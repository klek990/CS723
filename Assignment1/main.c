/* Standard includes. */
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <system.h>
#include <altera_avalon_pio_regs.h>
#include <stdbool.h>
#include "io.h"
#include "sys/alt_irq.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"

/* Scheduler includes. */
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "freertos/semphr.h"
#include "FreeRTOS/timers.h"

/* The parameters passed to the reg test tasks.  This is just done to check
 the parameter passing mechanism is working correctly. */
#define mainREG_TEST_1_PARAMETER ((void *)0x12345678)
#define mainREG_TEST_2_PARAMETER ((void *)0x87654321)
#define mainREG_TEST_3_PARAMETER ((void *)0x12348765)
#define mainREG_TEST_4_PARAMETER ((void *)0x78654321)
#define mainREG_TEST_5_PARAMETER ((void *)0x76854321)
#define mainREG_TEST_6_PARAMETER ((void *)0x76854312)
#define mainREG_TEST_PRIORITY (tskIDLE_PRIORITY + 1)
#define SAMPLINGFREQUENCY 16e3
#define SystemStateQueueSize 20

/* STATES */
#define NORMALSTATE 0
#define LOADSTATE 1
#define MAINTENANCESTATE 2

/* Plot VGA stuff */
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 //minimum frequency to draw
#define PRVGADraw_Task_P     (tskIDLE_PRIORITY+1)
TaskHandle_t PRVGADraw;


/* Semaphores */
/* 0 = Normal, 1 = Load managing, 2 = Maintenance */
SemaphoreHandle_t xSystemStateSemaphore;

/* Frequency thresholds configured using keyboard */
SemaphoreHandle_t lowerThreshold;
SemaphoreHandle_t upperThreshold;

// Define Queues
static QueueHandle_t xSignalInfoQueue;
static QueueHandle_t xSystemStateQueue;
static QueueHandle_t xWallSwitchQueue;
static QueueHandle_t xSystemStabilityQueue;
static QueueHandle_t xVGAFrequencyData;

TimerHandle_t xtimer200MS;
TimerHandle_t xtimer500MS;

/* Structures for received signal (freq, RoC, period) and which loads to shed */
struct signalInfoStruct
{
	float currentRoc;
	float currentFreq;
	float currentPeriod;
} signalInfo;

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

static void processSignalTask(void *pvParameters);
static void pollWallSwitchesTask(void *pvParameters);
static void manageSystemStateTask(void *pvParameters);

/* Global frequency variables to be passed in queues */
float freqNext = 0, freqPrev = 0, freqRoc = 0, period = 0;
int samplesPrev = 0, samplesNext = 0, avgSamples = 0;

// For system state management
int prevStateBeforeMaintenance = 1;
bool maintenanceActivated = false;

// MUST BE PROTECTED
int currentSystemState = 0;

// Thresholds
float rocThreshold = 7;
float freqThreshold = 48;

/* loadControlTask globals */
int loadsToChange = 0;
int sumOfLoads = 0;


//Systemstability (do not refer to)
bool currIsStable = true;
bool prevIsStable = true;
bool xTimer500Expired = false;

bool firstLoadShed = false; 

//Load control 2 globals for test
//Start off all loads as 
int currentAssignedLoads = 0b11111;
SemaphoreHandle_t xCurrentOnLoadSemaphore;
int prevWallSwitchValue = 0;
int sendWallSwitchValue = 0;

// Callbacks

/*
void xTimer200MSCallback(TimerHandle_t xTimer)
{
	//Test Statement
	printf("TIMER 200 MS EXPIRED\n");

	// If first load is not shed within 200ms, shed load manually
	if(!firstLoadShed)
	{
		sumOfLoads += pow(2, loadsToChange);
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, sumOfLoads & 0b11111);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~sumOfLoads & 0b11111);
		printf("RoC out of bounds. Load shed: %d\n\n", sumOfLoads);
		loadsToChange++;

		// After first load is shed, start the 500ms timer 
		xTimerStart(xtimer500MS, 0);

		firstLoadShed = true;
	}
	firstLoadShed = false;
	xTimerStop(xtimer200MS, 0);
}
*/

void xTimer200MSCallback(TimerHandle_t xTimer)
{
	//Test Statement
	printf("TIMER 200 MS EXPIRED\n");

	/* If first load is not shed within 200ms, shed load manually */
	if(!firstLoadShed)
	{
		//TAKE THE SEMAPHORE
		xSemaphoreTake(xCurrentOnLoadSemaphore, 0);

		//DO LOAD SHEDDING
		currentAssignedLoads = currentAssignedLoads&(currentAssignedLoads - 1);

		//Write to LEDS
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, currentAssignedLoads & 0b11111);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~currentAssignedLoads & 0b11111);
		printf("Load shed by timer");

		/* After first load is shed, start the 500ms timer */
		xTimerStart(xtimer500MS, 0);

		//GIVE THE SEMAPHORE
		xSemaphoreGive(xCurrentOnLoadSemaphore);

	}
	xTimerStop(xtimer200MS, 0);
}

void xTimer500MSCallback(TimerHandle_t xTimer)
{
	if (currentSystemState == LOADSTATE)
	{
		bool isStable = false;
		//Test Statement
		printf("TIMER 500 MS EXPIRED\n");
		if(!currIsStable)
		{
			//Tell the loadControlQueue to decrement load
			isStable = false;
			if (xQueueSend(xSystemStabilityQueue, &isStable, 50/portTICK_PERIOD_MS) == pdPASS)
			{
				printf("UNSTABLE loadcontrol queue FROM TIMER\n");

			}
		}
		else 
		{
			//Tell the loadControlQueue to increment load if the system is stable
			isStable = true;
			if (xQueueSend(xSystemStabilityQueue, &isStable, 50/portTICK_PERIOD_MS) == pdPASS)
			{
				printf("STABLE loadcontrol queue FROM TIMER\n");

			}
		}
		xTimer500Expired = true;
	}
}

/* Read signal from onboard FAU and do calculations */
void readFrequencyISR(void *context)
{
	samplesPrev = samplesNext;
	samplesNext = IORD(FREQUENCY_ANALYSER_BASE, 0);

	avgSamples = (samplesNext + samplesPrev) / 2;

	freqPrev = SAMPLINGFREQUENCY / (float)samplesPrev;
	freqNext = SAMPLINGFREQUENCY / (float)samplesNext;

	period = freqNext / (float)2;

	/* RoC should be irrespective of  */
	freqRoc = ((freqNext - freqPrev) * SAMPLINGFREQUENCY) / avgSamples;
	if (freqRoc < 0)
	{
		freqRoc *= (float)-1;
	}

	// printf("Freq Next: %0.2f\n", freqNext);
	// printf("Freq Prev: %0.2f\n", freqPrev);
	// printf("Freq RoC: %0.2f\n", freqRoc);
	
	return;
}

/* Read keyboard */
static void readKeyboardISR(void *context, alt_u32 id)
{
	char ascii;
	int status = 0;
	unsigned char key = 0;
	KB_CODE_TYPE decode_mode;
	status = decode_scancode(context, &decode_mode, &key, &ascii);
	if (status == 0) // success
	{
		// print out the result
		switch (decode_mode)
		{
			case KB_ASCII_MAKE_CODE:
				printf("ASCII   : %c\n", ascii);
				break;
			case KB_LONG_BINARY_MAKE_CODE:
				// do nothing
			case KB_BINARY_MAKE_CODE:
				printf("MAKE CODE : %x\n", key);
				break;
			case KB_BREAK_CODE:
				// do nothing
			default:
				printf("DEFAULT   : %x\n", key);
				break;
		}
	}

	/* 'ascii' holds the character, can be represented as upper case of that letter (pressing k gives K)
	 * 	Further, we can simply read the letter input to manage the upper and lower threshold. If 'U' is
	 *	pressed, we then read the next set of number key strokes to assign a value to the upper threshold
	 * 	vice versa for lower threshold.*/
	
	IOWR(SEVEN_SEG_BASE, 0, ascii);
	return;
}

static void maintenanceStateISR(void *context)
{
	int passToQueue = MAINTENANCESTATE;
	if (xQueueSendFromISR(xSystemStateQueue, &passToQueue, NULL) == pdPASS)
	{
		printf("\nMaintenance State sucessfully sent to SystemStateQueue\n");
	}

	// Clear edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x01);
}

/* This should be used to send the signal information to the other tasks */
static void processSignalTask(void *pvParameters)
{
	struct signalInfoStruct sendSignalInfo;
	while (1)
	{
		if (freqRoc != INFINITY)
		{
			sendSignalInfo.currentFreq = freqNext;
			sendSignalInfo.currentRoc = freqRoc;
			sendSignalInfo.currentPeriod = period;

			if (xQueueSend(xSignalInfoQueue, &sendSignalInfo, 50/portTICK_PERIOD_MS) == pdPASS)
			{
				if (xQueueSend(xVGAFrequencyData, &sendSignalInfo.currentFreq, NULL) == pdPASS)
				{
					printf("%f\n", sendSignalInfo.currentFreq);
				}
			}
			else
			{
				// printf("Process signal task failed sending to queue\n");
			}
		}
	}
}

/* When load is set:
 * 		Red LED = on
 * 		Green LED = off
 * When load is shed:
 * 		Red LED = off
 * 		Green LED = on */
static void pollWallSwitchesTask(void *pvParameters){
	while (1)
	{
		/* Read which wall switch is triggered */
		prevWallSwitchValue = sendWallSwitchValue;
		sendWallSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0b11111;
		if(prevWallSwitchValue != sendWallSwitchValue){
			if (xQueueSend(xWallSwitchQueue, &sendWallSwitchValue, NULL) == pdPASS)
			{
				if (currentSystemState == MAINTENANCESTATE)
				{
					printf("Maintenance mode. Wall switch value sent to queue: %d \n", sendWallSwitchValue);
				}
			}
		}
		vTaskDelay(500);
	}
}

static void manageSystemStateTask(void *pvParameters)
{
	printf("in here\n");
	int latestStateValue, wallSwitchValue;
	while (1)
	{
		// Consume the value stored in the SystemStateQueue
		if (xQueueReceive(xSystemStateQueue, &(latestStateValue), 50/portTICK_PERIOD_MS) == pdPASS)
		{
			xSemaphoreTake(xSystemStateSemaphore, 0);
			// Critical Section

			// If maintenance not activated, normal operation
			if (!maintenanceActivated)
			{
				// check to see if maintenance state is the latestvalue
				if (latestStateValue == MAINTENANCESTATE)
				{
					// Save the current state before maintenance
					prevStateBeforeMaintenance = currentSystemState;

					// Update the current state to the maintanenace state
					currentSystemState = MAINTENANCESTATE;

					// Activate the maintenance flag
					maintenanceActivated = true;
				}
				else
				{
					printf("in this section\n");
					// Normal Operation
					currentSystemState = latestStateValue;
				}
			}
			else
			{
				// Ignore all other values unless it is button, then restore last non maintenance state
				// Maintenance State stops load management, there
				if (latestStateValue == MAINTENANCESTATE)
				{
					printf("in this if\n");
					// Restore the system to what it was before
					currentSystemState = prevStateBeforeMaintenance;
					maintenanceActivated = false;

					/*wallSwitchValue = IORD(SLIDE_SWITCH_BASE, 0);
					printf("Overwriting sumOfLoads with %d", wallSwitchValue);
					loadsToChange = __builtin_popcount(wallSwitchValue);
					sumOfLoads = wallSwitchValue;
					printf("Sum to shed: %d\n", wallSwitchValue);
					printf("Pop count to shed: %d\n", loadsToChange); */
				}
			}

			xSemaphoreGive(xSystemStateSemaphore);
			printf("\nQueue Value Consumed: %d, System State Is Now: %d\n", latestStateValue, currentSystemState);
		}
	}
}

static void checkSystemStabilityTask(void *pvParameters)
{
	struct signalInfoStruct receivedMessage;
	bool isStable = false;
	int systemStateUpdateValue;
	int pxRxedMessage;
	while (1)
	{
		if (xQueueReceive(xSignalInfoQueue, &(receivedMessage), 50/portTICK_PERIOD_MS) == pdPASS)
		{
			prevIsStable = currIsStable;
			currIsStable = !(receivedMessage.currentFreq < freqThreshold || receivedMessage.currentRoc > rocThreshold);
			// printf("currIsStable: %d", currIsStable);
			if(currentSystemState == NORMALSTATE){
				//Instability is frist detected

				// Check if ROC is greater than ROC threshold, or if Frequency is below FREQ threshold
				//if it is stable and not in the watching state then do nothing
				if (!currIsStable)
				{
					// System is UNSTABLE

					// Signify that the load management state is needed
					systemStateUpdateValue = LOADSTATE;
					if (xQueueSend(xSystemStateQueue, &systemStateUpdateValue, NULL) == pdPASS)
					{

						printf("\nLoad Managing State sucessfully sent to SystemStateQueue %d\n", systemStateUpdateValue);
						//send that the system is not stable
						isStable = false;

						if (xQueueSend(xSystemStabilityQueue, &isStable, NULL) == pdPASS)
						{
							printf("\nStability Information added to loadcontrol queue: %d\n", isStable);	
							//Start the 200ms timer to show that first loadshedding must happen
							xTimerStart(xtimer200MS, 0);
							firstLoadShed = false;
						}
					}
				}
			}
			else if(currentSystemState == LOADSTATE)
			{
				//If timer 500 hasnt expired AND the system status changes, restart the timer
				if((prevIsStable != currIsStable) && !xTimer500Expired)
				{
					xTimerReset(xtimer500MS, 0);
				}

				//If it has expired, reset the flag;
				if(xTimer500Expired){
					xTimer500Expired = false;
				}

				// Otherwise let the timer expire and it will check if the system has been stable or not

			}
		}
		vTaskDelay(100);
	}
}

/* Switch loads on/off based on information in loadControlQueue (pollWallSwitches & checkSystemStability) */
static void loadControlTask(void *pvParameters)
{
	int localSystemState = 0;
	int wallSwitchTriggered = 0;
	bool isStable = false;
	
	while (1)
	{
		// printf("CurrentSystemState: %d\n", currentSystemState);
		/* If in load managing, check systemStability */
		if (currentSystemState == LOADSTATE) 
		{
			if (xQueueReceive(xSystemStabilityQueue, &isStable, 50/portTICK_PERIOD_MS) == pdPASS)
			{
				/* If not stable, begin to shed loads */
				if (!isStable)
				{
					printf("LoadInfoStability: %d\n", isStable);
					printf("UNSTABLE: %f\n", freqRoc);

					/* If all the loads are not shed, keep shedding loads from lowest to highest significance */
					if (sumOfLoads < 31)
					{
						sumOfLoads += pow(2, loadsToChange);
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, sumOfLoads & 0b11111);
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~sumOfLoads & 0b11111);
						printf("RoC out of bounds. Load shed: %d\n\n", sumOfLoads);
						loadsToChange++;

						/* Start 500ms timer when first load is shed, set flag to true. */
						if(!firstLoadShed)
						{
							xTimerStart(xtimer500MS, 0);
							firstLoadShed = true;
						}
					}
				}

				/* If system is stable, start turning loads back on from highest priority */
				else if (isStable)
				{
					printf("STABLE: %f\n", freqRoc);

					if (loadsToChange >= 0)
					{
						sumOfLoads -= pow(2, loadsToChange-1);
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, sumOfLoads & 0b11111);
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~sumOfLoads & 0b11111);
						printf("System stable. Turning on Load: %d\n\n", sumOfLoads);
						loadsToChange--;

						if (sumOfLoads == 0)
						{
							/* Set system state to normal (0) and stop the 500ms timer */
							localSystemState = NORMALSTATE;
							if (xQueueSend(xSystemStateQueue, &(localSystemState), 50/portTICK_PERIOD_MS) == pdPASS)
							{
								printf("Normal mode\n");
							}
						}
					}
				}
			}
		}

		/* Wall switch interaction when in maintenance state */
		if (currentSystemState == MAINTENANCESTATE)
		{
			if (xQueueReceive(xWallSwitchQueue, &wallSwitchTriggered, 50/portTICK_PERIOD_MS))
			{
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, wallSwitchTriggered);
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~wallSwitchTriggered & 0b11111);
			}
		}
	}
}

/* VGA Display */
void PRVGADraw_Task(void *pvParameters )
{
	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);

	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);

	double freq[100], dfreq[100];
	int i = 99, j = 0;
	Line line_freq, line_roc;

	while(1){

		//receive frequency data from queue
		while(uxQueueMessagesWaiting( xVGAFrequencyData ) != 0){
			xQueueReceive( xVGAFrequencyData, freq+i, 0 );
			printf("%f\n", freq[i]);
			
			//calculate frequency RoC

			if(i==0){
				dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);	
			}
			else{
				dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
			}

			if (dfreq[i] > 100.0){
				dfreq[i] = 100.0;
			}


			i =	++i%100; //point to the next data (oldest) to be overwritten

		}

		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

		for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(i+j)%100]) > MIN_FREQ) && ((int)(freq[(i+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
			}
		}
		vTaskDelay(10);

	}
}


static void loadControlTask2(void *pvParameters)
{
	int localSystemState = NORMALSTATE;
	int receivedSwitchValue = 0;
	bool isStable = false;
	while (1)
	{
		if (currentSystemState == NORMALSTATE){
			//DO NOTHING IN THE NORMAL STATE
		} 
		else if (currentSystemState == LOADSTATE){
			if (xQueueReceive(xSystemStabilityQueue, &isStable, 50/portTICK_PERIOD_MS) == pdPASS){
				//Take the semaphore
				xSemaphoreTake(xCurrentOnLoadSemaphore, 0);

				if (xQueueReceive(xWallSwitchQueue, &receivedSwitchValue, 50/portTICK_PERIOD_MS) == pdPASS)
				{
					printf("\nAcknowledged Manual Switch Change in LOAD STATE\n");
					//And because we are not allowed to turn any switches on, but we cann turn them off;
					currentAssignedLoads &= receivedSwitchValue;
				}
				if(isStable){
					//TURN ON MSB
					printf("System stable. Turning on Load");
					//FIGURE OUT WHERE MSB unset bit is:
					int pos = 0;
					int temp = currentAssignedLoads;
					for (int i=0; i < 5; i++){
						if ((temp & (1 << i)) == 0){
							pos = i;
						}
					}
					currentAssignedLoads |= (1 << pos);
				}
				else {
					//TURN OFF LSB 
					printf("System unStable. Turning off Load");
					currentAssignedLoads = currentAssignedLoads&(currentAssignedLoads - 1);

					//If the flag syaing it has been serviced hasnt been set, set it to true
					if(!firstLoadShed){
						firstLoadShed = true;
						xTimerStart(xtimer500MS, 0);
					}
				}
				//write to leds
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, currentAssignedLoads & 0b11111);
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~currentAssignedLoads & 0b11111);

				//Check to see if stability reached
				if(currentAssignedLoads == 0b11111){
					if (xQueueSend(xSystemStateQueue, &(localSystemState), 50/portTICK_PERIOD_MS) == pdPASS)
					{
						printf("Normal mode\n");
					}
				}
				//RELEASE THE SEMAPHORE
				xSemaphoreGive(xCurrentOnLoadSemaphore);
			} 
		}
		else {
			//Maintenance state
			if (xQueueReceive(xWallSwitchQueue, &receivedSwitchValue, 50/portTICK_PERIOD_MS))
			{
				xSemaphoreTake(xCurrentOnLoadSemaphore, 0);
				currentAssignedLoads = receivedSwitchValue;
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, currentAssignedLoads & 0b11111);
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0b00000);
				xSemaphoreGive(xCurrentOnLoadSemaphore);
			}
		}
	}
}

int initCreateTasks(void)
{
	// xTaskCreate( PRVGADraw_Task, "DrawTsk", configMINIMAL_STACK_SIZE, NULL, PRVGADraw_Task_P, &PRVGADraw );
	
	xTaskCreate(processSignalTask, "processSignal", configMINIMAL_STACK_SIZE,
				mainREG_TEST_1_PARAMETER, mainREG_TEST_PRIORITY + 5, NULL);

	xTaskCreate(pollWallSwitchesTask, "pollWallSwitches", configMINIMAL_STACK_SIZE,
				mainREG_TEST_2_PARAMETER, mainREG_TEST_PRIORITY, NULL);

	xTaskCreate(manageSystemStateTask, "manageSystemStateTask", configMINIMAL_STACK_SIZE,
				mainREG_TEST_3_PARAMETER, mainREG_TEST_PRIORITY + 6, NULL);

	xTaskCreate(checkSystemStabilityTask, "checkSystemStabilityTask", configMINIMAL_STACK_SIZE,
				mainREG_TEST_4_PARAMETER, mainREG_TEST_PRIORITY + 5, NULL);

	//xTaskCreate(loadControlTask, "loadControlTask", configMINIMAL_STACK_SIZE,
				//mainREG_TEST_5_PARAMETER, mainREG_TEST_PRIORITY + 4, NULL);

	xTaskCreate(loadControlTask2, "loadControlTask2", configMINIMAL_STACK_SIZE,
			mainREG_TEST_5_PARAMETER, mainREG_TEST_PRIORITY + 4, NULL);

	

	return 0;
}

/*
 * Create the demo tasks then start the scheduler.
 */
int main(void)
{
	/* Initially, all LED's green and red LED's off */
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 255);
	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, 0);
	alt_up_ps2_dev *keyboard = alt_up_ps2_open_dev(PS2_NAME);

	if (keyboard == NULL)
	{
		printf("can't find PS/2 device\n");
		return 1;
	}

	alt_up_ps2_clear_fifo(keyboard);

	/* Register FAU, keyboard and push button */
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, readFrequencyISR);
	alt_irq_register(PS2_IRQ, keyboard, readKeyboardISR);

	/* Register keyboard interrupt */
	IOWR_8DIRECT(PS2_BASE, 4, 1);

	/* Clear edge cap register */
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x01);

	// Enable Interrupts to button 1
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x01);

	/* Register Button ISR */
	alt_irq_register(PUSH_BUTTON_IRQ, 0, maintenanceStateISR);

	

	/* The RegTest tasks as described at the top of this file. */
	initCreateTasks();

	xSystemStateQueue = xQueueCreate(SystemStateQueueSize, sizeof(int));
	xWallSwitchQueue = xQueueCreate(SystemStateQueueSize, sizeof(int));
	xSystemStabilityQueue = xQueueCreate(SystemStateQueueSize, sizeof(int));
	xSignalInfoQueue = xQueueCreate(SystemStateQueueSize, sizeof(struct signalInfoStruct));
	xVGAFrequencyData = xQueueCreate( 100, sizeof(float) );

	
	if (xSystemStateQueue == NULL)
	{
		printf("Unable to Create Integer SystemStateQueue\n");
	}
	else
	{
		printf("SystemStateQueue Created Successfully\n");
	}

	xSystemStateSemaphore = xSemaphoreCreateBinary();

	if (xSystemStateSemaphore == NULL)
	{
		printf("Unable to Create systemState Semaphore\n");
	}
	else
	{
		xSemaphoreGive(xSystemStateSemaphore);
		printf("System State Semaphore successfully created\n");
	}

	xCurrentOnLoadSemaphore = xSemaphoreCreateBinary();

	if (xCurrentOnLoadSemaphore == NULL)
	{
		printf("Unable to Create xCurrentOnLoadSemaphore\n");
	}
	else
	{
		xSemaphoreGive(xCurrentOnLoadSemaphore);
		printf("xCurrentOnLoadSemaphore successfully created\n");
	}

	xtimer200MS = xTimerCreate("timer200MS", 200 / portTICK_PERIOD_MS, pdFALSE, (void *)0, xTimer200MSCallback);
	if (xtimer200MS == NULL)
	{
		printf("200 MS Timer not successfully created\n");
	}

	xtimer500MS = xTimerCreate("timer500MS", 500 / portTICK_PERIOD_MS, pdFALSE, (void *)1, xTimer500MSCallback);
	if (xtimer500MS == NULL)
	{
		printf("500 MS Timer not successfully created\n");
	}

	/* Finally start the scheduler. */
	vTaskStartScheduler();

	/* Will only reach here if there is insufficient heap available to start
	 the scheduler. */
	for (;;)
		;
}
