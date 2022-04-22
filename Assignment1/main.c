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
#define mainREG_TEST_PRIORITY (tskIDLE_PRIORITY + 1)
#define SAMPLINGFREQUENCY 16e3

#define SystemStateQueueSize 20

#define normalState 0
#define loadState 1
#define maintenanceState 2

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

TimerHandle_t xtimer200MS;
TimerHandle_t xtimer500MS;

/* Structures for received signal (freq, RoC, period) and which loads to shed */
struct signalInfoStruct
{
	float currentRoc;
	float currentFreq;
	float currentPeriod;
} signalInfo;

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
float freqThreshold = 50;

/* loadControlTask globals */
int loadsToChange = 0;
int sumOfLoads = 0;


//Systemstability (do not refer to)
bool currIsStable = true;
bool prevIsStable = true;
bool xTimer500Expired = false;

bool firstLoadShed = false; 

// Callbacks

void xTimer200MSCallback(TimerHandle_t xTimer)
{
	//Test Statement
	printf("TIMER 200 MS EXPIRED\n");

	/* If first load is not shed within 200ms, shed load manually */
	if(!firstLoadShed)
	{
		sumOfLoads += pow(2, loadsToChange);
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, sumOfLoads & 0b11111);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~sumOfLoads & 0b11111);
		printf("RoC out of bounds. Load shed: %d\n\n", sumOfLoads);
		loadsToChange++;

		/* After first load is shed, start the 500ms timer */
		xTimerStart(xtimer500MS, 0);

		firstLoadShed = true;
	}
	firstLoadShed = false;
	xTimerStop(xtimer200MS, 0);
}

void xTimer500MSCallback(TimerHandle_t xTimer)
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

	//Check the Stability state on expiry
	//If it is stable 
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
	int passToQueue = maintenanceState;
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
				// printf("signalInfoStruct sent to queue\n");
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
static void pollWallSwitchesTask(void *pvParameters)
{
	int sendWallSwitchValue = 0;

	while (1)
	{
		/* Read which wall switch is triggered */
		sendWallSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0b11111;
		if (xQueueSend(xWallSwitchQueue, &sendWallSwitchValue, NULL) == pdPASS)
		{
			if (currentSystemState == maintenanceState)
			{
				printf("Maintenance mode. Wall switch value sent to queue: %d \n", sendWallSwitchValue);
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
				if (latestStateValue == maintenanceState)
				{
					// Save the current state before maintenance
					prevStateBeforeMaintenance = currentSystemState;

					// Update the current state to the maintanenace state
					currentSystemState = maintenanceState;

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
				if (latestStateValue == maintenanceState)
				{
					printf("in this if\n");
					// Restore the system to what it was before
					currentSystemState = prevStateBeforeMaintenance;
					maintenanceActivated = false;
					wallSwitchValue = IORD(RED_LEDS_BASE, 0);

					printf("Overwriting sumOfLoads with %d", wallSwitchValue);


					printf("loads to shed: %d\n", __builtin_popcount(wallSwitchValue));
					loadsToChange = __builtin_popcount(wallSwitchValue);
					sumOfLoads = wallSwitchValue;
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
	while (1)
	{
		if (xQueueReceive(xSignalInfoQueue, &(receivedMessage), 50/portTICK_PERIOD_MS) == pdPASS)
		{
			prevIsStable = currIsStable;
			currIsStable = !(receivedMessage.currentFreq < freqThreshold || receivedMessage.currentRoc > rocThreshold);
			if(currentSystemState == normalState){
				//Instability is frist detected

				// Check if ROC is greater than ROC threshold, or if Frequency is below FREQ threshold
				//if it is stable and not in the watching state then do nothing
				if (!currIsStable)
				{
					// System is UNSTABLE

					// Signify that the load management state is needed
					systemStateUpdateValue = loadState;
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
						}
					}
				}
			}
			else if(currentSystemState == loadState)
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
		vTaskDelay(500);
		//while(uxQueueMessagesWaiting(xSignalInfoQueue) == 0){;}
	}
}

/* Switch loads on/off based on information in loadControlQueue (pollWallSwitches & checkSystemStability) */
static void loadControlTask(void *pvParameters)
{
	int localSystemState = 0;
	int wallSwitchTriggered = 0, turnOffLEDS = 0;
	bool isStable = false;
	int currentLEDS = 0;
	
	while (1)
	{
		// printf("CurrentSystemState: %d\n", currentSystemState);
		/* If in load managing, check systemStability */
		if (currentSystemState == loadState) 
		{
			// currentLEDS = IORD(RED_LEDS_BASE, 0);
			// turnOffLEDS = IORD(SLIDE_SWITCH_BASE,0) & currentLEDS;

			// /* Only allow LED's to be turned off */
			// if (currentLEDS == ~turnOffLEDS)
			// {
			// 	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, turnOffLEDS);
			// 	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~turnOffLEDS);
			// }

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
						printf("Load power: %d", loadsToChange);
						sumOfLoads -= pow(2, loadsToChange-1);
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, sumOfLoads & 0b11111);
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~sumOfLoads & 0b11111);
						printf("System stable. Turning on Load: %d\n\n", sumOfLoads);
						loadsToChange--;

						if (sumOfLoads == 0)
						{
							/* Set system state to normal (0) and stop the 500ms timer */
							localSystemState = normalState;
							if (xQueueSend(xSystemStateQueue, &(localSystemState), 50/portTICK_PERIOD_MS) == pdPASS)
							{
								printf("Normal mode\n");
								xTimerStop(xtimer500MS, 0);
							}
						}
					}
				}
			}
		}

		/* Wall switch interaction when in maintenance state */
		if (currentSystemState == maintenanceState)
		{
			if (xQueueReceive(xWallSwitchQueue, &wallSwitchTriggered, 50/portTICK_PERIOD_MS))
			{
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, wallSwitchTriggered);
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~wallSwitchTriggered & 0b11111);
			}
		}
	}
}

int initCreateTasks(void)
{
	xTaskCreate(processSignalTask, "processSignal", configMINIMAL_STACK_SIZE,
				mainREG_TEST_1_PARAMETER, mainREG_TEST_PRIORITY + 5, NULL);

	xTaskCreate(pollWallSwitchesTask, "pollWallSwitches", configMINIMAL_STACK_SIZE,
				mainREG_TEST_2_PARAMETER, mainREG_TEST_PRIORITY, NULL);

	xTaskCreate(manageSystemStateTask, "manageSystemStateTask", configMINIMAL_STACK_SIZE,
				mainREG_TEST_3_PARAMETER, mainREG_TEST_PRIORITY + 6, NULL);

	xTaskCreate(checkSystemStabilityTask, "checkSystemStabilityTask", configMINIMAL_STACK_SIZE,
				mainREG_TEST_4_PARAMETER, mainREG_TEST_PRIORITY + 5, NULL);

	xTaskCreate(loadControlTask, "loadControlTask", configMINIMAL_STACK_SIZE,
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
