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
#define mainREG_TEST_4_PARAMETER ((void *)0x87654321)
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
static QueueHandle_t xLoadControlQueue;

TimerHandle_t xtimer200MS;
TimerHandle_t xtimer500MS;

/* Structures for received signal (freq, RoC, period) and which loads to shed */
struct signalInfoStruct
{
	float currentRoc;
	float currentFreq;
	float currentPeriod;
} signalInfo;

struct loadInfoStruct
{
	int wallSwitchLoad;
	bool isStable;
} loadInfo;

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
int currentSystemState = 1;

// Thresholds
float rocThreshold = 7;
float freqThreshold = 50;

// Callbacks

void xTimer200MSCallback(TimerHandle_t xTimer)
{
}

void xTimer500MSCallback(TimerHandle_t xTimer)
{
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

/*  */
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
	while (1)
	{
		signalInfo.currentFreq = freqNext;
		signalInfo.currentRoc = freqRoc;
		signalInfo.currentPeriod = period;

		if (xQueueSend(xSignalInfoQueue, &signalInfo, NULL) == pdPASS)
		{
			printf("signalInfoStruct sent to queue\n");
		}
		else
		{
			printf("Process signal task failed sending to queue\n");
		}
		vTaskDelay(1000);
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
	int wallSwitchToQueue = 0;

	while (1)
	{
		/* Read which wall switch is triggered */
		wallSwitchToQueue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0b11111;

		if (xQueueSend(xLoadControlQueue, &wallSwitchToQueue, NULL) == pdPASS)
		{
			if (maintenanceActivated)
			{
				printf("Maintennace mode. Wall switch value sent to queue: %d \n", wallSwitchToQueue);
			}
		}

		vTaskDelay(1000);
	}
}

static void manageSystemStateTask(void *pvParameters)
{
	int latestStateValue;
	while (1)
	{
		if (xSystemStateQueue != NULL)
		{
			// Consume the value stored in the SystemStateQueue
			if (xQueueReceive(xSystemStateQueue, &(latestStateValue), 50/portTICK_PERIOD_MS) == pdPASS)
			{
				xSemaphoreTake(xSystemStateSemaphore, portMAX_DELAY);
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

						// Restore the system to what it was before
						currentSystemState = prevStateBeforeMaintenance;
						maintenanceActivated = false;
					}
				}

				xSemaphoreGive(xSystemStateSemaphore);
				printf("\nQueue Value Consumed: %d, System State Is Now: %d\n", latestStateValue, currentSystemState);
			}
			else
			{

			}
		}
	}
}

static void checkSystemStabilityTask(void *pvParameters)
{
	struct signalInfoStruct receivedMessage;
	int systemStateUpdateValue;
	while (1)
	{	
		if (xQueueReceive(xSignalInfoQueue, &(receivedMessage), 0) == pdPASS)
		{
			// Get the absolute roc Value
			if (receivedMessage.currentRoc < 0)
			{
				receivedMessage.currentRoc = receivedMessage.currentRoc * -1;
			}

			// Check if ROC is greater than ROC threshold, or if Frequency is below FREQ threshold
			if (receivedMessage.currentFreq < freqThreshold || receivedMessage.currentRoc > rocThreshold)
			{
				// System is UNSTABLE

				// Signify that the load management state is needed
				systemStateUpdateValue = loadState;
				if (xQueueSend(xSystemStateQueue, &systemStateUpdateValue, NULL) == pdPASS)
				{
					printf("\nLoad Managing State sucessfully sent to SystemStateQueue\n");
				}

				// Signify that the lowest priority load must be shed

				// start the 500 ms timer
				// delayThisTask by 200ms to ensure load shedding

				// Implement logic to being watching
			}
		}
		vTaskDelay(1000);
	}
}
int loadsToChange = 0;
int countDownBit = 3;
int sumOfLoads = 0;

/* Keeps track of rollover  */
int shedLoad = 0;

/* Switch loads on/off based on information in loadControlQueue (pollWallSwitches & checkSystemStability) */
static void loadControlTask(void *pvParameters)
{

	int wallSwitchTriggered = 0;
	float receivedRoc = 0;
	struct signalInfoStruct receivedSignalInfo;
	struct loadInfoStruct receivedLoadInfo;

	/* Testing if load managing correct */
	currentSystemState = 1;

	while (1)
	{
		if (xQueueReceive(xSignalInfoQueue, &(receivedSignalInfo), 0) == pdPASS)
		{
			wallSwitchTriggered = receivedLoadInfo.wallSwitchLoad;
			if (xQueueReceive(xLoadControlQueue, &(receivedLoadInfo), NULL) == pdPASS)
			{
				receivedRoc = receivedSignalInfo.currentRoc;
				if (!loadInfo.isStable)
				{

					printf("UNSTABLE: %f\n", receivedRoc);
					if (maintenanceActivated)
					{
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, wallSwitchTriggered);
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~wallSwitchTriggered & 0b11111);
					}

					/* If RoC threshold is exceeded and is in load managing state, start shedding loads one by one until
					 *  stability criteria satisfied. */
					else if (currentSystemState == 1 && receivedRoc > rocThreshold && sumOfLoads < 31)
					{
						sumOfLoads += pow(2, loadsToChange);
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, sumOfLoads & 0b11111);
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~sumOfLoads & 0b11111);
						printf("RoC out of bounds: %f. Load shed: %d\n\n", receivedRoc, sumOfLoads);
						loadsToChange++;

						/* FOR TESTING PURPOSES */
						// if (sumOfLoads == 31)
						// {
						// 	loadInfo.isStable = 1;
						// }
					}
				}

				/* If system is stable, start turning loads back on from highest priority */
				else if (loadInfo.isStable)
				{
					/* receiveROC for testing only */
					receivedRoc = receivedSignalInfo.currentRoc;
					printf("STABLE: %f\n", receivedRoc);

					if (maintenanceActivated)
					{
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, wallSwitchTriggered);
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~wallSwitchTriggered & 0b11111);
						// printf("System unstable. loadInfo wall switch value: %d\n", receivedLoadInfo.wallSwitchLoad);
					}
					else if (currentSystemState == 1 && loadsToChange >= 0)
					{
						printf("Load power: %d", loadsToChange);
						sumOfLoads -= pow(2, loadsToChange - 1);
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, sumOfLoads & 0b11111);
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ~sumOfLoads & 0b11111);
						printf("System stable at: %f. Turning on Load: %d\n\n", receivedRoc, sumOfLoads);
						loadsToChange--;

						/* FOR TESTING PURPOSES */
						// if (sumOfLoads == 0)
						// {
						// 	loadInfo.isStable = 0;
						// }
					}
				}
			}
		}
		vTaskDelay(1000);
	}
}

int initCreateTasks(void)
{
	xTaskCreate(processSignalTask, "processSignal", configMINIMAL_STACK_SIZE,
				mainREG_TEST_1_PARAMETER, mainREG_TEST_PRIORITY + 5, NULL);

	xTaskCreate(pollWallSwitchesTask, "pollWallSwitches", configMINIMAL_STACK_SIZE,
				mainREG_TEST_2_PARAMETER, mainREG_TEST_PRIORITY, NULL);

	xTaskCreate(manageSystemStateTask, "manageSystemStateTask", configMINIMAL_STACK_SIZE,
				mainREG_TEST_3_PARAMETER, mainREG_TEST_PRIORITY + 3, NULL);

	xTaskCreate(checkSystemStabilityTask, "checkSystemStabilityTask", configMINIMAL_STACK_SIZE,
				mainREG_TEST_3_PARAMETER, mainREG_TEST_PRIORITY + 5, NULL);

	xTaskCreate(loadControlTask, "loadControlTask", configMINIMAL_STACK_SIZE,
				mainREG_TEST_4_PARAMETER, mainREG_TEST_PRIORITY + 4, NULL);

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
	xLoadControlQueue = xQueueCreate(SystemStateQueueSize, sizeof(int));
	xSignalInfoQueue = xQueueCreate(SystemStateQueueSize, sizeof(struct signalInfoStruct));
	if (xSystemStateQueue == NULL)
	{
		printf("\nUnable to Create Integer SystemStateQueue");
	}
	else
	{
		printf("\nSystemStateQueue Created Successfully");
	}

	xSystemStateSemaphore = xSemaphoreCreateBinary();

	if (xSystemStateSemaphore == NULL)
	{
		printf("\nUnable to Create systemState Semaphore");
	}
	else
	{
		xSemaphoreGive(xSystemStateSemaphore);
		printf("\n System State Semaphore successfully created");
	}

	xtimer200MS = xTimerCreate("timer200MS", 200 / portTICK_PERIOD_MS, pdFALSE, (void *)0, xTimer200MSCallback);
	if (xtimer200MS == NULL)
	{
		printf("200 MS Timer not successfully created");
	}

	xtimer500MS = xTimerCreate("timer500MS", 500 / portTICK_PERIOD_MS, pdFALSE, (void *)1, xTimer500MSCallback);
	if (xtimer500MS == NULL)
	{
		printf("500 MS Timer not successfully created");
	}

	/* Finally start the scheduler. */
	vTaskStartScheduler();

	/* Will only reach here if there is insufficient heap available to start
	 the scheduler. */
	for (;;)
		;
}
