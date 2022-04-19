/* Standard includes. */
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <system.h>
#include <altera_avalon_pio_regs.h>
#include "io.h"
#include "sys/alt_irq.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"

/* Scheduler includes. */
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "freertos/semphr.h"

/* The parameters passed to the reg test tasks.  This is just done to check
 the parameter passing mechanism is working correctly. */
#define mainREG_TEST_1_PARAMETER ((void *)0x12345678)
#define mainREG_TEST_2_PARAMETER ((void *)0x87654321)
#define mainREG_TEST_2_PARAMETER ((void *)0x12348765)
#define mainREG_TEST_PRIORITY (tskIDLE_PRIORITY + 1)
#define SAMPLINGFREQUENCY 16e3

#define SystemStateQueueSize 20

#define normalState 0
#define loadState 1
#define managementState 2


/* Semaphores */
/* 0 = Normal, 1 = Load managing, 2 = Maintenance */
SemaphoreHandle_t systemState;

/* Frequency thresholds configured using keyboard */
SemaphoreHandle_t lowerThreshold;
SemaphoreHandle_t upperThreshold;

//Define Queue for SystemState
static QueueHandle_t xSystemStateQueue;


static void processSignalTask(void *pvParameters);
static void pollWallSwitchesTask(void *pvParameters);
static void manageSystemStateTask(void *pvParameters);

/* Global frequency variables to be passed in queues */
float freqNext = 0, freqPrev = 0, freqRoC = 0;

/* Read signal from onboard FAU and do calculations */
void readFrequencyISR()
{
	int samplesPrev = 0, samplesNext = 0, avgSamples = 0;
	samplesPrev = samplesNext;
	samplesNext = IORD_ALTERA_AVALON_PIO_DATA(FREQUENCY_ANALYSER_BASE);

	avgSamples = (samplesNext + samplesPrev) / 2;

	freqPrev = SAMPLINGFREQUENCY / (float)samplesPrev;
	freqNext = SAMPLINGFREQUENCY / (float)samplesNext;

	freqRoC = ((freqNext - freqPrev) * SAMPLINGFREQUENCY) / avgSamples;

	printf("Freq Next: %0.2f\n", freqNext);
	printf("Freq Prev: %0.2f\n", freqPrev);
	printf("Freq RoC: %0.2f\n", freqRoC);

	return;
}

/*  */
void readKeyboardISR(void *context, alt_u32 id)
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

void maintenanceStateISR(void *context){
	int passToQueue = managementState;
	if(xQueueSendFromISR(xSystemStateQueue, &passToQueue, NULL) == pdPASS){
		printf("\nMaintenance State sucessfully sent to SystemStateQueue");
	};
	//Clear edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_PIO_BASE, 0x00);
}

/* This should be used to send the signal information to the other tasks */
static void processSignalTask(void *pvParameters)
{
	while (1)
	{
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
	int wallSwitch = 0;

	/* Initially, all LED's green */
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 255);

	while (1)
	{
		/* Read which wall switch is triggered */
		wallSwitch = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0b11111111;

		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, wallSwitch);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE & 0b11111111, ~wallSwitch);

		vTaskDelay(1000);
	}
}

static void manageSystemStateTask(void *pvParameters)
{	
	int latestStateValue;


	while(1)
	{
		
		if(xSystemStateQueue != NULL){
			//Consume the value stored in the SystemStateQueue
			if(xQueueReceive(xSystemStateQueue, &(latestStateValue), 0) == pdPASS) {
				printf("\nQueue Value Consumed: System State Is Now: %d", latestStateValue);
			}
			else {
				printf("\nQueue Read Failed");
			}
			//Take the semaphore


		}
		//If there are two consecutive "2", the state needs to be set to 
	}
}




int initCreateTasks(void){
	xTaskCreate(processSignalTask, "processSignal", configMINIMAL_STACK_SIZE, 
		mainREG_TEST_1_PARAMETER, mainREG_TEST_PRIORITY + 5, NULL);

	xTaskCreate(pollWallSwitchesTask, "pollWallSwitches", configMINIMAL_STACK_SIZE, 
		mainREG_TEST_2_PARAMETER, mainREG_TEST_PRIORITY, NULL);

	xTaskCreate(manageSystemStateTask, "manageSystemStateTask", configMINIMAL_STACK_SIZE, 
		mainREG_TEST_3_PARAMETER, mainREG_TEST_PRIORITY + 3, NULL);

	return 0;
}

/*
 * Create the demo tasks then start the scheduler.
 */
int main(void)
{
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
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_PIO_BASE, 0x00);

	//Enable Interrupts to button 1
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_PIO_BASE, 0x01);

	/* Register Button ISR */
	alt_irq_register(PUSH_BUTTON_PIO_IRQ, 0, maintenanceStateISR);

	/* The RegTest tasks as described at the top of this file. */
	initCreateTasks();

	xSystemStateQueue = xQueueCreate(SystemStateQueueSize, sizeof(int));
	if(xSystemStateQueue == NULL){
		printf("\nUnable to Create Integer SystemStateQueue");
	} 
	else {
		printf("\nSystemStateQueue Created Successfully")
	}

	/* Finally start the scheduler. */
	vTaskStartScheduler();



	/* Will only reach here if there is insufficient heap available to start
	 the scheduler. */
	for (;;)
		;
}
