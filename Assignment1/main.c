/* Standard includes. */
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <system.h>
#include <altera_avalon_pio_regs.h>

/* Scheduler includes. */
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "freertos/semphr.h"

/* The parameters passed to the reg test tasks.  This is just done to check
 the parameter passing mechanism is working correctly. */
#define mainREG_TEST_1_PARAMETER    ( ( void * ) 0x12345678 )
#define mainREG_TEST_2_PARAMETER    ( ( void * ) 0x87654321 )
#define mainREG_TEST_PRIORITY       ( tskIDLE_PRIORITY + 1)
#define SAMPLINGFREQUENCY 16e3

/* Semaphores */
/* 0 = Normal, 1 = Load managing, 2 = Maintenance */
SemaphoreHandle_t systemState;

/* Frequency thresholds configured using keyboard */
SemaphoreHandle_t lowerThreshold;
SemaphoreHandle_t upperThreshold;

static void processSignalTask(void *pvParameters);
static void pollWallSwitchesTask(void *pvParameters);

/* Global frequency variables to be passed in queues */
float freqNext = 0, freqPrev = 0, freqRoC = 0;

/* Read signal from onboard FAU and do calculations */
void readFrequencyISR() 
{
	int samplesPrev = 0, samplesNext = 0, avgSamples = 0;
	samplesPrev = samplesNext;
	samplesNext = IORD_ALTERA_AVALON_PIO_DATA(FREQUENCY_ANALYSER_BASE);

	avgSamples = (samplesNext + samplesPrev)/2;

	freqPrev = SAMPLINGFREQUENCY/(float)samplesPrev;
	freqNext = SAMPLINGFREQUENCY/(float)samplesNext;

	freqRoC = ((freqNext - freqPrev)*SAMPLINGFREQUENCY)/avgSamples;

	printf("Freq Next: %0.2f\n", freqNext);
	printf("Freq Prev: %0.2f\n", freqPrev);
	printf("Freq RoC: %0.2f\n", freqRoC);

	return;
}

/*  */
void readKeyboardISR (void* context, alt_u32 id)
{

	return;
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

/*
 * Create the demo tasks then start the scheduler.
 */
int main(void)
{
	/* Register FAU */
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, readFrequencyISR);

	/* The RegTest tasks as described at the top of this file. */
	xTaskCreate( processSignalTask, "processSignal", configMINIMAL_STACK_SIZE, mainREG_TEST_1_PARAMETER, mainREG_TEST_PRIORITY, NULL);
	xTaskCreate( pollWallSwitchesTask, "pollWallSwitches", configMINIMAL_STACK_SIZE, mainREG_TEST_2_PARAMETER, mainREG_TEST_PRIORITY, NULL);

	/* Finally start the scheduler. */
	vTaskStartScheduler();

	/* Will only reach here if there is insufficient heap available to start
	 the scheduler. */
	for (;;);
}
