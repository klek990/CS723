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

/* The parameters passed to the reg test tasks.  This is just done to check
 the parameter passing mechanism is working correctly. */
#define mainREG_TEST_1_PARAMETER    ( ( void * ) 0x12345678 )
#define mainREG_TEST_2_PARAMETER    ( ( void * ) 0x87654321 )
#define mainREG_TEST_PRIORITY       ( tskIDLE_PRIORITY + 1)
static void processSignalTask(void *pvParameters);
static void pollWallSwitchesTask(void *pvParameters);
int samplingFrequency = 16e3;

/*
 * Create the demo tasks then start the scheduler.
 */
int main(void)
{
	/* The RegTest tasks as described at the top of this file. */
	xTaskCreate( processSignalTask, "processSignal", configMINIMAL_STACK_SIZE, mainREG_TEST_1_PARAMETER, mainREG_TEST_PRIORITY, NULL);
	xTaskCreate( pollWallSwitchesTask, "pollWallSwitches", configMINIMAL_STACK_SIZE, mainREG_TEST_2_PARAMETER, mainREG_TEST_PRIORITY, NULL);

	/* Finally start the scheduler. */
	vTaskStartScheduler();

	/* Will only reach here if there is insufficient heap available to start
	 the scheduler. */
	for (;;);
}

static void processSignalTask(void *pvParameters)
{
	float freqNext = 0, freqPrev = 0, freqRoC = 0;
	int samplesPrev = 0, samplesNext = 0, avgSamples = 0;

	while (1)
	{
		samplesPrev = samplesNext;
		samplesNext = IORD_ALTERA_AVALON_PIO_DATA(FREQUENCY_ANALYSER_BASE);

		avgSamples = (samplesNext + samplesPrev)/2;

		freqPrev = samplingFrequency/(float)samplesPrev;
		freqNext = samplingFrequency/(float)samplesNext;

		freqRoC = ((freqNext - freqPrev)*samplingFrequency)/avgSamples;

		printf("Freq Next: %0.2f\n", freqNext);
		printf("Freq Prev: %0.2f\n", freqPrev);
		printf("Freq RoC: %0.2f\n", freqRoC);

		printf("Previous Samples: %d\n", samplesPrev);
		printf("New Samples: %d\n", samplesNext);
		printf("Average samples: %d\n\n", avgSamples);
		vTaskDelay(1000);
	}
}

static void pollWallSwitchesTask(void *pvParameters)
{
	int wallSwitch = 0;
	
	/* Initally, all LED's green */
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 255);

	while (1)
	{


		/* Read which wall switch is triggered */
		wallSwitch = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0b11111111;

		/*When load is set:
		* 		Red LED = on
		* 		Green LED = off
		* When load is shed:
		* 		Red LED = off
		* 		Green LED = on */
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, wallSwitch);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE & 0b11111111, ~wallSwitch);

		printf("Switch active: %d\n", wallSwitch);
		vTaskDelay(1000);
	}

}
