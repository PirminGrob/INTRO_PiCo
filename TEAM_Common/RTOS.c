/**
0 * \file
 * \brief Real Time Operating System (RTOS) main program.
 * \author Erich Styger, erich.styger@hslu.ch
 */

#include "Platform.h"
#if PL_CONFIG_HAS_RTOS
#include "RTOS.h"
#include "FRTOS1.h"
#include "Application.h"

void RTOS_Init(void) {
	BaseType_t res;
	xTaskHandle taskHndl;

	res = xTaskCreate(PiCo_Blinky_Task, "PiCoB", configMINIMAL_STACK_SIZE + 100,
			(void *) NULL, tskIDLE_PRIORITY, &taskHndl);
	if (res != pdPASS) {
	}

	res = xTaskCreate(PiCo_Key_Task, "PiCoKey", configMINIMAL_STACK_SIZE + 100,
			(void *) NULL, tskIDLE_PRIORITY, NULL);
	if (res != pdPASS) {
	}

	res = xTaskCreate(PiCo_Event_Task, "PiCoEvent",
			configMINIMAL_STACK_SIZE + 100, (void *) NULL, tskIDLE_PRIORITY + 1,
			NULL);
	if (res != pdPASS) {
	}
}

void RTOS_Deinit(void) {
	/* nothing needed for now */
}

#endif /* PL_CONFIG_HAS_RTOS */
