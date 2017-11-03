/**
 * \file
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
	  res=xTaskCreate(PiCo_Blinky_Task, "PiCoB", configMINIMAL_STACK_SIZE+50, (void * ) NULL, tskIDLE_PRIORITY+1, &taskHndl);
	  if (res != pdPASS){}
}

void RTOS_Deinit(void) {
  /* nothing needed for now */
}

#endif /* PL_CONFIG_HAS_RTOS */
