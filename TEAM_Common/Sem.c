/**
 * \file
 * \brief Semaphore usage
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * Module using semaphores.
 */

/**
 * \file
 * \brief Semaphore usage
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * Module using semaphores.
 */

#include "Platform.h" /* interface to the platform */
#if PL_CONFIG_HAS_SEMAPHORE
#include "FRTOS1.h"
#include "Sem.h"
#include "LED.h"

static xSemaphoreHandle sem = NULL;

static void vSlaveTask(void *pvParameters) {
  /*! \todo Implement functionality */
	for(;;){
		xSemaphoreTake(sem,portMAX_DELAY);
//	    uint8_t buf[48];
//	    UTIL1_strcpy(buf, sizeof(buf), "Buuubbelleee");
//	    UTIL1_strcat(buf, sizeof(buf), "\r\n");
//	    SHELL_SendString(buf);
	}
}

static void vMasterTask(void *pvParameters) {
  /*! \todo send semaphore from master task to slave task */
	for(;;){
		vTaskDelay(1000/portTICK_PERIOD_MS);
		xSemaphoreGive(sem);
	}
}

void SEM_Deinit(void) {
}

/*! \brief Initializes module */
void SEM_Init(void) {
	BaseType_t res;
	sem = xSemaphoreCreateBinary () ;
	 if (sem == NULL) {
	 // Failed ! Not enough heap memory?. /

	 } else {
		//  The semaphore can now be used . Calling
		//	xSemaphoreTake () on the semaphore w i l l f a i l until
		//	the semaphore has f i r s t been given .  /
			res = xTaskCreate(vSlaveTask, "SlaveTask", configMINIMAL_STACK_SIZE + 100,
					(void *) NULL, tskIDLE_PRIORITY, NULL);
			if (res != pdPASS) {
			}

			res = xTaskCreate(vMasterTask, "MasterTask", configMINIMAL_STACK_SIZE + 100,
					(void *) NULL, tskIDLE_PRIORITY, NULL);
			if (res != pdPASS) {
			}
	 }

}
#endif /* PL_CONFIG_HAS_SEMAPHORE */
