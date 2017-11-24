/**
 * \file
 * \brief Main application interface
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This provides the main application entry point.
 */

#ifndef SOURCES_FS2016_COMMON_APPLICATION_H_
#define SOURCES_FS2016_COMMON_APPLICATION_H_

#include "Platform.h"

#if PL_CONFIG_HAS_EVENTS
#include "Event.h"

//extern xSemaphoreHandle SEM_REF_CALIBRATED = NULL;

void APP_EventHandler(EVNT_Handle event);
#endif

void PiCo_Blinky_Task(void * pvParameters);

void PiCo_Key_Task(void * pvParameters);

void PiCo_Event_Task(void * pvParameters);

void APP_Start(void);

void LED_HeartBeat(void *p);

#endif /* SOURCES_FS2016_COMMON_APPLICATION_H_ */
