/**
 * \file
 * \brief Main application file
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This provides the main application entry point.
 */

#include "Platform.h"
#include "Application.h"
#include "Event.h"
#include "LED.h"
#include "WAIT1.h"
#include "CS1.h"
#include "KeyDebounce.h"
#include "CLS1.h"
#include "KIN1.h"
#include "Trigger.h"
#if PL_CONFIG_HAS_KEYS
  #include "Keys.h"
#endif
#if PL_CONFIG_HAS_SHELL
  #include "CLS1.h"
  #include "Shell.h"
  #include "RTT1.h"
#endif
#if PL_CONFIG_HAS_BUZZER
  #include "Buzzer.h"
#endif
#if PL_CONFIG_HAS_RTOS
  #include "FRTOS1.h"
  #include "RTOS.h"
#include "task.h"
#endif
#if PL_CONFIG_HAS_QUADRATURE
  #include "Q4CLeft.h"
  #include "Q4CRight.h"
#endif
#if PL_CONFIG_HAS_MOTOR
  #include "Motor.h"
#endif
#if PL_CONFIG_BOARD_IS_ROBO_V2
  #include "PORT_PDD.h"
#endif
#if PL_CONFIG_HAS_LINE_FOLLOW
  #include "LineFollow.h"
#endif
#if PL_CONFIG_HAS_LCD_MENU
  #include "LCD.h"
#endif
#if PL_CONFIG_HAS_SNAKE_GAME
  #include "Snake.h"
#endif
#if PL_CONFIG_HAS_REFLECTANCE
  #include "Reflectance.h"
#endif
#include "Sumo.h"

#if PL_CONFIG_HAS_EVENTS

//xSemaphoreHandle SEM_REF_CALIBRATED = NULL;

static void BtnMsg(int btn, const char *msg) {
#if PL_CONFIG_HAS_SHELL
  #if PL_CONFIG_HAS_SHELL_QUEUE
    uint8_t buf[48];

    UTIL1_strcpy(buf, sizeof(buf), "Button pressed: ");
    UTIL1_strcat(buf, sizeof(buf), msg);
    UTIL1_strcat(buf, sizeof(buf), ": ");
    UTIL1_strcatNum32s(buf, sizeof(buf), btn);
    UTIL1_strcat(buf, sizeof(buf), "\r\n");
    SHELL_SendString(buf);
  #else
    CLS1_SendStr("Button pressed: ", CLS1_GetStdio()->stdOut);
    CLS1_SendStr(msg, CLS1_GetStdio()->stdOut);
    CLS1_SendStr(": ", CLS1_GetStdio()->stdOut);
    CLS1_SendNum32s(btn, CLS1_GetStdio()->stdOut);
    CLS1_SendStr("\r\n", CLS1_GetStdio()->stdOut);
  #endif
#endif
}

void APP_EventHandler(EVNT_Handle event) {
  /*! \todo handle events */
  switch(event) {
  case EVNT_STARTUP:
    {
      int i;
      for (i=0;i<5;i++) {
        LED1_Neg();
        WAIT1_Waitms(50);
      }
      LED1_Off();
    }
    break;
  case EVNT_LED_HEARTBEAT:
    LED2_Neg();

    break;
#if PL_CONFIG_NOF_KEYS>=1
  case EVNT_SW1_PRESSED:
    BtnMsg(1, "Pressed");
#if PL_CONFIG_HAS_BUZZER
    BUZ_PlayTune(BUZ_TUNE_BUTTON);
#endif
     break;
  case EVNT_SW1_LPRESSED:
      BtnMsg(1, "long pressed ");
#if PL_CONFIG_HAS_REFLECTANCE
      REF_CalibrateStartStop();
#endif

#if PL_CONFIG_HAS_BUZZER
    BUZ_PlayTune(BUZ_TUNE_BUTTON_LONG);
#endif
       break;
  case EVNT_SW1_RELEASED:
      BtnMsg(1, "released");
       break;
#endif
#if PL_CONFIG_NOF_KEYS>=2
  case EVNT_SW2_PRESSED:
    BtnMsg(2, "Pressed");
     break;
  case EVNT_SW2_LPRESSED:
      BtnMsg(2, "long pressed ");
       break;
  case EVNT_SW2_RELEASED:
      BtnMsg(2, "released");
       break;
#endif
#if PL_CONFIG_NOF_KEYS>=3
  case EVNT_SW3_PRESSED:
    BtnMsg(3, "Pressed");
     break;
  case EVNT_SW3_LPRESSED:
      BtnMsg(3, "long pressed ");
       break;
  case EVNT_SW3_RELEASED:
      BtnMsg(3, "released");
       break;
#endif
#if PL_CONFIG_NOF_KEYS>=4
  case EVNT_SW4_PRESSED:
    BtnMsg(4, "Pressed");
     break;
  case EVNT_SW4_LPRESSED:
      BtnMsg(4, "long pressed ");
       break;
  case EVNT_SW4_RELEASED:
      BtnMsg(4, "released");
       break;
#endif
#if PL_CONFIG_NOF_KEYS>=5
  case EVNT_SW5_PRESSED:
    BtnMsg(5, "Pressed");
     break;
  case EVNT_SW5_LPRESSED:
      BtnMsg(5, "long pressed ");
       break;
  case EVNT_SW5_RELEASED:
      BtnMsg(5, "released");
       break;
#endif
#if PL_CONFIG_NOF_KEYS>=6
  case EVNT_SW6_PRESSED:
    BtnMsg(6, "Pressed");
     break;
  case EVNT_SW6_LPRESSED:
      BtnMsg(6, "long pressed ");
       break;
  case EVNT_SW6_RELEASED:
      BtnMsg(6, "released");
       break;
#endif
#if PL_CONFIG_NOF_KEYS>=7
  case EVNT_SW7_PRESSED:
    BtnMsg(7, "Pressed");
     break;
  case EVNT_SW7_LPRESSED:
      BtnMsg(7, "long pressed ");
       break;
  case EVNT_SW7_RELEASED:
      BtnMsg(7, "released");
       break;
#endif
    default:
      break;
   } /* switch */
}
#endif /* PL_CONFIG_HAS_EVENTS */

#if PL_CONFIG_HAS_MOTOR /* currently only used for robots */
static const KIN1_UID RoboIDs[] = {
  /* 0: L8, V1 */ {{0x00,0x09,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x4E,0x4E,0x27,0x99,0x10,0x02,0x00,0x25}},
  /* 1: L7, V1 */ {{0x00,0x20,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x4E,0x45,0x27,0x99,0x10,0x02,0x00,0x07}},
};
#endif

static void APP_AdoptToHardware(void) {
  KIN1_UID id;
  uint8_t res;

  res = KIN1_UIDGet(&id);
  if (res!=ERR_OK) {
    for(;;); /* error */
  }
#if PL_CONFIG_HAS_MOTOR
  if (KIN1_UIDSame(&id, &RoboIDs[0])) { /* L8 */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CRight_SwapPins(TRUE);
#endif
    //MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert left motor */
    //MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert left motor */
  } else if (KIN1_UIDSame(&id, &RoboIDs[1])) { /* L7 */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CRight_SwapPins(TRUE);
#endif
	MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert left motor */
	//MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert left motor */
  }
#endif
#if PL_CONFIG_HAS_QUADRATURE && PL_CONFIG_BOARD_IS_ROBO_V2
  /* pull-ups for Quadrature Encoder Pins */
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 10, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 10, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 11, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 11, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 16, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 16, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 17, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 17, PORT_PDD_PULL_ENABLE);
#endif
}

void LED_HeartBeat(void *p){
	(void) p;//Compiler keine Warnung
	LED_Neg(1);
	TRG_SetTrigger(TRG_LED_BLINK, 250/TRG_TICKS_MS, LED_HeartBeat, NULL);
}

void PiCo_Blinky_Task_2(void * pvParameters){
	(void) pvParameters;//Compiler keine Warnung
     TickType_t xLastWakeTime = xTaskGetTickCount();
     for(;;){
         LED2_Neg();
         vTaskDelayUntil(&xLastWakeTime, 2000/portTICK_PERIOD_MS);
     }
     vTaskDelete(NULL);
}

void PiCo_Blinky_Task(void * pvParameters){
     TickType_t xLastWakeTime = xTaskGetTickCount();
     BaseType_t res;
     xTaskHandle taskHndl;
     res=xTaskCreate(PiCo_Blinky_Task_2, "PiCoB2", configMINIMAL_STACK_SIZE+50, (void * ) 10, tskIDLE_PRIORITY+2, &taskHndl);
     if (res != pdPASS){}
     vTaskDelay(2000/portTICK_PERIOD_MS);
    // vTaskDelete(taskHndl);
     for(;;){
         LED1_Neg();
         vTaskDelayUntil(&xLastWakeTime, 500/portTICK_PERIOD_MS);
     }
}


void PiCo_Key_Task(void * pvParameters){
     //TickType_t xLastWakeTime = xTaskGetTickCount();
     //if (res != pdPASS){}
     //vTaskDelay(2000/portTICK_PERIOD_MS);
     //vTaskDelete(taskHndl);
     for(;;){
    	 KEY_Scan();
         vTaskDelay(100/portTICK_PERIOD_MS);
     }
}

void PiCo_Event_Task(void * pvParameters){
     //TickType_t xLastWakeTime = xTaskGetTickCount();
     //if (res != pdPASS){}
     //vTaskDelay(2000/portTICK_PERIOD_MS);
     //vTaskDelete(taskHndl);
     for(;;){
    	 EVNT_HandleEvent(APP_EventHandler,TRUE);
         vTaskDelay(50/portTICK_PERIOD_MS);
     }
}

#if PL_CONFIG_HAS_MOTOR && PL_CONFIG_HAS_REFLECTANCE
#define SENS_THRESHOLD 300
static void PiCo_Brumm_Brumm(void * pvParameters){
	(void)pvParameters;
	uint16_t sens_val[REF_NOF_SENSORS];
	uint8_t cnt;
	//uint16_t del_times[10] = {20, 10, 50, 30, 35, 5, 90, 4, 40, 42};
	//uint8_t del_i = 0;
//	while(xSemaphoreTake(SEM_REF_CALIBRATED,0)!=pdTRUE);
	//bötten
	while(!REF_IsReady()){
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
	MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_LEFT), MOT_DIR_FORWARD);
	MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), MOT_DIR_FORWARD);
	MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0x8000);
	MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0x8000);
	for(;;){
		REF_GetSensorValues(&sens_val[0], REF_NOF_SENSORS);
		cnt = 0;
		for(uint8_t i = 0; i < REF_NOF_SENSORS; i++){
			if(sens_val[i] > SENS_THRESHOLD) cnt++;
		}
		if(cnt == REF_NOF_SENSORS){
			MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_LEFT), MOT_DIR_FORWARD);
			MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), MOT_DIR_FORWARD);
			MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0x8000);
			MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0x8000);
		} else if(sens_val[REF_NOF_SENSORS - 1] < SENS_THRESHOLD || sens_val[REF_NOF_SENSORS - 2] < SENS_THRESHOLD || sens_val[REF_NOF_SENSORS - 3] < SENS_THRESHOLD){
			MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_LEFT), MOT_DIR_BACKWARD);
			MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), MOT_DIR_FORWARD);
			MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0x8000);
			MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0xFFFF);
			//del_i = (del_i + 1) % 10;
			//vTaskDelay(del_times[del_i] / portTICK_PERIOD_MS);
			//vTaskDelay(del_i * 100 / portTICK_PERIOD_MS);
			vTaskDelay(10 / portTICK_PERIOD_MS);
		} else if(sens_val[0] < SENS_THRESHOLD || sens_val[1] < SENS_THRESHOLD || sens_val[2] < SENS_THRESHOLD){
			MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_LEFT), MOT_DIR_FORWARD);
			MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), MOT_DIR_BACKWARD);
			MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0xFFFF);
			MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0x8000);
			//del_i = (del_i + 1) % 10;
			//vTaskDelay(del_times[del_i] / portTICK_PERIOD_MS);
			//vTaskDelay(del_i * 100 / portTICK_PERIOD_MS);
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		vTaskDelay(5/portTICK_PERIOD_MS);
	}
}
#endif

void APP_Start(void) {
  PL_Init();
  APP_AdoptToHardware();
#if PL_CONFIG_HAS_MOTOR && PL_CONFIG_HAS_REFLECTANCE
	BaseType_t res = xTaskCreate(PiCo_Brumm_Brumm, "BrummBrumm", configMINIMAL_STACK_SIZE + 300,
			(void *) NULL, tskIDLE_PRIORITY, NULL);
	if (res != pdPASS) {
	}
#endif
/*  vSemaphoreCreateBinary(SEM_REF_CALIBRATED);
  if(SEM_REF_CALIBRATED == NULL){
	  for(;;); // error
  }
  (void)xSemaphoreTake(SEM_REF_CALIBRATED, 0);*/
  //TRG_SetTrigger(TRG_LED_BLINK, 5000/TRG_TICKS_MS, LED_HeartBeat, NULL);
  __asm volatile("cpsie i"); /* enable interrupts */
  //EVNT_SetEvent(EVNT_STARTUP);

  //for(;;) {
	  //LED1_Neg();
	  //WAIT1_Waitms(100);
	  //KEY_Scan();
	  //EVNT_HandleEvent(APP_EventHandler,TRUE);
	  //BUZ_Play(BUZ_TUNE_WELCOME);
	  //CLS1_SendStr("Hallloooooo Buuuubeeeeeleeee",CLS1_GetStdio()->stdOut);

  //}
}


