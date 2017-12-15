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
#include "PORT_PDD.h"
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
#include "Drive.h"
#include "Distance.h"

static xSemaphoreHandle sem = NULL;
static int32_t speed_l = 0, speed_r = 0, robo = 0;

#if PL_CONFIG_HAS_EVENTS

//xSemaphoreHandle SEM_REF_CALIBRATED = NULL;
int longPressedFLAG =0;
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
      longPressedFLAG = 1;
#if PL_CONFIG_HAS_REFLECTANCE
      REF_CalibrateStartStop();
#endif

#if PL_CONFIG_HAS_BUZZER
    BUZ_PlayTune(BUZ_TUNE_BUTTON_LONG);
#endif
       break;
  case EVNT_SW1_RELEASED:
      BtnMsg(1, "released");
      if(longPressedFLAG) {
    	  longPressedFLAG = 0;
      }
      else{
    	  //LF_StartFollowing();
    	  xSemaphoreGive(sem);
      }
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
  /* 0: 23, V2 */ {{0x00,0x29,0x00,0x00,0x67,0xCD,0xB4,0x51,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
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
if (KIN1_UIDSame(&id, &RoboIDs[0])) { /* 23 */
	  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 10, PORT_PDD_PULL_UP);
	  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 10, PORT_PDD_PULL_ENABLE);
	  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 11, PORT_PDD_PULL_UP);
	  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 11, PORT_PDD_PULL_ENABLE);
	  PORT_PDD_SetPinPullSelect(PORTA_BASE_PTR, 14, PORT_PDD_PULL_UP);
	  PORT_PDD_SetPinPullEnable(PORTA_BASE_PTR, 14, PORT_PDD_PULL_ENABLE);
	  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 16, PORT_PDD_PULL_UP);
	  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 16, PORT_PDD_PULL_ENABLE);
	  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 17, PORT_PDD_PULL_UP);
	  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 17, PORT_PDD_PULL_ENABLE);
	  MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE);
	#if PL_CONFIG_HAS_QUADRATURE
		(void)Q4CRight_SwapPins(TRUE);
	#endif
		//MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert left motor */
		//MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert left motor */
		speed_l = 3000;
		speed_r = 3000;
		robo = 2;
	  } else if (KIN1_UIDSame(&id, &RoboIDs[1])) { /* L7 */
	#if PL_CONFIG_HAS_QUADRATURE
			(void)Q4CRight_SwapPins(TRUE);
			(void)Q4CLeft_SwapPins(TRUE);
	#endif
		MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert left motor */
		//MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert left motor */
		speed_l = 3000;
		speed_r = 3000;
		robo=1;
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
#define SENS_THRESHOLD 50
static void PiCo_Brumm_Brumm(void * pvParameters){
	(void)pvParameters;
	/*MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_LEFT), MOT_DIR_FORWARD);
	MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), MOT_DIR_FORWARD);
//	MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0x0000);
//	MOT_SetVal(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0x0000);
	MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 50);
	MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 50);
	for(;;){
	}*/
	uint16_t sens_val[REF_NOF_SENSORS];
	uint8_t cnt;
	uint16_t del_times[10] = {200, 100, 200, 100, 200, 100, 200, 100, 200, 100};
	uint8_t del_i = 0;
//	while(xSemaphoreTake(SEM_REF_CALIBRATED,0)!=pdTRUE);
	//bötten
	while(!REF_IsReady()){
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
	MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 100);
	MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 100);
	for(;;){
		REF_GetSensorValues(&sens_val[0], REF_NOF_SENSORS);
		cnt = 0;
		for(uint8_t i = 0; i < REF_NOF_SENSORS; i++){
			if(sens_val[i] > SENS_THRESHOLD) cnt++;
		}
		if(cnt == REF_NOF_SENSORS){
			MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 90);
			MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 90);
		} else if(sens_val[REF_NOF_SENSORS - 1] < SENS_THRESHOLD || sens_val[REF_NOF_SENSORS - 2] < SENS_THRESHOLD || sens_val[REF_NOF_SENSORS - 3] < SENS_THRESHOLD){
			MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), -100);
			MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0);
			del_i = (del_i + 1) % 10;
			vTaskDelay(del_times[del_i] / portTICK_PERIOD_MS);
			//vTaskDelay(del_i * 100 / portTICK_PERIOD_MS);
			//vTaskDelay(10 / portTICK_PERIOD_MS);
		} else if(sens_val[0] < SENS_THRESHOLD || sens_val[1] < SENS_THRESHOLD || sens_val[2] < SENS_THRESHOLD){
			MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0);
			MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), -100);
			del_i = (del_i + 1) % 10;
			vTaskDelay(del_times[del_i] / portTICK_PERIOD_MS);
			//vTaskDelay(del_i * 100 / portTICK_PERIOD_MS);
			//vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		vTaskDelay(50/portTICK_PERIOD_MS);
	}
}
#endif
#define SOICHEIB_SPEED_LEFT 75
#define SOICHEIB_SPEED_RIGHT 75
#define SOICHEIB_TURN_TIME_90 300
#define SOICHEIB_SENS_THRESHOLD 50
static void Soicheib(void * pvParameters){
	(void)pvParameters;
	uint8_t sumo = 0;
	vTaskDelay(1000/portTICK_PERIOD_MS);
	for(;;){
		if(xSemaphoreTake(sem, 0) == pdTRUE){
			sumo = !sumo;
			if(sumo){
				//start battle
				for(uint8_t i = 0; i < 10; i++){
					vTaskDelay(500/portTICK_PERIOD_MS);
					//BUZ_PlayTune(BUZ_TUNE_BUTTON);
				}
				//vTaskDelay(5000/portTICK_PERIOD_MS);
				//todo: tof
				//DRV_SetSpeed(SPEED_LEFT, SPEED_RIGHT);
				//DRV_SetMode(DRV_MODE_SPEED);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), SOICHEIB_SPEED_LEFT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), SOICHEIB_SPEED_RIGHT);
			} else {
				//stop battle
				//DRV_SetMode(DRV_MODE_STOP);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0);
			}
		}
		if(sumo){
			/*//battle
			int16_t tof_val[4];
			tof_val[0] = DIST_GetDistance(DIST_SENSOR_FRONT);
			tof_val[1] = DIST_GetDistance(DIST_SENSOR_LEFT);
			tof_val[2] = DIST_GetDistance(DIST_SENSOR_RIGHT);
			tof_val[3] = DIST_GetDistance(DIST_SENSOR_REAR);
			if(tof_val[1] != -1){
				// turn left
				BUZ_PlayTune(BUZ_TUNE_BUTTON_LONG);
				//DRV_SetSpeed(-SPEED_LEFT,SPEED_RIGHT);
				vTaskDelay(TURN_TIME_90/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
			} else if(tof_val[2] != -1){
				// turn right
				BUZ_PlayTune(BUZ_TUNE_BUTTON_LONG);
				//DRV_SetSpeed(SPEED_LEFT,-SPEED_RIGHT);
				vTaskDelay(TURN_TIME_90/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
			} else if(tof_val[3] != -1){
				// turn 180
				BUZ_PlayTune(BUZ_TUNE_BUTTON_LONG);
				//DRV_SetSpeed(SPEED_LEFT,-SPEED_RIGHT);
				vTaskDelay(2*TURN_TIME_90/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
			}*/
			uint16_t sens_val[REF_NOF_SENSORS];
			uint8_t cnt;
			REF_GetSensorValues(&sens_val[0], REF_NOF_SENSORS);
			cnt = 0;
			for(uint8_t i = 0; i < REF_NOF_SENSORS; i++){
				if(sens_val[i] > SOICHEIB_SENS_THRESHOLD) cnt++;
			}
			if(cnt == REF_NOF_SENSORS){
				// drive forward
			} else if(sens_val[REF_NOF_SENSORS - 1] < SOICHEIB_SENS_THRESHOLD || sens_val[REF_NOF_SENSORS - 2] < SOICHEIB_SENS_THRESHOLD || sens_val[REF_NOF_SENSORS - 3] < SOICHEIB_SENS_THRESHOLD){
				// turn left
				BUZ_PlayTune(BUZ_TUNE_BUTTON);
				//DRV_SetSpeed(-SPEED_LEFT*2,-SPEED_RIGHT/2);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), -100);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), -50);
				vTaskDelay(250/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), SOICHEIB_SPEED_LEFT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), SOICHEIB_SPEED_RIGHT);
			} else if(sens_val[0] < SOICHEIB_SENS_THRESHOLD || sens_val[1] < SOICHEIB_SENS_THRESHOLD || sens_val[2] < SOICHEIB_SENS_THRESHOLD){
				// turn right
				BUZ_PlayTune(BUZ_TUNE_BUTTON);
				//DRV_SetSpeed(SPEED_LEFT/2,-SPEED_RIGHT*2);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), -50);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), -100);
				vTaskDelay(250/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), SOICHEIB_SPEED_LEFT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), SOICHEIB_SPEED_RIGHT);
			}
		} else {
			//wait
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

#define BUNDNER_SPEED_LEFT 75
#define BUNDNER_SPEED_RIGHT 75
#define BUNDNER_TURN_TIME_90 300
#define BUNDNER_SENS_THRESHOLD 50
static void bundner_bock(void * pvParameters){
	(void)pvParameters;
	uint8_t sumo = 0;
	vTaskDelay(1000/portTICK_PERIOD_MS);
	for(;;){
		if(xSemaphoreTake(sem, 0) == pdTRUE){
			sumo = !sumo;
			if(sumo){
				//start battle
				for(uint8_t i = 0; i < 10; i++){
					vTaskDelay(500/portTICK_PERIOD_MS);
					//BUZ_PlayTune(BUZ_TUNE_BUTTON);
				}
				//vTaskDelay(5000/portTICK_PERIOD_MS);
				//todo: tof
				//DRV_SetSpeed(SPEED_LEFT, SPEED_RIGHT);
				//DRV_SetMode(DRV_MODE_SPEED);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), BUNDNER_SPEED_LEFT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), BUNDNER_SPEED_RIGHT);
			} else {
				//stop battle
				//DRV_SetMode(DRV_MODE_STOP);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0);
			}
		}
		if(sumo){
			/*//battle
			int16_t tof_val[4];
			tof_val[0] = DIST_GetDistance(DIST_SENSOR_FRONT);
			tof_val[1] = DIST_GetDistance(DIST_SENSOR_LEFT);
			tof_val[2] = DIST_GetDistance(DIST_SENSOR_RIGHT);
			tof_val[3] = DIST_GetDistance(DIST_SENSOR_REAR);
			if(tof_val[1] != -1){
				// turn left
				BUZ_PlayTune(BUZ_TUNE_BUTTON_LONG);
				//DRV_SetSpeed(-SPEED_LEFT,SPEED_RIGHT);
				vTaskDelay(TURN_TIME_90/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
			} else if(tof_val[2] != -1){
				// turn right
				BUZ_PlayTune(BUZ_TUNE_BUTTON_LONG);
				//DRV_SetSpeed(SPEED_LEFT,-SPEED_RIGHT);
				vTaskDelay(TURN_TIME_90/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
			} else if(tof_val[3] != -1){
				// turn 180
				BUZ_PlayTune(BUZ_TUNE_BUTTON_LONG);
				//DRV_SetSpeed(SPEED_LEFT,-SPEED_RIGHT);
				vTaskDelay(2*TURN_TIME_90/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
			}*/
			uint16_t sens_val[REF_NOF_SENSORS];
			uint8_t cnt;
			REF_GetSensorValues(&sens_val[0], REF_NOF_SENSORS);
			cnt = 0;
			for(uint8_t i = 0; i < REF_NOF_SENSORS; i++){
				if(sens_val[i] > BUNDNER_SENS_THRESHOLD) cnt++;
			}
			if(cnt == REF_NOF_SENSORS){
				// drive forward
			} else if(sens_val[REF_NOF_SENSORS - 1] < BUNDNER_SENS_THRESHOLD || sens_val[REF_NOF_SENSORS - 2] < BUNDNER_SENS_THRESHOLD || sens_val[REF_NOF_SENSORS - 3] < BUNDNER_SENS_THRESHOLD){
				// turn left
				BUZ_PlayTune(BUZ_TUNE_BUTTON);
				//DRV_SetSpeed(-SPEED_LEFT*2,-SPEED_RIGHT/2);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), -100);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), -50);
				vTaskDelay(250/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), BUNDNER_SPEED_LEFT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), BUNDNER_SPEED_RIGHT);
			} else if(sens_val[0] < BUNDNER_SENS_THRESHOLD || sens_val[1] < BUNDNER_SENS_THRESHOLD || sens_val[2] < BUNDNER_SENS_THRESHOLD){
				// turn right
				BUZ_PlayTune(BUZ_TUNE_BUTTON);
				//DRV_SetSpeed(SPEED_LEFT/2,-SPEED_RIGHT*2);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), -50);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), -100);
				vTaskDelay(250/portTICK_PERIOD_MS);
				//DRV_SetSpeed(SPEED_LEFT,SPEED_RIGHT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), BUNDNER_SPEED_LEFT);
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), BUNDNER_SPEED_RIGHT);
			}
		} else {
			//wait
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

void APP_Start(void) {
  PL_Init();
  APP_AdoptToHardware();
  vSemaphoreCreateBinary(sem);
  xSemaphoreTake(sem,0);
  if(robo==1){
  if(xTaskCreate(Soicheib, "Soicheib", configMINIMAL_STACK_SIZE + 500, (void *)NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS){
	  for(;;); // error
  }
  }else if(robo==2){
  if(xTaskCreate(bundner_bock, "bundner bock", configMINIMAL_STACK_SIZE + 500, (void *)NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS){
	  for(;;); // error
  }
  }
//#if PL_CONFIG_HAS_MOTOR && PL_CONFIG_HAS_REFLECTANCE
//	BaseType_t res = xTaskCreate(PiCo_Brumm_Brumm, "BrummBrumm", configMINIMAL_STACK_SIZE + 300,
//			(void *) NULL, tskIDLE_PRIORITY, NULL);
//	if (res != pdPASS) {
//	}
//#endif
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


