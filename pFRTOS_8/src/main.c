/*
    Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
    Copyright (C) 2012 - 2018 Xilinx, Inc. All Rights Reserved.

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software. If you wish to use our Amazon
    FreeRTOS name, please do so in a fair use way that does not cause confusion.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
    FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
    COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    http://www.FreeRTOS.org
    http://aws.amazon.com/freertos


    1 tab == 4 spaces!
*/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "stdio.h"
#include "math.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
#include "PmodGPIO.h"
#include "xgpio.h"
#include "sleep.h"
#include "xsysmon.h"
#include <xtime_l.h>
#include "xiic.h"
#include "PmodKYPD.h"
#include "xsdps.h"
#include "ff.h"
#include "xil_cache.h"
#include "xplatform_info.h"

#define C_BASEADDR 0x43C00000
/*---------------------------TASKS-----------------------------*/
static void velocimeterTask( void *pvParameters );
static void keyPadTask( void *pvParameters );
static void accelerometerTask( void *pvParameters );
static void displayTask( void *pvParameters );
static void presenceTask( void *pvParameters );
static void ultrasonicTask( void *pvParameters );
static void useSystemTask( void *pvParameters);
static void riskControlTask( void *pvParameters);
static void writeRecordTask( void *pvParameters);
static void buzzerTask( void *pvParameters);
/*-----------------------------------------------------------*/
/*------------------------Task Handle--------------------------*/
static TaskHandle_t xVelocimeterTask;
static TaskHandle_t xKeyPadTask;
static TaskHandle_t xAccelerometerTask;
static TaskHandle_t xDisplayTask;
static TaskHandle_t xPresenceTask;
static TaskHandle_t xUsonicTask;
static TaskHandle_t xUseSystemTask;
static TaskHandle_t xRiskControlTask;
static TaskHandle_t xRecordTask;
static TaskHandle_t xBuzzerTask;
/*-------------------------------------------------------------*/
/*-------------------------Semaphores--------------------------*/
SemaphoreHandle_t xSemaphoreBuzzer = NULL;
SemaphoreHandle_t xSemaphoreAutopilot = NULL;
SemaphoreHandle_t xSemaphoreKB = NULL;
SemaphoreHandle_t xSemaphoreSpeed = NULL;
SemaphoreHandle_t xSemaphoreAltitude = NULL;
SemaphoreHandle_t xSemaphoreTilt = NULL;
SemaphoreHandle_t xSemaphorePresence = NULL;
SemaphoreHandle_t xSemaphoreRisk = NULL;
SemaphoreHandle_t xSemaphoreSD = NULL;
/*-------------------------------------------------------------*/
static void vTimerCallback3SecExpired(xTimerHandle pxTimer);
static void vTimerCallback15SecExpired(xTimerHandle pxTimer);

//IIC Defines & Declarations //Accelerometer
	#define IIC_DEVICE XPAR_AXI_IIC_0_DEVICE_ID
	#define MPU 0x68
	//Buffers
	u8 bufferSend[128];
	u8 bufferRecv[128];
	int ByteCount;
	int StatusIIC;
	//Config
	XIic_Config *ConfigPtr;
	XIic XiicInstance;
	//Conversion Rates
	#define A_R 16384.0 // 32768/2
	//Conversion Rad to Degrees 180/PI
	#define RAD_A_DEG 57.295779
	//RAW Data
	int16_t AcX, AcY, AcZ;

//Accelerometer Protected Object (P.O)
	//Angles
	float Acc[2];

	int specRiskTilt = 0;
	char alertTilt[32] = " ";
	int countCorrectReadingsTilt = 0;
	int AxisX = 0;
	int AxisY = 0;

	void setup();
	void readAccelerometer();

//KeyPad & System Defines (P.O)
	#define DEFAULT_KEYTABLE "0FED789C456B123A"
	char code[6] = " ";

	XGpio options;
	u8 debug, started;

	PmodGPIO myDeviceJD_io, myDeviceJE_o;
	PmodKYPD myDeviceKP;

	void setupKP();
	void setupBoard();
	void setupGPIO();

//Ultrasonic (P.O)
	int minAltitude = 141;
	int maxAltitude = 170;
	#define AltitudeRatio (10000/12000) //10/12=0.833333
	int realValueRatio = 12000/170; //Variable

	int specRiskSpeed = 0;
	char alertSpeed[32] = " ";
	int countCorrectReadingsAltitude = 0;
	int countWrongReadingsAltitude[2] = {0, 0};
	int altitudeDB = 0;

	void readUltrasonicSensor();

//Velocimeter (P.O)
	int specRiskAltitude = 0;
	char alertAltitude[32] = " ";
	int countCorrectReadingsSpeed = 0;
	int countWrongReadingsSpeed[4] = {0, 0, 0, 0};
	float speedDB = 0.0;

	void readVelocimeter();

//Presence (P.O)
	int specRiskPresence = 0;
	char alertPresence[32] = " ";
	int countCorrectReadingsPresence = 0;
	int countWrongReadingsPresence[2] = {0, 0};
	int presenceDB = 0;

	void readPresence();

//RiskControl (P.O)
	int genRisk = 0;

	void pmodLedWork(u8 led);
	void rgbLedWork(u8 risk);
	void useActuator(u8 risk,u8 counter);

//SD Card
	FIL fil;
	FATFS fatfs;
	char FileName[32] = "Record_.txt";
	char *SD_File;

	u8 SourceAddress[1024*2] __attribute__ ((aligned(32)));

	char send2uSD[64];

	int SDFileWrite(char *record);
	char * getLevelRisk(int risk);

xTimerHandle timerHndl3SecTimeout;
xTimerHandle timerHndl15SecTimeout;

int main( void ){
	setupBoard();
	setupGPIO();
	xil_printf("Main task system!\r\n" );

	timerHndl3SecTimeout = xTimerCreate(
		"timer3Sec", /* name */
		pdMS_TO_TICKS(3000), /* period/time */
		pdFALSE, /* auto reload */
		(void*)1, /* timer ID */
		vTimerCallback3SecExpired); /* callback */
		if (timerHndl3SecTimeout==NULL) {
			for(;;); /* failure! */
		}

	timerHndl15SecTimeout = xTimerCreate(
		"timer15sec",
		pdMS_TO_TICKS(15000),
		pdFALSE,
		(void*)2,
		vTimerCallback15SecExpired);
		if (timerHndl15SecTimeout==NULL) {
			for(;;);
		}

	//Tasks
	xTaskCreate( keyPadTask,
		( const char * ) "KeyPad",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY+1,
		&xKeyPadTask );

	xTaskCreate( writeRecordTask,
		( const char * ) "Record",
		2048,
		NULL,
		tskIDLE_PRIORITY+2,
		&xRecordTask );

	xTaskCreate(displayTask,
		( const char * ) "Display",
		1024,
		NULL,
		tskIDLE_PRIORITY+3,
		&xDisplayTask );

	xTaskCreate( useSystemTask,
		( const char * ) "System",
		1024,
		NULL,
		tskIDLE_PRIORITY+4,
		&xUseSystemTask );

	xTaskCreate( ultrasonicTask,
		( const char * ) "USonic",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY+5,
		&xUsonicTask );

	xTaskCreate( presenceTask,
		 ( const char * ) "Presence",
		 configMINIMAL_STACK_SIZE,
		 NULL,
		 tskIDLE_PRIORITY+6,
		 &xPresenceTask );

	xTaskCreate( velocimeterTask,
		 ( const char * ) "Speed",
		 configMINIMAL_STACK_SIZE,
		 NULL,
		 tskIDLE_PRIORITY+7,
		 &xVelocimeterTask );

	xTaskCreate( accelerometerTask,
		( const char * ) "Tilt",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY+8,
		&xAccelerometerTask );

	xTaskCreate( riskControlTask,
		( const char * ) "RiskControl",
		1024,
		NULL,
		tskIDLE_PRIORITY+9,
		&xRiskControlTask );

	xTaskCreate( buzzerTask,
		( const char * ) "Buzzer",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY+2,
		&xBuzzerTask);

	//Binary Semaphores
	xSemaphoreBuzzer = xSemaphoreCreateBinary();
	xSemaphoreAutopilot = xSemaphoreCreateBinary();

	if(xSemaphoreBuzzer != NULL){
		/* The semaphore was created successfully and
		can be used. */
	}
	if(xSemaphoreAutopilot != NULL){
		/* The semaphore was created successfully and
		can be used. */
	}

	//Mutex Semaphores
	xSemaphoreKB = xSemaphoreCreateMutex();
	xSemaphoreAltitude = xSemaphoreCreateMutex();
	xSemaphorePresence = xSemaphoreCreateMutex();
	xSemaphoreSpeed = xSemaphoreCreateMutex();
	xSemaphoreTilt = xSemaphoreCreateMutex();
	xSemaphoreRisk = xSemaphoreCreateMutex();
	xSemaphoreSD = xSemaphoreCreateMutex();

	if(xSemaphoreKB != NULL){
		/* The semaphore was created successfully and
		can be used. */
	}
	if(xSemaphoreAltitude != NULL){
		/* The semaphore was created successfully and
		can be used. */
	}
	if(xSemaphorePresence != NULL){
		/* The semaphore was created successfully and
		can be used. */
	}
	if(xSemaphoreSpeed != NULL){
		/* The semaphore was created successfully and
		can be used. */
	}
	if(xSemaphoreTilt != NULL){
		/* The semaphore was created successfully and
		can be used. */
	}
	if(xSemaphoreRisk != NULL){
		/* The semaphore was created successfully and
		can be used. */
	}
	if(xSemaphoreSD != NULL){
		/* The semaphore was created successfully and
		can be used. */
	}

	vTaskStartScheduler();

	for( ;; );
}


/*-----------------------------------------------------------*/

static void keyPadTask( void *pvParameters ){
	TickType_t xLastTimeAwake;
	u16 keystate;
	XStatus status, last_status = KYPD_NO_KEY;
	u8 key, last_key = 'x';
	char codeKP[6];

	strncpy(codeKP," ",6);

	xLastTimeAwake = 0;

	int kp = 0;

	setupKP();

	for( ;; ){
		keystate = KYPD_getKeyStates(&myDeviceKP);
		status = KYPD_getKeyPressed(&myDeviceKP, keystate, &key);

		// Print key detect if a new key is pressed or if status has changed
		if (status == KYPD_SINGLE_KEY && (status != last_status || key != last_key)) {
			sprintf(codeKP+kp,"%c",(char) key);
			xil_printf("Key Pressed: %s\r\n",codeKP);
			kp++;
			last_key = key;

		}else if (status == KYPD_MULTI_KEY && status != last_status)
			xil_printf("Error: Multiple keys pressed\r\n");

		if(kp == 6){
			xSemaphoreTake(xSemaphoreKB,portMAX_DELAY);
				strcpy(code,codeKP);
			xSemaphoreGive(xSemaphoreKB);
			kp = 0;
			strncpy(codeKP," ",6);
		}

		last_status = status;

		vTaskDelayUntil(&xLastTimeAwake, 200/portTICK_RATE_MS);
	}
}

static void ultrasonicTask( void *pvParameters ){
	TickType_t xLastTimeAwake;
	xLastTimeAwake = 0;

	for( ;; ){
		if(started == 1){
			xSemaphoreTake( xSemaphoreAltitude, portMAX_DELAY );
			readUltrasonicSensor();
			xSemaphoreGive( xSemaphoreAltitude );
		}
		vTaskDelayUntil(&xLastTimeAwake, 300/portTICK_RATE_MS);
	}
}

static void presenceTask( void *pvParameters ){
	TickType_t xLastTimeAwake;
	xLastTimeAwake = 0;

	for( ;; ){
		if(started == 1){
			xSemaphoreTake( xSemaphorePresence, portMAX_DELAY );
			readPresence();
			xSemaphoreGive( xSemaphorePresence );
		}
		vTaskDelayUntil(&xLastTimeAwake, 600/portTICK_RATE_MS);
	}
}

static void accelerometerTask( void *pvParameters ){
	TickType_t xLastTimeAwake;
	xLastTimeAwake = 0;

	setup();

	for( ;; ){
		if(started == 1){
			xSemaphoreTake( xSemaphoreTilt, portMAX_DELAY );
			readAccelerometer();
			xSemaphoreGive( xSemaphoreTilt );
		}
		vTaskDelayUntil(&xLastTimeAwake, 250/portTICK_RATE_MS);
	}

}

static void velocimeterTask( void *pvParameters ){
	TickType_t xLastTimeAwake;
	xLastTimeAwake = 0;

	for( ;; ){
		if(started == 1){
			xSemaphoreTake( xSemaphoreSpeed, portMAX_DELAY );
			readVelocimeter();
			xSemaphoreGive( xSemaphoreSpeed );
		}
		vTaskDelayUntil(&xLastTimeAwake, 500/portTICK_RATE_MS);
	}
}

static void useSystemTask( void *pvParameters ){
	TickType_t xLastTimeAwake;
	xLastTimeAwake = 0;
	char message[128];
	message[0] = '\0';
	char codeSys[6];

	for( ;; ){
		if(xSemaphoreTake( xSemaphoreKB, ( TickType_t ) 30) == pdTRUE){
				strcpy(codeSys,code);
				strcpy(code," ");
			xSemaphoreGive( xSemaphoreKB );
		}else{
			strcpy(codeSys," ");
		}

		if(strcmp(codeSys,"C1234A") == 0){
			started = 1;
			sprintf(message,"\r\nCode: %s\r\n- System Working\r\n",codeSys);
			printf("%s",message);
			xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
				if(SDFileWrite(message) != XST_SUCCESS){
					printf("Can´t access SD to start!\r\n");
				}
			xSemaphoreGive(xSemaphoreSD);
			strcpy(codeSys," ");
		}else if(strcmp(codeSys,"D1234A") == 0){
			started = 0;
			rgbLedWork(0);
			pmodLedWork(0);
			sprintf(message,"\r\nCode: %s\r\n- System Deactivated\r\n",codeSys);
			printf("%s",message);
			xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
			if(SDFileWrite(message) != XST_SUCCESS){
				printf("Can´t access SD!\r\n");
			}
			xSemaphoreGive(xSemaphoreSD);
			strcpy(codeSys," ");
		}else if(strcmp(codeSys,"F0001A") == 0){
			debug = 1;
			sprintf(message,"\r\nCode: %s\r\n- Debug Mode: ON\r\n",codeSys);
			printf("%s",message);
			xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
			if(SDFileWrite(message) != XST_SUCCESS){
				printf("Can´t access SD!\r\n");
			}
			xSemaphoreGive(xSemaphoreSD);
			strcpy(codeSys," ");
		}else if(strcmp(codeSys,"F0000A") == 0){
			debug = 0;
			sprintf(message,"\r\nCode: %s\r\n- Debug Mode: OFF\r\n",codeSys);
			printf("%s",message);
			xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
			if(SDFileWrite(message) != XST_SUCCESS){
				printf("Can´t access SD to start!\r\n");
			}
			xSemaphoreGive(xSemaphoreSD);
			strcpy(codeSys," ");
		}else if(strcmp(codeSys," ") != 0){
			if((codeSys[0] == 'B') && (codeSys[5] == 'A')){
				maxAltitude = (((int) (codeSys[1]-48))*1000 +
							((int) (codeSys[2]-48))*100 +
							((int) (codeSys[3]-48))*10 +
							((int) (codeSys[4]-48)));
				minAltitude = (maxAltitude*10000)/12000;
				realValueRatio = 12000/maxAltitude;
				sprintf(message,"\r\nCode: %s\r\n- New Min Altitude: %d\r\n- New Max Altitude: %d\r\n- New Ratio Conversion to Real: %d\r\n",codeSys,minAltitude,maxAltitude,realValueRatio);
				printf("%s",message);
				xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
				if(SDFileWrite(message) != XST_SUCCESS){
					printf("Can´t access SD to start!\r\n");
				}
				xSemaphoreGive(xSemaphoreSD);
				strcpy(codeSys," ");

			}else if((codeSys[0] == 'E') && (codeSys[5] == 'E')){
				if(started == 0){
					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(FileName,"Record_%c%c%c%c.txt", codeSys[1], codeSys[2], codeSys[3], codeSys[4]);
						sprintf(message,"\r\nCode: %s\r\nChanged FileName to %s\r\n",codeSys,FileName);
						printf("%s",message);
						if(SDFileWrite(message) != XST_SUCCESS){
							printf("Can´t access SD to start!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}else{
					printf("System is started. Can't change FileName right now!\r\n");
				}
				strcpy(codeSys," ");
			}else{
				sprintf(message,"\r\nCode: %s\r\n- Wrong Code!\r\n",codeSys);
				printf("%s",message);
				xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
					if(SDFileWrite(message) != XST_SUCCESS){
						printf("Can´t access SD to start!\r\n");
					}
				xSemaphoreGive(xSemaphoreSD);
				strcpy(codeSys," ");
			}
		}

		if(started == 1){
			XGpio_DiscreteWrite(&options, 1, 1);
			usleep(10000);
			XGpio_DiscreteWrite(&options, 1, 0);
		}

		vTaskDelayUntil(&xLastTimeAwake, 2000/portTICK_RATE_MS);
	}
}

static void riskControlTask( void *pvParameters){
	TickType_t timed,old_timed,xLastTimeAwake;
	TickType_t totalCorrect = 0;

	char messageSD[64] = " ";

	u8 specRisk[4];
	u8 counterL;
	u8 counterG;
	u8 counterC;
	int intGenRisk;
	int oldRiskValue = 0;

	xLastTimeAwake = 0;
	old_timed = 0;
	timed = 0;

	for( ;; ){
		if(started == 1){
			xSemaphoreTake( xSemaphoreAltitude, portMAX_DELAY );
				specRisk[0] = specRiskAltitude;
			xSemaphoreGive( xSemaphoreAltitude );

			xSemaphoreTake( xSemaphoreTilt, portMAX_DELAY );
				specRisk[1] = specRiskTilt;
			xSemaphoreGive( xSemaphoreTilt );

			xSemaphoreTake( xSemaphorePresence, portMAX_DELAY );
				specRisk[2] = specRiskPresence;
			xSemaphoreGive( xSemaphorePresence );

			xSemaphoreTake( xSemaphoreSpeed, portMAX_DELAY );
				specRisk[3] = specRiskSpeed;
			xSemaphoreGive( xSemaphoreSpeed );

			xSemaphoreTake(xSemaphoreRisk, portMAX_DELAY );
				oldRiskValue = genRisk;
			xSemaphoreGive(xSemaphoreRisk);

			if(oldRiskValue != 3){
				counterL = 0;
				counterG = 0;
				counterC = 0;

				for(int i=0;i<sizeof(specRisk);i++){
					if(specRisk[i]==1){
						counterL++;
					}else if(specRisk[i]==2){
						counterG++;
					}else if(specRisk[i]==3){
						counterC++;
					}
				}

				intGenRisk = 0;

				if(counterL > 0){
					if(counterL < 3){
						intGenRisk = 1;
					}else{
						counterG++;
					}
				}
				if(counterG > 0){
					if(counterG < 2){
						intGenRisk = 2;
					}else{
						counterC++;
					}
				}
				if(counterC > 0){
					intGenRisk = 3;
				}
			}

			if(intGenRisk == 0){
				if(totalCorrect < pdMS_TO_TICKS(5000UL)){
					timed = xTaskGetTickCount();
					totalCorrect += timed-old_timed;
				}if((totalCorrect >= pdMS_TO_TICKS(5000UL)) && (oldRiskValue != 0)){
					if(oldRiskValue == 2){
						oldRiskValue = 1;
						xSemaphoreTake(xSemaphoreRisk, portMAX_DELAY );
							genRisk = oldRiskValue;
						xSemaphoreGive(xSemaphoreRisk);
						xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
							sprintf(messageSD,"\r\n[%ld] Risk level rectified to %d (%s)\r\n",(xTaskGetTickCount()*10),oldRiskValue,getLevelRisk(oldRiskValue));
							if(SDFileWrite(messageSD) != XST_SUCCESS){
								printf("Can´t access SD - Risk!\r\n");
							}
						xSemaphoreGive(xSemaphoreSD);
						totalCorrect = 0;
						useActuator(1, 1);
					}else if(oldRiskValue == 1){
						oldRiskValue = 0;
						xSemaphoreTake(xSemaphoreRisk, portMAX_DELAY );
							genRisk = oldRiskValue;
						xSemaphoreGive(xSemaphoreRisk);

						xSemaphoreTake( xSemaphoreAltitude, portMAX_DELAY );
							strcpy(alertAltitude, " ");
						xSemaphoreGive( xSemaphoreAltitude );

						xSemaphoreTake( xSemaphoreTilt, portMAX_DELAY );
							strcpy(alertTilt, " ");
						xSemaphoreGive( xSemaphoreTilt );

						xSemaphoreTake( xSemaphorePresence, portMAX_DELAY );
							strcpy(alertPresence, " ");
						xSemaphoreGive( xSemaphorePresence );

						xSemaphoreTake( xSemaphoreSpeed, portMAX_DELAY );
							strcpy(alertSpeed, " ");
						xSemaphoreGive( xSemaphoreSpeed );

						xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
							sprintf(messageSD,"\r\n[%ld] Risk level rectified to %d (%s)\r\n",(xTaskGetTickCount()*10),oldRiskValue,getLevelRisk(oldRiskValue));
							if(SDFileWrite(messageSD) != XST_SUCCESS){
								printf("Can´t access SD - Risk!\r\n");
							}
						xSemaphoreGive(xSemaphoreSD);
						useActuator(0, 0);
					}else if(oldRiskValue == 0){
						useActuator(0, 0);
					}
				}
			}else{
				totalCorrect = 0;
				timed = xTaskGetTickCount();

				if(intGenRisk == 1){
					xSemaphoreTake(xSemaphoreRisk, portMAX_DELAY );
						genRisk = 1;
					xSemaphoreGive(xSemaphoreRisk);

					//Activate Level 1 response if 1st time
					if(oldRiskValue != intGenRisk){
						useActuator(intGenRisk, counterL);
					}
					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(messageSD,"\r\n[%ld] Level %d activated!\r\n",(xTaskGetTickCount()*10),intGenRisk);
						if(SDFileWrite(messageSD) != XST_SUCCESS){
							printf("Can´t access SD - Risk!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);

				} else if(intGenRisk == 2){
					xSemaphoreTake(xSemaphoreRisk, portMAX_DELAY );
						genRisk = 2;
					xSemaphoreGive(xSemaphoreRisk);

					//Activate Level 2 response if first time
					if(oldRiskValue != intGenRisk){
						useActuator(intGenRisk,counterG);
					}
					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(messageSD,"\r\n[%ld] Level %d activated!\r\n",(xTaskGetTickCount()*10),intGenRisk);
						if(SDFileWrite(messageSD) != XST_SUCCESS){
							printf("Can´t access SD - Risk!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				} else if(intGenRisk == 3){
					xSemaphoreTake(xSemaphoreRisk, portMAX_DELAY );
						genRisk = 3;
					xSemaphoreGive(xSemaphoreRisk);
					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(messageSD,"\r\n[%ld] Level %d activated!\r\n",(xTaskGetTickCount()*10),intGenRisk);
						if(SDFileWrite(messageSD) != XST_SUCCESS){
							printf("Can´t access SD - Risk!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);

					if(oldRiskValue != intGenRisk){
						useActuator(intGenRisk, counterC);
					}
				}
			}
			old_timed = timed;
		}
		vTaskDelayUntil(&xLastTimeAwake, 500/portTICK_RATE_MS);
	}
}

static void displayTask( void *pvParameters ){
	TickType_t xLastTimeAwake;
	char alertDAltitude[32] = " ";
	char alertDPresence[32] = " ";
	char alertDSpeed[32] = " ";
	char alertDTilt[32] = " ";
	u8 specRisk[4];
	float AccDis[2];
	int PresenceDis;
	float SpeedDis;
	int AltitudeDis;
	int genRiskD;

	xLastTimeAwake = 0;

	for( ;; ){
		if(started == 1){
			xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
				sprintf(send2uSD,"\r\n[%ld] Display Task Executed\r\n",(xTaskGetTickCount()*10));
				if(SDFileWrite(send2uSD) != XST_SUCCESS){
					printf("Can´t access SD - Display!\r\n");
				}
			xSemaphoreGive(xSemaphoreSD);

			xSemaphoreTake( xSemaphoreAltitude, portMAX_DELAY );
				specRisk[0] = specRiskAltitude;
				strcpy(alertDAltitude, alertAltitude);
				AltitudeDis = altitudeDB;
			xSemaphoreGive( xSemaphoreAltitude );

			xSemaphoreTake( xSemaphoreTilt, portMAX_DELAY );
				specRisk[1] = specRiskTilt;
				strcpy(alertDTilt, alertTilt);
				AccDis[0] = Acc[0];
				AccDis[1] = Acc[1];
			xSemaphoreGive( xSemaphoreTilt);

			xSemaphoreTake( xSemaphorePresence, portMAX_DELAY );
				specRisk[2] = specRiskPresence;
				strcpy(alertDPresence, alertPresence);
				PresenceDis = presenceDB;
			xSemaphoreGive( xSemaphorePresence );

			xSemaphoreTake( xSemaphoreSpeed, portMAX_DELAY );
				specRisk[3] = specRiskSpeed;
				strcpy(alertDSpeed, alertSpeed);
				SpeedDis = speedDB;
			xSemaphoreGive( xSemaphoreSpeed );

			xSemaphoreTake( xSemaphoreRisk, portMAX_DELAY );
				genRiskD = genRisk;
			xSemaphoreGive(xSemaphoreRisk);

			printf("\r\n[%ld] Display Task\r\n",(xTaskGetTickCount()*10));

			printf("\r\nValues\r\n");
				printf("\tAltitude Values: %d meters (Sim: %d cm)\r\n",AltitudeDis*realValueRatio,AltitudeDis);
				printf("\tTilt Values: X = %0.2f º, Y = %0.2f º\r\n",AccDis[0],AccDis[1]);
				printf("\tPresence Values: %d\r\n",PresenceDis);
				printf("\tSpeed Values: %0.2f km/h\r\n",SpeedDis);

			if(debug == 1){
				printf("\r\nSpecific Risk (Debug)\r\n");
				printf("\tAltitude Values: %d (%s)\r\n",specRisk[0],getLevelRisk(specRisk[0]));
				printf("\tTilt Values: %d (%s)\r\n",specRisk[1],getLevelRisk(specRisk[1]));
				printf("\tPresence Values: %d (%s)\r\n",specRisk[2],getLevelRisk(specRisk[2]));
				printf("\tSpeed Values: %d (%s)\r\n",specRisk[3],getLevelRisk(specRisk[3]));
			}

			printf("\r\nGeneral Risk: %d (%s)\r\n",genRiskD,getLevelRisk(genRiskD));

			if(genRiskD != 0){
				printf("\r\nAlerts\r\n");
				if(strcmp(alertDAltitude," ")){
					printf("\tAltitude: %s \r\n",alertDAltitude);
				}
				if(strcmp(alertDTilt," ")){
					printf("\tTilt: %s \r\n",alertDTilt);
				}
				if(strcmp(alertDPresence," ")){
					printf("\tPresence: %s \r\n",alertDPresence);
				}
				if(strcmp(alertDSpeed," ")){
					printf("\tSpeed: %s \r\n",alertDSpeed);
				}
			}
		}
		vTaskDelayUntil(&xLastTimeAwake, 2000/portTICK_RATE_MS);
	}
}

static void writeRecordTask(void *pvParameters){
	int Status;
	char dataRecord[1024*2];
	TickType_t xLastTimeAwake;
	XTime timeX;

	dataRecord[0] = '\0';
	xLastTimeAwake = 0;

	for(;;){
		if(started){
			//Create the record to write at once
			XTime_GetTime(&timeX);
			sprintf(dataRecord,"\r\n[%lld] Started SD Recording.\r\n\r\n",(timeX/(COUNTS_PER_SECOND/1000)));

			xSemaphoreTake( xSemaphoreKB, portMAX_DELAY);
				sprintf(dataRecord + strlen(dataRecord),"\tModes enabled:\r\n\t\tActive/Started: %s\n\t\tDebug: %s\r\n\r\n",((started)?"True":"False"),((debug)?"True":"False"));
			xSemaphoreGive( xSemaphoreKB);

			xSemaphoreTake( xSemaphoreRisk, portMAX_DELAY);
				sprintf(dataRecord + strlen(dataRecord),"\tGeneral Risk:\r\n\t\tLevel: %d (%s)\r\n\r\n",genRisk,getLevelRisk(genRisk));
			xSemaphoreGive( xSemaphoreRisk);

			xSemaphoreTake( xSemaphoreTilt, portMAX_DELAY);
				sprintf(dataRecord + strlen(dataRecord),"\tSpecific Risk (Accelerometer/Tilt):\r\n\t\tRisk Level: %d (%s)\r\n\t\tAlert: \"%s\"\r\n\t\tActual Tilt Angle (X, Y): (%0.2f, %0.2f)\r\n\t\tCounters (Correct, Wrong AxisX, Wrong AxisY): (%d, %d, %d)\r\n\r\n",
					specRiskTilt,getLevelRisk(specRiskTilt),alertTilt,Acc[0],Acc[1],countCorrectReadingsTilt,AxisX,AxisY);
			xSemaphoreGive( xSemaphoreTilt);

			xSemaphoreTake( xSemaphoreAltitude, portMAX_DELAY);
				sprintf(dataRecord + strlen(dataRecord),"\tSpecific Risk (Altimeter/Altitude):\r\n\t\tRisk Level: %d (%s)\r\n\t\tAlert: \"%s\"\r\n\t\tActual Altitude \"Simulated\" (Real): %d m (%d cm)\r\n\t\tSet Altitude (Min - Max): (%d - %d)\r\n\t\tCounters (Wrong Low, Correct, Wrong High): (%d, %d, %d)\r\n\r\n",
					specRiskAltitude,getLevelRisk(specRiskAltitude),alertAltitude,(altitudeDB*realValueRatio),altitudeDB,minAltitude,maxAltitude,countWrongReadingsAltitude[0],countCorrectReadingsAltitude,countWrongReadingsAltitude[1]);
			xSemaphoreGive( xSemaphoreAltitude);

			xSemaphoreTake( xSemaphoreSpeed, portMAX_DELAY);
				sprintf(dataRecord + strlen(dataRecord),"\tSpecific Risk (Velocimeter/Speed):\r\n\t\tRisk Level: %d (%s)\r\n\t\tAlert: \"%s\"\r\n\t\tActual Speed: %0.2f km/h\r\n\t\tCounters (Wrong Too Low, Wrong Low, Correct, Wrong High, Wrong Too High): (%d, %d, %d, %d, %d)\r\n\r\n",
					specRiskSpeed,getLevelRisk(specRiskSpeed),alertSpeed,speedDB,countWrongReadingsSpeed[0],countWrongReadingsSpeed[1],countCorrectReadingsSpeed,countWrongReadingsSpeed[2],countWrongReadingsSpeed[3]);
			xSemaphoreGive( xSemaphoreSpeed);

			xSemaphoreTake( xSemaphorePresence, portMAX_DELAY);
				sprintf(dataRecord + strlen(dataRecord),"\tSpecific Risk (Presence):\r\n\t\tRisk Level: %d (%s)\r\n\t\tAlert: \"%s\"\r\n\t\tActual Presence: %d\r\n\t\tCounters (Wrong Low, Correct, Wrong High): (%d, %d, %d)\r\n\r\n",
					specRiskPresence,getLevelRisk(specRiskPresence),alertPresence,presenceDB,countWrongReadingsPresence[0],countCorrectReadingsPresence,countWrongReadingsPresence[1]);
			xSemaphoreGive( xSemaphorePresence);

			XTime_GetTime(&timeX);
			sprintf(dataRecord + strlen(dataRecord),"[%lld] Ended SD Recording.\r\n",(timeX/(COUNTS_PER_SECOND/1000)));

			//Write into SD
			xSemaphoreTake( xSemaphoreSD, portMAX_DELAY);
				Status = SDFileWrite(dataRecord);
				if (Status != XST_SUCCESS) {
					xil_printf("Writing record into SD failed!\r\n");
				}
			xSemaphoreGive( xSemaphoreSD);
		}
		vTaskDelayUntil(&xLastTimeAwake, 5000/portTICK_RATE_MS);
	}
}

static void buzzerTask(void *pvParameters){ //Not using
	TickType_t xLastTimeAwake;
	xLastTimeAwake = 0;
	int level = 0;

	for( ;; ){
		xSemaphoreTake(xSemaphoreBuzzer,portMAX_DELAY);
			xSemaphoreTake(xSemaphoreRisk, portMAX_DELAY );
				level = genRisk;
			xSemaphoreGive(xSemaphoreRisk);

			if(level == 1){
				GPIO_setPin(&myDeviceJE_o, 4, 1);
				usleep(5000);
				GPIO_setPin(&myDeviceJE_o, 4, 0);
				usleep(5000);
			}else if(level == 2){
				GPIO_setPin(&myDeviceJE_o, 4, 1);
				usleep(2000);
				GPIO_setPin(&myDeviceJE_o, 4, 0);
				usleep(2000);
			}else if(level == 3){
				GPIO_setPin(&myDeviceJE_o, 4, 1);
				usleep(600);
				GPIO_setPin(&myDeviceJE_o, 4, 0);
				usleep(600);
			}else{
				GPIO_setPin(&myDeviceJE_o, 4, 0);
			}

		xSemaphoreGive(xSemaphoreBuzzer);

		vTaskDelayUntil(&xLastTimeAwake, 15/portTICK_RATE_MS);
	}
}

/*
 * //////////////////////FUNCTIONS ///////////////////////////////////////////
 */

void readAccelerometer(){
	float AuxAcc[2] = {0.0, 0.0};

	for(int i=0; i<3; i++){
		bufferSend[0]= 0x3B;
		ByteCount = XIic_Send(XPAR_AXI_IIC_0_BASEADDR, MPU, bufferSend, 1, XIIC_STOP);
		if(ByteCount == 0){
			printf("\r\nError: GY-521!\r\n");
			xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
				if(SDFileWrite("\r\nError: GY-521!\r\n") != XST_SUCCESS){
					printf("Can´t access SD - Accelerometer!\r\n");
				}
			xSemaphoreGive(xSemaphoreSD);
			break;
		}
		ByteCount = XIic_Recv(XPAR_AXI_IIC_0_BASEADDR, MPU, bufferRecv, 6, XIIC_STOP);

		AcX = bufferRecv[0]<<8|bufferRecv[1];
		AcY = bufferRecv[2]<<8|bufferRecv[3];
		AcZ = bufferRecv[4]<<8|bufferRecv[5];

		AuxAcc[0] += atan((-1)*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_A_DEG;
		AuxAcc[1] += atan((-1)*(AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_A_DEG;
	}

	Acc[1] = AuxAcc[1]/3;
	Acc[0] = AuxAcc[0]/3;

	//Check AxisX
	if((Acc[0] >= (-30.0)) && (Acc[0] <= (30.0))){
		AxisX = 0;
	}else{
		if(AxisX<20){
			AxisX++;
		}
	}
	//Check AxisY
	if((Acc[1] >= (-20.0)) && (Acc[1] <= (20.0))){
		AxisY = 0;
	}else{
		if(AxisY<20){
			AxisY++;
		}
	}

	//Check counters
	if(specRiskTilt!=3){
		if((AxisX == 0) &&(AxisY == 0)){ //Within tilt range
			if(countCorrectReadingsTilt<20){
				countCorrectReadingsTilt++;
			}
			if(countCorrectReadingsTilt == 2){
				//specRiskTilt->3 or 0, cannot rectify
				if(specRiskTilt == 2){
					specRiskTilt = 1;
					countCorrectReadingsTilt = 0;
					strcpy(alertTilt, "Rectified Angle");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertTilt,specRiskTilt);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Accelerometer!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);

				}
				else if(specRiskTilt == 1){
					specRiskTilt = 0;
					strcpy(alertTilt, " ");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] No Risk Angle\r\n",(xTaskGetTickCount()*10));
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Accelerometer!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
			}
		}else{
			if(specRiskTilt != 3){
				countCorrectReadingsTilt = 0;
				if((AxisX < 3) || (AxisY < 3)){
					if(AxisX >= 3){
						if(AxisX >= 5){
							specRiskTilt = 2;

							xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
								sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertTilt,specRiskTilt);
								if(SDFileWrite(send2uSD) != XST_SUCCESS){
									printf("Can´t access SD - Accelerometer!\r\n");
								}
							xSemaphoreGive(xSemaphoreSD);
						}else{
							specRiskTilt = 1;
							strcpy(alertTilt, "High Roll Angle");

							xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
								sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertTilt,specRiskTilt);
								if(SDFileWrite(send2uSD) != XST_SUCCESS){
									printf("Can´t access SD - Accelerometer!\r\n");
								}
							xSemaphoreGive(xSemaphoreSD);
						}
					}
					if(AxisY >= 3){
						if(AxisY >= 5){
							specRiskTilt = 2;

							xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
								sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertTilt,specRiskTilt);
								if(SDFileWrite(send2uSD) != XST_SUCCESS){
									printf("Can´t access SD - Accelerometer!\r\n");
								}
							xSemaphoreGive(xSemaphoreSD);
						}else{
							specRiskTilt = 1;
							strcpy(alertTilt, "High Pitch Angle");

							xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
								sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertTilt,specRiskTilt);
								if(SDFileWrite(send2uSD) != XST_SUCCESS){
									printf("Can´t access SD - Accelerometer!\r\n");
								}
							xSemaphoreGive(xSemaphoreSD);
						}
					}
				}else{
					if((AxisX >= 5) && (AxisY >= 5)){
						specRiskTilt = 3;

						xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
							sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertTilt,specRiskTilt);
							if(SDFileWrite(send2uSD) != XST_SUCCESS){
								printf("Can´t access SD - Accelerometer!\r\n");
							}
						xSemaphoreGive(xSemaphoreSD);
					}else{
						specRiskTilt = 2;
						strcpy(alertTilt, "High Roll and Pitch Angle");

						xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
							sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertTilt,specRiskTilt);
							if(SDFileWrite(send2uSD) != XST_SUCCESS){
								printf("Can´t access SD - Accelerometer!\r\n");
							}
						xSemaphoreGive(xSemaphoreSD);
					}
				}
			}
		}
	}
}

void readUltrasonicSensor(){
	long distance = 0;
	long altitude = 0;
	XTime tStart,tEnd;
	u32 tUsed;

	//Retrieving data

	for(int iUs = 0; iUs < 3; iUs++){
		GPIO_setPin(&myDeviceJD_io, 7, 1);
		usleep(10);
		GPIO_setPin(&myDeviceJD_io, 7, 0);

		while (GPIO_getPin(&myDeviceJD_io, 3) == 0){
		}

		XTime_GetTime(&tStart);

		while (GPIO_getPin(&myDeviceJD_io, 3) == 1){
		}

		XTime_GetTime(&tEnd);

		tUsed = ((tEnd - tStart)/(COUNTS_PER_SECOND/1000000));
		distance = tUsed/59;

		if(distance > altitude){
			altitude = distance;
		}

		usleep(200);
	}

	altitudeDB = altitude;

	//Comparing and detect phase

	if(specRiskAltitude!=3){
		if(altitude < minAltitude){ //Altitude under 10000 m (Low Altitude)
			countCorrectReadingsAltitude = 0;
			if(countWrongReadingsAltitude[0] < 20){
				countWrongReadingsAltitude[0]++;
			}
			countWrongReadingsAltitude[1] = 0;
			if(countWrongReadingsAltitude[0] >= 3){
				if(countWrongReadingsAltitude[0] >=6){
					specRiskAltitude = 2;
					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertAltitude,specRiskAltitude);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Ultrasonic!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}else{
					specRiskAltitude = 1;
					strcpy(alertAltitude, "Low Altitude");
					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertAltitude,specRiskAltitude);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Ultrasonic!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
			}
		}else if((altitude >= minAltitude) && (altitude <= maxAltitude)){ //Altitude between 10000 & 12000 m (OK)
			if(countCorrectReadingsAltitude < 20){
				countCorrectReadingsAltitude++;
			}
			countWrongReadingsAltitude[0] = 0;
			countWrongReadingsAltitude[1] = 0;
			if(countCorrectReadingsAltitude == 3){ //Rectify specRisk
				//specRiskAltitude->3 or 0, cannot rectify
				if(specRiskAltitude == 2){
					specRiskAltitude = 1;
					countCorrectReadingsAltitude = 0;
					strcpy(alertAltitude, "Rectified Altitude");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertAltitude,specRiskAltitude);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Ultrasonic!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
				else if(specRiskAltitude == 1){
					specRiskAltitude = 0;
					strcpy(alertAltitude, " ");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] No Risk Altitude\r\n",(xTaskGetTickCount()*10));
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Ultrasonic!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
			}
		} else if(altitude > maxAltitude){ // Altitude over 12000 m (High Altitude)
			countCorrectReadingsAltitude = 0;
			countWrongReadingsAltitude[0] = 0;
			if(countWrongReadingsAltitude[1]<20){
				countWrongReadingsAltitude[1]++;
			}
			if(countWrongReadingsAltitude[1] >= 3){
				if(countWrongReadingsAltitude[1] >=6){
					specRiskAltitude = 3;

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertAltitude,specRiskAltitude);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Ultrasonic!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
				else{
					specRiskAltitude = 2;
					strcpy(alertAltitude, "High Altitude");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertAltitude,specRiskAltitude);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Ultrasonic!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
			}
		}else{ //Other or error
			printf("Ultrasonic Sensor Out of Value");
		}
	}
}

void readVelocimeter(){
	float speed;
	u16 data14 = 0;

	data14 = Xil_In32(C_BASEADDR + 0x278);

	//
	// 0->65535 equivalent 754->900 =>
	// data14/65535*146 is 0->146(diference 900-754)
	//
	speed = ((data14/65535.0)*146.0)+754.0;

	speedDB = speed;

	if(specRiskSpeed!=3){
		if((speed >= 754.0) && (speed < 783.2)){ //1st sector (Too Slow)
			countCorrectReadingsSpeed = 0;
			if(countWrongReadingsSpeed[0]<20)
				countWrongReadingsSpeed[0]++;
			countWrongReadingsSpeed[1] = 0;
			countWrongReadingsSpeed[2] = 0;
			countWrongReadingsSpeed[3] = 0;
			if(countWrongReadingsSpeed[0] >= 3){
				specRiskSpeed = 2;
				strcpy(alertSpeed, "Too Low Speed");

				xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
					sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertSpeed,specRiskSpeed);
					if(SDFileWrite(send2uSD) != XST_SUCCESS){
						printf("Can´t access SD - Velocimeter!\r\n");
					}
				xSemaphoreGive(xSemaphoreSD);
			}
		}else if((speed >= 783.2) && (speed < 812.4)){ //2nd sector (Slow)
			countCorrectReadingsSpeed = 0;
			countWrongReadingsSpeed[0] = 0;
			if(countWrongReadingsSpeed[1]<20)
				countWrongReadingsSpeed[1]++;
			countWrongReadingsSpeed[2] = 0;
			countWrongReadingsSpeed[3] = 0;
			if(countWrongReadingsSpeed[1] >= 3){
				specRiskSpeed = 1;
				strcpy(alertSpeed, "Low Speed");

				xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
					sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertSpeed,specRiskSpeed);
					if(SDFileWrite(send2uSD) != XST_SUCCESS){
						printf("Can´t access SD - Velocimeter!\r\n");
					}
				xSemaphoreGive(xSemaphoreSD);
			}
		}else if((speed >= 812.4) && (speed < 841.6)){ //3rd sector (OK)
			if(countCorrectReadingsSpeed<20)
				countCorrectReadingsSpeed++;
			countWrongReadingsSpeed[0] = 0;
			countWrongReadingsSpeed[1] = 0;
			countWrongReadingsSpeed[2] = 0;
			countWrongReadingsSpeed[3] = 0;
			if(countCorrectReadingsSpeed == 2){ //Rectify specRisk
				//specRiskSpeed->3 or 0, cannot rectify
				if(specRiskSpeed == 2){
					specRiskSpeed = 1;
					countCorrectReadingsSpeed = 0;
					strcpy(alertSpeed, "Rectified Speed");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertSpeed,specRiskSpeed);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Velocimeter!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
				else if(specRiskSpeed == 1){
					specRiskSpeed = 0;
					strcpy(alertSpeed, " ");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] No Risk Speed\r\n",(xTaskGetTickCount()*10));
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Velocimeter!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
			}
		}else if((speed >= 841.6) && (speed < 870.8)){ //4th sector (Fast)
			countCorrectReadingsSpeed = 0;
			countWrongReadingsSpeed[0] = 0;
			countWrongReadingsSpeed[1] = 0;
			if(countWrongReadingsSpeed[2]<20)
				countWrongReadingsSpeed[2]++;
			countWrongReadingsSpeed[3] = 0;
			if(countWrongReadingsSpeed[2] >= 3){
				specRiskSpeed = 1;
				strcpy(alertSpeed, "High Speed");

				xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
					sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertSpeed,specRiskSpeed);
					if(SDFileWrite(send2uSD) != XST_SUCCESS){
						printf("Can´t access SD - Velocimeter!\r\n");
					}
				xSemaphoreGive(xSemaphoreSD);
			}
		}else if((speed >= 870.8) && (speed <= 900)){ //5th sector (Too Fast)
			countCorrectReadingsSpeed = 0;
			countWrongReadingsSpeed[0] = 0;
			countWrongReadingsSpeed[1] = 0;
			countWrongReadingsSpeed[2] = 0;
			if(countWrongReadingsSpeed[3]<20)
				countWrongReadingsSpeed[3]++;
			if(countWrongReadingsSpeed[3] >= 3){
				specRiskSpeed = 2;
				strcpy(alertSpeed, "Too High Speed");

				xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
					sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertSpeed,specRiskSpeed);
					if(SDFileWrite(send2uSD) != XST_SUCCESS){
						printf("Can´t access SD - Velocimeter!\r\n");
					}
				xSemaphoreGive(xSemaphoreSD);
			}
		}else{ //Other or error
			printf("Velocimeter Out of Value");
		}
	}
}

void readPresence(){
	int presence = 0;

	if(GPIO_getPin(&myDeviceJD_io, 1) == 0){ //Detected first pilot?
		presence++;
	}
	if( GPIO_getPin(&myDeviceJD_io, 2) == 0){ //There is a second pilot?
		presence++;
	}
	if(GPIO_getPin(&myDeviceJD_io, 4) == 1){ //There is movement near the door?
		presence++;
	}

	presenceDB = presence;

	if(specRiskPresence!=3){
		if(presence == 2){ //2 people in airplane cockpit
			if(countCorrectReadingsPresence<20)
					countCorrectReadingsPresence++;
			countWrongReadingsPresence[0] = 0;
			countWrongReadingsPresence[1] = 0;
			if(countCorrectReadingsPresence == 3){ //Rectify specRisk
				//specRiskPresence->3 or 0, cannot rectify
				if(specRiskPresence == 2){
					specRiskPresence = 1;
					countCorrectReadingsPresence = 0;
					strcpy(alertPresence, "Rectified Presence");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertPresence,specRiskPresence);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Presence!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
				else if(specRiskPresence == 1){
					specRiskPresence = 0;
					strcpy(alertPresence, " ");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] No Risk Presence\r\n",(xTaskGetTickCount()*10));
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Presence!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
			}
		} else if(presence <= 1){ //only 1 person detected (or no pilots)
			countCorrectReadingsPresence = 0;
			if(countWrongReadingsPresence[0]<20)
				countWrongReadingsPresence[0]++;
			countWrongReadingsPresence[1] = 0;
			if(countWrongReadingsPresence[0] >= 3){
				if(countWrongReadingsPresence[0] >=6){
					specRiskPresence = 3;
					strcpy(alertPresence, "Risk: Low Presence");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertPresence,specRiskPresence);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Presence!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
				else{
					specRiskPresence = 2;
					strcpy(alertPresence, "Low Presence");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertPresence,specRiskPresence);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Presence!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
			}
		} else if(presence > 2){ // 2+ people in airplane cockpit
			countCorrectReadingsPresence = 0;
			countWrongReadingsPresence[0] = 0;
			if(countWrongReadingsPresence[1]<20)
				countWrongReadingsPresence[1]++;
			if(countWrongReadingsPresence[1] >= 3){
				if(countWrongReadingsPresence[1] >=6){
					specRiskPresence = 2;
					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertPresence,specRiskPresence);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Presence!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
				else{
					specRiskPresence = 1;
					strcpy(alertPresence, "High Presence");

					xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
						sprintf(send2uSD,"\r\n[%ld] %s - Level: %d\r\n",(xTaskGetTickCount()*10),alertPresence,specRiskPresence);
						if(SDFileWrite(send2uSD) != XST_SUCCESS){
							printf("Can´t access SD - Presence!\r\n");
						}
					xSemaphoreGive(xSemaphoreSD);
				}
			}
		}
	}
}

void pmodLedWork(u8 led){
	GPIO_setPin(&myDeviceJE_o, 5, 0);
	GPIO_setPin(&myDeviceJE_o, 6, 0);
	GPIO_setPin(&myDeviceJE_o, 7, 0);
	GPIO_setPin(&myDeviceJE_o, 8, 0);

	if(led >= 1){
		GPIO_setPin(&myDeviceJE_o, 5, 1);
	}
	if(led >= 2){
		GPIO_setPin(&myDeviceJE_o, 6, 1);
	}
	if(led >= 3){
		GPIO_setPin(&myDeviceJE_o, 7, 1);
	}
	if(led >= 4){
		GPIO_setPin(&myDeviceJE_o, 8, 1);
	}

	xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
		sprintf(send2uSD,"\r\n[%ld] Activated PmodLED - LEDs ON: %d\r\n",(xTaskGetTickCount()*10),led);
		if(SDFileWrite(send2uSD) != XST_SUCCESS){
			printf("Can´t access SD - PmodLED!\r\n");
		}
	xSemaphoreGive(xSemaphoreSD);
}

void rgbLedWork(u8 risk){
	if(risk == 1){ //Light Risk = Blue
		GPIO_setPin(&myDeviceJE_o, 1, 0);
		GPIO_setPin(&myDeviceJE_o, 2, 0);
		GPIO_setPin(&myDeviceJE_o, 3, 1);
	}else if(risk == 2){ //Grave/Serious Risk = Yellow
		GPIO_setPin(&myDeviceJE_o, 1, 1);
		GPIO_setPin(&myDeviceJE_o, 2, 1);
		GPIO_setPin(&myDeviceJE_o, 3, 0);
	}else if(risk == 3){ //Critical Risk = Red
		GPIO_setPin(&myDeviceJE_o, 1, 1);
		GPIO_setPin(&myDeviceJE_o, 2, 0);
		GPIO_setPin(&myDeviceJE_o, 3, 0);
	}else{ //Risk 0 or not defined = OFF
		GPIO_setPin(&myDeviceJE_o, 1, 0);
		GPIO_setPin(&myDeviceJE_o, 2, 0);
		GPIO_setPin(&myDeviceJE_o, 3, 0);
	}

	xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
		sprintf(send2uSD,"\r\n[%ld] Activated RGB LED - %s\r\n",(xTaskGetTickCount()*10),getLevelRisk(risk));
		if(SDFileWrite(send2uSD) != XST_SUCCESS){
			printf("Can´t access SD - RGB LED!\r\n");
		}
	xSemaphoreGive(xSemaphoreSD);
}

int SDFileWrite(char *record){
	FRESULT Res;
	UINT NumBytesWritten;
	u32 BuffCnt;
	u32 MaxSize = (1024*2);
	TCHAR *Path = "0:/"; //Main partition
	char text[MaxSize];


	//Copy message to Buffer
	strcpy(text, record);

	for(BuffCnt = 0; BuffCnt < strlen(text); BuffCnt++){
		SourceAddress[BuffCnt] = (int) text[BuffCnt];
	}

	//Mount SD_card
	Res = f_mount(&fatfs, Path, 0);
	if (Res != FR_OK) {
		xil_printf("SD f_mount failed \r\n");
		return XST_FAILURE;
	}

	SD_File = (char *)FileName;

	//Open the file
	Res = f_open(&fil, SD_File, FA_OPEN_APPEND | FA_WRITE );
	if (Res) {
		xil_printf("SD f_open failed \r\n");
		return XST_FAILURE;
	}

	//Write into file the buffer
	Res = f_write(&fil, (const void*)SourceAddress, strlen(text), &NumBytesWritten);
	if (Res) {
		xil_printf("SD f_write failed \r\n");
		return XST_FAILURE;
	}

	//Close the file
	Res = f_close(&fil);
	if (Res) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

void useActuator(u8 risk, u8 counter){
	char message[64];
	if(risk == 3){
		pmodLedWork(4);
		rgbLedWork(risk);

		xSemaphoreGive(xSemaphoreBuzzer);
		if (xTimerReset(timerHndl3SecTimeout, 0)!=pdPASS) { //start timer 3 seconds
			for(;;);
		}
		xSemaphoreGive(xSemaphoreAutopilot);
		sprintf(message,"\r\n[%ld] * Autopilot started! *\r\n", (xTaskGetTickCount()*10));
		xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
			if(SDFileWrite(message) != XST_SUCCESS){
				printf("Can´t access SD - RGB LED!\r\n");
			}
		xSemaphoreGive(xSemaphoreSD);
		printf("%s",message);
		if (xTimerReset(timerHndl15SecTimeout, 0)!=pdPASS) { // start timer 15 seconds
			for(;;);
		}
	}else if(risk == 2){
		pmodLedWork(3);
		rgbLedWork(risk);

		xSemaphoreGive(xSemaphoreBuzzer);
		if (xTimerReset(timerHndl3SecTimeout, 0)!=pdPASS) { // start timer 3 seconds
			for(;;);
		}
	}else if(risk == 1){
		pmodLedWork(counter);
		rgbLedWork(risk);

		xSemaphoreGive(xSemaphoreBuzzer);
		if (xTimerReset(timerHndl3SecTimeout, 0)!=pdPASS) { // start timer 3 seconds
			for(;;);
		}
	}else if(risk == 0){
		pmodLedWork(0);
		rgbLedWork(0);
	}
}

/*-----------------------------------------------------------*/

//EXTRA FUNCTIONS

void setup(){
	ConfigPtr = XIic_LookupConfig(IIC_DEVICE);
	StatusIIC = XIic_CfgInitialize(&XiicInstance, ConfigPtr, ConfigPtr->BaseAddress);
	StatusIIC = XIic_SetAddress(&XiicInstance, XII_ADDR_TO_SEND_TYPE, MPU);
	StatusIIC = XIic_Start(&XiicInstance);

	bufferSend[0]=0x6B;
	bufferSend[1]=0x00;
	ByteCount = XIic_Send(XPAR_AXI_IIC_0_BASEADDR, MPU, bufferSend, 2, XIIC_STOP);
}

void setupKP(){
	strcpy(code," ");
	KYPD_begin(&myDeviceKP, XPAR_PMODKYPD_0_AXI_LITE_GPIO_BASEADDR);
	KYPD_loadKeyTable(&myDeviceKP, (u8*) DEFAULT_KEYTABLE);
	Xil_Out32(myDeviceKP.GPIO_addr, 0xF);
}

void setupBoard(){
	XGpio_Initialize(&options, 0);
	XGpio_SetDataDirection(&options, 2, 0xF);
	XGpio_SetDataDirection(&options, 1, 0x0);
}

void setupGPIO(){
	GPIO_begin(&myDeviceJD_io, XPAR_PMODGPIO_1_AXI_LITE_GPIO_BASEADDR, 0x0F);
	GPIO_begin(&myDeviceJE_o, XPAR_PMODGPIO_2_AXI_LITE_GPIO_BASEADDR, 0x00);
}


char * getLevelRisk(int risk){
	if(risk == 0){
		return "No Risk";
	}else if(risk == 1){
		return "Light Risk";
	}else if(risk == 2){
		return "Severe Risk";
	}else if(risk == 3){
		return "Critical Risk";
	}else{
		return '\0';
	}
}

static void vTimerCallback3SecExpired(xTimerHandle pxTimer) {
	xSemaphoreTake(xSemaphoreBuzzer,portMAX_DELAY);
}

static void vTimerCallback15SecExpired(xTimerHandle pxTimer) {
	xSemaphoreTake(xSemaphoreAutopilot,portMAX_DELAY);
	pmodLedWork(0);
	rgbLedWork(0);
	xSemaphoreTake(xSemaphoreRisk, portMAX_DELAY );
		genRisk = 0;
	xSemaphoreGive(xSemaphoreRisk);
	xSemaphoreTake(xSemaphoreAltitude, portMAX_DELAY );
	    specRiskAltitude = 0;
	xSemaphoreGive(xSemaphoreAltitude);
	xSemaphoreTake(xSemaphorePresence, portMAX_DELAY );
	    specRiskPresence = 0;
	xSemaphoreGive(xSemaphorePresence);
	xSemaphoreTake(xSemaphoreSpeed, portMAX_DELAY );
	    specRiskSpeed = 0;
	xSemaphoreGive(xSemaphoreSpeed);
	xSemaphoreTake(xSemaphoreTilt, portMAX_DELAY );
	    specRiskTilt = 0;
	xSemaphoreGive(xSemaphoreTilt);
	xSemaphoreTake(xSemaphoreSD,portMAX_DELAY);
		sprintf(send2uSD,"[%ld] No risk due to autopilot\r\n",(xTaskGetTickCount()*10));
		if(SDFileWrite(send2uSD) != XST_SUCCESS){
			printf("Can´t access SD - Risk!\r\n");
		}
	xSemaphoreGive(xSemaphoreSD);
}
