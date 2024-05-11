#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "tm4c123gh6pm.h"
#include <stdbool.h>
#include "init.h"


xSemaphoreHandle xBinarySemaphore; //Binary automatic
xSemaphoreHandle xLockSemaphore; // CHILD Lock
xSemaphoreHandle xMutex; //lock UP/DOWN
xSemaphoreHandle xStopSemaphore; //	STOP
//xSemaphoreHandle xStopSemaphore;
QueueHandle_t xQueue; //	Driver queue
QueueHandle_t xQueueP; //Passenger queue

// volatile int* recvValue;
// volatile int sendValue = 0;
static volatile bool child=false;
static volatile bool automatic=false;

void up() {
			GPIO_PORTE_DATA_R = 0x2;
}

void down() {
			GPIO_PORTE_DATA_R = 0x4;
}	

void stop() {
		GPIO_PORTE_DATA_R = 0;
}


void readDriver() {
		int sendValue;
		
	for(;;){
		
			// Window UP
			if((GPIO_PORTF_DATA_R & 0x1) == 0x0){
				sendValue = 0;
				automatic=true;

				
				
				/////////// Check for Manual or Auto Mode
				vTaskDelay(pdMS_TO_TICKS( 2000 ));
				if((GPIO_PORTF_DATA_R & 0x1) == 0x0) {
					sendValue = 1;
					automatic=false;

				}
				///////////
				xQueueSendToBack(xQueue, &sendValue, 100);
				
				/*
				if(automatic) {
					xQueueSendToBack(xQueue, &sendValue, 0);
				}
				else {  //not auto
					xQueueSendToBack(xQueue, &sendValue, 0);
				} */
			} 
			
			
			else if((GPIO_PORTF_DATA_R & 0x10) == 0x0) {
				sendValue = 2;
				automatic=true;

				vTaskDelay(pdMS_TO_TICKS( 2000 ));
				if((GPIO_PORTF_DATA_R & 0x10) == 0x0) {
					sendValue = 3;
					automatic=false;

				}
				///////////
				xQueueSendToBack(xQueue, &sendValue, 100);
				
				/*
				// Window DOWN
				automatic = true;
				sendValue = 2;
				////////////// Check for Manual or Auto Mode
				vTaskDelay(pdMS_TO_TICKS( 2000 ));
				if((GPIO_PORTF_DATA_R & 0x10) == 0x0) {
					automatic = false;
					sendValue = 3;
				}
				//////////////
				if(automatic) {
					xQueueSendToBack(xQueue, &sendValue, 0);
				}
				else {  //not auto
					xQueueSendToBack(xQueue, &sendValue, 0);
				} 
				*/
				
			}			
		}
}





void driver() {
	
	int recvValue;
	BaseType_t xStatus= pdFAIL;
	xQueueReceive(xQueue,&recvValue, 0);
	
	while(1) {
		xStatus= xQueueReceive(xQueue, &recvValue, portMAX_DELAY);
	  xSemaphoreTake(xMutex, portMAX_DELAY);
		if(xStatus==pdPASS){
			if ((recvValue)==0) {
				up();
				xSemaphoreTake(xStopSemaphore, portMAX_DELAY);
				stop();
			}
			
			else if ((recvValue)==1){
				up();
				while((GPIO_PORTF_DATA_R & 0x1) == 0x0);
				stop();
			}
			
			else if ((recvValue)==2) {
				down();
				xSemaphoreTake(xStopSemaphore, portMAX_DELAY);
				stop();
		  	}
			
			else if ((recvValue)==3) {		
				down();
				while((GPIO_PORTF_DATA_R & 0x10) == 0x0);
				stop();			}
		}
		
		xSemaphoreGive(xMutex);
		
	}
}

void passenger() {
	
	int value;
	BaseType_t xStatus= pdFAIL;
	xQueueReceive(xQueueP,&value, 0);
	
	while(1) {
		xStatus= xQueueReceive(xQueueP, &value, portMAX_DELAY);
	  xSemaphoreTake(xMutex, portMAX_DELAY);
		if(xStatus==pdPASS){
			if (value==0) {
				up();
				xSemaphoreTake(xStopSemaphore, portMAX_DELAY);
				stop();
			}
			
			else if (value==1){
				up();
				while((GPIO_PORTE_DATA_R & 0x8) == 0x0);
				stop();
			}
			
			else if (value==2) {
				down();
				xSemaphoreTake(xStopSemaphore, portMAX_DELAY);
				stop();
		  	}
			
			else if (value==3) {		
				down();
				while((GPIO_PORTE_DATA_R & 0x10) == 0x0);
				stop();			}
		}
		
		xSemaphoreGive(xMutex);
		
	}
}

void readPassenger() {
	int value = 0 ;
		while(1) {
			if(((GPIO_PORTE_DATA_R & 0x08) == 0x0) && !child) {
				value = 0;
				automatic=true;
				vTaskDelay(pdMS_TO_TICKS( 2000 ));
				if((GPIO_PORTE_DATA_R & 0x08) == 0x0) {
					 value = 1;
					automatic=false;

				}
					xQueueSendToBack(xQueueP, &value, 100);
				}
			if(((GPIO_PORTE_DATA_R & 0x10) == 0x0) && !child) {
				value = 2;
				automatic=true;

				vTaskDelay(pdMS_TO_TICKS( 2000 ));
				if((GPIO_PORTE_DATA_R & 0x10) == 0x0) {
					value = 3;
					automatic=false;

				}
					xQueueSendToBack(xQueueP, &value, 100);
			}			
		}
}


void GPIOA_Handler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if((GPIO_PORTA_DATA_R&0x10) == 0x0)
	{
	xSemaphoreGiveFromISR(xStopSemaphore, &xHigherPriorityTaskWoken);
	}else if((GPIO_PORTA_DATA_R&0x20) == 0x0)
	{
		xSemaphoreGiveFromISR(xStopSemaphore, &xHigherPriorityTaskWoken);
	}
	//clear interrupt flag
	GPIOA->ICR = 0x30;
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); 
}



void GPIOD_Handler(void){
	if((GPIO_PORTD_DATA_R&0x01) == 0x0)
	{
	child=true;
	GPIOF->DATA = 0X02;

		
	}else if((GPIO_PORTD_DATA_R&0x02) == 0x0)
	{
	child=false;
	GPIOF->DATA = 0X08;
	}
	//clear interrupt flag
	GPIOD->ICR = 0x03; 
}

void GPIOB_Handler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(automatic) {
					xSemaphoreGiveFromISR(xStopSemaphore, &xHigherPriorityTaskWoken);
	}
	else if(!automatic) {
		stop();
	}
	
	//GPIOF->DATA = 0X4;
	GPIOB->ICR = 0x1; // Clear any Previous Interrupt
}


int main() {
	__ASM("CPSIE i");
	PortA_Init();
	PortE_Init();
	PortF_Init();
	PortD_Init();
	PortB_Init();

	xBinarySemaphore = xSemaphoreCreateBinary();
	xMutex = xSemaphoreCreateMutex();                            
	xStopSemaphore  = xSemaphoreCreateBinary();
	//xStopSemaphoreD  = xSemaphoreCreateBinary();
	xQueue  = xQueueCreate(1,sizeof(int));
	xQueueP  = xQueueCreate(1,sizeof(int));
	
	
	xTaskCreate(readDriver,"task1",50,NULL,1,NULL);
	xTaskCreate(readPassenger,"task3",50,NULL,1,NULL);
  xTaskCreate(driver,"task2",50,NULL,2,NULL);
	xTaskCreate(passenger,"task4",50,NULL,2,NULL);
	vTaskStartScheduler();
for(;;);
	
}

