#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "tm4c123gh6pm.h"
#include "init.h"


//MOTOR0,1+PASSENGER 2,3       2->UP,4 3->DOWN,8
void PortE_Init(void){
	SYSCTL->RCGCGPIO |= 0x00000010; // 1)
	while((SYSCTL_PRGPIO_R & 0x00000010) == 0){}
  GPIOE->DIR = 0x06; // 5) 
	GPIOE->PUR = 0x18; //                                   
	GPIOE->DEN = 0x1E; // 7)                              
}

//DRIVER
void PortF_Init(void){
	SYSCTL->RCGCGPIO |= 0x00000020; // 1) F clock
	while((SYSCTL_PRGPIO_R & 0x00000020) == 0){}
	GPIOF->LOCK = 0x4C4F434B; 
	GPIOF->CR = 0x1F;  
	GPIOF->AMSEL= 0x00; 
	GPIOF->PCTL = 0x00000000;
	GPIOF->DIR = 0x0E; 
	GPIOF->AFSEL = 0x00; 
	GPIOF->AFSEL = 0x00; 
	GPIOF->PUR = 0x11; 
	GPIOF->DEN = 0x1F; 
	GPIOF->DATA = 0x08;
		
}

// pin 2 & 3
void PortA_Init(void){
	SYSCTL->RCGCGPIO |= 0x00000001; // 1)
	while((SYSCTL_PRGPIO_R & 0x00000001) == 0){}
	GPIOA->AMSEL= 0x00; // 3) disable analog function
	GPIOA->PCTL = 0x00000000; // 4) GPIO clear bit PCTL
	GPIOA->DIR = 0x00; // 5) 
	GPIOA->AFSEL = 0x00; // 6) no alternate function
	GPIOA->AFSEL = 0x00; // 6) no alternate function
	GPIOA->PUR = 0x30; // 
	GPIOA->DEN = 0x30; // 7) 
		// Setup the interrupt on PortD
	//GPIOA->IS &= ~0x0c; // Make bits PF0 and PF4 level sensitive
	//GPIOA->IBE |= 0x0c; 
		
	GPIOA->IS  &= ~(1<<4)|~(1<<5); /* make bit 4, 0 edge sensitive */
  GPIOA->IBE &=~(1<<4)|~(1<<5);  /* trigger is controlled by IEV */
  GPIOA->IEV &= ~(1<<5)|~(1<<4);   /* falling edge trigger */
	GPIOA->ICR = 0x30; // Clear any Previous Interrupt
	GPIOA->IM |=0x30; // Unmask  the interrupts
	NVIC_EnableIRQ(PortA_IRQn); // Enable the Interrupt for PortF in NVIC
}

void PortD_Init(void){
	SYSCTL->RCGCGPIO |= 0x00000008; // 1)
	while((SYSCTL_PRGPIO_R & 0x0000008) == 0){}
	//GPIOD->LOCK = 0x4C4F434B; // 2) 
	//GPIOD->CR = 0x1F; // 
	//GPIOD->AMSEL= 0x00; // 3) disable analog function
	//GPIOD->PCTL = 0x00000000; // 4) GPIO clear bit PCTL
  GPIOD->DIR = 0x00; // 5) 
	//GPIOD->AFSEL = 0x00; // 6) no alternate function
	//GPIOD->AFSEL = 0x00; // 6) no alternate function
	GPIOD->PUR = 0x03; //                                   
	GPIOD->DEN = 0x03; // 7)                              
	//GPIOD->DATA = 0x00;
	// Setup the interrupt on PortD
	GPIOD->IS  &= ~(1<<0)|~(1<<1); /* make bit 4, 0 edge sensitive */
  GPIOD->IBE &=~(1<<0)|~(1<<1);  /* trigger is controlled by IEV */
  GPIOD->IEV &= ~(1<<0)|~(1<<1);   /* falling edge trigger */
	GPIOD->ICR = 0x3; // Clear any Previous Interrupt
	GPIOD->IM |=0x3; // Unmask  the interrupts
	NVIC_EnableIRQ(PortD_IRQn); // Enable the Interrupt for PortF in NVIC
}

void PortB_Init(void){
		SYSCTL->RCGCGPIO |= 0x00000002; // 1)
	while((SYSCTL_PRGPIO_R & 0x0000002) == 0){}
	//GPIOB->LOCK = 0x4C4F434B; // 2) 
	//GPIOB->CR = 0x1F; // 
	GPIOB->AMSEL= 0x00; // 3) disable analog function
	GPIOB->PCTL = 0x00000000; // 4) GPIO clear bit PCTL
  GPIOB->DIR = 0x00; // 5) 
	//GPIOB->AFSEL = 0x00; // 6) no alternate function
	//GPIOB->AFSEL = 0x00; // 6) no alternate function
	GPIOB->PUR = 0x01; //                                   
	GPIOB->DEN = 0x01; // 7)                              
	//GPIOB->DATA = 0x00;
		// Setup the interrupt on PortF
	GPIOB->ICR = 0x1; // Clear any Previous Interrupt
	GPIOB->IM |=0x1; // Unmask the interrupts for PF0 and PF4
	GPIOB->IS |= 0x1; // Make bits PF0 and PF4 level sensitive
	GPIOB->IEV &= ~0x1; // Sense on Low Level
	NVIC_EnableIRQ(PortB_IRQn); // Enable the Interrupt for PortF in NVIC
}
