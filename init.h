#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "tm4c123gh6pm.h"
#define PortD_IRQn 3
#define PortF_IRQn 30
#define PortA_IRQn 0
#define PortB_IRQn 1


void PortE_Init(void);
void PortF_Init(void);
void PortD_Init(void);
void PortA_Init(void);
void PortB_Init(void);
