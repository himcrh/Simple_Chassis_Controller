#ifndef __MY_TASK__
#define __MY_TASK__

/*From cubemx*/
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "usart.h"

/*From mine*/
#include "can_bsp.h"
#include "rm2006.h"
#include "Chassis_bsp.h"
#include "Line_bsp.h"
#include "mechanical_arm_uart.h"
#include "fashion_star_uart_servo.h"
#include "Arm_Move_bsp.h"

extern uint8_t Uart_Rx_Buffer[10];
extern uint8_t Sence_Rx_Buffer[8];
extern uint8_t operation_code;

/*≥Ã–Ú≥ı ºªØ*/
void Program_Init(void);



#endif
