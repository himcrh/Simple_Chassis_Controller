#ifndef __LINE_BSP__
#define __LINE_BSP__

/*From cubemx*/
#include "stm32f4xx.h"
#include "stdbool.h"
#include "gpio.h"
/*From mine*/

typedef struct Line_IO{
	
	GPIO_TypeDef * IO_Port;	//Pin Port
	uint16_t IO_Pin;				//Pin
	bool Pin_State;					//State
	bool Triggered;					//是否触发过
	
	
}Line_IO;

typedef struct Handle_IO{
	
	bool All_Pin_Up;				//全部触发
	bool All_Pin_Down;			//全部未触发
	bool All_Pin_Triggered;	//全部触发过
	int8_t Cur_Counter;	//当前触发个数
	
}Handle_IO;

/*车的俯视图，区分前后左右，对于每一个巡线模块，从左到右计数增大*/
extern Line_IO Fr_Line[8],Bk_Line[8],Lt_Line[8],Rt_Line[8];
extern Handle_IO Fr_Handle,Bk_Handle,Lt_Handle,Rt_Handle;

void Emptying_Triggered(Line_IO Cur_Line[] ,Handle_IO* Cur_Handle);
	
#endif
