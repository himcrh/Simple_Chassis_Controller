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
	bool Triggered;					//�Ƿ񴥷���
	
	
}Line_IO;

typedef struct Handle_IO{
	
	bool All_Pin_Up;				//ȫ������
	bool All_Pin_Down;			//ȫ��δ����
	bool All_Pin_Triggered;	//ȫ��������
	int8_t Cur_Counter;	//��ǰ��������
	
}Handle_IO;

/*���ĸ���ͼ������ǰ�����ң�����ÿһ��Ѳ��ģ�飬�����Ҽ�������*/
extern Line_IO Fr_Line[8],Bk_Line[8],Lt_Line[8],Rt_Line[8];
extern Handle_IO Fr_Handle,Bk_Handle,Lt_Handle,Rt_Handle;

void Emptying_Triggered(Line_IO Cur_Line[] ,Handle_IO* Cur_Handle);
	
#endif
