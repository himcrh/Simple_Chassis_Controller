#include "Line_bsp.h"

Line_IO Fr_Line[8] = {
	{.IO_Port = Fr_IO_L1_GPIO_Port,.IO_Pin = Fr_IO_L1_Pin,.Pin_State = false},	//L1
	{.IO_Port = Fr_IO_L2_GPIO_Port,.IO_Pin = Fr_IO_L2_Pin,.Pin_State = false},	//L2
	{.IO_Port = Fr_IO_L3_GPIO_Port,.IO_Pin = Fr_IO_L3_Pin,.Pin_State = false},	//L3
	{.IO_Port = Fr_IO_L4_GPIO_Port,.IO_Pin = Fr_IO_L4_Pin,.Pin_State = false},	//L4
	{.IO_Port = Fr_IO_L5_GPIO_Port,.IO_Pin = Fr_IO_L5_Pin,.Pin_State = false},	//L5
	{.IO_Port = Fr_IO_L6_GPIO_Port,.IO_Pin = Fr_IO_L6_Pin,.Pin_State = false},	//L6
	{.IO_Port = Fr_IO_L7_GPIO_Port,.IO_Pin = Fr_IO_L7_Pin,.Pin_State = false},	//L7
	{.IO_Port = Fr_IO_L8_GPIO_Port,.IO_Pin = Fr_IO_L8_Pin,.Pin_State = false},	//L8
};

Line_IO Bk_Line[8] = {
	{.IO_Port = Bk_IO_L1_GPIO_Port,.IO_Pin = Bk_IO_L1_Pin,.Pin_State = false},	//L1
	{.IO_Port = Bk_IO_L2_GPIO_Port,.IO_Pin = Bk_IO_L2_Pin,.Pin_State = false},	//L2
	{.IO_Port = Bk_IO_L3_GPIO_Port,.IO_Pin = Bk_IO_L3_Pin,.Pin_State = false},	//L3
	{.IO_Port = Bk_IO_L4_GPIO_Port,.IO_Pin = Bk_IO_L4_Pin,.Pin_State = false},	//L4
	{.IO_Port = Bk_IO_L5_GPIO_Port,.IO_Pin = Bk_IO_L5_Pin,.Pin_State = false},	//L5
	{.IO_Port = Bk_IO_L6_GPIO_Port,.IO_Pin = Bk_IO_L6_Pin,.Pin_State = false},	//L6
	{.IO_Port = Bk_IO_L7_GPIO_Port,.IO_Pin = Bk_IO_L7_Pin,.Pin_State = false},	//L7
	{.IO_Port = Bk_IO_L8_GPIO_Port,.IO_Pin = Bk_IO_L8_Pin,.Pin_State = false},	//L8
};

Line_IO Lt_Line[8] = {
	{.IO_Port = Lt_IO_L1_GPIO_Port,.IO_Pin = Lt_IO_L1_Pin,.Pin_State = false},	//L1
	{.IO_Port = Lt_IO_L2_GPIO_Port,.IO_Pin = Lt_IO_L2_Pin,.Pin_State = false},	//L2
	{.IO_Port = Lt_IO_L3_GPIO_Port,.IO_Pin = Lt_IO_L3_Pin,.Pin_State = false},	//L3
	{.IO_Port = Lt_IO_L4_GPIO_Port,.IO_Pin = Lt_IO_L4_Pin,.Pin_State = false},	//L4
	{.IO_Port = Lt_IO_L5_GPIO_Port,.IO_Pin = Lt_IO_L5_Pin,.Pin_State = false},	//L5
	{.IO_Port = Lt_IO_L6_GPIO_Port,.IO_Pin = Lt_IO_L6_Pin,.Pin_State = false},	//L6
	{.IO_Port = Lt_IO_L7_GPIO_Port,.IO_Pin = Lt_IO_L7_Pin,.Pin_State = false},	//L7
};

Line_IO Rt_Line[8] = {
	{.IO_Port = Rt_IO_L1_GPIO_Port,.IO_Pin = Rt_IO_L1_Pin,.Pin_State = false},	//L1
	{.IO_Port = Rt_IO_L2_GPIO_Port,.IO_Pin = Rt_IO_L2_Pin,.Pin_State = false},	//L2
	{.IO_Port = Rt_IO_L3_GPIO_Port,.IO_Pin = Rt_IO_L3_Pin,.Pin_State = false},	//L3
	{.IO_Port = Rt_IO_L4_GPIO_Port,.IO_Pin = Rt_IO_L4_Pin,.Pin_State = false},	//L4
	{.IO_Port = Rt_IO_L5_GPIO_Port,.IO_Pin = Rt_IO_L5_Pin,.Pin_State = false},	//L5
	{.IO_Port = Rt_IO_L6_GPIO_Port,.IO_Pin = Rt_IO_L6_Pin,.Pin_State = false},	//L6
	{.IO_Port = Rt_IO_L7_GPIO_Port,.IO_Pin = Rt_IO_L7_Pin,.Pin_State = false},	//L7
};

Handle_IO Fr_Handle,Bk_Handle,Lt_Handle,Rt_Handle;

void Emptying_Triggered(Line_IO Cur_Line[] ,Handle_IO* Cur_Handle){
	for(int i=0;i<8;i++){
		Cur_Line[i].Triggered = false;
	}
	Cur_Handle->All_Pin_Triggered = false;
}

void Line_Init(void){
	

}
