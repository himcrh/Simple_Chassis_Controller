#include "my_task.h"

uint8_t Uart_Rx_Buffer[10];
uint8_t Sence_Rx_Buffer[8];
uint8_t operation_code;

void Program_Init(void){
	
	/*CAN1初始化*/
	CAN_Init(&hcan1);
	
	/*底盘信息初始化*/
	Chassis_Info_Init();
	
	Usart_Init();
}

/*底盘运动控制进程*/
void Chassis_Run(void *argument){
	/*INIT*/

	/*LOOP*/
	while(1){
		
		
		HAL_UART_Receive_IT(&huart6,&operation_code,1);
//		HAL_UART_Receive_IT(&huart4,Sence_Rx_Buffer,8);
		/*如果存在任务模式的切换，则将程序计数器清零*/
		static int Last_Move_Status = -1;
		if(Last_Move_Status != Move_Info.Move_Status){
			Last_Move_Status = Move_Info.Move_Status;
			Move_Info.Process_Status = 0;
		}
			
		switch(Move_Info.Move_Status){
			/*十字校准模式*/
			case cross_calibration:{
				Move_Status_calibration();
				break;
			}
			/*Debug运动模式*/
			case Move_Free:{
				Move_Status_Free();
				break;
			}
			case Move_Task:{
				Move_Status_NewTask();
				//Move_Status_Task();
				break;
			}
			
		}

		osDelay(5);
	}
}

float Servo_Angle = 70.0f;
/*巡线更新进程*/
void Line_Refresh(void *argument){
	/*INIT*/
	
	/*LOOP*/
	while(1){
		Fr_Handle.Cur_Counter = Bk_Handle.Cur_Counter = Lt_Handle.Cur_Counter = Rt_Handle.Cur_Counter = 0;
		bool Fr_Have = true,Bk_Have = true,Lt_Have = true,Rt_Have = true;
		
		for(int i=0;i<8;i++){
			Fr_Line[i].Pin_State = HAL_GPIO_ReadPin(Fr_Line[i].IO_Port,Fr_Line[i].IO_Pin);
			Fr_Line[i].Triggered = Fr_Line[i].Pin_State ? true:Fr_Line[i].Triggered;
			Bk_Line[i].Pin_State = HAL_GPIO_ReadPin(Bk_Line[i].IO_Port,Bk_Line[i].IO_Pin);
			Bk_Line[i].Triggered = Bk_Line[i].Pin_State ? true:Bk_Line[i].Triggered;
			
			Fr_Handle.Cur_Counter += Fr_Line[i].Pin_State?1:0;
			Bk_Handle.Cur_Counter += Bk_Line[i].Pin_State?1:0;
			Fr_Have = Fr_Have & Fr_Line[i].Triggered;
			Bk_Have = Bk_Have & Bk_Line[i].Triggered;
			if(i<7){
				Rt_Line[i].Pin_State = HAL_GPIO_ReadPin(Rt_Line[i].IO_Port,Rt_Line[i].IO_Pin);
				Rt_Line[i].Triggered = Rt_Line[i].Pin_State ? true:Rt_Line[i].Triggered;
				Lt_Line[i].Pin_State = HAL_GPIO_ReadPin(Lt_Line[i].IO_Port,Lt_Line[i].IO_Pin);
				Lt_Line[i].Triggered = Lt_Line[i].Pin_State ? true:Lt_Line[i].Triggered;
				
				Lt_Handle.Cur_Counter += Lt_Line[i].Pin_State?1:0;
				Rt_Handle.Cur_Counter += Rt_Line[i].Pin_State?1:0;
				Lt_Have = Lt_Have & Lt_Line[i].Triggered;
				Rt_Have = Rt_Have & Rt_Line[i].Triggered;
			}
			
		}
		Fr_Handle.All_Pin_Up = Fr_Handle.Cur_Counter==8?true:false;
		Bk_Handle.All_Pin_Up = Bk_Handle.Cur_Counter==8?true:false;
		Lt_Handle.All_Pin_Up = Lt_Handle.Cur_Counter==7?true:false;
		Rt_Handle.All_Pin_Up = Rt_Handle.Cur_Counter==7?true:false;
		
		Fr_Handle.All_Pin_Down = Fr_Handle.Cur_Counter==0?true:false;
		Bk_Handle.All_Pin_Down = Bk_Handle.Cur_Counter==0?true:false;
		Lt_Handle.All_Pin_Down = Lt_Handle.Cur_Counter==0?true:false;
		Rt_Handle.All_Pin_Down = Rt_Handle.Cur_Counter==0?true:false;
		
		Fr_Handle.All_Pin_Triggered = Fr_Have;
		Bk_Handle.All_Pin_Triggered = Bk_Have;
		Lt_Handle.All_Pin_Triggered = Lt_Have;
		Rt_Handle.All_Pin_Triggered = Rt_Have;
		osDelay(5);
	}
}

/*基本机械臂运动进程*/
void Arm_Move_Base(void *argument){


	while(1){
		FSUS_SetServoAngle(&uart5,4, Arm_Ctrl_Info.Servo_4, Arm_Ctrl_Info.Speed_4, 0, 0);
		FSUS_SetServoAngle(&uart5,3, Arm_Ctrl_Info.Servo_3, Arm_Ctrl_Info.Speed_3, 0, 0);
		FSUS_SetServoAngle(&uart5,2, Arm_Ctrl_Info.Servo_2, Arm_Ctrl_Info.Speed_2, 0, 0);
		FSUS_SetServoAngle(&uart5,1, Arm_Ctrl_Info.Servo_1, Arm_Ctrl_Info.Speed_1, 0, 0);
		FSUS_SetServoAngle(&uart5,0, Arm_Ctrl_Info.Servo_0, Arm_Ctrl_Info.Speed_0, 0, 0);
		Usart_SendAll(&uart5);
		osDelay(10);
	}
	
}

/*机械臂控制进程*/
void Arm_Move_Ctrl(void *argument){
	
	while(1){
		switch(Arm_Ctrl_Info.Status){
			case Arm_Ctrl_Debug:{
				break;
			}
			case Arm_Ctrl_Init:{
					Arm_Move_Init();
				break;
			}
		}
		osDelay(5);
	}
}
