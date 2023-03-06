#include "Chassis_bsp.h"

WHEEL Wheel_Info[4];//轮子信息
MOVE Move_Info;			//移动信息
TASK Task_Info;			//任务信息
void Chassis_Info_Init(void){
	
	/*轮子正反转参数*/
	Wheel_Info[0].Dir_Inv = true;
	Wheel_Info[1].Dir_Inv = false;
	Wheel_Info[2].Dir_Inv = false;
	Wheel_Info[3].Dir_Inv = true;
	
	/*将物料盘方向设置为正*/
	Move_Info.Chassis_dir = 1;
		
	/*初始设置为debug模式*/
	Move_Info.Move_Status = Move_Free;
	
	/*串口屏显示初始化*/
	uint8_t Uart_Tx_Buffer[22]={0xff,0xff,0xff,0x74,0x31,0x2E,0x74,0x78,0x74,0x3D,0x22,0x30,0x30,0x30,0x2B,0x30,0x30,0x30,0x22,0xff,0xff,0xff};
	HAL_UART_Transmit(&huart6,Uart_Tx_Buffer,22,0xffff);
}

void Speed_Robot_Coordinate(float VX_W, float VY_W, float VZ_W){
	
/*无陀螺仪，先屏蔽掉部分运算*/
#define gyroscope_preparation false

#if gyroscope_preparation
	
	/*陀螺仪偏移角度计算*/
	double angle2cnt = Cur_GYRO.Z_Pos * Pi / 180;
	/*解算到机器人坐标系运动向量*/
	float VX = -((float)sin(angle2cnt)) * VY_W + ((float)cos(angle2cnt)) * VX_W;
	float VY = ((float)cos(angle2cnt)) * VY_W + ((float)sin(angle2cnt)) * VX_W;
	float VZ = VZ_W;
#else
	float VX=0.0f;
	float VY=0.0f;
	switch(Move_Info.Chassis_dir){
		case 0:{
			VX=VX_W;
			VY=VY_W;
			break;
		}
		case 1:{
			VX=-VX_W;
			VY=-VY_W;
			break;
		}
		case 2:{
			VX=VY_W;
			VY=-VX_W;
			break;
		}
		case 3:{
			VX=-VY_W;
			VY=VX_W;
			break;
		}
	}
	
	float VZ=VZ_W;
	
#endif

	
	/*四麦轮速度解算，右下角轮子起始，顺时针id增大*/
	Wheel_Info[0].Wheel_Spd = VY + VX - VZ;
	Wheel_Info[1].Wheel_Spd = VY - VX + VZ;
	Wheel_Info[2].Wheel_Spd = VY + VX + VZ;
	Wheel_Info[3].Wheel_Spd = VY - VX - VZ;
	
	/*驱动电机运动*/
	Wheels_Run();
}

void Wheels_Run(void){
	
	for(int8_t ID=1;ID<=4;ID++){
		int8_t id = ID-1;
		/*轮子反向*/
		if(Wheel_Info[id].Dir_Inv)
			Wheel_Info[id].Wheel_Spd = -Wheel_Info[id].Wheel_Spd;
		/*轮子CAN消息设置*/
		M2006_Set_Speed(Wheel_Info[id].Wheel_Spd,ID);
	}
	/*发送CAN报文控制M2006*/
	DJI_CAN_Send_Data(&hcan1, can_Sendbuf_3508, 0x200 ,8);
}

/*自由移动模式，基本用于debug*/
void Move_Status_Free(void){
	static int Speed_0,Speed_1,Speed_2,Speed_3;//四个轮子速度变量用于debug
	Speed_0 = M2006_Get_Speed(1);
	Speed_1 = M2006_Get_Speed(2);
	Speed_2 = M2006_Get_Speed(3);
	Speed_3 = M2006_Get_Speed(4);
	
	Speed_Robot_Coordinate(Move_Info.VX_IN,Move_Info.VY_IN,Move_Info.VZ_IN);
	
}


/*校准移动模式，使用巡线模块进行十字校准*/
void Move_Status_calibration(void){
	
	float VX,VY,VZ=0;

	if(Run_Into_End(&VX,&VY,&VZ,1)){
		Move_Info.Move_Status = Move_Free;
	}
	
	Speed_Robot_Coordinate(VX,VY,VZ);
}

/*左右旋转 + 校准*/
/*
dir 0-左旋 1-右旋
value 0-90° 1-180°
*/
bool Direction_Rotation(bool direction,bool value,float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int rotate_counter = 0;
	static int8_t minor_count = 0;
	if(function_init){
		rotate_counter = 0;
		minor_count = 0;
		function_init = false;
	}
	
	/*0-左旋	1-右旋*/
	*VZ = direction?6000:-6000;
	switch(rotate_counter){
		case 0:{
			if(direction){if(Fr_Line[7].Pin_State) rotate_counter=1;}
			else{if(Fr_Line[0].Pin_State) rotate_counter=1;}
				
			break;
		}
		case 1:{
			if(Fr_Line[3].Pin_State){
				rotate_counter=2;
				minor_count++;
			}
			break;
		}
		case 2:{
			/*旋转完成*/
			if(minor_count == (int8_t)value+1){
				/*校准*/
				if(Position_Calibration(VX,VY,VZ)){
					function_init = true;
					return true;
				}
			}
			else
				rotate_counter=0;
			break;
		}
	}
	return false;
	
}

/*正方向校准*/
bool Positive_Direction_Calibration(float *VX,float *VY,float *VZ){
	static int Calibration_Counter = 0;
	/*正方向更新*/
	Move_Info.Chassis_dir = 0;
	
	/*设定旋转偏移值为0*/
	float Fr_rotation_offset = 0.0f;
	float Bk_rotation_offset = 0.0f;
	float rotation_offset = 0;
	
	for(int i=0;i<8;i++){
		if(Fr_Line[i].Pin_State)
			Fr_rotation_offset += (float)i+(i>3?-3.0f:-4.0f);
		if(Bk_Line[i].Pin_State)
			Bk_rotation_offset += (float)i+(i>3?-3.0f:-4.0f);
	//Bk_rotation_offset += Bk_Line[i].Pin_State?i-4.0f:0;
	}
	/*如果前后都触发*/
	if(!Fr_Handle.All_Pin_Down && !Bk_Handle.All_Pin_Down){
		rotation_offset += Fr_rotation_offset/(float)Fr_Handle.Cur_Counter + Bk_rotation_offset/(float)Bk_Handle.Cur_Counter;
	}
		*VZ = rotation_offset*200;
	
	if(*VZ <= 1.0f){
		*VX = Fr_rotation_offset*300;
	}
	
	if(fabsf(*VZ) <= 1.0f && fabsf(*VY) <= 1.0f && fabsf(*VX)<= 1.0f){
		Calibration_Counter++;
		if(Calibration_Counter >= 30){
			Calibration_Counter = 0;
			Move_Info.Chassis_dir = 1;
			return true;
		}
	}
	else
		Calibration_Counter = 0;
	return false;
}

/*位置校准*/
bool Position_Calibration(float *VX,float *VY,float *VZ){
	
	static int Calibration_Counter = 0;
	/*正方向更新*/
	Move_Info.Chassis_dir = 0;
	
	/*如果四方向触发有问题，让其先旋转正常*/
	if((Lt_Handle.All_Pin_Down||Rt_Handle.All_Pin_Down) && (Fr_Handle.All_Pin_Down || Bk_Handle.All_Pin_Down))
	{
		*VZ = 6000;
		return false;
	}
	
	/*设定旋转偏移值为0*/
	float Rt_rotation_offset = 0.0f;
	float Lt_rotation_offset = 0.0f;
	float Fr_rotation_offset = 0.0f;
	float Bk_rotation_offset = 0.0f;
	float rotation_offset = 0;
	
	for(int i=0;i<7;i++){
		Lt_rotation_offset += Lt_Line[i].Pin_State?i-3.0f:0;
		Rt_rotation_offset += Rt_Line[i].Pin_State?i-3.0f:0;
	}
	for(int i=0;i<8;i++){
		if(Fr_Line[i].Pin_State)
			Fr_rotation_offset += (float)i+(i>3?-3.0f:-4.0f);
		if(Bk_Line[i].Pin_State)
			Bk_rotation_offset += (float)i+(i>3?-3.0f:-4.0f);
		//Bk_rotation_offset += Bk_Line[i].Pin_State?i-4.0f:0;
	}
		
	
	/*如果前后都触发*/
	if(!Fr_Handle.All_Pin_Down && !Bk_Handle.All_Pin_Down){
		rotation_offset += Fr_rotation_offset/(float)Fr_Handle.Cur_Counter + Bk_rotation_offset/(float)Bk_Handle.Cur_Counter;
	}
	if(*VZ <= 0.5f){
		/*如果左右都触发*/
		if(!Lt_Handle.All_Pin_Down && !Rt_Handle.All_Pin_Down){
//			rotation_offset += Lt_rotation_offset/(float)Lt_Handle.Cur_Counter + Rt_rotation_offset/(float)Rt_Handle.Cur_Counter;
		}
	}
	
	*VZ = rotation_offset*400;
	
	if(*VZ <= 1.0f){
		*VX = Fr_rotation_offset*500;
		*VY = Lt_rotation_offset*500;
	}
	
	if(fabsf(*VZ) <= 1.0f && fabsf(*VY) <= 1.0f && fabsf(*VX)<= 1.0f){
		Calibration_Counter++;
		if(Calibration_Counter >= 5){
			Calibration_Counter = 0;
			Move_Info.Chassis_dir = 1;
			return true;
		}
		
	}
//	else
//		Calibration_Counter = 0;
	return false;
}


/*巡线跑*/
bool Line_Patrol_VY(float *VX,float *VY,float *VZ,float Desired_Speed,int Counter){
	
	static bool function_init = true;
	static int walk_count = 0;	//计数走过多少格
	static int Program_Counter = 0;
	static float Desired_Speed_Last = 0;
	/*程序初始化*/
	if(function_init){
		Emptying_Triggered(Fr_Line,&Fr_Handle);
		Emptying_Triggered(Bk_Line,&Bk_Handle);
		function_init = false;
		walk_count = 0;
		Program_Counter = 0;
		Desired_Speed_Last = 0;
	}
	if(fabsf(Desired_Speed_Last)<=fabsf(Desired_Speed)-60 ){
		if(Desired_Speed > 1.0f)
			Desired_Speed_Last += 80;
		else
			Desired_Speed_Last -= 80;
		Desired_Speed = Desired_Speed_Last;
	}
	
	if(walk_count>=4){
		if(Desired_Speed > 1.0f)
			Desired_Speed = 3000;
		else
			Desired_Speed = -3000;
	}
	
	switch(Program_Counter){
		case 0:{
			*VY = Desired_Speed;
			/*正向,物料盘方向*/
			if(Desired_Speed > 1.0f){
				if(Fr_Handle.All_Pin_Triggered && Fr_Handle.Cur_Counter<=2){
					Emptying_Triggered(Fr_Line,&Fr_Handle);
					walk_count++;
				}
				*VZ = Rotating_Line_Patrol(Bk_Line,&Bk_Handle,0);
			}
			/*负向*/
			else{
				if(Bk_Handle.All_Pin_Triggered && Bk_Handle.Cur_Counter<=2){
					Emptying_Triggered(Bk_Line,&Bk_Handle);
					walk_count++;
				}
				*VZ = Rotating_Line_Patrol(Fr_Line,&Fr_Handle,0);
			}
			if(Counter == walk_count){
				Program_Counter = 1;
			}
			break;
		}
		case 1:{
			*VY = Desired_Speed;
			if(Desired_Speed > 1.0f && (Lt_Line[0].Pin_State || Rt_Line[6].Pin_State))
				Program_Counter = 2;
			if(Desired_Speed < -1.0f && (Lt_Line[6].Pin_State || Rt_Line[0].Pin_State))
				Program_Counter = 2;
			
			break;
		}
		case 2:{
			if(Position_Calibration(VX,VY,VZ)){
				
				function_init = true;
				return true;
			}
			break;
		}
	}

	return false;
}

/*返回巡线旋转轴的值*/
//dir 0-前后 1-左右
float Rotating_Line_Patrol(Line_IO Cur_Line[],Handle_IO *Cur_Handle,bool Dir){
	float VZ = 0.0f;
	/*如果压在线上暂时停止Z轴变换*/
	if(Cur_Handle->Cur_Counter > 2)
		return VZ;
	
	float Cur_rotation_offset = 0.0f;
	
	if(Dir){
		for(int i=0;i<7;i++){
			if(Cur_Line[i].Pin_State)
				Cur_rotation_offset += (float)i-3.0f;
		}
	}
	else{
		for(int i=0;i<8;i++){
			if(Cur_Line[i].Pin_State)
				Cur_rotation_offset += (float)i+(i>3?-3.0f:-4.0f);
		}
	}
	VZ = Cur_rotation_offset*300.0f;
	return VZ;
}
/*巡线跑*/
bool Line_Patrol_VX(float *VX,float *VY,float *VZ,float Desired_Speed,int Counter){
	
	static bool function_init = true;
	static int walk_count = 0;	//计数走过多少格
	static int Program_Counter = 0;
	static float Desired_Speed_Last= 0;
	/*程序初始化*/
	if(function_init){
		Emptying_Triggered(Lt_Line,&Lt_Handle);
		Emptying_Triggered(Rt_Line,&Rt_Handle);
		function_init = false;
		walk_count = 0;
		Program_Counter = 0;
		Desired_Speed_Last= 0;
	}
	if(fabsf(Desired_Speed_Last)<=fabsf(Desired_Speed)-40 ){
		if(Desired_Speed > 1.0f)
			Desired_Speed_Last += 80;
		else
			Desired_Speed_Last -= 80;
		Desired_Speed = Desired_Speed_Last;
	}
	if(walk_count>=4){
		if(Desired_Speed > 1.0f)
			Desired_Speed = 3000;
		else
			Desired_Speed = -3000;
	}
	switch(Program_Counter){
		case 0:{
			*VX = Desired_Speed;
			/*正向,物料盘右侧*/
			if(Desired_Speed > 1.0f){
				if(Rt_Handle.All_Pin_Triggered && Rt_Handle.Cur_Counter<=2){
					Emptying_Triggered(Rt_Line,&Rt_Handle);
					walk_count++;
				}
				*VZ = Rotating_Line_Patrol(Lt_Line,&Lt_Handle,1);
			}
			/*负向*/
			else{
				if(Lt_Handle.All_Pin_Triggered && Lt_Handle.Cur_Counter<=2){
					Emptying_Triggered(Lt_Line,&Lt_Handle);
					walk_count++;
				}
				*VZ = Rotating_Line_Patrol(Rt_Line,&Rt_Handle,1);
			}
			if(Counter == walk_count){
				Program_Counter = 1;
			}
			break;
		}
		case 1:{
			*VX = Desired_Speed;
			if(Desired_Speed > 1.0f && (Fr_Line[1].Pin_State || Bk_Line[6].Pin_State))
				Program_Counter = 2;
			if(Desired_Speed < -1.0f && (Bk_Line[1].Pin_State || Fr_Line[6].Pin_State))
				Program_Counter = 2;
			
			break;
		}
		case 2:{
			if(Position_Calibration(VX,VY,VZ)){
				function_init = true;
				return true;
			}
			break;
		}
	}

	return false;
}

bool Scan_Qr_Code(float *VX,float *VY,float *VZ){
		
	static bool function_init = true;
	static int Program_Counter = 0;
	if(function_init){
		function_init = false;
		Arm_Ctrl_Info.Servo_4 = 80;
		Program_Counter = 0;
		memset(Uart_Rx_Buffer,0,sizeof(Uart_Rx_Buffer));
	}
	switch(Program_Counter){
		case 0:{
			*VY = 2000;
			if(Rt_Line[3].Pin_State)
				Program_Counter = 1;
			break;
		}
		case 1:{
			if(Line_Patrol_VX(VX,VY,VZ,5500,2))
				Program_Counter = 2;
			break;
		}
		case 2:{
			if(Line_Patrol_VY(VX,VY,VZ,5500,2))
				Program_Counter = 3;
			break;
		}
		case 3:{
			HAL_UART_Receive_IT(&huart1,Uart_Rx_Buffer,9);
			/*扫码成功*/
			if(Uart_Rx_Buffer[7]==0x0d && Uart_Rx_Buffer[8]==0x0a){
				function_init = true;
				return true;
			}
			break;
		}
	}
	return false;
}

bool Visual_Recognition(float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int Program_Counter = 0;
	if(function_init){
		function_init = false;
		memset(Sence_Rx_Buffer,0,sizeof(Sence_Rx_Buffer));
		Program_Counter = 0;
	}
	switch(Program_Counter){
		case 0:{
			if(Line_Patrol_VX(VX,VY,VZ,5500,3))
				Program_Counter = 1;
			break;
		}
		case 1:{
			static int identification_count = 0;
			HAL_UART_Receive_IT(&huart4,Sence_Rx_Buffer,8);
			
			if(Sence_Rx_Buffer[0] == '?'&&Sence_Rx_Buffer[7]=='!'){
				if(identification_count++ > 2){
					identification_count = 0;
					Program_Counter = 2;
				}
				else
					memset(Sence_Rx_Buffer,0,sizeof(Sence_Rx_Buffer));
			}
			break;
		}
		case 2:{
			if(Line_Patrol_VY(VX,VY,VZ,-5500,1)){
				function_init = true;
				return true;
			}
			break;
		}
	}
	return false;

}

bool Clamp_The_Upper_Material(float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int Program_Counter = 0;
	static int Code_Counter = -1;
	if(function_init){
		function_init = false;
		Program_Counter = 0;
		Code_Counter = -1;
	}
	switch(Program_Counter){
		case 0:{
			if(Position_Calibration(VX,VY,VZ))
				Program_Counter = 1;
			break;
		}
		case 1:{
			*VY = -1500;
			if(Rt_Line[0].Pin_State||Lt_Line[6].Pin_State)
				Program_Counter = 2;
			break;
		}
		case 2:{
			if(Arm_Ctrl_Info.Status == Arm_Ctrl_Debug){
				Code_Counter++;
				if(Code_Counter == 3){
					Code_Counter = -1;
					Program_Counter = 3;
					break;
				}
			}
			for(int i=0;i<=2;i++){
				if(Task_Info.Task_Code_Fr[Code_Counter]==Task_Info.Up_Order[i])
					Arm_Order_Info.outer = i+1;
			}
			Arm_Order_Info.inner = Task_Info.Task_Code_Fr[Code_Counter];
			Arm_Ctrl_Info.Status = Arm_Ctrl_Init;
			break;
		}
		case 3:{
			*VY = 3500;
			if(Rt_Line[3].Pin_State||Lt_Line[3].Pin_State)
				Program_Counter = 4;
			break;
		}
		case 4:{
			if(Position_Calibration(VX,VY,VZ)){
				function_init = true;
				return true;
			}
			break;
		}
	}
	return false;
}


bool Run_To_Processing(float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int Program_Counter = 0;
	if(function_init){
		function_init = false;
		Program_Counter = 0;
	}
	switch(Program_Counter){
#if !Field_Inverse
		case 0:{
			if(Line_Patrol_VY(VX,VY,VZ,5500,2))
				Program_Counter = 1;
			break;
		}
		case 1:{
			if(Direction_Rotation(0,0,VX,VY,VZ)){
				function_init = true;
				return true;
			}
			break;
		}
#endif
#if Field_Inverse
		case 0:{
			if(Line_Patrol_VY(VX,VY,VZ,5500,4))
				Program_Counter = 1;
			break;
		}
		case 1:{
			if(Direction_Rotation(0,1,VX,VY,VZ)){
				Program_Counter = 2;
			}
			break;
		}
		case 2:{
			if(Line_Patrol_VX(VX,VY,VZ,5500,2)){
				function_init = true;
				return true;
			}
			break;
		}
#endif
	}
	return false;
}
bool Rough_Machining_Upper(float *VX,float *VY,float *VZ){
	
	static bool function_init = true;
	static int Program_Counter = 0;
	static int Code_Counter = -1;
	if(function_init){
		Emptying_Triggered(Fr_Line,&Fr_Handle);
		function_init = false;
		Program_Counter = 0;
		Code_Counter = -1;
	}
	switch(Program_Counter){
		case 0:{
			if(Position_Calibration(VX,VY,VZ))
				Program_Counter = 1;
			break;
		}
		case 1:{
			*VY = -1500;
			if(Fr_Handle.All_Pin_Triggered && (Lt_Line[6].Pin_State||Rt_Line[0].Pin_State)){
					Program_Counter = 2;
			}
				
			break;
		}
		case 2:{
			if(Positive_Direction_Calibration(VX,VY,VZ)){
				Program_Counter = 3;
			}
			break;
		}
		case 3:{
			if(Arm_Ctrl_Info.Status == Arm_Ctrl_Debug){
				
				if(Positive_Direction_Calibration(VX,VY,VZ)){
					Code_Counter++;
					if(Code_Counter == 3){
						Code_Counter = -1;
						Program_Counter = 4;
						break;
					}	
					Arm_Order_Info.inner = Task_Info.Task_Code_Fr[Code_Counter];
					Arm_Order_Info.outer = Task_Info.Task_Code_Fr[Code_Counter]+6;
					Arm_Ctrl_Info.Status = Arm_Ctrl_Init;				
				}
			}
			
//			Arm_Order_Info.inner = Task_Info.Task_Code_Fr[Code_Counter];
//			Arm_Order_Info.outer = Task_Info.Task_Code_Fr[Code_Counter]+6;
//			Arm_Ctrl_Info.Status = Arm_Ctrl_Init;
			
			break;
		}
		case 4:{
			if(Arm_Ctrl_Info.Status == Arm_Ctrl_Debug){
				Code_Counter++;
				if(Code_Counter == 3){
					Code_Counter = -1;
					Program_Counter = 5;
					break;
				}
			}
			Arm_Order_Info.inner = Task_Info.Task_Code_Fr[Code_Counter];
			Arm_Order_Info.outer = Task_Info.Task_Code_Fr[Code_Counter]+9;
			Arm_Ctrl_Info.Status = Arm_Ctrl_Init;
			break;
		}
		case 5:{
			*VY = 3500;
			if(Rt_Line[3].Pin_State||Lt_Line[3].Pin_State)
				Program_Counter = 6;
			break;
		}
		case 6:{
			if(Position_Calibration(VX,VY,VZ)){
				function_init = true;
				return true;
			}
			break;
		}
	}
	return false;
}
bool Rough_Machining_Lower(float *VX,float *VY,float *VZ){
	
	static bool function_init = true;
	static int Program_Counter = 0;
	static int Code_Counter = -1;
	if(function_init){
		Emptying_Triggered(Fr_Line,&Fr_Handle);
		function_init = false;
		Program_Counter = 0;
		Code_Counter = -1;
	}
	switch(Program_Counter){
		case 0:{
			if(Position_Calibration(VX,VY,VZ))
				Program_Counter = 1;
			break;
		}
		case 1:{
			*VY = -1500;
			if(Fr_Handle.All_Pin_Triggered && (Lt_Line[6].Pin_State||Rt_Line[0].Pin_State)){
					Program_Counter = 2;
			}
			break;
		}
		case 2:{
			if(Positive_Direction_Calibration(VX,VY,VZ)){
				Program_Counter = 3;
			}
			break;
		}
		case 3:{
			if(Arm_Ctrl_Info.Status == Arm_Ctrl_Debug){
				
				if(Positive_Direction_Calibration(VX,VY,VZ)){
					Code_Counter++;
					if(Code_Counter == 3){
						Code_Counter = -1;
						Program_Counter = 4;
						break;
					}	
					Arm_Order_Info.inner = Task_Info.Task_Code_Se[Code_Counter];
					Arm_Order_Info.outer = Task_Info.Task_Code_Se[Code_Counter]+6;
					Arm_Ctrl_Info.Status = Arm_Ctrl_Init;				
				}
			}
			break;
		}
		case 4:{
			if(Arm_Ctrl_Info.Status == Arm_Ctrl_Debug){
				Code_Counter++;
				if(Code_Counter == 3){
					Code_Counter = -1;
					Program_Counter = 5;
					break;
				}
			}
			Arm_Order_Info.inner = Task_Info.Task_Code_Se[Code_Counter];
			Arm_Order_Info.outer = Task_Info.Task_Code_Se[Code_Counter]+9;
			Arm_Ctrl_Info.Status = Arm_Ctrl_Init;
			break;
		}
		case 5:{
			*VY = 3500;
			if(Rt_Line[3].Pin_State||Lt_Line[3].Pin_State)
				Program_Counter = 6;
			break;
		}
		case 6:{
			if(Position_Calibration(VX,VY,VZ)){
				function_init = true;
				return true;
			}
			break;
		}
	}
	return false;
}


bool Product_Area_Upper(float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int Program_Counter = 0;
	static int Code_Counter = -1;
	if(function_init){
		Emptying_Triggered(Fr_Line,&Fr_Handle);
		function_init = false;
		Program_Counter = 0;
		Code_Counter = -1;
	}
	switch(Program_Counter){
		case 0:{
			if(Position_Calibration(VX,VY,VZ))
				Program_Counter = 1;
			break;
		}
		case 1:{
			*VY = -1500;
			if(Lt_Line[0].Pin_State||Rt_Line[6].Pin_State)
				Program_Counter = 2;
			break;
		}
		case 2:{
			if(Positive_Direction_Calibration(VX,VY,VZ)){
				Program_Counter = 3;
			}
			break;
		}
		case 3:{
			if(Arm_Ctrl_Info.Status == Arm_Ctrl_Debug){
				
				if(Positive_Direction_Calibration(VX,VY,VZ)){
					Code_Counter++;
					if(Code_Counter == 3){
						Code_Counter = -1;
						Program_Counter = 4;
						break;
					}	
				Arm_Order_Info.inner = Task_Info.Task_Code_Fr[Code_Counter];
				Arm_Order_Info.outer = Task_Info.Task_Code_Fr[Code_Counter]+12;
					Arm_Ctrl_Info.Status = Arm_Ctrl_Init;				
				}
			}
			break;
		}
		case 4:{
			*VY = 3500;
			if(Rt_Line[3].Pin_State||Lt_Line[3].Pin_State)
				Program_Counter = 5;
			break;
		}
		case 5:{
			if(Position_Calibration(VX,VY,VZ)){
				function_init = true;
				return true;
			}
			break;
		}
	}
	return false;
}
bool Product_Area_Lower(float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int Program_Counter = 0;
	static int Code_Counter = -1;
	if(function_init){
		Emptying_Triggered(Fr_Line,&Fr_Handle);
		function_init = false;
		Program_Counter = 0;
		Code_Counter = -1;
	}
	switch(Program_Counter){
		case 0:{
			if(Position_Calibration(VX,VY,VZ))
				Program_Counter = 1;
			break;
		}
		case 1:{
			*VY = -1500;
			if(Lt_Line[0].Pin_State||Rt_Line[6].Pin_State)
				Program_Counter = 2;
			break;
		}
		case 2:{
			if(Positive_Direction_Calibration(VX,VY,VZ)){
				Program_Counter = 3;
			}
			break;
		}
		case 3:{
			if(Arm_Ctrl_Info.Status == Arm_Ctrl_Debug){
				
				if(Positive_Direction_Calibration(VX,VY,VZ)){
					Code_Counter++;
					if(Code_Counter == 3){
						Code_Counter = -1;
						Program_Counter = 4;
						break;
					}	
					Arm_Order_Info.inner = Task_Info.Task_Code_Se[Code_Counter];
					Arm_Order_Info.outer = Task_Info.Task_Code_Se[Code_Counter]+15;
					Arm_Ctrl_Info.Status = Arm_Ctrl_Init;				
				}
			}
			break;
		}
		case 4:{
			*VY = 3500;
			if(Rt_Line[3].Pin_State||Lt_Line[3].Pin_State)
				Program_Counter = 5;
			break;
		}
		case 5:{
			if(Position_Calibration(VX,VY,VZ)){
				function_init = true;
				return true;
			}
			break;
		}
	}
	return false;
}
bool Run_To_Product_Area(float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int Program_Counter = 0;
	if(function_init){
		function_init = false;
		Program_Counter = 0;
	}
	switch(Program_Counter){
#if !Field_Inverse
		case 0:{
			if(Line_Patrol_VY(VX,VY,VZ,5500,2))
				Program_Counter = 1;
			break;
		}
		case 1:{
			if(Direction_Rotation(0,0,VX,VY,VZ)){
				Program_Counter = 2;
			}
			break;
		}
		case 2:{
			if(Line_Patrol_VY(VX,VY,VZ,-5500,2)){
				function_init = true;
				return true;
			}
			break;
		}
#endif
		
#if Field_Inverse
		case 0:{
			if(Line_Patrol_VY(VX,VY,VZ,5500,2))
				Program_Counter = 1;
			break;
		}
		case 1:{
			if(Direction_Rotation(1,0,VX,VY,VZ)){
				Program_Counter = 2;
			}
			break;
		}
		case 2:{
			if(Line_Patrol_VY(VX,VY,VZ,-5500,2)){
				function_init = true;
				return true;
			}
			break;
		}
#endif
	}
	return false;
}
bool Run_To_stock(float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int Program_Counter = 0;
	if(function_init){
		function_init = false;
		Program_Counter = 0;
	}
	switch(Program_Counter){
#if !Field_Inverse
		case 0:{
			if(Line_Patrol_VY(VX,VY,VZ,6500,4))
				Program_Counter = 1;
			break;
		}
		case 1:{
			if(Direction_Rotation(0,1,VX,VY,VZ)){
				Program_Counter = 2;
			}
			break;
		}
		case 2:{
			if(Line_Patrol_VX(VX,VY,VZ,6500,2)){
				function_init = true;
				return true;
			}
			break;
		}
#endif		

#if Field_Inverse
		case 0:{
			if(Direction_Rotation(1,0,VX,VY,VZ)){
				Program_Counter = 1;
			}
			break;
		}
		case 1:{
			if(Line_Patrol_VY(VX,VY,VZ,-6500,2)){
				function_init = true;
				return true;
			}
			break;
		}
#endif
	}
	return false;
}

bool Run_To_End(float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int Program_Counter = 0;
	if(function_init){
		function_init = false;
		Program_Counter = 0;
	}
	switch(Program_Counter){
#if !Field_Inverse
		case 0:{
			if(Line_Patrol_VX(VX,VY,VZ,6500,3)){
				function_init = true;
				return true;
			}
			break;
		}
#endif
#if Field_Inverse
		case 0:{
			if(Line_Patrol_VY(VX,VY,VZ,6500,5)){
				Program_Counter = 1;
			}	
			break;
		}
		case 1:{
			if(Line_Patrol_VX(VX,VY,VZ,6500,2)){
				function_init = true;
				return true;
			}	
		}
#endif
	}
	return false;
}

/*Dir 0-从右进 1-从上进*/
bool Run_Into_End(float *VX,float *VY,float *VZ,bool Dir){
	static bool function_init = true;
	static int Program_Counter = 0;
	if(function_init){
		Emptying_Triggered(Lt_Line,&Lt_Handle);
		Emptying_Triggered(Rt_Line,&Rt_Handle);
		Emptying_Triggered(Fr_Line,&Fr_Handle);
		function_init = false;
		Program_Counter = 0;
	}
	switch(Program_Counter){
#if !Field_Inverse
		case 0:{
			*VX = Dir?-2500:2500;
			if(Lt_Handle.All_Pin_Triggered || Rt_Handle.All_Pin_Triggered)
				Program_Counter = 1;
			break;
		}
		case 1:{
			*VY = -2500;
			if(Fr_Handle.All_Pin_Up && Bk_Handle.All_Pin_Up){
				function_init = true;
				return true;
			}
			break;
		}
#endif
#if Field_Inverse
		case 0:{
			*VY = 3000;
			if(Fr_Handle.All_Pin_Triggered)
				Program_Counter = 1;
			break;
		}
		case 1:{
			*VX = 3000;
			if(Lt_Handle.All_Pin_Up && Rt_Handle.All_Pin_Up){
				function_init = true;
				return true;
			}
			break;
		}
#endif
	}
	return false;
}

bool Clamp_The_lower_Material(float *VX,float *VY,float *VZ){
	static bool function_init = true;
	static int Program_Counter = 0;
	static int Code_Counter = -1;
	if(function_init){
		Emptying_Triggered(Fr_Line,&Fr_Handle);
		function_init = false;
		Program_Counter = 0;
		Code_Counter = -1;
	}
	switch(Program_Counter){
		case 0:{
			if(Position_Calibration(VX,VY,VZ))
				Program_Counter = 1;
			break;
		}
		case 1:{
			*VY = -1500;
			if(Fr_Handle.All_Pin_Triggered && !Fr_Line[0].Pin_State)
				Program_Counter = 2;
			break;
		}
		case 2:{
			if(Arm_Ctrl_Info.Status == Arm_Ctrl_Debug){
				
				if(Positive_Direction_Calibration(VX,VY,VZ)){
					Code_Counter++;
					if(Code_Counter == 3){
						Code_Counter = -1;
						Program_Counter = 3;
						break;
					}	
					for(int i=0;i<=2;i++){
						if(Task_Info.Task_Code_Se[Code_Counter]==Task_Info.Down_Order[i])
							Arm_Order_Info.outer = i+4;
					}
					Arm_Order_Info.inner = Task_Info.Task_Code_Se[Code_Counter];
					Arm_Ctrl_Info.Status = Arm_Ctrl_Init;			
				}
			}
			break;
		}
		case 3:{
			*VY = 3500;
			if(Rt_Line[3].Pin_State||Lt_Line[3].Pin_State)
				Program_Counter = 4;
			break;
		}
		case 4:{
			if(Position_Calibration(VX,VY,VZ)){
				function_init = true;
				return true;
			}
			break;
		}
	}
	return false;
}
void Move_Status_NewTask(void){
	float VX=0.0f,VY=0.0f,VZ=0.0f;
	switch(Move_Info.Process_Status){
		/*出发到扫二维码*/
		case 0:{
			if(Scan_Qr_Code(&VX,&VY,&VZ))
				Move_Info.Process_Status = 1;
			break;
		}
		case 1:{
			if(Visual_Recognition(&VX,&VY,&VZ))
				Move_Info.Process_Status = 2;
			break;
		}
		case 2:{
			if(Clamp_The_Upper_Material(&VX,&VY,&VZ))
				Move_Info.Process_Status = 3;
			break;
		}
		case 3:{
			if(Run_To_Processing(&VX,&VY,&VZ))
				Move_Info.Process_Status = 4;
			break;
		}
		case 4:{
			if(Rough_Machining_Upper(&VX,&VY,&VZ))
				Move_Info.Process_Status = 5;
			break;
		}
		case 5:{
			if(Run_To_Product_Area(&VX,&VY,&VZ))
				Move_Info.Process_Status = 6;
			break;
		}
		case 6:{
			if(Product_Area_Upper(&VX,&VY,&VZ))
				Move_Info.Process_Status = 7;
			break;
		}
		case 7:{
			if(Run_To_stock(&VX,&VY,&VZ))
				Move_Info.Process_Status = 8;
			break;
		}
		case 8:{
			if(Clamp_The_lower_Material(&VX,&VY,&VZ))
				Move_Info.Process_Status = 9;
			break;
		}
		case 9:{
			if(Run_To_Processing(&VX,&VY,&VZ))
				Move_Info.Process_Status = 10;
			break;
		}
		case 10:{
			if(Rough_Machining_Lower(&VX,&VY,&VZ)){
				Move_Info.Process_Status = 11;
			}
			break;
		}
		case 11:{
			if(Run_To_Product_Area(&VX,&VY,&VZ))
				Move_Info.Process_Status = 12;
			break;
		}
		case 12:{
			if(Product_Area_Lower(&VX,&VY,&VZ))
				Move_Info.Process_Status = 13;
			break;
		}
		case 13:{
			if(Run_To_End(&VX,&VY,&VZ))
				Move_Info.Process_Status = 14;
			break;
		}
		case 14:{
			if(Run_Into_End(&VX,&VY,&VZ,0))
				Move_Info.Process_Status = 15;
			break;
		}
		case 15:{
			Move_Info.Move_Status = Move_Free;
			break;
		}
	}
	Speed_Robot_Coordinate(VX,VY,VZ);
	return;
}

