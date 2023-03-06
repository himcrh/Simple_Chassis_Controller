#ifndef __CHASSIS_BSP__
#define __CHASSIS_BSP__

/*From cubemx*/
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "math.h"

/*From mine*/
#include "rm2006.h"
#include "Line_bsp.h"
#include "my_task.h"
#include "Arm_Move_bsp.h"

#define Field_Inverse false

/*轮子信息类型*/
typedef struct WHEEL{
	
	float Wheel_Spd;
	bool Dir_Inv;			//是否反转轮子方向(向前转为正)
	float Wheel_Radius;		//轮子半径(mm)

}WHEEL;

/*任务要求类型*/
typedef struct TASK{
	/*第一轮任务码*/
	uint8_t Task_Code_Fr[3];
	/*第二轮任务码*/
	uint8_t Task_Code_Se[3];
	
	uint8_t Up_Order[3];
	uint8_t Down_Order[3];
}TASK;

/*运动模式*/
typedef struct MOVE{
	int Move_Status;
	int VX_IN,VY_IN,VZ_IN;
	int8_t Chassis_dir;			//底盘正方向 0-舵机方向为正 1-物料盘方向为正 2-舵机右侧为正 3-舵机左侧为正
	int Process_Status;			//程序状态机
}MOVE;

typedef enum{
	cross_calibration,	//十字方向校准模式
	Move_Free,					//自由移动模式
	Move_Task						//任务移动模式
}MOVE_MODE;

extern WHEEL Wheel_Info[4];//轮子信息
extern MOVE Move_Info;		 //移动信息
extern TASK Task_Info;		 //任务信息

/*外部调用函数*/
void Chassis_Info_Init(void);
bool Positive_Direction_Calibration(float *VX,float *VY,float *VZ);
bool Position_Calibration(float *VX,float *VY,float *VZ);
bool Direction_Rotation(bool direction,bool value,float *VX,float *VY,float *VZ);
bool Line_Patrol_VY(float *VX,float *VY,float *VZ,float Desired_Speed,int Counter);
bool Line_Patrol_VX(float *VX,float *VY,float *VZ,float Desired_Speed,int Counter);
float Rotating_Line_Patrol(Line_IO Cur_Line[],Handle_IO *Cur_Handle,bool Dir);
bool Scan_Qr_Code(float *VX,float *VY,float *VZ);
bool Visual_Recognition(float *VX,float *VY,float *VZ);

bool Clamp_The_Upper_Material(float *VX,float *VY,float *VZ);
bool Clamp_The_lower_Material(float *VX,float *VY,float *VZ);
bool Rough_Machining_Upper(float *VX,float *VY,float *VZ);
bool Rough_Machining_Lower(float *VX,float *VY,float *VZ);
bool Product_Area_Upper(float *VX,float *VY,float *VZ);
bool Product_Area_Lower(float *VX,float *VY,float *VZ);



/*跑到半成品区*/
bool Run_To_Product_Area(float *VX,float *VY,float *VZ);
/*跑到粗加工区*/
bool Run_To_Processing(float *VX,float *VY,float *VZ);
/*跑到库存区*/
bool Run_To_stock(float *VX,float *VY,float *VZ);
/*跑到终点*/
bool Run_To_End(float *VX,float *VY,float *VZ);
bool Run_Into_End(float *VX,float *VY,float *VZ,bool Dir);

/*运动模式*/
void Move_Status_Free(void);				//自由移动模式，基本用于debug
void Move_Status_calibration(void);	//校准移动模式，使用巡线模块进行十字校准
void Move_Status_NewTask(void);

/*内部函数声明*/
void Wheels_Run(void);
void Speed_Robot_Coordinate(float VX_W, float VY_W, float VZ_W);

#endif
