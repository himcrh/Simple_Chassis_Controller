#ifndef __ARM_MOVE_BSP__
#define __ARM_MOVE_BSP__

/*From Cubemx*/
#include "cmsis_os2.h"

typedef struct Arm_Ctrl{
	
	float Servo_0;
	float Servo_1;
	float Servo_2;
	float Servo_3;
	float Servo_4;
	uint16_t Speed_0;
  uint16_t Speed_1;
	uint16_t Speed_2;
	uint16_t Speed_3;
	uint16_t Speed_4;
	uint16_t Speed_all;
	int Status;
}Arm_Ctrl;

typedef enum mechanical_arm_motion_mode{
	Arm_Ctrl_Debug,
	Arm_Ctrl_Init,
}mechanical_arm_motion_mode;



void Arm_Move_Init(void);

typedef struct Arm_Order{
	
  int inner;
	int outer;
}Arm_Order;

typedef struct Arm_Locus{
  
	float x;
	float y;
	float z;
	//调参设置变量
	float x_0;
	float y_0;
	float z_0;
	float x_1;
	float y_1;
	float z_1;
	float x_2;
	float y_2;
	float z_2;
	float x_3;
	float y_3;
	float z_3;
}Arm_Locus;

typedef struct Arm_Theta{
	
  float theta_0;
  float theta_1;
  float theta_2;
  float theta_3;
  float theta_4;
}Arm_Theta;

extern Arm_Ctrl Arm_Ctrl_Info;
extern Arm_Order Arm_Order_Info;

void Arm_Move_Path(int inner,int outer);

void Arm_Move_Inner(int inner_1,int outer_1);
	
void Arm_Move_Outer(int outer_2);

void Inverse_Kinematic(float x_0, float y_0, float z_0);

void Arm_Move_Back(void);

void Arm_Move_Front(void);

void Arm_Move_Reversal(void);

void Arm_Move_Transition_F(float x_tf,float y_tf,float z_tf);

void Arm_Move_Transition_B(float x_tb,float y_tb,float z_tb);

void Arm_Invesal(uint16_t Speed);

void Arm_xyz(float x,float y,float z);

#endif
