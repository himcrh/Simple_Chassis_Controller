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

/*������Ϣ����*/
typedef struct WHEEL{
	
	float Wheel_Spd;
	bool Dir_Inv;			//�Ƿ�ת���ӷ���(��ǰתΪ��)
	float Wheel_Radius;		//���Ӱ뾶(mm)

}WHEEL;

/*����Ҫ������*/
typedef struct TASK{
	/*��һ��������*/
	uint8_t Task_Code_Fr[3];
	/*�ڶ���������*/
	uint8_t Task_Code_Se[3];
	
	uint8_t Up_Order[3];
	uint8_t Down_Order[3];
}TASK;

/*�˶�ģʽ*/
typedef struct MOVE{
	int Move_Status;
	int VX_IN,VY_IN,VZ_IN;
	int8_t Chassis_dir;			//���������� 0-�������Ϊ�� 1-�����̷���Ϊ�� 2-����Ҳ�Ϊ�� 3-������Ϊ��
	int Process_Status;			//����״̬��
}MOVE;

typedef enum{
	cross_calibration,	//ʮ�ַ���У׼ģʽ
	Move_Free,					//�����ƶ�ģʽ
	Move_Task						//�����ƶ�ģʽ
}MOVE_MODE;

extern WHEEL Wheel_Info[4];//������Ϣ
extern MOVE Move_Info;		 //�ƶ���Ϣ
extern TASK Task_Info;		 //������Ϣ

/*�ⲿ���ú���*/
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



/*�ܵ����Ʒ��*/
bool Run_To_Product_Area(float *VX,float *VY,float *VZ);
/*�ܵ��ּӹ���*/
bool Run_To_Processing(float *VX,float *VY,float *VZ);
/*�ܵ������*/
bool Run_To_stock(float *VX,float *VY,float *VZ);
/*�ܵ��յ�*/
bool Run_To_End(float *VX,float *VY,float *VZ);
bool Run_Into_End(float *VX,float *VY,float *VZ,bool Dir);

/*�˶�ģʽ*/
void Move_Status_Free(void);				//�����ƶ�ģʽ����������debug
void Move_Status_calibration(void);	//У׼�ƶ�ģʽ��ʹ��Ѳ��ģ�����ʮ��У׼
void Move_Status_NewTask(void);

/*�ڲ���������*/
void Wheels_Run(void);
void Speed_Robot_Coordinate(float VX_W, float VY_W, float VZ_W);

#endif
