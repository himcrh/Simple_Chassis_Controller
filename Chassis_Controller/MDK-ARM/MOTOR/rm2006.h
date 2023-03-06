/*********************************************************************************
*@�������ܼ�飺
*@	M2006_Set_I��		M2006��������
*@	M2006_Set_Speed��	M2006�ٶȱջ���ͨ��������Ϊ���
*@	M2006_Set_Pos��		M2006λ�ñջ���ͨ���ٶ���Ϊ���������PID
*@
*@	M2006_Get_Feedback��M2006���ݷ�����ȡ��CAN�ж���ʹ��
*@	M_2006_Get_Torque��	M2006ת����Ϣ��ȡ
*@	M2006_Get_Speed��	�ٶ���Ϣ��ȡ
*@	M2006_Get_Pos��		λ����Ϣ��ȡ
*@	pos_rec��			λ����Ϣ���£�CAN�ж���ʹ�ã����㹻�ߵĲ����ʽ���߽�����
*@
*@	Ang2Cnt��			�Ƕ�������ת��Ϊ��������Cntֵ
*@	Cnt2Ang��			������Cntֵת��Ϊʵ��ת���ĽǶ�
*@
*@	M2006��3508��ͨ�ģ�����ֻ�ǵ�����ٱȲ�ͬ�Լ�3508����һ������¶ȷ���
*@	��˸��ļ��޸ĵ���ļ��ٱȺ����ֱ������3508�Ŀ���
*@  
*@ @file	:	rm2006.c
*@ @author	:	SHJ
*@ @version	:	v2.1���޸���ͬʱ���ƶ����������ݱ���д��Bug��ÿ���������һ�׶�����PID
*@				����ʱ���Զ�ÿ���������Եĵ�������λ�ñջ�����ڲ�����Ϊcntֵ
*@ @date	:	2019-1-30
*********************************************************************************/


#ifndef __RM2006_H
#define __RM2006_H	 
#include "can.h"
#include "string.h"


#define CNT_PER_ROUND			(8192)								//�������߳�
#define CNT_PER_ROUND_OUT(x)	(CNT_PER_ROUND*RM_3508Reduction_Ratio[(x-1)])
#define CLAMP(x, lower, upper)	(x >= upper ? upper : (x <= lower ? lower : x))

typedef struct
{
	int Err;
	int Err_1;
	int Err_sum;
	
	float Kp;
	float Ki;
	float Kd;
	
	int Max;
	int Min;

	float P_Out;
	float I_Out;
	float D_Out;
	float PID_Out;
}M2006_PID;

//extern CAN_HandleTypeDef   CAN1_Handler;   //CAN1���
extern uint8_t M2006_Feedback_Buf[8][6];      //�����������
extern int M2006_Pos[8];
extern uint8_t can_Sendbuf_3508[8];

extern int d;

uint8_t		M2006_Set_I(int target_i,uint8_t motor_id);
void	    M2006_Set_Speed(int goal_speed,int ID);
void	    M2006_Set_Pos(int goal_cnt,int ID);

void	M2006_Get_Feedback(uint32_t std_id,uint8_t* data_p);
int		M_2006_Get_Torque(uint8_t motor_id);
int		M2006_Get_Speed(uint8_t motor_id);
int		M2006_Get_Pos(uint8_t motor_id);
void	pos_rec(uint8_t motor_id);									//CAN�ж���ʵʱ����λ����Ϣ
//void CAN_SendData(CAN_HandleTypeDef *hcant, uint8_t *pData);
int		DJI_Ang2Cnt(float angle,int ID);
double	DJI_Cnt2Ang(int32_t cnt,int ID);
uint8_t DJI_CAN_Send_Data(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID, uint16_t Len);
//uint8_t CAN1_Send_Msg(CAN_HandleTypeDef *CAN_Handle,uint8_t* msg,uint8_t len,int id);


#endif

