#include "rm2006.h"
//����ʱ����5~10ms
const uint8_t RM_3508Reduction_Ratio[8] = {36,36,36,36,36,36,36,36};//������ٱ�����
/*
********************************************************************************
M2006 ����������ĸ�ʽ��
DATA[0]:ת�ӻ�е�Ƕȸ�8λ
DATA[1]:ת�ӻ�е�Ƕȵ�8λ
DATA[2]:ת��ת�ٸ�8λ
DATA[3]:ת��ת�ٵ�8λ
DATA[4]:ʵ�����ת�ظ�8λ
DATA[5]:ʵ�����ת�ص�8λ
DATA[6]:Null
DATA[6]:Null

����Ƶ�ʣ�1KHz(CAN�ж�Ƶ��)
��ʶ����0x200+���ID
********************************************************************************
*/

//���ڴ洢���������ȫ�ֱ���
uint8_t M2006_Feedback_Buf[8][6];		//�������ֵ(ȫ�ֱ���)
int M2006_Pos[8];					//ÿһ��Ԫ�ض�Ӧһ��ID�ĵ������Ϣ

/*
********************************************************************************
M2006 ������ձ��ĸ�ʽ��
TxMessage.StdId = 0x200
���ID:1
DATA[0]:���Ƶ�����8λ
DATA[1]:���Ƶ�����8λ
���ID:2
DATA[2]:���Ƶ�����8λ
DATA[3]:���Ƶ�����8λ
���ID:3
DATA[4]:���Ƶ�����8λ
DATA[5]:���Ƶ�����8λ
���ID:4
DATA[6]:���Ƶ�����8λ
DATA[7]:���Ƶ�����8λ

TxMessage.StdId = 0x1FF
���ID:5
DATA[0]:���Ƶ�����8λ
DATA[1]:���Ƶ�����8λ
���ID:6
DATA[2]:���Ƶ�����8λ
DATA[3]:���Ƶ�����8λ
���ID:7
DATA[4]:���Ƶ�����8λ
DATA[5]:���Ƶ�����8λ
���ID:8
DATA[6]:���Ƶ�����8λ
DATA[7]:���Ƶ�����8λ

����ת�ص���ֵ��Χ��-10000~10000����Ӧ����������-10~10A
********************************************************************************
*/

//PID������ʼ��	(ÿ�����һ�ײ���)
static M2006_PID M2006_Speed_Pid[8] =
{
	{.Kp = 8,.Ki = 0.1,.Kd = 1.5,.Max = 15000,.Min = -15000},//ID = 1
	{.Kp = 8,.Ki = 0.1,.Kd = 1.5,.Max = 15000,.Min = -15000},//ID = 2
	{.Kp = 10.5,.Ki = 0.05,.Kd = 0.5,.Max = 15000,.Min = -15000},//ID = 3
	{.Kp = 8,.Ki = 0.1,.Kd = 1.5,.Max = 15000,.Min = -15000},//ID = 4
	{.Kp = 15,.Ki = 0.6275,.Kd = 2.5,.Max = 10000,.Min = -10000},//ID = 5
	{.Kp = 30,.Ki = 0.0,.Kd = 10,.Max = 10000,.Min = -10000},//ID = 6
	{.Kp = 6.5,.Ki = 0,.Kd = 1.5,.Max = 10000,.Min = -10000},//ID = 7
	{.Kp = 6.5,.Ki = 0,.Kd = 1.5,.Max = 10000,.Min = -10000},//ID = 8
};
//1-4��3508�Ĳ�����5-8��2006�Ĳ���
static M2006_PID M2006_Pos_Pid[8] = 
{
	{.Kp = 0.07,.Ki = 0.00001,.Kd = 0.2,.Max = 260,.Min = -14000},	//ID = 1
	{.Kp = 0.05,.Ki = 0.001,.Kd = 0.0,.Max = 14000,.Min = -14000},	//ID = 2
	{.Kp = 0.0075,.Ki = 0.00001,.Kd = 0.005,.Max = 260,.Min = -260},	//ID = 3
	{.Kp = 0.0075,.Ki = 0.00001,.Kd = 0.005,.Max = 260,.Min = -260},	//ID = 4
	{.Kp = 0.0075,.Ki = 0.00001,.Kd = 0.005,.Max = 260,.Min = -260},	//ID = 5
	{.Kp = 0.05,.Ki = 0.001,.Kd = 0.0,.Max = 14000,.Min = -14000},	//ID = 6
	{.Kp = 0.07,.Ki = 0,.Kd = 0.2,.Max = 14000,.Min = -14000},	//ID = 7
	{.Kp = 0.07,.Ki = 0,.Kd = 0.2,.Max = 14000,.Min = -14000},	//ID = 8
};


/*********************************************************************************
  *@  name      : M2006_Set_I
  *@  function  : M2006�����������
  *@  input     : Ŀ����������id
  *@  output    : �ɹ�����0��ʧ�ܷ���1
*********************************************************************************/
uint8_t can_Sendbuf_3508[8];
uint8_t M2006_Set_I(int target_i,uint8_t motor_id)
{
	if( motor_id>=1 && motor_id<=8 ) 
	{
		int send_id = 0;
		send_id = send_id;
		
		if( target_i<=-15000 )
			target_i=-15000;
		else if( target_i>=15000 )
			target_i=15000;

		if(motor_id<=4)
			send_id=0x200;  
		else 
		{
			send_id=0x1ff;
			motor_id -=4;
		}

		can_Sendbuf_3508[2*motor_id-2]=target_i>>8;                  //����ֵ��8λ
		can_Sendbuf_3508[2*motor_id-1]=target_i & 0x00ff;            //����ֵ��8λ

		//CAN_SendData(&hcan,can_Sendbuf_3508);
//		DJI_CAN_Send_Data(&hcan1, can_Sendbuf_3508, 0x200 ,8);
		return 0;
	}
	else 
	return 1;
}


/********************************************************************************
  *@  name      : M2006_Set_Speed
  *@  function  : rm3508�ٶ�����
  *@  input     : Ŀ���ٶȣ�-15000~15000���ɽ��ܣ������id
  *@  output    : ��
********************************************************************************/
void M2006_Set_Speed(int goal_speed,int ID)
{
	uint8_t id = ID-1;
	M2006_Speed_Pid[id].Err_1 = M2006_Speed_Pid[id].Err;
	M2006_Speed_Pid[id].Err 	= goal_speed - M2006_Get_Speed(ID);
	M2006_Speed_Pid[id].Err_sum += M2006_Speed_Pid[id].Err; 
	M2006_Speed_Pid[id].Err_sum = CLAMP(M2006_Speed_Pid[id].Err_sum,-1000,1000);
	
	M2006_Speed_Pid[id].P_Out = M2006_Speed_Pid[id].Kp * M2006_Speed_Pid[id].Err;
	M2006_Speed_Pid[id].I_Out = M2006_Speed_Pid[id].Ki * M2006_Speed_Pid[id].Err_sum;
	M2006_Speed_Pid[id].D_Out = M2006_Speed_Pid[id].Kd * (M2006_Speed_Pid[id].Err - M2006_Speed_Pid[id].Err_1); 
	
	M2006_Speed_Pid[id].PID_Out = M2006_Speed_Pid[id].P_Out + M2006_Speed_Pid[id].I_Out + M2006_Speed_Pid[id].D_Out;
	M2006_Speed_Pid[id].PID_Out = CLAMP(M2006_Speed_Pid[id].PID_Out,M2006_Speed_Pid[id].Min,M2006_Speed_Pid[id].Max);

	M2006_Set_I(M2006_Speed_Pid[id].PID_Out,ID);	
}


/********************************************************************************
  *@  name      : M2006_Set_Pos
  *@  function  : rm3508λ������
  *@  input     : Ŀ��Ƕȣ�����Ƕȣ������id
  *@  output    : ��
********************************************************************************/
void M2006_Set_Pos(int goal_cnt,int ID)
{
	uint8_t id = ID-1;
	M2006_Pos_Pid[id].Err_1 	= M2006_Pos_Pid[id].Err;
	M2006_Pos_Pid[id].Err 	= goal_cnt - M2006_Get_Pos(ID);
	
	M2006_Pos_Pid[id].Err_sum += M2006_Pos_Pid[id].Err; 
	M2006_Pos_Pid[id].Err_sum = CLAMP(M2006_Pos_Pid[id].Err_sum,-20000,20000);
	
	M2006_Pos_Pid[id].P_Out = M2006_Pos_Pid[id].Kp * M2006_Pos_Pid[id].Err;
	M2006_Pos_Pid[id].I_Out = M2006_Pos_Pid[id].Ki * M2006_Pos_Pid[id].Err_sum;
	M2006_Pos_Pid[id].D_Out = M2006_Pos_Pid[id].Kd * (M2006_Pos_Pid[id].Err - M2006_Pos_Pid[id].Err_1); 
	
	M2006_Pos_Pid[id].PID_Out = M2006_Pos_Pid[id].P_Out + M2006_Pos_Pid[id].I_Out + M2006_Pos_Pid[id].D_Out;
	M2006_Pos_Pid[id].PID_Out = CLAMP(M2006_Pos_Pid[id].PID_Out,M2006_Pos_Pid[id].Min,M2006_Pos_Pid[id].Max );

	if(M2006_Pos_Pid[id].PID_Out<1&&M2006_Pos_Pid[id].PID_Out>-1)
		M2006_Pos_Pid[id].PID_Out = 0;
	
//	if(Pos_Pid[id].PID_Out == 0)
//		Pos_Pid[id].Kp = 1;
//	else 
//		Pos_Pid[id].Kp = 0.0075;
	
	M2006_Set_Speed(M2006_Pos_Pid[id].PID_Out,ID);	

}
/*********************************************************************************
  *@  name      : M2006_Get_Feedback
  *@  function  : ��ȡM2006����ķ���������ȫ�ֱ���M2006_Feedback_Buf[8][6];
  *@  input     : message_id,message����ָ��
  *@  output    : 
*********************************************************************************/
void M2006_Get_Feedback(uint32_t std_id,uint8_t* data_p)
{
	int i;
	for(i=1;i<9;i++)        //ǰ�ĵ��ƥ��
	{
		if(std_id==0x200+i)
		{
			memcpy(M2006_Feedback_Buf[i-1],data_p,6);
			pos_rec(i);
			return;
		}
	}
}
/*********************************************************************************
  *@  name      : M_2006_Get_Real_I
  *@  function  : ��ȡM2006�����ʵ��ת����Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid�����ת��,��ȡʧ�ܷ���0
*********************************************************************************/
int M_2006_Get_Torque(uint8_t motor_id)
{
	int torque = 0;
	if(M2006_Feedback_Buf[motor_id-1][2]>>7==1)
		torque = -( 0xffff-(  (M2006_Feedback_Buf[motor_id-1][4]<<8)+M2006_Feedback_Buf[motor_id-1][5])  ) ;
	else 
		torque = (M2006_Feedback_Buf[motor_id-1][4]<<8)+M2006_Feedback_Buf[motor_id-1][5];
	return torque;
}
/*********************************************************************************
  *@  name      : M2006_Get_Speed
  *@  function  : ��ȡM2006����ķ������ٶ���Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid������ٶ�,��ȡʧ�ܷ���0
*********************************************************************************/
int M2006_Get_Speed(uint8_t motor_id)
{
	int speed = 0;
	if(M2006_Feedback_Buf[motor_id-1][2]>>7==1)
		speed = -( 0xffff-(  (M2006_Feedback_Buf[motor_id-1][2]<<8)+M2006_Feedback_Buf[motor_id-1][3])  ) ;
	else 
		speed = (M2006_Feedback_Buf[motor_id-1][2]<<8)+M2006_Feedback_Buf[motor_id-1][3];
	return speed;
}
/*********************************************************************************
  *@  name      : M2006_Get_Pos
  *@  function  : ��ȡM2006�����ǰ��λ����Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid�����λ�ã���������CNTֵ
*********************************************************************************/
int M2006_Get_Pos(uint8_t motor_id)
{
	return M2006_Pos[motor_id - 1];
}
/*********************************************************************************
  *@  name      : pos_rec
  *@  function  : ��ȡ2006����ķ�����λ����Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid�����λ����Ϣ,��ȡʧ�ܷ���-1
*********************************************************************************/
void pos_rec(uint8_t motor_id)
{
	int id=motor_id-1;
	int32_t	m2006_tmp[8];
	static int32_t	m2006_base[8] = {0};	//��������Ѿ�ת����Ȧ����һȦ8192
	static int32_t m2006tmp_pre[8] = {0};

	m2006_tmp[id]=(M2006_Feedback_Buf[id][0]<<8)+M2006_Feedback_Buf[id][1];
	if ( m2006_tmp[id] - m2006tmp_pre[id] > 4095 )  //ת��8191��0ʱ��¼Ȧ��
		m2006_base[id] -= 8191;
	else if ( m2006_tmp[id] - m2006tmp_pre[id] < -4095 )
		m2006_base[id] += 8191;
	
	m2006tmp_pre[id] = m2006_tmp[id];
	M2006_Pos[id] = m2006_base[id] + m2006_tmp[id];	

}
/********************************************************************************
  *@  name      : DJI_Ang2Cnt
  *@  function  : �Ƕ�ת��Ϊʵ�ʵ��Ӧ��ת����λ���� //δ���������ת��תһȦ��ֵΪ8192
  *@  input     : Ŀ��Ƕȣ�����Ƕȣ������id  //id��ͬ�����ٱȲ�ͬ
  *@  output    : ���λ��
********************************************************************************/
int DJI_Ang2Cnt(float angle,int ID)  
{

	int cnt;
	cnt = (int)(CNT_PER_ROUND_OUT(ID) * angle/360);
	return cnt;
}
/********************************************************************************
  *@  name      : DJI_Cnt2Ang
  *@  function  : ���λ��ת��Ϊ�Ƕ� //δ���������ת��תһȦ��ֵΪ8192
  *@  input     : ���λ�ã����id  //id��ͬ�����ٱȲ�ͬ
  *@  output    : ���ת���ĽǶ�
********************************************************************************/
double DJI_Cnt2Ang(int32_t cnt,int ID)
{
	int angled;
	angled = (double)((cnt * 360.0)/CNT_PER_ROUND_OUT(ID));
	return angled;
}

//void CAN_SendData(CAN_HandleTypeDef *hcant, uint8_t *pData){
//	
//	CAN_TxHeaderTypeDef TxMessage;
//	TxMessage.StdId = 0x200;
//	TxMessage.DLC = 8; /*Ĭ��һ֡���䳤��Ϊ8*/
//	TxMessage.IDE = CAN_ID_STD;
//	TxMessage.RTR = CAN_RTR_DATA;
//	//�ȴ�������
//	HAL_CAN_GetTxMailboxesFreeLevel(hcant);

//	while (!HAL_CAN_GetTxMailboxesFreeLevel(hcant)); //�ȴ�������
//	HAL_CAN_AddTxMessage(hcant, &TxMessage, pData, (uint32_t *)CAN_TX_MAILBOX1);
//}

uint8_t DJI_CAN_Send_Data(CAN_HandleTypeDef* hcant, uint8_t *pData, uint16_t ID, uint16_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
	uint8_t FreeTxNum = 0;
	CAN_TxHeaderTypeDef TxMessage;
	
	TxMessage.IDE = CAN_ID_STD;  //��׼֡,CAN_ID_EXT��չ֡;
	TxMessage.RTR = CAN_RTR_DATA;  //����֡,CAN_RTR_REMOTEң��֡
	TxMessage.StdId = ID;
	TxMessage.DLC = Len;
	
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcant);	
		
	while(FreeTxNum==0)  //�ȴ�������
	{
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcant);
	}
	
	HAL_Delay(1); //û����ʱ���п��ܻᷢ��ʧ��
	
	HAL_RetVal = HAL_CAN_AddTxMessage(hcant,&TxMessage,pData,(uint32_t*)CAN_TX_MAILBOX0);
	
	if(HAL_RetVal!=HAL_OK)
	{
		return 2;
	}
	
	return 0;
}




