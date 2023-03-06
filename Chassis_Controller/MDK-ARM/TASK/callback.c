/*From cubemx*/
#include "can.h"
#include "usart.h"
#include "string.h"

/*From mine*/
#include "rm2006.h"
#include "my_task.h"
#include "Chassis_bsp.h"

/*CAN1中断回调函数，用于接收2006返回数据*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	if(hcan->Instance == hcan1.Instance)
	{
		CAN_RxHeaderTypeDef RxMessage;
		uint8_t RxData[8] = {0};
	
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxData);

		/*处理2006回传数据*/
		M2006_Get_Feedback(RxMessage.StdId,RxData);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*二维码扫描模块*/
	if(huart->Instance == huart1.Instance){
		uint8_t Uart_Tx_Buffer[22]={0xff,0xff,0xff,0x74,0x31,0x2E,0x74,0x78,0x74,0x3D,0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0xff,0xff,0xff};
		memcpy(&Uart_Tx_Buffer[11],&Uart_Rx_Buffer[0],7);
		HAL_UART_Transmit(&huart6,Uart_Tx_Buffer,22,0xffff);
		
		if(Uart_Rx_Buffer[7]==0x0d && Uart_Rx_Buffer[8]==0x0a){
			for(int i=0;i<=2;i++){
				Task_Info.Task_Code_Fr[i] = Uart_Rx_Buffer[i]-'0';
				Task_Info.Task_Code_Se[i] = Uart_Rx_Buffer[i+4]-'0';
			}
		}
			
		
	}
	if(huart->Instance == huart6.Instance){
		if(operation_code == 0xfb){
			Move_Info.Move_Status = Move_Task;
		}
		if(operation_code == 0xfa){
			Move_Info.Move_Status = Move_Free;
		}
	}
	if(huart->Instance == huart4.Instance){
		if(Sence_Rx_Buffer[0] == '?'&&Sence_Rx_Buffer[7]=='!'){
			for(int i=0;i<=2;i++){
				Task_Info.Up_Order[i] = Sence_Rx_Buffer[i+1];
				Task_Info.Down_Order[i] = Sence_Rx_Buffer[i+4];
			}
			
		}
	}
}
