#include "mechanical_arm_uart.h"


#if UART5_ENABLE
  uint8_t uart5SendBuf[USART_SEND_BUF_SIZE+1];
	uint8_t uart5RecvBuf[USART_RECV_BUF_SIZE+1];
	RingBufferTypeDef uart5SendRingBuf;
	RingBufferTypeDef uart5RecvRingBuf;
	Usart_DataTypeDef uart5;
	 
#endif


void Usart_Init(void){
		#if 0
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
		
    // 串口配置结构体
    USART_InitTypeDef USART_InitStructure;
    // 外部中断结构体
    NVIC_InitTypeDef NVIC_InitStructure;
		#endif

    #if UART5_ENABLE
			// 赋值结构体usart指针
			uart5.pUSARTx = UART5;
			// 初始化缓冲区(环形队列)
			RingBuffer_Init(&uart5SendRingBuf,  USART_SEND_BUF_SIZE, uart5SendBuf);
			RingBuffer_Init(&uart5RecvRingBuf, USART_RECV_BUF_SIZE,  uart5RecvBuf);
			uart5.recvBuf = &uart5RecvRingBuf;
			uart5.sendBuf = &uart5SendRingBuf;
    #endif
    
}

/* 发送单个字节 */
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch)
{
    /* 发送一个字节到USART */
		HAL_UART_Transmit(&huart5,&ch,1,0xffff);
//    USART_SendData(pUSARTx, ch);
//	/* 等待发送寄存器为空 */
//    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/* 发送8位的字节流 */
void Usart_SendByteArr(USART_TypeDef *pUSARTx, uint8_t *byteArr, uint16_t size){
    uint16_t bidx;
    for (bidx=0; bidx<size; bidx++){
        Usart_SendByte(pUSARTx, byteArr[bidx]);
    }
    /* 等待发送寄存器为空 */
    // while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/* 发送字符串 */
void Usart_SendString(USART_TypeDef *pUSARTx, char *str){
    uint16_t sidx=0;
    do
    {
        Usart_SendByte(pUSARTx, (uint8_t)(*(str + sidx)));
        sidx++;
    } while(*(str + sidx) != '\0');
    /* 等待发送寄存器为空 */
	// USART_FLAG_TXE:  Transmit data register empty flag 发送数据寄存器为空
	// RESET = 0
	// SET = 1
    // while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

//// 将串口发送缓冲区的内容全部发出去
//void Usart_SendAll(Usart_DataTypeDef *usart){
//	uint8_t value;
//	uint8_t Val_All[13] = {0};
//	int i = 0;
//	while(RingBuffer_GetByteUsed(usart->sendBuf)){
//		value = RingBuffer_Pop(usart->sendBuf);
//		Val_All[i] = value;
//		i++;
//		// printf("Usart_SendAll pop: %d", value);
////		Usart_SendByte(usart->pUSARTx, value);
//	}
//	HAL_UART_Transmit(&huart5,Val_All,12,0xffffff);
//}

// 将串口发送缓冲区的内容全部发出去
void Usart_SendAll(Usart_DataTypeDef *usart){
	uint8_t value;
	static uint8_t Val_All[70] = {0};
	static int i = 0;
	while(RingBuffer_GetByteUsed(usart->sendBuf)){
		value = RingBuffer_Pop(usart->sendBuf);
		Val_All[i] = value;
		i++;
		// printf("Usart_SendAll pop: %d", value);
//		Usart_SendByte(usart->pUSARTx, value);
	}
	if(i>=60){
//	HAL_UART_Transmit_DMA(&huart5,Val_All,12);
		HAL_UART_Transmit(&huart5,Val_All,i,0xfffffff);
		i = 0;
		memset(Val_All,0,sizeof(Val_All));
	}
//	HAL_UART_Transmit(&huart5,Val_All,12,0xffffff);
}

