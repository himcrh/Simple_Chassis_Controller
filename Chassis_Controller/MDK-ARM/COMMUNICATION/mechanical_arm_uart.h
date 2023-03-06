#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "ring_buffer.h"

#define USART_RECV_BUF_SIZE 500
#define USART_SEND_BUF_SIZE 500

// UART Enable
#define UART5_ENABLE 1

typedef struct
{  
    USART_TypeDef *pUSARTx;
    //发送端缓冲区
    RingBufferTypeDef *sendBuf;
		//接收端缓冲区
    RingBufferTypeDef *recvBuf;
} Usart_DataTypeDef;

//memset(&gc_block, 0, sizeof(parser_block_t))
#if UART5_ENABLE
  extern Usart_DataTypeDef uart5;
  extern uint8_t uart5SendBuf[USART_SEND_BUF_SIZE+1];
	extern uint8_t uart5RecvBuf[USART_RECV_BUF_SIZE+1];
	extern RingBufferTypeDef uart5SendRingBuf;
	extern RingBufferTypeDef uart5RecvRingBuf;
#endif


// 配置串口
void Usart_Init(void);
// 发送字节
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch);
// 发送字节数组
void Usart_SendByteArr(USART_TypeDef *pUSARTx, uint8_t *byteArr, uint16_t size);
// 发送字符串
void Usart_SendString(USART_TypeDef *pUSARTx, char *str);
// 将串口发送缓冲区的内容全部发出去
void Usart_SendAll(Usart_DataTypeDef *usart);

#endif
