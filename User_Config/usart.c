#include "usart.h"
#include "Start_Task.h"
#include "stdlib.h"
#include "string.h"
#include "Timer.h"

extern rc_info_t rc;

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf)
{
	rc->ch1 = (dbus_buf[0] | dbus_buf[1] << 8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (dbus_buf[1] >> 3 | dbus_buf[2] << 5) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (dbus_buf[2] >> 6 | dbus_buf[3] << 2 | dbus_buf[4] << 10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (dbus_buf[4] >> 1 | dbus_buf[5] << 7) & 0x07FF;
	rc->ch4 -= 1024;

	/* prevent remote control zero deviation */
	if(rc->ch1 <= 5 && rc->ch1 >= -5)
		rc->ch1 = 0;
	if(rc->ch2 <= 5 && rc->ch2 >= -5)
		rc->ch2 = 0;
	if(rc->ch3 <= 5 && rc->ch3 >= -5)
		rc->ch3 = 0;
	if(rc->ch4 <= 5 && rc->ch4 >= -5)
		rc->ch4 = 0;

	rc->sw1 = ((dbus_buf[5] >> 4) & 0x000C) >> 2;
	rc->sw2 = (dbus_buf[5] >> 4) & 0x0003;
	rc->iw = (dbus_buf[16] | dbus_buf[17] << 8) & 0x07FF;

	if ((abs(rc->ch1) > 660) || \
	  (abs(rc->ch2) > 660) || \
	  (abs(rc->ch3) > 660) || \
	  (abs(rc->ch4) > 660))
	{
		memset(rc, 0, sizeof(rc_info_t));
		return ;
	}
}
  
/*USART3*/
uint8_t  dbus_buf[2][DBUS_MAX_LEN];
static void USART3_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	
	DMA_DeInit(DMA1_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dbus_buf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = DBUS_MAX_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);
	/*使用双缓冲模式*/
	DMA_DoubleBufferModeConfig(DMA1_Stream1,(uint32_t)dbus_buf[1],DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream1,ENABLE);
	
	DMA_Cmd(DMA1_Stream1, DISABLE);
	DMA_Cmd(DMA1_Stream1, ENABLE);

}


USART_InitTypeDef USART_InitStructure3;

void USART3_DEVICE(void)/*DBUS*/
{
	/**USART3 GPIO Configuration
	PC11     ------> USART3_RX
	PC10     ------> USART3_TX
	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	USART_DeInit(USART3);
	USART_InitStructure3.USART_BaudRate = 100000;
	USART_InitStructure3.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure3.USART_StopBits = USART_StopBits_1;
	USART_InitStructure3.USART_Parity = USART_Parity_Even;
	USART_InitStructure3.USART_Mode = USART_Mode_Rx;
	USART_InitStructure3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&USART_InitStructure3);
	
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_IDLE);
	USART_ClearITPendingBit(USART3,USART_FLAG_IDLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART3_DMA();
}


/*USART3 中断函数*/
void USART3_IRQHandler(void)
{
	static uint16_t rbuf_size;
	if(USART_GetFlagStatus(USART3,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(USART3);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)//采用DMA双缓冲区进行接收数据，当使用memory0时
		{
			/*重置DMA*/
			DMA_Cmd(DMA1_Stream1,DISABLE);
			rbuf_size = DBUS_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);
			if(rbuf_size == DBUS_BUFLEN*2)
			{
				remote_ctrl(&rc,dbus_buf[0]);
			}
			DMA_SetCurrDataCounter(DMA1_Stream1,DBUS_MAX_LEN);
			DMA1_Stream1->CR |= DMA_SxCR_CT;
			
			DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF2|DMA_FLAG_HTIF2);
			DMA_Cmd(DMA1_Stream1,ENABLE);	
		}
		else//当使用memory1时
		{
			/*重置DMA*/
			DMA_Cmd(DMA1_Stream1,DISABLE);
			rbuf_size = DBUS_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream2);
			if(rbuf_size == DBUS_BUFLEN*2)
			{
				remote_ctrl(&rc,dbus_buf[1]);
			}
			DMA_SetCurrDataCounter(DMA1_Stream1,DBUS_MAX_LEN);
			DMA1_Stream1 ->CR &= ~(DMA_SxCR_CT);
			
			DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF2|DMA_FLAG_HTIF2);
			DMA_Cmd(DMA1_Stream1,ENABLE);
		}
		err_list.rc = 20;
		USART_ClearFlag(USART3, USART_FLAG_IDLE);//清除空闲中断标志位
		USART_ClearITPendingBit(USART3,USART_FLAG_IDLE);
	}
}
