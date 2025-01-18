#include "can.h"

/*CAN1配置*//*CAN1_rx D0*//*CAN1_TX D1*/
void CAN1_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);//can时钟使能
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, ENABLE);//can外设复位
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, DISABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//GPIO备用功能模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;//高速模式
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);//重映射
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
	
	CAN_InitStructure.CAN_ABOM = ENABLE;//软件自动离线管理
	CAN_InitStructure.CAN_AWUM = DISABLE;//睡眠模式通过软件唤醒
	CAN_InitStructure.CAN_TTCM = DISABLE;//非时间触发通道模式
	CAN_InitStructure.CAN_TXFP = DISABLE;//优先级由报文标识符决定
	CAN_InitStructure.CAN_NART = DISABLE;//禁止报文自动传送
	CAN_InitStructure.CAN_RFLM = DISABLE;//报文不锁定，新的覆盖旧的
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;//模式设置
	CAN_InitStructure.CAN_Prescaler = 3;//预分频系数
	CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;//重新同步跳跃宽度
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;//
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;//
	CAN_Init(CAN1,&CAN_InitStructure);//初始化can1
	
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;//屏蔽位宽模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;//32位宽
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;//32ID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;//
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32位mask
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;//
	CAN_FilterInitStructure.CAN_FilterNumber = 0;//过滤器0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;//激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);//初始化滤波器
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;// 主优先级为5
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/*CAN2配置*//*CAN2_RX B5*//*CAN2_TX B6*/
void CAN2_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);
	
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_Prescaler = 3;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_Init(CAN2,&CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterNumber = 14;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 	5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void CAN_INIT(void)
{
	CAN1_INIT();
	CAN2_INIT();
}
