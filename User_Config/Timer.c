#include "stm32f4xx.h" // Device header
#include "Timer.h"

uint8_t TIM2_Flag;
uint16_t TIM2_Count;
err_t err_list;

void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_InternalClockConfig(TIM2);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 500 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 4200 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM2, ENABLE);
}

uint8_t TIM2_GetFlag(void)
{
	return TIM2_Flag;
}

void TIM2_ClearFlag(void)
{
	TIM2_Flag = 0;
}

void TIM2_SetCount(uint16_t Count)
{
	TIM2_Count = Count;
}

uint16_t TIM2_GetCount(void)
{
	return TIM2_Count;
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		if (TIM2_Count > 0)
		{
			TIM2_Count--;
		}
		else
		{
			TIM2_Flag = 1;
		}

		if (err_list.fire[0] > 0)
			err_list.fire[0]--;
		if (err_list.fire[1] > 0)
			err_list.fire[1]--;
		if (err_list.fire[2] > 0)
			err_list.fire[2]--;
		if (err_list.fire[3] > 0)
			err_list.fire[3]--;
		if (err_list.fire[4] > 0)
			err_list.fire[4]--;
		if (err_list.fire[5] > 0)
			err_list.fire[5]--;
		if (err_list.yaw_2006 > 0)
			err_list.yaw_2006--;
		if (err_list.yaw_6020 > 0)
			err_list.yaw_6020--;
		if (err_list.reload > 0)
			err_list.reload--;
		if (err_list.push > 0)
			err_list.push--;
		if (err_list.rc > 0)
			err_list.rc--;

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
