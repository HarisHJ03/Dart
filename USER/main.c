#include "stm32f4xx.h"
#include "Start_Task.h"

#include "gpio.h"
#include "can.h"
#include "usart.h"
#include "Timer.h"
#include "Delay.h"

uint8_t Savety_Flag;

int main(void)
{
	// 初始化
	GPIO_INIT();
	CAN_INIT();
	USART3_DEVICE();
	Timer_Init();
	PID_INIT();

	// 重置定时器
	TIM2_SetCount(20);
	TIM2_ClearFlag();
	// 等待所有设备在线超过20*0.05s = 1s
	while (1)
	{
		Cut_Power();
		// 如果有不在线就重置定时器
		if (!ALLMotor_OnLine())
		{
			TIM2_SetCount(20);
			TIM2_ClearFlag();
		}
		// 只有在线了，且定时器响了才会跳出循环
		else if (TIM2_GetFlag())
			break;
	}

	// 上电后必须等到右拨杆拨到最下面一次
	while (!Savety_Flag)
	{
		PID_INIT();
		Cut_Power();
		if (rc.sw2 == 2)
			Savety_Flag = 1;
	}
	// 重置定时器
	TIM2_SetCount(20);
	TIM2_ClearFlag();
	// 初始化底部各个电机的位置和数据
	while (!Init_Mode())
		Cut_Power();
	// 开始跑主任务
	while (1)
	{
		if (ALLMotor_OnLine())
		{
			PID_INIT();
			Dart_MainControl();
		}
		else
		{
			while (!ALLMotor_OnLine())
			{
				Cut_Power();
				Reset_Reload(1);
			}
		}
	}
}
