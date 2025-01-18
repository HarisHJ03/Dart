#include "stm32f4xx.h"
#include "gpio.h"

void GPIO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	
	/*LED灯*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOH, &GPIO_InitStructure);
	// 由原理图可知，引脚拉低，LED亮，所以一开始全部拉高电平
	GPIO_TURNLED(WHITE);
}

void GPIO_TURNLED(LED_COLOR color)
{
	//这里RGB对应的H12 H11 H10，传进来的COLOR左移10位就会对应上
	GPIO_Write(GPIOH, color << 10);
}
