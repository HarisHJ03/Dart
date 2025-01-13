#ifndef __TIMER_H__
#define __TIMER_H__

typedef struct
{
	uint8_t fire[6], yaw_2006, yaw_6020, reload, push, rc;
}err_t;

void Timer_Init(void);

uint8_t TIM2_GetFlag(void);
void TIM2_ClearFlag(void);
void TIM2_SetCount(uint16_t Count);
uint16_t TIM2_GetCount(void);

extern err_t err_list;

#endif
