#include "stm32f4xx.h"                  // Device header
#include "PWM.h"

void Servo_SetAngle(float Angle)
{
	PWM_SetCompare3(Angle / 180 * 2000 + 500);
}
void Servo_Init(void)
{
	PWM_Init();
	Servo_SetAngle(210);
}
