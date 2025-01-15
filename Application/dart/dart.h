#ifndef DART_H
#define DART_H

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void DartInit(void);

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void DartTask(void);

typedef enum 
{
	NONE,
	RELAOD,
	YAW,
	PUSH,
}Safe_State;

#endif // CHASSIS_H
