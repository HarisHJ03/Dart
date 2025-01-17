/*
	机械最后修改时间：2024-7-17
	电控最后修改时间：2024-7-17

	电控设备：
	C板
	摩擦轮2对4个3508（不带减速箱）：挂载在CAN1
	飞镖轮盘1个3508（带减速箱）：挂载在CAN2
	推杆1个2006：挂载在CAN2
	YAW轴丝杆1个2006：挂载在CAN2
	YAW轴转轴一个6020：挂载在CAN2
*/

#include "stm32f4xx.h"

#include "Start_Task.h"

#include "bsp_can.h"
#include "pid.h"
#include "gpio.h"
#include "Delay.h"
#include "Timer.h"

/*
标志位作用详解

Fire_Finish_Flag:
表示是否是处于发射结束，上电默认结束置1，在Fire_Mode中通过操作可以置0，随后执行Push_Mode操作执行完后置1

Fire_Savety_Flag:
是否可以执行发射的安全标志位，这个是用于刚进入Fire_Mode模式，保证调试安全和操作安全必要，修改这个地方一定要注意逻辑
没有进入Fire_Mode的其他所有模式将其置0，每次刚进入Fire_Mode判断必为0，这时检测到左边拨杆在中间并且摇杆值在中下才置1
这样做是为了防止左拨杆是打开摩擦轮状态（即不在中间，这时摩擦轮会转）并且左摇杆推到上面（这时候就会启动推杆）
没有安全判断的话进入的发射模式直接就发射了，非常危险，如果以后有新的Mode，切记切记，这个标志位必须在所有地方都置0

Push_Delay_Flag:
用于实现两连发操作过程判断是否执行了等待延时，由于两连发的连贯动作，推杆推到顶再回退到底后，需要等待转盘转动的时间，
这时候推杆必须保持不动，等待一定时间后才能重新上推发射第二发
进入Fire_Mode前置0，发射完第一发后，推杆回退到底部，判断为0，置1且延时50*0.05s=2.5s来等待转盘转动

Push_Num_Flag;
这个用于记录两连发执行至第几发，执行发射前置0，发射第一发后判断为0，置1并执行第二次发射，发射第二法后判断为1，退出发射模式

Push_Reset_Flag;
这个用于决定推杆是使用角度环还是速度环，因为想要推杆发射过程中是匀速推，而没有操作的时候是锁住回退到底部的角度
正常置1，执行Fire_Mode的时候置0，启用速度环，执行完二连发后置1，重新启用角度环

Check_Locked_Flag;
用于检测该锁住角度的电机是否在允许误差范围内，给有需要的模式调用
再Check_Mode中通过判断各个电机的锁住情况给出结果

Adjust_Savety_Flag;
跟Fire_Savety_Flag一样，保证调整过程的安全进入
在其他模式中置0，进入后判断符合安全条件再置1,

Can1_Flag, Can2_Flag：
can1，can2的中断标志位，再对应CAN中断中置1，由Send_Mode读取，实现有接收才有发送，而不是无脑发
*/
uint8_t Fire_Finish_Flag = 1;
uint8_t Fire_Savety_Flag;
uint8_t Push_Delay_Flag;
uint8_t Push_Num_Flag;
uint8_t Push_Reset_Flag;
uint8_t Adjust_Savety_Flag;
uint8_t Check_Locked_Flag;
uint8_t Can1_Flag, Can2_Flag;


uint8_t test_state;

// 一个枚举，提高代码可读性
enum beat_state
{
	STOP,
	OUTPOST,
	BASE
};

int16_t test_speed1[4]={2000,3000,4000,5000};//低于3000大的摩擦轮不启动
int16_t test_speed2[4]={2000,3000,4000,5000};//每发飞镖的转速
int32_t test_yaw[4]={0,0,0,0};//每发飞镖的yaw补偿
uint8_t ID;//四发飞镖定ID

// 三对摩擦轮转速预设
int16_t target_speed1[3] = {0, 5550, 6000}; // 0停转 1前哨站 2基地5800, 8500
int16_t target_speed2[3] = {0, 5550, 6000}; // 0停转 1前哨站 2基地
int16_t target_speed3[3] = {0, 2000, 4000}; // 0停转 1前哨站 2基地
// yaw轴角度存储数组，储存前哨站和基地的位置
int32_t target_angle[3]; // 0无 1前哨站 2基地

// 遥控结构体变量
rc_info_t rc;

/*--电机结构体变量--*/
extern moto_measure_t moto_fire[6];											// 3508 * 6
extern moto_measure_t moto_yaw_2006, moto_yaw_6020, moto_reload, moto_push; // 2006 6020 3508 2006

/*--电机pid结构体变量--*/
pid_t fire1, fire2, fire3, fire4, fire5, fire6, yaw_a, yaw_s, reload_a, reload_s, push_a, push_s;

/*--PID预设参数--*/
float fire_s1_pid[3] = {35.0f, 0.003f, 5.0f}; // 三对摩擦轮用三组PID参数
float fire_s2_pid[3] = {35.0f, 0.003f, 5.0f};
float fire_s3_pid[3] = {35.0f, 0.003f, 5.0f};

float yaw_a_pid[3] = {30.0f, 0, 0.005f}; // yaw转轴6020角度环
float yaw_s_pid[3] = {15.0f, 0, 5.0f};	 // yaw丝杆2006速度环

float reload_a_pid[3] = {0.1f, 0, 0}; // 换弹3508角度环和速度环
float reload_s_pid[3] = {40.0f, 0, 5.0f};

float push_a_pid[3] = {0.07f, 0, 0.005f}; // 推杆2006角度环和速度环
float push_s_pid[3] = {10.0f, 0, 5.0f};

/*--pid计算过程的存储变量--*/
int16_t fire_target_speed[3], fire_calc[6]; // 3对摩擦轮的目标转速 6个摩擦轮的最终电流值
int16_t fire_zero[6] = {0};					// 全0且不能改，用于断电给send函数发送的数组

int32_t reload_target_angle;			  // reload目标角度
int16_t reload_target_speed, reload_calc; // reload角度环计算的目标速度 reload速度环计算出最终电流值

int32_t yaw_target_angle;			// yaw目标角度
int16_t yaw_target_speed, yaw_calc; // yaw角度环计算或调整时给定的速度 yaw速度环计算最终电流值

int32_t push_target_angle;			  // push目标角度
int16_t push_target_speed, push_calc; // push角度环计算或调整时给定的速度 push速度环计算最终电流值

// PID初始化
void PID_INIT(void)
{
	PID_Struct_Init(&fire1, fire_s1_pid[0], fire_s1_pid[1], fire_s1_pid[2], 16000, 5000, INIT); // fire1
	PID_Struct_Init(&fire2, fire_s1_pid[0], fire_s1_pid[1], fire_s1_pid[2], 16000, 5000, INIT); // fire2
	PID_Struct_Init(&fire3, fire_s2_pid[0], fire_s2_pid[1], fire_s2_pid[2], 16000, 5000, INIT); // fire3
	PID_Struct_Init(&fire4, fire_s2_pid[0], fire_s2_pid[1], fire_s2_pid[2], 16000, 5000, INIT); // fire4
	PID_Struct_Init(&fire5, fire_s3_pid[0], fire_s3_pid[1], fire_s3_pid[2], 16000, 5000, INIT); // fire5
	PID_Struct_Init(&fire6, fire_s3_pid[0], fire_s3_pid[1], fire_s3_pid[2], 16000, 5000, INIT); // fire6

	PID_Struct_Init(&yaw_a, yaw_a_pid[0], yaw_a_pid[1], yaw_a_pid[2], 16000, 5000, INIT); // yaw
	PID_Struct_Init(&yaw_s, yaw_s_pid[0], yaw_s_pid[1], yaw_s_pid[2], 16000, 5000, INIT); // yaw

	PID_Struct_Init(&reload_a, reload_a_pid[0], reload_a_pid[1], reload_a_pid[2], 15000, 5000, INIT); // reload
	PID_Struct_Init(&reload_s, reload_s_pid[0], reload_s_pid[1], reload_s_pid[2], 2000, 300, INIT);	  // reload

	PID_Struct_Init(&push_a, push_a_pid[0], push_a_pid[1], push_a_pid[2], 16000, 5000, INIT); // push
	PID_Struct_Init(&push_s, push_s_pid[0], push_s_pid[1], push_s_pid[2], 16000, 5000, INIT); // push
}

/*
	用于检测是否有任何一个设备掉线，err_list工作逻辑，由于所有设备都是通信中断，每个设备进中断就会将自己的err_list中
	对应的值刷新，定时中断又会定时将这些值自减到0，如果一个设备掉线一段时间没刷新自己的值，那就会被定时中断减到0，这时候
	这个函数里就会判断到有设备掉线了
*/
uint8_t ALLMotor_OnLine(void)
{
	// 检查所有设备
	if (!err_list.fire[0] || !err_list.fire[1] || !err_list.fire[2] || !err_list.fire[3] /*|| !err_list.fire[4] || !err_list.fire[5]*/
		|| !err_list.yaw_2006 || !err_list.yaw_6020 || !err_list.reload || !err_list.push || !err_list.rc)
	{
		// 只要有一个为0就return 0，并且板子亮红灯，如果没有设备掉线就亮绿灯
		GPIO_TURNLED(RED);
		return 0;
	}
	else
	{
		GPIO_TURNLED(GREEN);
		return 1;
	}
}

/*
	上电时候调用一次
*/
uint8_t Init_Mode(void)
{
	// 读取6020上电角度，记录为基地角和前哨站角，防止没有初始化会锁定所在机械限制范围角度外，防止机械拆装导致出现电机疯转，每次比赛前进入adjust_mode人肉瞄准刷新目标值
	target_angle[BASE] = moto_yaw_6020.total_ecd;
	target_angle[OUTPOST] = moto_yaw_6020.total_ecd;
	// reload角也锁定为上电角度
	reload_target_angle = moto_reload.total_ecd;
	// 上电校准推杆退回最底部,所以给定一个回退速度
	push_calc = pid_calc(&push_s, moto_push.speed_rpm, 200);
	can_send_botton(0, 0, push_calc);
	// 如果能转动说明还没推到底，重置定时器
	if (moto_push.speed_rpm > 50 || moto_push.speed_rpm < -50)
	{
		TIM2_SetCount(10);
		TIM2_ClearFlag();
	}
	// 如果推不动了且定时器响了说明已经卡了0.5s，这时候认为收到底了
	else if (TIM2_GetFlag())
	{
		// 这时候先设置定时器等待2.5s，没有这个定时的话，2006会持续输出力矩，可能导致角度不准确
		TIM2_SetCount(50);
		TIM2_ClearFlag();
		while (!TIM2_GetFlag())
			;
		// 等到整个推杆没力气了，记录当前角度为底部锁定角度
		push_target_angle = moto_push.total_ecd;
		// 返回初始化完成
		return 1;
	}
	// 返回初始化未完成
	return 0;
}

void Cut_Power(void)
{
	// 重置所有标志位
	Fire_Finish_Flag = 1;
	Fire_Savety_Flag = 0;
	Push_Delay_Flag = 0;
	Push_Num_Flag = 0;
	Push_Reset_Flag = 1;
	Check_Locked_Flag = 0;
	Adjust_Savety_Flag = 0;

	// 所有发送为零，确保断电
	if (Can1_Flag == 1)
	{
		can_send_fire(fire_zero);

		Can1_Flag = 0;
	}
	if (Can2_Flag == 1)
	{
		can_send_botton(0, 0, 0);

		Can2_Flag = 0;
	}
}

void Reset_Reload(uint8_t state) // state:1表示被动断电（设备掉线），0表示手动断电（右拨杆打最下）
{
	// 手动断电右拨杆向下时，将左拨杆下拨刷新reload电机0点
	if (rc.sw1 == 2 || state) // 左1右2
	{
		reload_target_angle = moto_reload.total_ecd;
		ID=0;
	}
}

static void Send_Mode(void)
{
	if (Can1_Flag == 1)
	{
		// 四个3508速度闭环
		fire_calc[0] = pid_calc(&fire1, moto_fire[0].speed_rpm, -fire_target_speed[0]);
		fire_calc[1] = pid_calc(&fire2, moto_fire[1].speed_rpm, fire_target_speed[0]);
		fire_calc[2] = pid_calc(&fire3, moto_fire[2].speed_rpm, -fire_target_speed[1]);
		fire_calc[3] = pid_calc(&fire4, moto_fire[3].speed_rpm, fire_target_speed[1]);
		fire_calc[4] = pid_calc(&fire3, moto_fire[2].speed_rpm, -fire_target_speed[2]);
		fire_calc[5] = pid_calc(&fire4, moto_fire[3].speed_rpm, fire_target_speed[2]);

		// 发送4个电机值
		can_send_fire(fire_calc);

		Can1_Flag = 0;
	}
	if (Can2_Flag == 1)
	{
		// yaw轴
		// 当发射模式，即右拨杆打上，只做2006角度闭环，如果打到中间就只做速度闭环
		if (rc.sw2 == 1)
			yaw_target_speed = pid_calc(&yaw_a, moto_yaw_6020.total_ecd, yaw_target_angle);
		// 底部丝杆2006速度闭环
		yaw_calc = pid_calc(&yaw_s, moto_yaw_2006.speed_rpm, yaw_target_speed);

		// 换弹3508角度串速度闭环
		reload_target_speed = pid_calc(&reload_a, moto_reload.total_ecd, reload_target_angle);
		reload_calc = pid_calc(&reload_s, moto_reload.speed_rpm, reload_target_speed);

		// 推杆
		// 当Push_Reset_Flag不为0，即为要回到最底部，这时候用角度环，如果是在发射过程中，则为0，这时候用速度环推上去
		if (Push_Reset_Flag)
			push_target_speed = pid_calc(&push_a, moto_push.total_ecd, push_target_angle);
		// 速度环
		push_calc = pid_calc(&push_s, moto_push.speed_rpm, push_target_speed);

		can_send_botton(yaw_calc, reload_calc, push_calc);

		Can2_Flag = 0;
	}
}

/*
	这里两连发都是采第一发打完转180度，第二发打完转90度，保证转盘的左右质量分布均匀，reload3508电机控制更稳定
*/
static void Push_Mode(void)
{
	// 将其他模式标志位重置
	Fire_Savety_Flag = 0;
	Fire_Finish_Flag = 0;
	Adjust_Savety_Flag = 0;
	
	// 刚进入Push_Reset_Flag为0，表示没有结束推杆发射
	if (!Push_Reset_Flag)//判断推杆位置
	{
		
		// 如果能转动说明还没推到顶，重置定时器
		if (moto_push.speed_rpm > 50 || moto_push.speed_rpm < -50)
		{
			TIM2_SetCount(10);
			TIM2_ClearFlag();
		}
		// 如果推不动了且定时器响了说明已经卡了0.5s，这时候认为推到顶了
		else if (TIM2_GetFlag())
		{
			// 判断速度是推到顶还是底
			if (push_target_speed < 0) // 目标速度小于0是推到顶，fire_mode最后给的是-1000上推
			{
				// 设置速度大于0，开始反转回退，并且重新启动定时器等待推杆收到底部
				push_target_speed = 1000;
				TIM2_SetCount(30);
				TIM2_ClearFlag();
			}
			else // 目标速度大于0是收到底
			{
				// 这时候停转，并且Push_Reset_Flag置1，这时候推杆会重新回到锁定角度状态
				push_target_speed = 0;
				Push_Reset_Flag = 1;
			}
			
		}
	}
	else//装弹+偶数发
	{
		
		// 如果打出了第一发，这时候Push_Num_Flag为0,
		if (!Push_Num_Flag) // 如果是第一发
		{
			// 这时候先判断是否已经执行了推杆等待拨盘转动的延时，刚进入时Push_Delay_Flag==0
			if (!Push_Delay_Flag) // 如果还没等待过
			{
				// 拨盘目标角度加180度，这用的是编码位，按理来说应该是8192*19/2=77824，但是实际测试就是得78700会比较准确转过180度
				reload_target_angle += 78700;
				// 将是否执行过等待的标志位置1
				Push_Delay_Flag = 1;
				// 然后设置定时器延时50*0.05=2.5s
				TIM2_SetCount(50);
				TIM2_ClearFlag();
			}
			else if (TIM2_GetFlag()) // 如果定时器响了
			{
				// 重置Push的标志位，并重新设置推杆速度，往上推，设置定时器，这时候会执行第二次发射
				Push_Delay_Flag = 0;
				Push_Num_Flag = 1; // Push_Num_Flag置1
				Push_Reset_Flag = 0;
				push_target_speed = -1000;//发射第二发
				TIM2_SetCount(30);
				TIM2_ClearFlag();
			}
		}
		else // 打完两发了，第三发进入这里
		{
			// 转盘转过90度
			reload_target_angle += 39350;
			// 重置Fire标志位，回到Fire模式
			Fire_Finish_Flag = 1;
			Fire_Savety_Flag = 1;
			Push_Reset_Flag = 1;
		}
	}
	
}

static void Fire_Mode(void) // 发射准备，右边拨杆打到最上
{
	// 重置其他模式标志位
	Push_Delay_Flag = 0;
	Push_Num_Flag = 0;
	Push_Reset_Flag = 1;
	Adjust_Savety_Flag = 0;

	//
	if (Fire_Savety_Flag && Check_Locked_Flag)
	{
		// 判断左拨杆
		switch (rc.sw1)
		{
		// 基地模式
		case 1: // 拨上，设置为基地角度
			fire_target_speed[0] = target_speed1[BASE];
			fire_target_speed[1] = target_speed2[BASE];
			fire_target_speed[2] = target_speed3[BASE];
			yaw_target_angle = target_angle[BASE];
			break;
		// 前哨站模式
		case 2: // 拨下，记录当前为前哨站角度
			{
				switch(ID)
				{
					case 0:
						fire_target_speed[0] = test_speed1[0];
						fire_target_speed[1] = test_speed2[0];
						yaw_target_angle = test_yaw[0]+target_angle[OUTPOST];
					break;
					case 1:
						fire_target_speed[0] = test_speed1[1];
						fire_target_speed[1] = test_speed2[1];
						yaw_target_angle = test_yaw[1]+target_angle[OUTPOST];
					break;
					case 2:
						fire_target_speed[0] = test_speed1[2];
						fire_target_speed[1] = test_speed2[2];
						yaw_target_angle = test_yaw[2]+target_angle[OUTPOST];
					break;
					case 3:
						fire_target_speed[0] = test_speed1[3];
						fire_target_speed[1] = test_speed2[3];
						yaw_target_angle = test_yaw[3]+target_angle[OUTPOST];
					break;
					default:
					break;
				}
				
				
			}
//			fire_target_speed[0] = target_speed1[OUTPOST];
//			fire_target_speed[1] = target_speed2[OUTPOST];
//			fire_target_speed[2] = target_speed3[OUTPOST];
//			yaw_target_angle = target_angle[OUTPOST];
			break;
		// 锁死模式
		case 3: // 中间，锁定发射机构
			test_state=1;
			fire_target_speed[0] = target_speed1[STOP];
			fire_target_speed[1] = target_speed2[STOP];
			fire_target_speed[2] = target_speed3[STOP];
			yaw_target_angle = moto_yaw_6020.total_ecd;
			break;
		default:
			break;
		}
		// 当摇杆上推且不为锁死模式
		if (rc.ch4 >= 600 && rc.sw1 != 3)
		{
			// 重置Fire模式标志位，设置Push模式标志位，然后就会进入Push模式了
			Fire_Finish_Flag = 0;
			Fire_Savety_Flag = 0;
			Push_Num_Flag = 0;
			Push_Delay_Flag = 0;
			Push_Reset_Flag = 0;
			// 推杆设置速度向上推，然后重置定时器
			push_target_speed = -1000;//发射
			TIM2_SetCount(30);
			TIM2_ClearFlag();
		}
	}
	else if (Check_Locked_Flag) // 刚进入Fire模式并且各个锁角度的电机正常锁住就会一直进入这里判断，防止操作失误误发射
	{
		if (rc.sw1 == 3 && rc.ch4 <= 20) // 左拨杆打到中间并且此时没有推动左摇杆向前发射
		{
			Fire_Savety_Flag = 1; // 发射安全检测完成置1
		}
		// 没有通过Fire安全判断锁死发射机构
		test_state=2;
		fire_target_speed[0] = target_speed1[STOP];
		fire_target_speed[1] = target_speed2[STOP];
		fire_target_speed[2] = target_speed3[STOP];
		yaw_target_angle = moto_yaw_6020.total_ecd;
	}
	else // 如果连锁定都不正常直接锁死整个发射机构
	{
		test_state=3;
//		fire_target_speed[0] = target_speed1[STOP];
//		fire_target_speed[1] = target_speed2[STOP];
//		fire_target_speed[2] = target_speed3[STOP];//修改为第二三发中间摩擦轮不会停，但注意发射完成也不停
		yaw_target_angle = moto_yaw_6020.total_ecd;
	}
}

static void Adjust_Mode(void) // 姿态调整，右边拨杆打到中间
{
	// 重置其他模式标志位
	Fire_Finish_Flag = 1;
	Fire_Savety_Flag = 0;
	Push_Delay_Flag = 0;
	Push_Num_Flag = 0;
	Push_Reset_Flag = 1;

	// 锁死发射机构
	test_state=4;
	fire_target_speed[0] = target_speed1[STOP];
	fire_target_speed[1] = target_speed1[STOP];
	fire_target_speed[2] = target_speed1[STOP];
	yaw_target_angle = moto_yaw_6020.total_ecd;

	if (Adjust_Savety_Flag && Check_Locked_Flag) // 如果通过了安全判断且所有电机锁定在规定角度范围内
	{
		// 储存上次推杆状态
		static uint8_t last_rc_ch2 = 0;
		// 调整摩擦轮的目标速度大小存储变量
		int16_t d_speed = 0;
		if (rc.sw1 != 3) // 中间是不能调速的
		{
			if (rc.ch2 >= 600 && last_rc_ch2 == 0) // 如果右摇杆推上，且上一次的推杆转态last_rc_ch2是0
			{
				// 这时候速度变化量为+50
				d_speed = 50;
				// 推杆转态last_rc_ch2置1
				last_rc_ch2 = 1;
				// 亮蓝色灯表示增加了50
				GPIO_TURNLED(BLUE);
			}
			else if (rc.ch2 <= -600 && last_rc_ch2 == 0) // 如果右摇杆推下，且上一次的推杆转态last_rc_ch2是0
			{
				// 这时候速度变化量为-50
				d_speed = -50;
				// 推杆转态last_rc_ch2置1
				last_rc_ch2 = 1;
				// 亮蓝色灯表示减少了50
				GPIO_TURNLED(YELLOW);
			}
			else if (rc.ch2 >= -50 && rc.ch2 <= 50)
			{
				// 每次回到中间附近才置推杆转态last_rc_ch2为0
				last_rc_ch2 = 0;
				// 亮回绿色灯
				GPIO_TURNLED(GREEN);
			}
		}

		// 判断左拨杆
		switch (rc.sw1)
		{
		case 1:											  // 拨上，记录当前为基地角度
			yaw_target_speed = 0;						  // 这时候不可调yaw角度
			target_speed1[BASE] += d_speed;				  // 加上变更速度
			target_angle[BASE] = moto_yaw_6020.total_ecd; // 把当前角度记录为基地角度
			break;
		case 2:												 // 拨下，记录当前为前哨站角度
			yaw_target_speed = 0;							 // 这时候不可调yaw角度
			target_speed1[OUTPOST] += d_speed;				 // 加上变更速度
			target_angle[OUTPOST] = moto_yaw_6020.total_ecd; // 把当前角度记录为前哨站角度
			break;
		case 3: // 拨中间，这时候能调整yaw轴角度和控制转盘转过90度
			// 此时只做yaw速度闭环，速度由摇杆决定
			yaw_target_speed = rc.ch1 * 5;
			if (rc.ch4 >= 600) // 如果左摇杆推上
			{
				reload_target_angle += 39350; // 这里按理来说应该是8192*19/4，是3508一圈的编码位即360度除4转90度，但是发现偏差很大，经过测试这个值是比较稳定的
				Adjust_Savety_Flag = 0;		  // 重置安全标志位
			}
			break;
		default:
			break;
		}
	}
	else // 刚进入Adjust模式就会一直进入这里判断
	{
		if (rc.ch1 >= -20 && rc.ch1 <= 20 &&
			rc.ch2 >= -20 && rc.ch2 <= 20 &&
			rc.ch4 >= -20 && rc.ch4 <= 20 &&
			rc.sw1 == 3) // 只有当摇杆居中且打到中间过后，才能将安全判断标志位置1
		{
			Adjust_Savety_Flag = 1;
		}
		// 没有通过安全判断之前yaw轴不可动
		yaw_target_speed = 0;
	}
}

void Check_Mode(void)
{
	// 对于3508带1:19的减速箱，1编码位约为0.0023度，允许距目标值正负500个编码位偏差，换算后大约为1度
	if (moto_reload.total_ecd - reload_target_angle >= 250 ||
		moto_reload.total_ecd - reload_target_angle <= -250)
	{
		Check_Locked_Flag = 0;
	}
	// 对于6020,不带减速箱，1编码位约为0.0439度，允许距目标值正负24个编码位偏差，换算后大约为1度
	else if (moto_yaw_6020.total_ecd - yaw_target_angle >= 12 ||
			 moto_yaw_6020.total_ecd - yaw_target_angle <= -12)
	{
		Check_Locked_Flag = 0;
	}
	// 对于2006,带1：36减速箱，1编码位约为0.0012度，允许距目标值正负10000个编码位偏差，换算后大约为10度
	else if (moto_push.total_ecd - push_target_angle >= 5000 ||
			 moto_push.total_ecd - push_target_angle <= -5000)
	{
		Check_Locked_Flag = 0;
	}
	else
	{
		Check_Locked_Flag = 1;
	}
}

void Dart_MainControl(void)
{
	if (rc.sw2 == 2) // 右拨杆向下
	{
		Cut_Power();
		Reset_Reload(0); // 重置装填角度
	}
	else
	{
		Check_Mode();
		if (Fire_Finish_Flag)
		{
			switch (rc.sw2)
			{
			case 1:
				Fire_Mode();
				break;
			case 3:
				Adjust_Mode();
				break;
			default:
				break;
			}
		}
		else
		{
			switch (rc.sw2)
			{
			case 1:
				Push_Mode();
				break;
			case 3:
				Adjust_Mode();
				break;
			default:
				break;
			}
		}
		Send_Mode();
	}
}
