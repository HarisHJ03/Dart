#include "dart.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "referee_task.h"
#include "remote_control.h"
#include "bsp_led.h"
#include "bsp_dwt.h"

// 以下注释都是裸机搬运过来的残留，思路一致，请自行鉴别有用注释
Safe_State safe_state;
volatile float jscope[2];
// 飞镖控制数据
typedef struct
{
	float time_start_ms, time_span_ms;

	Robot_Status_e dart_status;
	dart_target_e dart_target; // 打击目标

	int16_t fire_target_speed[3];//3对摩擦轮的目标转速 6个摩擦轮的最终电流值
	int32_t reload_target_angle;//reload目标角度
	int32_t yaw_target_angle, yaw_target_speed;//yaw目标角度
	int32_t push_target_angle, push_target_speed;//push目标角度

	uint8_t ready_finish_flag;
	uint8_t ready_savety_flag;
	uint8_t fire_delay_flag;
	uint8_t fire_num_flag;  // 每次连发数
	uint8_t fire_reset_flag;
	uint8_t adjust_savety_flag;
	uint8_t check_locked_flag;
} Dart_Ctrl_Cmd_s;

static Dart_Ctrl_Cmd_s dart_ctrl_cmd = {
	//三对摩擦轮转速预设
	.ready_finish_flag = 1,
};

int32_t dart_yaw_targetangle[3]; // 打击目标的角度	0无 1前哨站 2基地
//三对摩擦轮转速预设
int16_t dart_fire_targetspeed1[3] = { 0, 7000, 4000 }; // 0停转 1前哨站 2基地
int16_t dart_fire_targetspeed2[3] = { 0, 7000, 4000 }; // 0停转 1前哨站 2基地
int16_t dart_fire_targetspeed3[3] = { 0, 7000, 4000 }; // 0停转 1前哨站 2基地

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static referee_info_t *referee_data; // 用于获取裁判系统的数据
static Robot_Status_e dart_status;
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
static DJIMotorInstance *motor_fire_l1, *motor_fire_r1, *motor_fire_l2, *motor_fire_r2, /* *motor_fire_l3, *motor_fire_r3, */ \
						*motor_yaw_ctrl, *motor_yaw_recv, *motor_reload, *motor_push; // 三对摩擦轮电机3508，yaw轴控制电机2006，反馈电机6020，换弹电机3508，推杆电机2006

/*
	用于检测是否有任何一个设备掉线，err_list工作逻辑，由于所有设备都是通信中断，每个设备进中断就会将自己的err_list中
	对应的值刷新，定时中断又会定时将这些值自减到0，如果一个设备掉线一段时间没刷新自己的值，那就会被定时中断减到0，这时候
	这个函数里就会判断到有设备掉线了
*/
static void OnLineConfirm(void)
{
	//检查所有设备
	if (0
		|| !motor_fire_l1->daemon->temp_count
		|| !motor_fire_r1->daemon->temp_count
		|| !motor_fire_l2->daemon->temp_count
		|| !motor_fire_r2->daemon->temp_count
		// || !motor_fire_l3->daemon->temp_count
		// || !motor_fire_r3->daemon->temp_count
		|| !motor_yaw_ctrl->daemon->temp_count
		|| !motor_yaw_recv->daemon->temp_count
		|| !motor_reload->daemon->temp_count
		|| !rc_data[TEMP].daemon->temp_count
	)
	{
		//只要有一个为0就return 0，并且板子亮红灯，如果没有设备掉线就亮绿灯
		dart_ctrl_cmd.dart_status = ROBOT_STOP;
		FlowRGBShow(LED_RED);
	} 
	else if (rc_data[TEMP].remote_data.rc.switch_right == RC_SW_DOWN)
	{
		dart_ctrl_cmd.dart_status = ROBOT_STOP;
		FlowRGBShow(LED_YELLOW);
	}
	else
	{
		dart_ctrl_cmd.dart_status = ROBOT_READY;
		FlowRGBShow(LED_GREEN);
	}
}

static void Disable_Power(void)
{
	//重置所有标志位
	dart_ctrl_cmd.ready_finish_flag = 1;
	dart_ctrl_cmd.ready_savety_flag = 0;
	dart_ctrl_cmd.fire_delay_flag = 0;
	dart_ctrl_cmd.fire_num_flag = 0;
	dart_ctrl_cmd.fire_reset_flag = 1;
	dart_ctrl_cmd.check_locked_flag = 0;
	dart_ctrl_cmd.adjust_savety_flag = 0;
	
	DJIMotorStop(motor_fire_l1);
	DJIMotorStop(motor_fire_r1);
	DJIMotorStop(motor_fire_l2);
	DJIMotorStop(motor_fire_r2);
	// DJIMotorStop(motor_fire_l3);
	// DJIMotorStop(motor_fire_r3);
	DJIMotorStop(motor_yaw_ctrl);
	DJIMotorStop(motor_yaw_recv);
	DJIMotorStop(motor_reload);
	DJIMotorStop(motor_push);
}

static void Enable_Power(void)
{
	DJIMotorEnable(motor_fire_l1);
	DJIMotorEnable(motor_fire_r1);
	DJIMotorEnable(motor_fire_l2);
	DJIMotorEnable(motor_fire_r2);
	// DJIMotorEnable(motor_fire_l3);
	// DJIMotorEnable(motor_fire_r3);
	DJIMotorEnable(motor_yaw_ctrl);
	DJIMotorEnable(motor_yaw_recv);
	DJIMotorEnable(motor_reload);
	DJIMotorEnable(motor_push);
}

static void Reset_Reload(void)//state:1表示被动断电（设备掉线），0表示手段断电（右拨杆打最下）
{
	// 手动断电右拨杆向下时，将左拨杆下拨刷新reload电机0点
	if (rc_data[TEMP].remote_data.rc.switch_left == RC_SW_DOWN )
	{
		dart_ctrl_cmd.reload_target_angle = motor_reload->measure.total_angle;
		dart_ctrl_cmd.push_target_angle = motor_push->measure.total_angle;
		dart_ctrl_cmd.yaw_target_angle = motor_yaw_recv->measure.total_angle;
		dart_yaw_targetangle[DART_TARGET_BASE]=motor_yaw_recv->measure.total_angle;
		dart_yaw_targetangle[DART_TARGET_OUTPOST]=motor_yaw_recv->measure.total_angle;
	}
}

static void Send_Mode(void)
{
	// 摩擦轮电机
	DJIMotorSetRef(motor_fire_l1, dart_ctrl_cmd.fire_target_speed[0]);
	DJIMotorSetRef(motor_fire_r1, -dart_ctrl_cmd.fire_target_speed[0]);
	DJIMotorSetRef(motor_fire_l2, dart_ctrl_cmd.fire_target_speed[1]);
	DJIMotorSetRef(motor_fire_r2, -dart_ctrl_cmd.fire_target_speed[1]);
	// DJIMotorSetRef(motor_fire_l3, dart_ctrl_cmd.fire_target_speed[2];
	// DJIMotorSetRef(motor_fire_r3, dart_ctrl_cmd.fire_target_speed[2]);

	// yaw轴
	// 当发射模式，即右拨杆打上，只做2006角度闭环，如果打到中间就只做速度闭环
	if (rc_data[TEMP].remote_data.rc.switch_right == RC_SW_UP)
	{
		DJIMotorOuterLoop(motor_yaw_ctrl, ANGLE_LOOP);
		DJIMotorSetRef(motor_yaw_ctrl, dart_ctrl_cmd.yaw_target_angle);
	}
	else if (rc_data[TEMP].remote_data.rc.switch_right == RC_SW_MID)
	{
		DJIMotorOuterLoop(motor_yaw_ctrl, SPEED_LOOP);
		DJIMotorSetRef(motor_yaw_ctrl, dart_ctrl_cmd.yaw_target_speed);
	}
	
	// 换弹3508角度串速度闭环
	DJIMotorSetRef(motor_reload, dart_ctrl_cmd.reload_target_angle);
	
	// 推杆
	// 当fire_reset_flag不为0，即为要回到最底部，这时候用角度环，如果是在发射过程中，则为0，这时候用速度环推上去
	if (dart_ctrl_cmd.fire_reset_flag)
	{
		DJIMotorOuterLoop(motor_push, ANGLE_LOOP);
		DJIMotorSetRef(motor_push, dart_ctrl_cmd.push_target_angle);
	}
	else
	{
		//速度环
		DJIMotorOuterLoop(motor_push, SPEED_LOOP);
		DJIMotorSetRef(motor_push, dart_ctrl_cmd.push_target_speed);
	}
}

/*
	这里两连发都是采第一发打完转180度，第二发打完转90度，保证转盘的左右质量分布均匀，reload3508电机控制更稳定
*/
static void Fire_Mode(void)
{
	// 将其他模式标志位重置
	dart_ctrl_cmd.ready_savety_flag = 0;
	dart_ctrl_cmd.ready_finish_flag = 0;
	dart_ctrl_cmd.adjust_savety_flag = 0;
	
	//刚进入Push_Reset_Flag为0，表示没有结束推杆发射
	if (!dart_ctrl_cmd.fire_reset_flag)
	{
		// 如果能转动说明还没推到顶，重置定时器
		if (motor_push->measure.speed_aps > 50 || motor_push->measure.speed_aps < -50)
		{
			dart_ctrl_cmd.time_span_ms = 500.0f;
			dart_ctrl_cmd.time_start_ms = DWT_GetTimeline_ms();
		}
		// 如果推不动了且定时器响了说明已经卡了0.5s，这时候认为推到顶了
		else if (DWT_GetTimeline_ms() - dart_ctrl_cmd.time_start_ms > dart_ctrl_cmd.time_span_ms)
		{
			// 判断速度是推到顶还是底
			if (dart_ctrl_cmd.push_target_speed < 0)//目标速度小于0是推到顶
			{
				// 设置速度大于0，开始反转回退，并且重新启动定时器等待推杆收到底部
				dart_ctrl_cmd.push_target_speed = 4000;
				dart_ctrl_cmd.time_span_ms = 1500.0f;
				dart_ctrl_cmd.time_start_ms = DWT_GetTimeline_ms();
			}
			else// 目标速度大于0是收到底
			{
				// 这时候停转，并且Push_Reset_Flag置1，这时候推杆会重新回到锁定角度状态
				dart_ctrl_cmd.push_target_speed = 0;
				dart_ctrl_cmd.fire_reset_flag = 1;
			}
		}
	}
	else
	{
		// 如果打出了第一发，这时候Push_Num_Flag为0,
		if (!dart_ctrl_cmd.fire_num_flag)//如果是第一发
		{
			// 这时候先判断是否已经执行了推杆等待拨盘转动的延时
			if (!dart_ctrl_cmd.fire_delay_flag)//如果还没等待过
			{
				// 拨盘目标角度加180度，这用的是编码位，按理来说应该是8192*19/2=77824，但是实际测试就是得78700会比较准确转过180度
				dart_ctrl_cmd.reload_target_angle += 181.0f * 19.0f;
				// 将是否执行过等待的标志位置1
				dart_ctrl_cmd.fire_delay_flag = 1;
				// 然后设置定时器延时50*0.05=2.5s
				dart_ctrl_cmd.time_span_ms = 2500.0f;
				dart_ctrl_cmd.time_start_ms = DWT_GetTimeline_ms();
			}
			else if (DWT_GetTimeline_ms() - dart_ctrl_cmd.time_start_ms > dart_ctrl_cmd.time_span_ms)//如果定时器响了
			{
				// 重置Push的标志位，并重新设置推杆速度，往上推，设置定时器，这时候会执行第二次发射
				dart_ctrl_cmd.fire_delay_flag = 0;
				dart_ctrl_cmd.fire_num_flag = 1;//Push_Num_Flag置1
				dart_ctrl_cmd.fire_reset_flag = 0;
				dart_ctrl_cmd.push_target_speed = -4000;
				dart_ctrl_cmd.time_span_ms = 1500.0f;
				dart_ctrl_cmd.time_start_ms = DWT_GetTimeline_ms();
			}
		}
		else// 打完两发了
		{
			// 这时候先判断是否已经执行了推杆等待拨盘转动的延时
			if (!dart_ctrl_cmd.fire_delay_flag)//如果还没等待过
			{
				// 拨盘目标角度加90度，这用的是编码位，按理来说应该是8192*19/4=38912，但是实际测试就是得39350会比较准确转过180度
				dart_ctrl_cmd.reload_target_angle += 91.0f * 19.0f;
				// 将是否执行过等待的标志位置1
				dart_ctrl_cmd.fire_delay_flag = 1;
				// 然后设置定时器延时50*0.05=2.5s
				dart_ctrl_cmd.time_span_ms = 2500.0f;
				dart_ctrl_cmd.time_start_ms = DWT_GetTimeline_ms();
			}
			else if (DWT_GetTimeline_ms() - dart_ctrl_cmd.time_start_ms > dart_ctrl_cmd.time_span_ms)//如果定时器响了
			{
				// 重置Fire标志位，回到Fire模式
				dart_ctrl_cmd.ready_finish_flag = 1;
				dart_ctrl_cmd.ready_savety_flag = 1;
				dart_ctrl_cmd.fire_delay_flag = 0;
				dart_ctrl_cmd.fire_reset_flag = 1;
			}
		}
	}
}

static void Ready_Mode(void) // 发射准备，右边拨杆打到最上
{
	// 重置其他模式标志位
	dart_ctrl_cmd.fire_delay_flag = 0;
	dart_ctrl_cmd.fire_num_flag = 0;
	dart_ctrl_cmd.fire_reset_flag = 1;
	dart_ctrl_cmd.adjust_savety_flag = 0;
	
	//
	if (dart_ctrl_cmd.ready_savety_flag && dart_ctrl_cmd.check_locked_flag)
	{
		// 判断左拨杆
		switch (rc_data[TEMP].remote_data.rc.switch_left)
		{
			// 基地模式
			case RC_SW_UP:// 拨上，设置为基地角度
				dart_ctrl_cmd.fire_target_speed[0] = dart_fire_targetspeed1[DART_TARGET_BASE];
				dart_ctrl_cmd.fire_target_speed[1] = dart_fire_targetspeed2[DART_TARGET_BASE];
				// dart_ctrl_cmd.fire_target_speed[2] = dart_fire_targetspeed3[DART_TARGET_BASE];
				dart_ctrl_cmd.yaw_target_angle = dart_yaw_targetangle[DART_TARGET_BASE];
				break;
			// 前哨站模式
			case RC_SW_DOWN:// 拨下，记录当前为前哨站角度
				dart_ctrl_cmd.fire_target_speed[0] = dart_fire_targetspeed1[DART_TARGET_OUTPOST];
				dart_ctrl_cmd.fire_target_speed[1] = dart_fire_targetspeed2[DART_TARGET_OUTPOST];
				// dart_ctrl_cmd.fire_target_speed[2] = dart_fire_targetspeed3[DART_TARGET_OUTPOST];
				dart_ctrl_cmd.yaw_target_angle = dart_yaw_targetangle[DART_TARGET_OUTPOST];
				break;
			// 锁死模式
			case RC_SW_MID:// 中间，锁定发射机构
				dart_ctrl_cmd.fire_target_speed[0] = dart_fire_targetspeed1[DART_TARGET_NONE];
				dart_ctrl_cmd.fire_target_speed[1] = dart_fire_targetspeed2[DART_TARGET_NONE];
				// dart_ctrl_cmd.fire_target_speed[2] = dart_fire_targetspeed3[DART_TARGET_NONE];
//				dart_ctrl_cmd.yaw_target_angle = motor_yaw_recv->measure.total_angle;
				break;
			default:
				break;
		}
		// 当摇杆上推且不为锁死模式
		if (rc_data[TEMP].remote_data.rc.rocker_l1 >= 600 && rc_data[TEMP].remote_data.rc.switch_left != RC_SW_MID)
		{
			// 重置Fire模式标志位，设置Push模式标志位，然后就会进入Push模式了
			dart_ctrl_cmd.ready_finish_flag = 0;
			dart_ctrl_cmd.ready_savety_flag = 0;
			dart_ctrl_cmd.fire_num_flag = 0;
			dart_ctrl_cmd.fire_delay_flag = 0;
			dart_ctrl_cmd.fire_reset_flag = 0;
			// 推杆设置速度向上推，然后重置定时器
			dart_ctrl_cmd.push_target_speed = -4000;

			dart_ctrl_cmd.time_span_ms = 1500.0f;
			dart_ctrl_cmd.time_start_ms = DWT_GetTimeline_ms();
		}
	}
	else if (dart_ctrl_cmd.check_locked_flag)// 刚进入Fire模式并且各个锁角度的电机正常锁住就会一直进入这里判断，防止操作失误误发射
	{
		if (rc_data[TEMP].remote_data.rc.switch_left == RC_SW_MID && rc_data[TEMP].remote_data.rc.rocker_l1 <= 20)// 左拨杆打到中间并且此时没有推动左摇杆向前发射
		{
			dart_ctrl_cmd.ready_savety_flag = 1; // 发射安全检测完成置1
		}
		// 没有通过Fire安全判断锁死发射机构
		dart_ctrl_cmd.fire_target_speed[0] = dart_fire_targetspeed1[DART_TARGET_NONE];
		dart_ctrl_cmd.fire_target_speed[1] = dart_fire_targetspeed2[DART_TARGET_NONE];
		// dart_ctrl_cmd.fire_target_speed[2] = dart_fire_targetspeed3[DART_TARGET_NONE];
//		dart_ctrl_cmd.yaw_target_angle = motor_yaw_recv->measure.total_angle;
	}
	else// 如果连锁定都不正常直接锁死整个发射机构
	{
		dart_ctrl_cmd.fire_target_speed[0] = dart_fire_targetspeed1[DART_TARGET_NONE];
		dart_ctrl_cmd.fire_target_speed[1] = dart_fire_targetspeed2[DART_TARGET_NONE];
		// dart_ctrl_cmd.fire_target_speed[2] = dart_fire_targetspeed3[DART_TARGET_NONE];
//		dart_ctrl_cmd.yaw_target_angle = motor_yaw_recv->measure.total_angle;
	}
}

static void Adjust_Mode(void) // 姿态调整，右边拨杆打到中间
{
	// 重置其他模式标志位
	dart_ctrl_cmd.ready_finish_flag = 1;
	dart_ctrl_cmd.ready_savety_flag = 0;
	dart_ctrl_cmd.fire_delay_flag = 0;
	dart_ctrl_cmd.fire_num_flag = 0;
	dart_ctrl_cmd.fire_reset_flag = 1;
	
	// 锁死发射机构
	dart_ctrl_cmd.fire_target_speed[0] = dart_fire_targetspeed1[DART_TARGET_NONE];
	dart_ctrl_cmd.fire_target_speed[1] = dart_fire_targetspeed2[DART_TARGET_NONE];
	// dart_ctrl_cmd.fire_target_speed[2] = dart_fire_targetspeed3[DART_TARGET_NONE];
	dart_ctrl_cmd.yaw_target_angle = motor_yaw_recv->measure.total_angle;
	
//	if (dart_ctrl_cmd.adjust_savety_flag && dart_ctrl_cmd.check_locked_flag)// 如果通过了安全判断且所有电机锁定在规定角度范围内
	if (dart_ctrl_cmd.adjust_savety_flag)//检查check_locked_flag会出现问题
	{
		// 储存上次推杆状态
		static uint8_t last_rc_ch2 = 0;
		// 调整摩擦轮的目标速度大小存储变量
		int16_t d_speed = 0;
		if (rc_data[TEMP].remote_data.rc.switch_left != 3)// 中间是不能调速的
		{
			if(rc_data[TEMP].remote_data.rc.rocker_r1 >= 600 && last_rc_ch2 == 0)// 如果右摇杆推上，且上一次的推杆转态last_rc_ch2是0
			{
				// 这时候速度变化量为+50
				d_speed = 50;
				// 推杆转态last_rc_ch2置1
				last_rc_ch2 = 1;
				// 亮青色灯表示增加了50
				FlowRGBShow(LED_CYAN);
			}
			else if(rc_data[TEMP].remote_data.rc.rocker_r1 <= -600 && last_rc_ch2 == 0)// 如果右摇杆推下，且上一次的推杆转态last_rc_ch2是0
			{
				// 这时候速度变化量为-50
				d_speed = -50;
				// 推杆转态last_rc_ch2置1
				last_rc_ch2 = 1;
				// 亮紫色灯表示减少了50
				FlowRGBShow(LED_PURPLE);
			}
			else if (rc_data[TEMP].remote_data.rc.rocker_r1 >= -50 && rc_data[TEMP].remote_data.rc.rocker_r1 <= 50)
			{
				// 每次回到中间附近才置推杆转态last_rc_ch2为0
				last_rc_ch2 = 0;
				// 亮回绿色灯
				FlowRGBShow(LED_GREEN);
			}
		}
		
		// 判断左拨杆
		switch (rc_data[TEMP].remote_data.rc.switch_left)
		{
			case RC_SW_UP:// 拨上，记录当前为基地角度
				dart_ctrl_cmd.yaw_target_speed = 0;// 这时候不可调yaw角度
				dart_fire_targetspeed1[DART_TARGET_BASE] += d_speed;// 加上变更速度
				dart_fire_targetspeed2[DART_TARGET_BASE] += d_speed;// 加上变更速度
				// dart_fire_targetspeed3[DART_TARGET_BASE] += d_speed;// 加上变更速度
				dart_yaw_targetangle[DART_TARGET_BASE] = motor_yaw_recv->measure.total_angle;//把当前角度记录为基地角度
				break;
			case RC_SW_DOWN:// 拨下，记录当前为前哨站角度
				dart_ctrl_cmd.yaw_target_speed = 0;//这时候不可调yaw角度
				dart_fire_targetspeed1[DART_TARGET_OUTPOST] += d_speed;// 加上变更速度
				dart_fire_targetspeed2[DART_TARGET_OUTPOST] += d_speed;// 加上变更速度
				// dart_fire_targetspeed3[DART_TARGET_OUTPOST] += d_speed;// 加上变更速度
				dart_yaw_targetangle[DART_TARGET_OUTPOST] = motor_yaw_recv->measure.total_angle;// 把当前角度记录为前哨站角度
				break;
			case RC_SW_MID:// 拨中间，这时候能调整yaw轴角度和控制转盘转过90度
				// 此时只做yaw速度闭环，速度由摇杆决定
				dart_ctrl_cmd.yaw_target_speed = rc_data[TEMP].remote_data.rc.rocker_r_ * 10;
				if (rc_data[TEMP].remote_data.rc.rocker_l1 >= 600)// 如果左摇杆推上
				{
					dart_ctrl_cmd.reload_target_angle += 90.0f * 19.0f;
					dart_ctrl_cmd.adjust_savety_flag = 0;// 重置安全标志位
				}
				break;
			default:
				break;
		}
	}
	else// 刚进入Adjust模式就会一直进入这里判断
	{
		if (rc_data[TEMP].remote_data.rc.rocker_r1 >= -20 && rc_data[TEMP].remote_data.rc.rocker_r1 <= 20 &&
			rc_data[TEMP].remote_data.rc.rocker_r_ >= -20 && rc_data[TEMP].remote_data.rc.rocker_r_ <= 20 &&
			rc_data[TEMP].remote_data.rc.rocker_l1 >= -20 && rc_data[TEMP].remote_data.rc.rocker_l1 <= 20 &&
			rc_data[TEMP].remote_data.rc.switch_left == RC_SW_MID)// 只有当摇杆居中且打到中间过后，才能将安全判断标志位置1
		{
			dart_ctrl_cmd.adjust_savety_flag = 1;
		}
		// 没有通过安全判断之前yaw轴不可动
		dart_ctrl_cmd.yaw_target_speed = 0;
	}
}

static void Check_Mode(void)
{
	// 允许换弹距目标值正负大约4度偏差
	if (motor_reload->measure.total_angle - dart_ctrl_cmd.reload_target_angle >= 2.0f  || 
		motor_reload->measure.total_angle - dart_ctrl_cmd.reload_target_angle <= -2.0f )
	{
		dart_ctrl_cmd.check_locked_flag = 0;
		safe_state=RELAOD;
	}
	// 允许yaw距目标值正负1度编码位偏差
	else if (motor_yaw_recv->measure.total_angle - dart_ctrl_cmd.yaw_target_angle >= 0.5f ||
		     motor_yaw_recv->measure.total_angle - dart_ctrl_cmd.yaw_target_angle <= -0.5f)
	{
		dart_ctrl_cmd.check_locked_flag = 0;
		safe_state=YAW;
	}
	// 对于6020,不带减速箱，1编码位约为0.0012度，允许距目标值正负10000个编码位偏差，换算后大约为10度
	else if (motor_push->measure.total_angle - dart_ctrl_cmd.push_target_angle >= 5.0f  ||
		     motor_push->measure.total_angle - dart_ctrl_cmd.push_target_angle <= -5.0f )
	{
		safe_state=PUSH;
		dart_ctrl_cmd.check_locked_flag = 0;
	}
	else
	{
		safe_state=NONE;
		dart_ctrl_cmd.check_locked_flag = 1;
	}
}

// 裸机版本，仅供参考
// /*--PID预设参数--*/
// float fire_s1_pid[3] = {35.0f, 0.003f, 5.0f};	//三对摩擦轮用三组PID参数
// float fire_s2_pid[3] = {35.0f, 0.003f, 5.0f};
// float fire_s3_pid[3] = {35.0f, 0.003f, 5.0f};
						 
// float yaw_a_pid[3] = {30.0f, 0, 0.005f};		//yaw转轴6020角度环
// float yaw_s_pid[3] = {15.0f, 0, 5.0f};			//yaw丝杆2006速度环

// float reload_a_pid[3] = {0.1f, 0, 0};			//换弹3508角度环和速度环
// float reload_s_pid[3] = {40.0f, 0, 5.0f};

// float push_a_pid[3] = {0.07f, 0, 0.005f};		//推杆2006角度环和速度环
// float push_s_pid[3] = {10.0f, 0, 5.0f};

void DartInit()
{
    // 初始化时飞镖处于停止状态
    dart_status = ROBOT_STOP;

    // 四/六 个摩擦轮的参数一样,改tx_id和反转标志位即可 can1
    Motor_Init_Config_s fire_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1, // 4.5
                .Ki = 1,  // 0
                .Kd = 0.01,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    fire_motor_config.can_init_config.tx_id = 1;
    fire_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_fire_l1 = DJIMotorInit(&fire_motor_config);

    fire_motor_config.can_init_config.tx_id = 2;
    fire_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_fire_r1 = DJIMotorInit(&fire_motor_config);

    fire_motor_config.can_init_config.tx_id = 3;
    fire_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_fire_l2 = DJIMotorInit(&fire_motor_config);

    fire_motor_config.can_init_config.tx_id = 4;
    fire_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_fire_r2 = DJIMotorInit(&fire_motor_config);

    // fire_motor_config.can_init_config.tx_id = 5;
    // fire_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    // motor_fire_l3 = DJIMotorInit(&fire_motor_config);

    // fire_motor_config.can_init_config.tx_id = 6;
    // fire_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    // motor_fire_r3 = DJIMotorInit(&fire_motor_config);
    
	// yaw轴接收角度6020电机
    Motor_Init_Config_s yawrecv_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 6,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .MaxOut = 0,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
	};
    motor_yaw_recv = DJIMotorInit(&yawrecv_motor_config);
    // yaw轴控制角度2006电机
    Motor_Init_Config_s yawctrl_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 2000, // 10
                .Ki = 20,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 16000,
            },
            .speed_PID = {
                .Kp = 10,  // 50
                .Ki = 0, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 16000,
            },
            .other_angle_feedback_ptr = &motor_yaw_recv->measure.total_angle,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    motor_yaw_ctrl = DJIMotorInit(&yawctrl_motor_config);

	// 换弹3508电机
    Motor_Init_Config_s reload_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 20, // 10
                .Ki = 250,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1000,
                .MaxOut = 4000,
            },
            .speed_PID = {
                .Kp = 2,  // 50
                .Ki = 350, // 350
                .Kd = 0.01,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    motor_reload = DJIMotorInit(&reload_motor_config);

	// 发射2006电机
    Motor_Init_Config_s push_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 20, // 10
                .Ki = 250,
                .Kd = 0.01,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 2,  // 50
                .Ki = 0, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    motor_push = DJIMotorInit(&push_motor_config);

    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI
}

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void DartTask()
{
	jscope[0]=motor_yaw_recv->measure.total_angle;
	OnLineConfirm();
    if (dart_ctrl_cmd.dart_status == ROBOT_READY)
    {
		Enable_Power();
		Check_Mode();
		if (dart_ctrl_cmd.ready_finish_flag)
		{
			switch (rc_data[TEMP].remote_data.rc.switch_right)
			{
			case RC_SW_UP:
				Ready_Mode();
				break;
			case RC_SW_MID:
				Adjust_Mode();
				break;
			default:
				break;
			}
		}
		else
		{
			switch (rc_data[TEMP].remote_data.rc.switch_right)
			{
			case RC_SW_UP:
				Fire_Mode();
				break;
			case RC_SW_MID:
				Adjust_Mode();
				break;
			default:
				break;
			}
		}
		Send_Mode();
    }
	else
	{
		Disable_Power();
		Reset_Reload();
		dart_ctrl_cmd.yaw_target_angle = motor_yaw_recv->measure.total_angle;
		dart_fire_targetspeed1[DART_TARGET_BASE]=dart_ctrl_cmd.yaw_target_angle;
		dart_fire_targetspeed2[DART_TARGET_BASE]=dart_ctrl_cmd.yaw_target_angle;
	}
}
