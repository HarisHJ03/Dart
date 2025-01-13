#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"

typedef enum
{
	//CAN1
	//6个发射3502分两组
	CAN1_TX_GROUP1_ID = 0x200,
	CAN1_TX_GROUP2_ID = 0x1FF,
	
	//CAN2
	//yaw轴2006 换弹3508 推杆2006全部放1组
	CAN2_TX_GROUP1_ID = 0x200,
} CAN_TX_ID;

typedef enum
{
	//CAN1
	//6个发射3508
	CAN_RX_FIRE1_ID = 0x201,
	CAN_RX_FIRE2_ID = 0x202,
	CAN_RX_FIRE3_ID = 0x203,
	CAN_RX_FIRE4_ID = 0x204,
	CAN_RX_FIRE5_ID = 0x205,
	CAN_RX_FIRE6_ID = 0x206,
	
	//CAN2
	//yaw轴2006
	CAN_RX_YAW_2006_ID = 0x201,
	//换弹3508
	CAN_RX_RELOAD_ID = 0x202,
	//推杆2006
	CAN_RX_PUSH_ID = 0x203,
	//yaw轴6020
	CAN_RX_YAW_6020_ID = 0X20A
} CAN_RX_ID;

typedef struct
{
	uint16_t ecd;			//当前角度
	uint16_t last_ecd;	//上次角度	
	
	int16_t  speed_rpm;		//速度
	
	int16_t  given_current;		//转矩电流

	int32_t  round_cnt;			//计算圈数
	
	int32_t  total_ecd;			//总角度

	uint16_t offset_ecd;		//角度偏移量
	
	uint32_t msg_cnt;			//传输数据次数几计次

	int32_t  ecd_raw_rate;	//每次转过角度
} moto_measure_t;

void can_send_fire(int16_t* iq);
void can_send_botton(int16_t iq1, int16_t iq2, int16_t iq3);

#endif

