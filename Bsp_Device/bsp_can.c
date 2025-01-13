#include "bsp_can.h"
#include "Timer.h"

CanRxMsg rx1_message;
CanRxMsg rx2_message;

moto_measure_t moto_fire[6];										 // 3508
moto_measure_t moto_yaw_2006, moto_yaw_6020, moto_reload, moto_push; // 2006 6020 3508 2006

void encoder_data_handler(moto_measure_t *ptr, CanRxMsg *message);
void get_moto_offset(moto_measure_t *ptr, CanRxMsg *message);

// canx通信ID处理函数
void STD_CAN_RxCpltCallback(CAN_TypeDef *_hcan, CanRxMsg *message)
{
	if (_hcan == CAN1)
	{
		static uint8_t i = 0;
		// 处理电机ID号
		i = message->StdId - 0x201;
		switch (message->StdId)
		{
		case CAN_RX_FIRE1_ID:
			err_list.fire[0] = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_fire[i], message);
			break;
		case CAN_RX_FIRE2_ID:
			err_list.fire[1] = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_fire[i], message);
			break;
		case CAN_RX_FIRE3_ID:
			err_list.fire[2] = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_fire[i], message);
			break;
		case CAN_RX_FIRE4_ID:
			err_list.fire[3] = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_fire[i], message);
			break;
		case CAN_RX_FIRE5_ID:
			err_list.fire[4] = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_fire[i], message);
			break;
		case CAN_RX_FIRE6_ID:
			err_list.fire[5] = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_fire[i], message);
			break;
		default:
			break;
		}
	}
	else if (_hcan == CAN2)
	{
		switch (message->StdId)
		{
		case CAN_RX_YAW_2006_ID:
			err_list.yaw_2006 = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_yaw_2006, message);
			break;
		case CAN_RX_RELOAD_ID:
			err_list.reload = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_reload, message);
			break;
		case CAN_RX_PUSH_ID:
			err_list.push = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_push, message);
			break;
		case CAN_RX_YAW_6020_ID:
			err_list.yaw_6020 = 20;
			// 处理电机数据宏函数
			encoder_data_handler(&moto_yaw_6020, message);
			break;
		default:
			break;
		}
	}
}

// can通信数据处理函数
void encoder_data_handler(moto_measure_t *ptr, CanRxMsg *message)
{
	ptr->last_ecd = ptr->ecd; // 角度
	ptr->ecd = (uint16_t)(message->Data[0] << 8 | message->Data[1]);

	if (ptr->ecd - ptr->last_ecd > 4096)
	{
		ptr->round_cnt--;
		ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
	}
	else if (ptr->ecd - ptr->last_ecd < -4096)
	{
		ptr->round_cnt++;
		ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
	}
	else
	{
		ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
	}

	ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;

	ptr->speed_rpm = (int16_t)(message->Data[2] << 8 | message->Data[3]);	  // 速度
	ptr->given_current = (int16_t)(message->Data[4] << 8 | message->Data[5]); // 转矩电流
}

// 角度值偏移量处理函数
void get_moto_offset(moto_measure_t *ptr, CanRxMsg *message)
{
	ptr->ecd = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
	ptr->offset_ecd = ptr->ecd;
}

// can1发送函数
void can_send_fire(int16_t *iq)
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_ID_STD;	  // 定义标识符的类型为标准标识符
	TxMessage.RTR = CAN_RTR_DATA; // 数据帧
	TxMessage.DLC = 0x08;		  // 数据长度为0x08

	TxMessage.StdId = CAN1_TX_GROUP1_ID; // 标准标识符
	TxMessage.Data[0] = iq[0] >> 8;
	TxMessage.Data[1] = iq[0];
	TxMessage.Data[2] = iq[1] >> 8;
	TxMessage.Data[3] = iq[1];
	TxMessage.Data[4] = iq[2] >> 8;
	TxMessage.Data[5] = iq[2];
	TxMessage.Data[6] = iq[3] >> 8;
	TxMessage.Data[7] = iq[3];

	CAN_Transmit(CAN1, &TxMessage);

	TxMessage.StdId = CAN1_TX_GROUP2_ID; // 标准标识符
	TxMessage.Data[0] = iq[4] >> 8;
	TxMessage.Data[1] = iq[4];
	TxMessage.Data[2] = iq[5] >> 8;
	TxMessage.Data[3] = iq[5];

	CAN_Transmit(CAN1, &TxMessage);
}

void can_send_botton(int16_t iq1, int16_t iq2, int16_t iq3)
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;

	TxMessage.StdId = CAN2_TX_GROUP1_ID; // 标准标识符
	TxMessage.Data[0] = iq1 >> 8;
	TxMessage.Data[1] = iq1;
	TxMessage.Data[2] = iq2 >> 8;
	TxMessage.Data[3] = iq2;
	TxMessage.Data[4] = iq3 >> 8;
	TxMessage.Data[5] = iq3;

	CAN_Transmit(CAN2, &TxMessage);
}

extern int Can1_Flag, Can2_Flag;
// can1中断
void CAN1_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		Can1_Flag = 1;

		// 接收CAN的信息，并将读到的信息存进rx1_message
		CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
		// 处理接收数据：具体是哪个电机的数据及数据的具体意义及大小
		STD_CAN_RxCpltCallback(CAN1, &rx1_message);

		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	}
}

// can2中断
void CAN2_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
	{
		Can2_Flag = 1;

		// 接收CAN的信息，并将读到的信息存进rx2_message
		CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
		// 处理接收数据：具体是哪个电机的数据及数据的具体意义及大小
		STD_CAN_RxCpltCallback(CAN2, &rx2_message);

		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	}
}
