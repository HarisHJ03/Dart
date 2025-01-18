#include "pc_rx_data.h"
//#include "dma.h"
#include "string.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//----------小电脑

/* pc system dma receive data object */
uart_dma_rxdata_t pc_rx_obj;
//----------解包任务
unpack_data_t pc_unpack_obj;

/* pc receive data fifo and buffer */
static SemaphoreHandle_t pc_rxdata_mutex;
fifo_s_t pc_rxdata_fifo;
static uint8_t pc_rxdata_buf[PC_RX_FIFO_BUFLEN];

void pc_rx_param_init(void)
{
#ifdef USE_USB_OTG_FS
	/* create the pc_rxdata_mutex mutex  */
	pc_rxdata_mutex = xSemaphoreCreateMutex();
	/* pc data fifo init */
	fifo_s_init(&pc_rxdata_fifo, pc_rxdata_buf, PC_RX_FIFO_BUFLEN, pc_rxdata_mutex); // 添加互斥量
	/* initial pc data unpack object */
	pc_unpack_obj.data_fifo = &pc_rxdata_fifo;
	pc_unpack_obj.p_header = (frame_header_t *)pc_unpack_obj.protocol_packet;
	pc_unpack_obj.index = 0;
	pc_unpack_obj.data_len = 0;
	pc_unpack_obj.unpack_step = STEP_HEADER_SOF;
#else
	/* create the pc_rxdata_mutex mutex  */
	pc_rxdata_mutex = xSemaphoreCreateMutex();
	/* pc data fifo init */
	fifo_s_init(&pc_rxdata_fifo, pc_rxdata_buf, PC_RX_FIFO_BUFLEN, pc_rxdata_mutex); // 添加互斥量
	/* initial pc data dma receiver object */
	pc_rx_obj.dma_stream = DMA2_Stream5;
	pc_rx_obj.data_fifo = &pc_rxdata_fifo;
	pc_rx_obj.buff_size = PC_RX_FIFO_BUFLEN;
	pc_rx_obj.buff[0] = pc_rxbuf[0];
	pc_rx_obj.buff[1] = pc_rxbuf[1];
	/* initial pc data unpack object */
	pc_unpack_obj.data_fifo = &pc_rxdata_fifo;
	pc_unpack_obj.p_header = (frame_header_t *)pc_unpack_obj.protocol_packet;
	pc_unpack_obj.index = 0;
	pc_unpack_obj.data_len = 0;
	pc_unpack_obj.unpack_step = STEP_HEADER_SOF;
#endif
}

robot_rx_data pc_recv_mesg; // 接收有pc发送的目标位置信息

#define pack_monitor 0
#if pack_monitor
int pc_seq = 0;
int once_lost_num = 0;
int lost_pack_percent = 0;

int pack_num_cnt = 0;
int lost_num_sum_t = 0;
int pack_lost = 0;
#endif
void pc_data_handler(uint8_t *p_frame)
{
	frame_header_t *p_header = (frame_header_t *)p_frame;
	memcpy(p_header, p_frame, HEADER_LEN);

	uint16_t data_length = p_header->data_length;
	uint16_t cmd_id = *(uint16_t *)(p_frame + HEADER_LEN);
	uint8_t *data_addr = p_frame + HEADER_LEN + CMD_LEN;

#if pack_monitor
	// lost pack monitor
	{
		pack_num_cnt++; // 包数量加1

		if (pack_num_cnt <= 100)
		{
			once_lost_num = p_header->seq - pc_seq - 1; // 检测丢包数量

			if (once_lost_num < 0)
			{
				once_lost_num += 256;
			}

			lost_num_sum_t += once_lost_num;
		}
		else
		{
			lost_pack_percent = lost_num_sum_t;
			lost_num_sum_t = 0;
			pack_num_cnt = 0;
		}

		if (once_lost_num != 0)
		{
			if pack_lost
				= 1; // 表示在一百帧数据内有丢包
		}
		else
		{
			pack_lost = 0;
		}

		pc_seq = p_header->seq;
	}
// end lost pack monitor
#endif
	taskENTER_CRITICAL();

	switch (cmd_id)
	{
	case GIMBAL_CTRL_ID:
	{
		memcpy(&pc_recv_mesg, data_addr, data_length);
	}
	break;
	}

	taskEXIT_CRITICAL();
}
