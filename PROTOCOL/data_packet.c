#include "data_packet.h"
#include "string.h"
//#include "judge_tx_data.h"
//#include "judge_rx_data.h"
#include "pc_tx_data.h"
#include "pc_rx_data.h"

/*******************************************************************/
/*裁判系统和小电脑的接收数据和打包*/
/*******************************************************************/

/*获取DMA状态数据*/
static void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
	*mem_id = DMA_GetCurrentMemoryTarget(dma_stream);

	*remain_cnt = DMA_GetCurrDataCounter(dma_stream);
}

// for debug
int dma_write_len = 0;
int fifo_overflow = 0;
/*
 * @ dma_obj ： judge_rx_obj  //初始化时已经把串口接收的数据缓冲区地址赋给了judge_rx_obj的成员buff
 */
void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type)
{
	int16_t tmp_len;
	uint8_t current_memory_id;
	uint16_t remain_data_counter;
	uint8_t *pdata = dma_obj->buff[0];

	get_dma_memory_msg(dma_obj->dma_stream, &current_memory_id, &remain_data_counter);

	if (UART_IDLE_IT == it_type)
	{
		if (current_memory_id)
		{
			dma_obj->write_index = dma_obj->buff_size * 2 - remain_data_counter;
		}
		else
		{
			dma_obj->write_index = dma_obj->buff_size - remain_data_counter;
		}
	}
	else if (UART_DMA_FULL_IT == it_type)
	{
#if 0
    if (current_memory_id)
    {
      dma_obj->write_index = dma_obj->buff_size;
    }
    else
    {
      dma_obj->write_index = dma_obj->buff_size*2;
    }
#endif
	}

	if (dma_obj->write_index < dma_obj->read_index)
	{
		dma_write_len = dma_obj->buff_size * 2 - dma_obj->read_index + dma_obj->write_index;

		tmp_len = dma_obj->buff_size * 2 - dma_obj->read_index;
		if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len)) // 把数据放进FIFO队列
			fifo_overflow = 1;
		else
			fifo_overflow = 0;
		dma_obj->read_index = 0;

		tmp_len = dma_obj->write_index;
		if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
			fifo_overflow = 1;
		else
			fifo_overflow = 0;
		dma_obj->read_index = dma_obj->write_index;
	}
	else
	{
		dma_write_len = dma_obj->write_index - dma_obj->read_index;

		tmp_len = dma_obj->write_index - dma_obj->read_index;
		if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
			fifo_overflow = 1;
		else
			fifo_overflow = 0;
		dma_obj->read_index = (dma_obj->write_index) % (dma_obj->buff_size * 2);
	}
}

static uint8_t *protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
	static uint8_t seq;

	uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
	frame_header_t *p_header = (frame_header_t *)tx_buf;

	p_header->sof = sof;		 // 帧头
	p_header->data_length = len; // 数据长度

	if (sof == UP_REG_ID)
	{
		if (seq++ >= 255)
			seq = 0;

		p_header->seq = seq;
	}
	else
	{
		p_header->seq = 0; // 包序号
	}

	memcpy(&tx_buf[HEADER_LEN], (uint8_t *)&cmd_id, CMD_LEN); // 复制cmd_id
	append_crc8_check_sum(tx_buf, HEADER_LEN);				  // 进行CRC8检验
	memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);		  // 复制数据内容
	append_crc16_check_sum(tx_buf, frame_length);			  // 进行CRC16检验

	return tx_buf; // 返回完整的数据帧
}

void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof)
{
	uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];
	static uint8_t a[119];

	uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN; // 发送那个数据包的总长度

	protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf); // 将数据进行打包，组成一个完整的数据帧
	memcpy(a, tx_buf, frame_length);						// 查看发送的完整数据，便于debug查看数据是否乱码

	/* use mutex operation */
	if (sof == UP_REG_ID)
	{
		fifo_s_puts(&pc_txdata_fifo, tx_buf, frame_length);
	}
	else if (sof == DN_REG_ID)
		fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length); // 将数据从tx_buf放到judge_txdata_fifo
	else
		return;
}

/*
 * @p_obj：judge_unpack_obj
 * @sof： DN_REG_ID
 */

void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof)
{
	uint8_t byte = 0;

	while (fifo_used_count(p_obj->data_fifo))
	{
		byte = fifo_s_get(p_obj->data_fifo);
		switch (p_obj->unpack_step)
		{
		case STEP_HEADER_SOF:
		{
			if (byte == sof)
			{
				p_obj->unpack_step = STEP_LENGTH_LOW;
				p_obj->protocol_packet[p_obj->index++] = byte;
			}
			else
			{
				p_obj->index = 0;
			}
		}
		break;

		case STEP_LENGTH_LOW:
		{
			p_obj->data_len = byte;
			p_obj->protocol_packet[p_obj->index++] = byte;
			p_obj->unpack_step = STEP_LENGTH_HIGH;
		}
		break;

		case STEP_LENGTH_HIGH:
		{
			p_obj->data_len |= (byte << 8);
			p_obj->protocol_packet[p_obj->index++] = byte;

			if (p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
			{
				p_obj->unpack_step = STEP_FRAME_SEQ;
			}
			else
			{
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;
			}
		}
		break;

		case STEP_FRAME_SEQ:
		{
			p_obj->protocol_packet[p_obj->index++] = byte;
			p_obj->unpack_step = STEP_HEADER_CRC8;
		}
		break;

		case STEP_HEADER_CRC8:
		{
			p_obj->protocol_packet[p_obj->index++] = byte;

			if (p_obj->index == HEADER_LEN)
			{
				if (verify_crc8_check_sum(p_obj->protocol_packet, HEADER_LEN))
				{
					p_obj->unpack_step = STEP_DATA_CRC16;
				}
				else
				{
					p_obj->unpack_step = STEP_HEADER_SOF;
					p_obj->index = 0;
				}
			}
		}
		break;

		case STEP_DATA_CRC16:
		{
			if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
			{
				p_obj->protocol_packet[p_obj->index++] = byte;
			}
			if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
			{
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;

				if (verify_crc16_check_sum(p_obj->protocol_packet, HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
				{
					if (sof == UP_REG_ID)
					{
						pc_data_handler(p_obj->protocol_packet);
					}
					else // DN_REG_ID
					{
						judgement_data_handler(p_obj->protocol_packet);
					}
				}
			}
		}
		break;

		default:
		{
			p_obj->unpack_step = STEP_HEADER_SOF;
			p_obj->index = 0;
		}
		break;
		}
	}
}

/************************************************************************/
/*裁判系统和小电脑的发送*/
/************************************************************************/

static void usart_send_data(USART_TypeDef *USARTx, uint8_t *pData, uint16_t Size)
{
	while (Size > 0U)
	{
		Size--;
		while ((USARTx->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
			; // 判断数据是否发送完成
		USARTx->DR = (*pData++ & (uint8_t)0xFF);
	}
}

#include "bsp_usb.h"
uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof)
{
#if (JUDGE_TX_FIFO_BUFLEN > PC_TX_FIFO_BUFLEN)
	uint8_t tx_buf[JUDGE_TX_FIFO_BUFLEN];
#else
	uint8_t tx_buf[PC_TX_FIFO_BUFLEN];
#endif

	uint32_t fifo_count = fifo_used_count(pfifo);

	if (fifo_count)
	{
		fifo_s_gets(pfifo, tx_buf, fifo_count); // 将数据转移到tx_buf

		if (sof == UP_REG_ID)
		{
#ifdef USE_USB_OTG_FS
			APP_FOPS.pIf_DataTx(tx_buf, fifo_count);
#else
			usart_send_data(USART1, tx_buf, fifo_count);
#endif
		}
		else if (sof == DN_REG_ID)
			usart_send_data(USART6, tx_buf, fifo_count); // 通过寄存器发送数据
		else
			return 0;
	}

	return fifo_count;
}
