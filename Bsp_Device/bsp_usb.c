#include "bsp_usb.h"
#include "usb_dcd_int.h"

/**
 ******************************************************************************
 * @file    bsp_usb.c
 * @author  Liao Wenhao
 * @version V1.0
 * @date    2023.05.16
 * @brief   Generic media access Layer.
 ******************************************************************************
 */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#pragma data_alignment = 4
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* Includes ------------------------------------------------------------------*/
// #include "usbd_cdc_vcp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LINE_CODING linecoding =
	{
		115200, /* baud rate*/
		0x00,	/* stop bits-1*/
		0x00,	/* parity - none*/
		0x08	/* nb. of bits 8*/
};
/* These are external variables imported from CDC core to be used for IN
   transfer management. */
extern uint8_t APP_Rx_Buffer[]; /* Write CDC received data in this buffer.
								   These data will be sent over USB IN endpoint
								   in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;	/* Increment this pointer or roll it back to
								   start address when writing received data
								   in the buffer APP_Rx_Buffer. */

/* Private function prototypes -----------------------------------------------*/
static uint16_t VCP_Init(void);
static uint16_t VCP_DeInit(void);
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t *Buf, uint32_t Len);
static uint16_t VCP_DataTx(uint8_t *Buf, uint32_t Len);
static uint16_t VCP_DataRx(uint8_t *Buf, uint32_t Len);

//static uint16_t VCP_COMConfig(uint8_t Conf);

CDC_IF_Prop_TypeDef VCP_fops =
	{
		VCP_Init,
		VCP_DeInit,
		VCP_Ctrl,
		VCP_DataTx,
		VCP_DataRx};

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  VCP_Init
 *         Initializes the Media on the STM32
 * @param  None
 * @retval Result of the operation (USBD_OK in all cases)
 */
static uint16_t VCP_Init(void)
{
	return USBD_OK;
}

/**
 * @brief  VCP_DeInit
 *         DeInitializes the Media on the STM32
 * @param  None
 * @retval Result of the operation (USBD_OK in all cases)
 */
static uint16_t VCP_DeInit(void)
{
	return USBD_OK;
}

/**
 * @brief  VCP_Ctrl
 *         Manage the CDC class requests
 * @param  Cmd: Command code
 * @param  Buf: Buffer containing command data (request parameters)
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the operation (USBD_OK in all cases)
 */
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t *Buf, uint32_t Len)
{
	switch (Cmd)
	{
	case SEND_ENCAPSULATED_COMMAND:
		break;
	case GET_ENCAPSULATED_RESPONSE:
		break;
	case SET_COMM_FEATURE:
		break;
	case GET_COMM_FEATURE:
		break;
	case CLEAR_COMM_FEATURE:
		break;
	case SET_LINE_CODING:
		linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) |
										(Buf[2] << 16) | (Buf[3] << 24));
		linecoding.format = Buf[4];
		linecoding.paritytype = Buf[5];
		linecoding.datatype = Buf[6];

		break;
	case GET_LINE_CODING:
		Buf[0] = (uint8_t)(linecoding.bitrate);
		Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
		Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
		Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
		Buf[4] = linecoding.format;
		Buf[5] = linecoding.paritytype;
		Buf[6] = linecoding.datatype;
		break;
	case SET_CONTROL_LINE_STATE:
		break;
	case SEND_BREAK:
		break;
	default:
		break;
	}
	return USBD_OK;
}

/**
 * @brief  VCP_DataTx
 *         该函数将通信口收到的数据发送到USB的IN端点 ，即通过USB发送出去
 *
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else VCP_FAIL
 */
static uint16_t VCP_DataTx(uint8_t *Buf, uint32_t Len)
{
	uint32_t i;

	// Put the data into the buffer. It will be processed by the USB stack.
	for (i = 0; i < Len; i++)
	{
		// Add a byte to the buffer
		APP_Rx_Buffer[APP_Rx_ptr_in] = Buf[i];

		// Update the circular buffer index
		APP_Rx_ptr_in++;

		// Loop the index if necessary
		if (APP_Rx_ptr_in == APP_RX_DATA_SIZE)
		{
			APP_Rx_ptr_in = 0;
		}
	}
	return USBD_OK;
}
/**
 * @brief  VCP_DataRx
 *         此函数处理从USB Device的 OUT端点收取数据，将其发送到指定通信口
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         until exiting this function. If you exit this function before transfer
 *         is complete on CDC interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data received 接收到的数据指针
 * @param  Len: Number of data received (in bytes) 接收到的数据长度
 * @retval Result of the operation: USBD_OK if all operations are OK else VCP_FAIL
 */
uint32_t USB_USART_REC_LEN = 64; // 定义USB_Usart接收数据长度
uint8_t USB_USART_RX_BUF[64];
uint32_t USB_USART_RX_STA = 0; // USB_Usart接收数据长度
static uint16_t VCP_DataRx(uint8_t *Buf, uint32_t Len)
{
	uint8_t i;
	USB_USART_RX_STA = Len;
	for (i = 0; i < Len; i++)
	{
		USB_USART_RX_BUF[i] = Buf[i];
	}
	return USBD_OK;
}

/**
 * @brief  VCP_COMConfig
 *         Configure the COM Port with default values or values received from host.
 * @param  Conf: can be DEFAULT_CONFIG to set the default configuration or OTHER_CONFIG
 *         to set a configuration received from the host.
 * @retval None.
 */
//static uint16_t VCP_COMConfig(uint8_t Conf)
//{
//	return USBD_OK;
//}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

extern USB_OTG_CORE_HANDLE USB_OTG_dev;

#ifdef USE_USB_OTG_FS
void OTG_FS_IRQHandler(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}
#endif
