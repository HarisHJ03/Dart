/**
 * @file usbd_conf.h
 * @author Liao Wenhao
 * @version 1.0
 * @date 2023-05
 * 
 * @copyright Copyright (c) 2023
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_CONF
  * @brief This file is the device library configuration file
  * @{
  */ 

/** @defgroup USBD_CONF_Exported_Defines
  * @{
  */ 


#define USBD_CFG_MAX_NUM					2			/* 配置描述最大个数 */
#define USBD_ITF_MAX_NUM					2			/* 接口描述符最大个数 */
#define USB_MAX_STR_DESC_SIZ				64			/* 字符串描述符最大长度 */

#define USBD_SELF_POWERED								/* 设备自供电 */

/** @defgroup USB_VCP_Class_Layer_Parameter
  * @{
  */ 
#define CDC_IN_EP							0x81	/* EP1 for data IN */
#define CDC_OUT_EP							0x01	/* EP1 for data OUT */
#define CDC_CMD_EP							0x82	/* EP2 for CDC commands */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#ifdef USE_USB_OTG_HS
	#define CDC_DATA_MAX_PACKET_SIZE		512		/* Endpoint IN & OUT Packet size */
	#define CDC_CMD_PACKET_SZE				8		/* Control Endpoint Packet size */

	#define CDC_IN_FRAME_INTERVAL			40		/* Number of micro-frames between IN transfers */
	#define APP_RX_DATA_SIZE				2048	/* Total size of IN buffer: APP_RX_DATA_SIZE*8/MAX_BAUDARATE*1000 should be > CDC_IN_FRAME_INTERVAL*8 */
#else
	#define CDC_DATA_MAX_PACKET_SIZE		64		/* 端点IN和OUT数据包大小 实际驱动内部会将 CDC_DATA_IN_PACKET_SIZE 和 CDC_DATA_OUT_PACKET_SIZE 定义为 CDC_DATA_MAX_PACKET_SIZE */
	#define CDC_CMD_PACKET_SZE				8		/* 控制端点包大小 */

	#define CDC_IN_FRAME_INTERVAL			5		/* IN包之间的时间间隔 */
	#define APP_RX_DATA_SIZE				2048	/* 用于IN数据传输的临时循环缓冲区的大小。Total size of IN buffer: APP_RX_DATA_SIZE*8/MAX_BAUDARATE*1000 应该要大于 CDC_IN_FRAME_INTERVAL */
#endif /* USE_USB_OTG_HS */

#define APP_FOPS							VCP_fops

/**
  * @}
  */ 


/**
  * @}
  */ 


#endif //__USBD_CONF__H__

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

