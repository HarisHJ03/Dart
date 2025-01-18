#ifndef __BSP_USB_H__
#define __BSP_USB_H__

#include "stm32f4xx.h"
/**
 ******************************************************************************
 * @file    bsp_usb.h
 * @author  Liao Wenhao 
 * @version V1.0
 * @date    2023.05.16
 * @brief   Generic media access Layer.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_VCP_H
#define __USBD_CDC_VCP_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_core.h"
#include "usbd_conf.h"


/* Exported typef ------------------------------------------------------------*/
/* The following structures groups all needed parameters to be configured for the 
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
   typedef struct
{
  uint32_t bitrate;		/* 波特率 */
  uint8_t  format;		/* 格式 */
  uint8_t  paritytype;	/*校验类型：0无校验 1奇校验 2 偶校验 */
  uint8_t  datatype;	/*数据位：8 or 7 */
}LINE_CODING;

/* Exported constants --------------------------------------------------------*/
/* The following define is used to route the USART IRQ handler to be used.
   The IRQ handler function is implemented in the usbd_cdc_vcp.c file. */
#define DEFAULT_CONFIG					0
#define OTHER_CONFIG					1

/* Exported macro ------------------------------------------------------------*/
extern CDC_IF_Prop_TypeDef VCP_fops;
/* Exported functions ------------------------------------------------------- */
static uint16_t VCP_Init(void);
static uint16_t VCP_DeInit(void);
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t *Buf, uint32_t Len);
static uint16_t VCP_DataTx(uint8_t *Buf, uint32_t Len);
static uint16_t VCP_DataRx(uint8_t *Buf, uint32_t Len);
#endif /* __USBD_CDC_VCP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#endif

