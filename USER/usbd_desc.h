/**
  ******************************************************************************
  * @file   usbd_desc.h
  * @author Liao Wenhao
  * @version 1.0
  * @date 2023-05
  * @brief   usbd_desc.c对应的头文件
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 ZCShou</center></h2>
  *
  * 该文件与usbd_desc.c配套使用，主要用来定义USB2.0规范中定义的各种描述符
  * USB驱动库允许用户来自定义设备的部分描述符：
  * 	(+) 设备描述符
  * 	(+) 设备限定描述符
  * 	(+) 字符串描述符
  * 		(+) 厂商代码字符串描述符
  * 		(+) 语言ID字符串描述符
  * 		(+) 产品ID字符串描述符
  * 		(+) 序列号字符串描述符
  * 对于某些描述符，例如：端点描述符，他们不能由用户定义，因此
  * 这些描述符不会提供给用户。
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __USB_DESC_H
#define __USB_DESC_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_req.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USB_DESC
  * @brief general defines for the usb device library file
  * @{
  */ 

/** @defgroup USB_DESC_Exported_Defines
  * @{
  */
/* 以下为USB2.0规范中规定的各种描述符的值，具体见USB2.0规范 */
#define USB_DEVICE_DESCRIPTOR_TYPE				0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE		0x02
#define USB_STRING_DESCRIPTOR_TYPE				0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE			0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE			0x05

/* 描述符大小 */
#define USB_SIZ_DEVICE_DESC						18				/* 设备描述符的大小，单位字节 */
#define USB_SIZ_STRING_LANGID					4

/* 用于生成USB设备号 这里有三个宏定义：DEVICE_ID1、DEVICE_ID2、DEVICE_ID3，这三个值（芯片中的地址，为只读寄存器）是STM32芯片产品唯一身份标识，可以用作USB字符序列号(96位)。详情参考STM32参考手册的设备电子签名章节。*/
#define DEVICE_ID1								(0x1FFF7A10)
#define DEVICE_ID2								(0x1FFF7A14)
#define DEVICE_ID3								(0x1FFF7A18)
/* USB设备号长度 */
#define USB_SIZ_STRING_SERIAL					0x1A
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Exported_TypesDefinitions
  * @{
  */
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_Variables
  * @{
  */ 
extern  uint8_t USBD_DeviceDesc  [USB_SIZ_DEVICE_DESC];
extern  uint8_t USBD_StrDesc[USB_MAX_STR_DESC_SIZ];
extern  uint8_t USBD_OtherSpeedCfgDesc[USB_LEN_CFG_DESC]; 
extern  uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC];
extern  uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID];
extern  USBD_DEVICE USR_desc; 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_FunctionsPrototype
  * @{
  */ 


uint8_t *     USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ManufacturerStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ProductStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length);

#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *     USBD_USR_USRStringDesc (uint8_t speed, uint8_t idx , uint16_t *length);  
#endif /* USB_SUPPORT_USER_STRING_DESC */  
  
/**
  * @}
  */ 

#endif /* __USBD_DESC_H */

/**
  * @}
  */ 

/**
* @}
*/ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
