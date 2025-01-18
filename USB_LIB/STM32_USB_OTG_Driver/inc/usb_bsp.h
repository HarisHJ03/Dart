/**
  ******************************************************************************
  * @file    usb_bsp.h
  * @author  ZCShou
  * @version V2.2.0
  * @date    2017.12.27
  * @brief   Specific api's relative to the used hardware platform
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 iESLab</center></h2>
  *
  *
  *
  * 参见ST例程
  * 
  * 
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_BSP__H__
#define __USB_BSP__H__

/* Includes ------------------------------------------------------------------*/
#include "usb_core.h"

/** @addtogroup USB_OTG_DRIVER
  * @{
  */
  
/** @defgroup USB_BSP
  * @brief This file is the 
  * @{
  */ 


/** @defgroup USB_BSP_Exported_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_BSP_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_BSP_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_BSP_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_BSP_Exported_FunctionsPrototype
  * @{
  */
void USB_OTG_BSP_Init (USB_OTG_CORE_HANDLE *pdev);
void USB_OTG_BSP_uDelay (const uint32_t usec);
void USB_OTG_BSP_mDelay (const uint32_t msec);
void USB_OTG_BSP_EnableInterrupt (USB_OTG_CORE_HANDLE *pdev);
/**
  * @}
  */ 

#endif /* __USB_BSP__H__ */

/**
  * @}
  */ 

/**
  * @}
  */ 
/************************ (C) COPYRIGHT iESLab *****END OF FILE****/

