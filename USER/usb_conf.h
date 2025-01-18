 /**
 * @file usb_conf.h
 * @author Liao Wenhao
 * @version 1.0
 * @date 2023-05
 * 
 * @copyright Copyright (c) 2023
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CONF__H__
#define __USB_CONF__H__

/* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"

/** @addtogroup USB_OTG_DRIVER
  * @{
  */
  
/** @defgroup USB_CONF
  * @brief USB low level driver configuration file
  * @{
  */ 

/** @defgroup USB_CONF_Exported_Defines
  * @{
  */

/*--------------------------- 主要是针对USB的Device模式，Host模式的配置项有所不同 -------------------------------*/
/* USB Core and PHY interface configuration.
   Tip: To avoid modifying these defines each time you need to change the USB
        configuration, you can declare the needed define in your toolchain
        compiler preprocessor.
   */

/*--------------------------- 第一部分：选择全速内核还是高速内核 -------------------------------*/
/* USB OTG core，即：STM32芯片内嵌的USB OTG 控制器。 STM32F105/07xx器件内嵌了一个USB OTG FS内核，而STM32F2xx和STM32F4xx器件内嵌了一个USB OTG FS内核和一个HS内核。*/
/****************** USB OTG FS PHY CONFIGURATION （全速内核配置） *******************************
*  The USB OTG FS Core supports one on-chip Full Speed PHY.
*  
*  The USE_EMBEDDED_PHY symbol is defined in the project compiler preprocessor 
*  when FS core is used.
*******************************************************************************/
#ifndef USE_USB_OTG_FS
	#define USE_USB_OTG_FS
#endif /* USE_USB_OTG_FS */

#ifndef USE_EMBEDDED_PHY 		/* 根据上面的注释，可以直接在编译工具链中定义该宏值 */
	#define USE_EMBEDDED_PHY
#endif

#ifdef USE_USB_OTG_FS 
	#define USB_OTG_FS_CORE
#endif

/****************** USB OTG HS PHY CONFIGURATION （高速内核配置）*******************************
*  The USB OTG HS Core supports two PHY interfaces:
*   (i)  An ULPI interface for the external High Speed PHY: the USB HS Core will 
*        operate in High speed mode 使用外置的 ULPI接口的 PHY，此时USB HS Core工作在高速模式
*   (ii) An on-chip Full Speed PHY: the USB HS Core will operate in Full speed mode 使用片上全速 PHY，此时USB HS Core工作在全速模式
*
*  You can select the PHY to be used using one of these two defines: 通过以下两个宏选择使用的PHY芯片
*   (i)  USE_ULPI_PHY: if the USB OTG HS Core is to be used in High speed mode 
*   (ii) USE_EMBEDDED_PHY: if the USB OTG HS Core is to be used in Full speed mode
*
*  Notes: 针对ST的评估板的注释说明无须理会
*   - The USE_ULPI_PHY symbol is defined in the project compiler preprocessor as 
*     default PHY when HS core is used.
*   - On STM322xG-EVAL and STM324xG-EVAL boards, only configuration(i) is available.
*     Configuration (ii) need a different hardware, for more details refer to your
*     STM32 device datasheet.
*******************************************************************************/
#ifndef USE_USB_OTG_HS
	/* #define USE_USB_OTG_HS */
#endif /* USE_USB_OTG_HS */

#ifndef USE_ULPI_PHY
	/* #define USE_ULPI_PHY */
#endif /* USE_ULPI_PHY */

#ifndef USE_EMBEDDED_PHY
	/* #define USE_EMBEDDED_PHY */
#endif /* USE_EMBEDDED_PHY */

#ifdef USE_USB_OTG_HS
	#define USB_OTG_HS_CORE
#endif

/*--------------------------- 第二部分：FIFO -------------------------------*/
/*******************************************************************************
*                      FIFO Size Configuration in Device mode
*  
*  (i) Receive data FIFO size = RAM for setup packets + 
*                   OUT endpoint control information +
*                   data OUT packets + miscellaneous
*      Space = ONE 32-bits words
*     --> RAM for setup packets = 10 spaces
*        (n is the nbr of CTRL EPs the device core supports) 
*     --> OUT EP CTRL info      = 1 space
*        (one space for status information written to the FIFO along with each 
*        received packet)
*     --> data OUT packets      = (Largest Packet Size / 4) + 1 spaces 
*        (MINIMUM to receive packets)
*     --> OR data OUT packets  = at least 2*(Largest Packet Size / 4) + 1 spaces 
*        (if high-bandwidth EP is enabled or multiple isochronous EPs)
*     --> miscellaneous = 1 space per OUT EP
*        (one space for transfer complete status information also pushed to the 
*        FIFO with each endpoint's last packet)
*
*  (ii)MINIMUM RAM space required for each IN EP Tx FIFO = MAX packet size for 
*       that particular IN EP. More space allocated in the IN EP Tx FIFO results
*       in a better performance on the USB and can hide latencies on the AHB.
*
*  (iii) TXn min size = 16 words. (n  : Transmit FIFO index)
*   (iv) When a TxFIFO is not used, the Configuration should be as follows: 
*       case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
*       --> Txm can use the space allocated for Txn.
*       case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
*       --> Txn should be configured with the minimum space of 16 words
*  (v) The FIFO is used optimally when used TxFIFOs are allocated in the top 
*       of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
*   (vi) In HS case12 FIFO locations should be reserved for internal DMA registers
*        so total FIFO size should be 1012 Only instead of 1024       
*******************************************************************************/
 
/****************** USB OTG HS CONFIGURATION （高速模式下的配置）**********************************/
#ifdef USB_OTG_HS_CORE
	#define RX_FIFO_HS_SIZE                          512
	#define TX0_FIFO_HS_SIZE                         128
	#define TX1_FIFO_HS_SIZE                         272
	#define TX2_FIFO_HS_SIZE                          0
	#define TX3_FIFO_HS_SIZE                          0
	#define TX4_FIFO_HS_SIZE                          0
	#define TX5_FIFO_HS_SIZE                          0

	/* #define USB_OTG_HS_LOW_PWR_MGMT_SUPPORT */
	/* #define USB_OTG_HS_SOF_OUTPUT_ENABLED */

	#ifdef USE_ULPI_PHY
	#define USB_OTG_ULPI_PHY_ENABLED
	#endif
	#ifdef USE_EMBEDDED_PHY
	#define USB_OTG_EMBEDDED_PHY_ENABLED
	#endif
	#define USB_OTG_HS_INTERNAL_DMA_ENABLED
	#define USB_OTG_HS_DEDICATED_EP1_ENABLED
#endif

/****************** USB OTG FS CONFIGURATION（全速模式下的配置） **********************************/
#ifdef USB_OTG_FS_CORE
	#define RX_FIFO_FS_SIZE							128
	#define TX0_FIFO_FS_SIZE						64
	#define TX1_FIFO_FS_SIZE						128
	#define TX2_FIFO_FS_SIZE						0
	#define TX3_FIFO_FS_SIZE						0
	#define TXH_NP_FS_FIFOSIZ						96
	#define TXH_P_FS_FIFOSIZ						96

	/* #define USB_OTG_FS_LOW_PWR_MGMT_SUPPORT */
	/* #define USB_OTG_FS_SOF_OUTPUT_ENABLED */
#endif

/****************** USB OTG MISC CONFIGURATION ********************************/
// #define VBUS_SENSING_ENABLED

/*--------------------------- 第三部分：Host模式还是Device模式 -------------------------------*/
/****************** USB OTG MODE CONFIGURATION ********************************/
/* #define USE_HOST_MODE */
#define USE_DEVICE_MODE
/* #define USE_OTG_MODE */

#ifndef USB_OTG_FS_CORE
	#ifndef USB_OTG_HS_CORE
		#error  "USB_OTG_HS_CORE or USB_OTG_FS_CORE should be defined"
	#endif
#endif

#ifndef USE_DEVICE_MODE
	#ifndef USE_HOST_MODE
		#error  "USE_DEVICE_MODE or USE_HOST_MODE should be defined"
	#endif
#endif

#ifndef USE_USB_OTG_HS
	#ifndef USE_USB_OTG_FS
		#error  "USE_USB_OTG_HS or USE_USB_OTG_FS should be defined"
	#endif
#else /* USE_USB_OTG_HS */
	#ifndef USE_ULPI_PHY
		#ifndef USE_EMBEDDED_PHY
			#error  "USE_ULPI_PHY or USE_EMBEDDED_PHY should be defined"
		#endif
	#endif
#endif

/*--------------------------- 第四部分：编译器兼容处理 -------------------------------*/
/****************** C Compilers dependant keywords ****************************/
/* In HS mode and when the DMA is used, all variables and data structures dealing
   with the DMA during the transaction process should be 4-bytes aligned */    
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined   (__GNUC__)        /* GNU Compiler */
    #define __ALIGN_END    __attribute__ ((aligned (4)))
    #define __ALIGN_BEGIN         
  #else                           
    #define __ALIGN_END
    #if defined   (__CC_ARM)      /* ARM Compiler */
      #define __ALIGN_BEGIN    __align(4)  
    #elif defined (__ICCARM__)    /* IAR Compiler */
      #define __ALIGN_BEGIN 
    #elif defined  (__TASKING__)  /* TASKING Compiler */
      #define __ALIGN_BEGIN    __align(4) 
    #endif /* __CC_ARM */  
  #endif /* __GNUC__ */ 
#else
  #define __ALIGN_BEGIN
  #define __ALIGN_END   
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__CC_ARM)         /* ARM Compiler */
  #define __packed    __packed
#elif defined (__ICCARM__)     /* IAR Compiler */
  #define __packed    __packed
#elif defined   ( __GNUC__ )   /* GNU Compiler */                        
  #define __packed    __attribute__ ((__packed__))
#elif defined   (__TASKING__)  /* TASKING Compiler */
  #define __packed    __unaligned
#endif /* __CC_ARM */

/**
  * @}
  */ 


/** @defgroup USB_CONF_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_CONF_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 


#endif /* __USB_CONF__H__ */



