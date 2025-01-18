/**
 ******************************************************************************
 * @file usbd_desc.c
 * @author Liao Wenhao
 * @version 1.0
 * @date 2023-05
 * @brief   USB Device 的各种描述符
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2018 ZCShou</center></h2>
 *
 * 该文件与usbd_desc.c配套使用，主要用来定义USB2.0规范中定义的各种描述�?
 * USB驱动库允许用户来自定义设备的部分描述符：
 * 	(+) 设备描述�?
 * 	(+) 设备限定描述�?
 * 	(+) 字符串描述符
 * 		(+) 厂商代码字符串描述符
 * 		(+) 语言ID字符串描述符
 * 		(+) 产品ID字符串描述符
 * 		(+) 序列号字符串描述�?
 * 对于某些描述符，例如：接口描述符、断点描述符，他们不能由用户定义，因�?
 * 这些描述符不会提供给用户�?
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_desc.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_DESC
 * @brief USBD descriptors module
 * @{
 */

/** @defgroup USBD_DESC_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_DESC_Private_Defines
 * @{
 */

/* 这两个ID需要向USB组织申请，不是免费的。当前该ID列表可以在以下网址查看http://www.linux-usb.org/usb.ids，通过查看可知0x0483是STMicroelectronics申请的，0x5740对应的是Virtual COM Port�?*/
#define USBD_VID 0x0483 /* 厂商ID */
#define USBD_PID 0x5740 /* 产品ID */

#define USBD_LANGID_STRING 0x409 /* 语言ID0x409指的是English(United States)，该值可以在USB_LANGIDs.pdf文档中找�? */

#define USBD_MANUFACTURER_STRING "Awakelion"
#define USBD_PRODUCT_HS_STRING "Virtual ComPort in HS mode"
#define USBD_PRODUCT_FS_STRING "Virtual ComPort in FS Mode"
#define USBD_CONFIGURATION_HS_STRING "VCP Config"
#define USBD_INTERFACE_HS_STRING "VCP Interface"
#define USBD_CONFIGURATION_FS_STRING "VCP Config"
#define USBD_INTERFACE_FS_STRING "VCP Interface"
/**
 * @}
 */

/** @defgroup USBD_DESC_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_DESC_Private_Variables
 * @{
 */

USBD_DEVICE USR_desc =
	{
		USBD_USR_DeviceDescriptor,
		USBD_USR_LangIDStrDescriptor,
		USBD_USR_ManufacturerStrDescriptor,
		USBD_USR_ProductStrDescriptor,
		USBD_USR_SerialStrDescriptor,
		USBD_USR_ConfigStrDescriptor,
		USBD_USR_InterfaceStrDescriptor,
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB标准设备描述符，具体结构见USB2.0规范 */
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END =
	{
		0x12,						/*bLength */
		USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
		0x00,						/*bcdUSB */
		0x02,
		0x00,				  /*bDeviceClass*/
		0x00,				  /*bDeviceSubClass*/
		0x00,				  /*bDeviceProtocol*/
		USB_OTG_MAX_EP0_SIZE, /*bMaxPacketSize*/
		LOBYTE(USBD_VID),	  /*idVendor*/
		HIBYTE(USBD_VID),	  /*idVendor*/
		LOBYTE(USBD_PID),	  /*idVendor*/
		HIBYTE(USBD_PID),	  /*idVendor*/
		0x00,				  /*bcdDevice rel. 2.00*/
		0x02,
		USBD_IDX_MFC_STR,	  /*Index of manufacturer  string*/
		USBD_IDX_PRODUCT_STR, /*Index of product string*/
		USBD_IDX_SERIAL_STR,  /*Index of serial number string*/
		USBD_CFG_MAX_NUM	  /*bNumConfigurations*/
};							  /* USB_DeviceDescriptor */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB 标准设备限定描述符，具体结构见USB2.0规范 */
__ALIGN_BEGIN uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
	{
		USB_LEN_DEV_QUALIFIER_DESC,
		USB_DESC_TYPE_DEVICE_QUALIFIER,
		0x00,
		0x02,
		0x00,
		0x00,
		0x00,
		0x40,
		0x01,
		0x00,
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* 以下均为 USB2.0规范规定的字符串描述符，其中前两个为独立处理的，后一个为通用的，在后面函数中赋�? */

/* 根据USB2.0规范，字符串描述符以UNICODE编码。由于这里只支持英文，因此这里bLength�?4，bDescriptorType�?3，LANGID�?0x409�?*/
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID] __ALIGN_END =
	{
		USB_SIZ_STRING_LANGID,
		USB_DESC_TYPE_STRING,
		LOBYTE(USBD_LANGID_STRING),
		HIBYTE(USBD_LANGID_STRING),
};

uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] =
	{
		USB_SIZ_STRING_SERIAL,
		USB_DESC_TYPE_STRING,
		/* 后续字节为空，在后面函数中填�? */
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* 这个为通用描述符，在后面函数中会填充其中的值，例如：厂商描述符、产品描述符等等*/
__ALIGN_BEGIN uint8_t USBD_StrDesc[USB_MAX_STR_DESC_SIZ] __ALIGN_END;

/**
 * @}
 */

/** @defgroup USBD_DESC_Private_FunctionPrototypes
 * @{
 */
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);
static void Get_SerialNum(void);
/**
 * @}
 */

/** @defgroup USBD_DESC_Private_Functions
 * @{
 */

/**
 * @brief  USBD_USR_DeviceDescriptor
 *         return the device descriptor
 * @param  speed : current device speed
 * @param  length : pointer to data length variable
 * @retval pointer to descriptor buffer
 */
uint8_t *USBD_USR_DeviceDescriptor(uint8_t speed, uint16_t *length)
{
	*length = sizeof(USBD_DeviceDesc);
	return (uint8_t *)USBD_DeviceDesc;
}

/**
 * @brief  USBD_USR_LangIDStrDescriptor
 *         return the LangID string descriptor
 * @param  speed : current device speed
 * @param  length : pointer to data length variable
 * @retval pointer to descriptor buffer
 */
uint8_t *USBD_USR_LangIDStrDescriptor(uint8_t speed, uint16_t *length)
{
	*length = sizeof(USBD_LangIDDesc);
	return (uint8_t *)USBD_LangIDDesc;
}

/**
 * @brief  USBD_USR_ProductStrDescriptor
 *         return the product string descriptor
 * @param  speed : current device speed
 * @param  length : pointer to data length variable
 * @retval pointer to descriptor buffer
 */
uint8_t *USBD_USR_ProductStrDescriptor(uint8_t speed, uint16_t *length)
{
	if (speed == 0)
	{
		USBD_GetString((uint8_t *)(uint8_t *)USBD_PRODUCT_HS_STRING, USBD_StrDesc, length);
	}
	else
	{
		USBD_GetString((uint8_t *)(uint8_t *)USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);
	}
	return USBD_StrDesc;
}

/**
 * @brief  USBD_USR_ManufacturerStrDescriptor
 *         return the manufacturer string descriptor
 * @param  speed : current device speed
 * @param  length : pointer to data length variable
 * @retval pointer to descriptor buffer
 */
uint8_t *USBD_USR_ManufacturerStrDescriptor(uint8_t speed, uint16_t *length)
{
	/* 这里是将ASCII编码的字符串转成UNICODE编码的字符串，同时UNICODE编码的字符串不是以NULL作为结束。USBD_GetString()方法定义在usbd_req.c文件�? */
	USBD_GetString((uint8_t *)(uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

/**
 * @brief  USBD_USR_SerialStrDescriptor
 *         return the serial number string descriptor
 * @param  speed : current device speed
 * @param  length : pointer to data length variable
 * @retval pointer to descriptor buffer
 */
uint8_t *USBD_USR_SerialStrDescriptor(uint8_t speed, uint16_t *length)
{
	*length = USB_SIZ_STRING_SERIAL;

	/* Update the serial number string descriptor with the data from the unique ID*/
	Get_SerialNum();

	return (uint8_t *)USBD_StringSerial;
}

/**
 * @brief  USBD_USR_ConfigStrDescriptor
 *         return the configuration string descriptor
 * @param  speed : current device speed
 * @param  length : pointer to data length variable
 * @retval pointer to descriptor buffer
 */
uint8_t *USBD_USR_ConfigStrDescriptor(uint8_t speed, uint16_t *length)
{
	if (speed == USB_OTG_SPEED_HIGH)
	{
		USBD_GetString((uint8_t *)(uint8_t *)USBD_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
	}
	else
	{
		USBD_GetString((uint8_t *)(uint8_t *)USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length);
	}
	return USBD_StrDesc;
}

/**
 * @brief  USBD_USR_InterfaceStrDescriptor
 *         return the interface string descriptor
 * @param  speed : current device speed
 * @param  length : pointer to data length variable
 * @retval pointer to descriptor buffer
 */
uint8_t *USBD_USR_InterfaceStrDescriptor(uint8_t speed, uint16_t *length)
{
	if (speed == 0)
	{
		USBD_GetString((uint8_t *)(uint8_t *)USBD_INTERFACE_HS_STRING, USBD_StrDesc, length);
	}
	else
	{
		USBD_GetString((uint8_t *)(uint8_t *)USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
	}
	return USBD_StrDesc;
}

/**
 * @brief  Create the serial number string descriptor
 * @param  None
 * @retval None
 */
static void Get_SerialNum(void)
{
	uint32_t deviceserial0, deviceserial1, deviceserial2;

	/* 这里取序列号只取�?48位，取了deviceserial0�?32位和deviceserial1的高16�? �?
		 序列号字符串描述符字节长度为0x1A = 12*2(序列号内�?) + 2(前面两个字节)*/
	deviceserial0 = *(uint32_t *)DEVICE_ID1;
	deviceserial1 = *(uint32_t *)DEVICE_ID2;
	deviceserial2 = *(uint32_t *)DEVICE_ID3;

	deviceserial0 += deviceserial2;

	if (deviceserial0 != 0)
	{
		IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8); /* 从第二个字节开始放，前两个字节�? 字符串描述的长度和类�?*/
		IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
	}
}

/**
 * @brief  Convert Hex 32Bits value into char
 * @param  value: value to convert
 * @param  pbuf: pointer to the buffer
 * @param  len: buffer length
 * @retval None
 */
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
	uint8_t idx = 0;

	for (idx = 0; idx < len; idx++)
	{
		if (((value >> 28)) < 0xA)
		{
			pbuf[2 * idx] = (value >> 28) + '0';
		}
		else
		{
			pbuf[2 * idx] = (value >> 28) + 'A' - 10;
		}

		value = value << 4;

		pbuf[2 * idx + 1] = 0;
	}
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
