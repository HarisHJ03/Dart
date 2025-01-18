#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"

#define DBUS_MAX_LEN 36
#define DBUS_BUFLEN 18

void USART3_DEVICE(void);/*DBUS*/

extern uint8_t dbus_buf[2][DBUS_MAX_LEN];

#endif
