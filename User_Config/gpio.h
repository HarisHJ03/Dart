#ifndef __GPIO_H
#define __GPIO_H

typedef enum
{
	BLACK,
	BLUE,
	GREEN,
	CYAN,
	RED,
	PURPLE,
	YELLOW,
	WHITE
}LED_COLOR;

void GPIO_INIT(void);
void GPIO_TURNLED(LED_COLOR color);

#endif
