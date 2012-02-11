#include <stdlib.h>
#include "LPC17xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"

static inline LPC_GPIO_TypeDef *block(int which)
{
	switch (which)
	{
    	case 0:
			return LPC_GPIO0;
		case 1:
			return LPC_GPIO1;
		case 2:
			return LPC_GPIO2;
		case 3:
			return LPC_GPIO3;
		case 4:
			return LPC_GPIO4;
		default:
			return NULL;
	}
}

void GPIO_SetDirection(int which, unsigned int in_bitmap, unsigned int out_bitmap)
{
	block(which)->FIODIR |= out_bitmap;
	block(which)->FIODIR &= ~(in_bitmap);
}

void GPIO_Write(int which, unsigned int set_bitmap, unsigned int clear_bitmap)
{
	block(which)->FIOSET = set_bitmap;
	block(which)->FIOCLR = clear_bitmap;
}

void GPIO_Toggle(int which, unsigned int bitmap)
{
	taskENTER_CRITICAL();
	{
		unsigned int oldval = GPIO_Read(which);
		GPIO_Write(which, ~oldval & bitmap, oldval & bitmap);
	}
	taskEXIT_CRITICAL();
}

unsigned int GPIO_Read(int which)
{
	return block(which)->FIOPIN;
}

