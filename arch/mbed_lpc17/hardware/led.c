/*
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "led.h"

#define LED_FIO1_BITS			( LED_2 | LED_3 | LED_4 | LED_5 )
#define LED_NUM_LEDS			( 4 )

#define LED_2 ( 1UL << 18UL )
#define LED_3 ( 1UL << 20UL )
#define LED_4 ( 1UL << 23UL )
#define LED_5 ( 1UL << 21UL )

static unsigned long ulLEDs[] = { LED_3, LED_2, LED_5, LED_4 };

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

void led_init( void )
{
	/* LEDs on port 1. */
	LPC_GPIO1->FIODIR  = LED_FIO1_BITS;
	
	/* Start will all LEDs off. */
	LPC_GPIO1->FIOCLR = LED_FIO1_BITS;
}
/*-----------------------------------------------------------*/

void led_set( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue )
{
	if( uxLED < LED_NUM_LEDS )
	{
		/* Set of clear the output. */
		if( xValue )
		{
			LPC_GPIO1->FIOCLR = ulLEDs[ uxLED ];
		}
		else
		{
			LPC_GPIO1->FIOSET = ulLEDs[ uxLED ];
		}
	}
}
/*-----------------------------------------------------------*/

void led_toggle( unsigned portBASE_TYPE uxLED )
{
	if( uxLED < LED_NUM_LEDS )
	{
		if( LPC_GPIO1->FIOPIN & ulLEDs[ uxLED ] )
		{
			LPC_GPIO1->FIOCLR = ulLEDs[ uxLED ];
		}
		else
		{
			LPC_GPIO1->FIOSET = ulLEDs[ uxLED ];
		}
	}
}
/*-----------------------------------------------------------*/

unsigned portBASE_TYPE led_get( unsigned portBASE_TYPE uxLED )
{
	if( uxLED < LED_NUM_LEDS )
	{
		return ( LPC_GPIO1->FIOPIN & ulLEDs[ uxLED ] );
	}
	else
	{
		return 0;
	}
}
/*-----------------------------------------------------------*/







