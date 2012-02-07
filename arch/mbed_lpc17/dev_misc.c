/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "LPC17xx.h"
#include "gpio.h"
#include "uart.h"
#include "led.h"
#include "dev_misc.h"

void dev_enable_ints()
{
//	sei();
}

void dev_disable_ints()
{
//	cli();
}

void delay_ms(double time_ms) 
{
	vTaskDelay(time_ms/portTICK_RATE_MS);
}

void delay_us(double time_us)
{
//	_delay_us(time_us); not really used by now
}

void sleep_mode()
{

}

extern int grbl_main();

void grbl_task(void * args)
{
	grbl_main();	
	while(1);
}

/* Bit definitions. */
#define PCONP_PCGPIO    0x00008000
#define PLLFEED_FEED1   0x000000AA
#define PLLFEED_FEED2   0x00000055

static void SetupHardware( void );

extern void grbl_task( void *pvParameters );
extern void blink_task( void *pvParameters );

/*-----------------------------------------------------------*/

int main( void )
{
	SetupHardware();

	uart_init(STD_UART,128,128,0);
	uart_SetBaud(STD_UART,9600);	

    	xTaskCreate( grbl_task, ( signed char * ) "grbl", configMINIMAL_STACK_SIZE*7, ( void * ) NULL, tskIDLE_PRIORITY+1, NULL );

	xTaskCreate( blink_task, ( signed char * ) "blink", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+2, NULL );
	
	vTaskStartScheduler();

	for( ;; );
}	

void blink_task(void * param)
{
	const portTickType xDelay = 500/portTICK_RATE_MS;
	
	for(;;){
		led_toggle(0);
		led_toggle(1);
		vTaskDelay( xDelay );
	}
	while(1);	
}

void SetupHardware( void )
{
	/* Disable peripherals power. */
	LPC_SC->PCONP = 0;

	/* Enable GPIO power. */
	LPC_SC->PCONP = PCONP_PCGPIO;

	/* Disable TPIU. */
	LPC_PINCON->PINSEL10 = 0;

	if ( LPC_SC->PLL0STAT & ( 1 << 25 ) )
	{
		/* Enable PLL, disconnected. */
		LPC_SC->PLL0CON = 1;
		LPC_SC->PLL0FEED = PLLFEED_FEED1;
		LPC_SC->PLL0FEED = PLLFEED_FEED2;
	}
	
	/* Disable PLL, disconnected. */
	LPC_SC->PLL0CON = 0;
	LPC_SC->PLL0FEED = PLLFEED_FEED1;
	LPC_SC->PLL0FEED = PLLFEED_FEED2;
	    
	/* Enable main OSC. */
	LPC_SC->SCS |= 0x20;
	while( !( LPC_SC->SCS & 0x40 ) );
	
	/* select main OSC, 12MHz, as the PLL clock source. */
	LPC_SC->CLKSRCSEL = 0x1;
	
	LPC_SC->PLL0CFG = 0x20031;
	LPC_SC->PLL0FEED = PLLFEED_FEED1;
	LPC_SC->PLL0FEED = PLLFEED_FEED2;
	      
	/* Enable PLL, disconnected. */
	LPC_SC->PLL0CON = 1;
	LPC_SC->PLL0FEED = PLLFEED_FEED1;
	LPC_SC->PLL0FEED = PLLFEED_FEED2;
	
	/* Set clock divider. */
	LPC_SC->CCLKCFG = 0x03;
	
	/* Configure flash accelerator. */
	LPC_SC->FLASHCFG = 0x403a;
	
	/* Check lock bit status. */
	while( ( ( LPC_SC->PLL0STAT & ( 1 << 26 ) ) == 0 ) );
	    
	/* Enable and connect. */
	LPC_SC->PLL0CON = 3;
	LPC_SC->PLL0FEED = PLLFEED_FEED1;
	LPC_SC->PLL0FEED = PLLFEED_FEED2;
	while( ( ( LPC_SC->PLL0STAT & ( 1 << 25 ) ) == 0 ) );

	
	/* Configure the clock for the USB. */
	  
	if( LPC_SC->PLL1STAT & ( 1 << 9 ) )
	{
		/* Enable PLL, disconnected. */
		LPC_SC->PLL1CON = 1;
		LPC_SC->PLL1FEED = PLLFEED_FEED1;
		LPC_SC->PLL1FEED = PLLFEED_FEED2;
	}
	
	/* Disable PLL, disconnected. */
	LPC_SC->PLL1CON = 0;
	LPC_SC->PLL1FEED = PLLFEED_FEED1;
	LPC_SC->PLL1FEED = PLLFEED_FEED2;
	
	LPC_SC->PLL1CFG = 0x23;
	LPC_SC->PLL1FEED = PLLFEED_FEED1;
	LPC_SC->PLL1FEED = PLLFEED_FEED2;
	      
	/* Enable PLL, disconnected. */
	LPC_SC->PLL1CON = 1;
	LPC_SC->PLL1FEED = PLLFEED_FEED1;
	LPC_SC->PLL1FEED = PLLFEED_FEED2;
	while( ( ( LPC_SC->PLL1STAT & ( 1 << 10 ) ) == 0 ) );
	
	/* Enable and connect. */
	LPC_SC->PLL1CON = 3;
	LPC_SC->PLL1FEED = PLLFEED_FEED1;
	LPC_SC->PLL1FEED = PLLFEED_FEED2;
	while( ( ( LPC_SC->PLL1STAT & ( 1 << 9 ) ) == 0 ) );

	/*  Setup the peripheral bus to be the same as the PLL output (64 MHz). */
	LPC_SC->PCLKSEL0 = 0x05555555;

  // Connect UART0 pins
	LPC_PINCON->PINSEL0 |= 0x00000050;

  // Connect ethernet pins
	LPC_PINCON->PINSEL2 = 0x50150105;
	LPC_PINCON->PINSEL3 = ( LPC_PINCON->PINSEL3 & ~0x0000000F ) | 0x00000005;

	/* Configure the LEDs. */
	led_init();
}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will get called if a task overflows its stack. */

	( void ) pxTask;
	( void ) pcTaskName;
	uart_printf("overflow");

	for( ;; );
}

