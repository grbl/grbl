#include "serial.h"
#include "dev_misc.h"

#include <stm32f10x.h>
#include <stm32f10x_conf.h>

int __errno;
static int delay_ms_flag;

void setup_rcc();
void setup_gpio();
void setup_nvic();

#define LED_PORT GPIOC
#define LED_1 GPIO_Pin_8
#define LED_2 GPIO_Pin_9

void pend_sv_handler()
{
	return;
}

void sv_call_handler()
{
	return;
}

void systick_handler()
{
	return;
}

void dev_enable_ints()
{
//	sei();
}

void dev_disable_ints()
{
//	cli();
}

void toggle_led()
{
	static int led = 1;
	GPIO_WriteBit(LED_PORT, LED_1,led);
	GPIO_WriteBit(LED_PORT, LED_2,led);
	led ^= 1;
}

void delay_ms(double time_ms) 
{

	delay_ms_flag = 0;
  	TIM3->ARR =  time_ms;
	
	TIM_SetCounter(TIM3, 0);	
	TIM_Cmd(TIM3, ENABLE);
	
	while(!delay_ms_flag);

	TIM_Cmd(TIM3, DISABLE);
}

void delay_us(double time_us)
{
//	_delay_us(time_us); 
}

void sleep_mode()
{

}

void tim3_isr() __attribute__ ((optimize(0))); //might break if optimized
void tim3_isr()
{
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);	
	delay_ms_flag = 1;
}

extern int grbl_main();
int main() {
	setup_rcc();
	setup_gpio();
	setup_nvic();

	/* Setup delay timer. Configured to to 1ms per update*/
	TIM_TimeBaseInitTypeDef timer_settings;
	timer_settings.TIM_Period = 5;
	timer_settings.TIM_Prescaler = 24000-1;
	timer_settings.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_settings.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &timer_settings);
	/* Clear TIM2 update pending flag */
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);	
	/* TIM IT enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);

	grbl_main();	
	
	while(1);
	return 0;
}

// Configures the peripheral clocks
void setup_rcc(void)
{
	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP | 
				RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3,
			       ENABLE);

	/* Enable GPIOA and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
			       RCC_APB2Periph_GPIOC |
			       RCC_APB2Periph_USART1 |
			       RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

//Configures the different GPIO ports
void setup_gpio(void)
{
	GPIO_InitTypeDef gpio_init;

	/* Configure UART tx pin */
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	/* Configure UART rx pin */
	gpio_init.GPIO_Pin = GPIO_Pin_10;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	//config LED pin
	gpio_init.GPIO_Pin = LED_1 | LED_2;	
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_PORT, &gpio_init);
}

//Configure the nested vectored interrupt controller
void setup_nvic(void)
{
	NVIC_InitTypeDef nvic_init;

	/* Set the Vector Table base address as specified in .ld file */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	/* 4 bits for Interupt priorities so no sub priorities */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); //-->TODO

	/* Configure USART interrupt */
	nvic_init.NVIC_IRQChannel = USART1_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xf;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure EXTI interrupt 
	nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xc;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init); */

	/* Configure stepper interrupt */
	nvic_init.NVIC_IRQChannel = TIM2_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0x2;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure delay_ms interrupt */
	nvic_init.NVIC_IRQChannel = TIM3_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0x2;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
}

