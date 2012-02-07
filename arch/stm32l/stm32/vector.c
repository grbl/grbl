/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stm32f10x.h>

#define WEAK __attribute__ ((weak))

/* Symbols exported by linker script */
extern unsigned _etext, _data, _edata, _ebss;

#ifndef STACK_SIZE
#define STACK_SIZE 1024
#endif

__attribute__ ((section(".stack")))
unsigned long stack[STACK_SIZE / sizeof(unsigned long)];

void main(void);
void reset_handler(void);
void blocking_handler(void);
void null_handler(void);
void WEAK nmi_handler(void);
void WEAK hard_fault_handler(void);
void WEAK mem_manage_handler(void);
void WEAK bus_fault_handler(void);
void WEAK usage_fault_handler(void);
extern void sv_call_handler();
#define sv_call_handler sv_call_handler
void WEAK debug_monitor_handler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void pend_sv_handler();
#define pend_sv_handler pend_sv_handler
extern void systick_handler();
#define sys_tick_handler systick_handler
void WEAK wwdg_isr(void);
void WEAK pvd_isr(void);
void WEAK tamper_isr(void);
void WEAK rtc_isr(void);
void WEAK flash_isr(void);
void WEAK rcc_isr(void);
void WEAK exti0_isr(void);
void WEAK exti1_isr(void);
void WEAK exti2_isr(void);
void WEAK exti3_isr(void);
void WEAK exti4_isr(void);
void WEAK dma1_channel1_isr(void);
void WEAK dma1_channel2_isr(void);
void WEAK dma1_channel3_isr(void);
void WEAK dma1_channel4_isr(void);
void WEAK dma1_channel5_isr(void);
void WEAK dma1_channel6_isr(void);
void WEAK dma1_channel7_isr(void);
void WEAK adc1_2_isr(void);
void WEAK usb_hp_can_tx_isr(void);
void WEAK usb_lp_can_rx0_isr(void);
void WEAK can_rx1_isr(void);
void WEAK can_sce_isr(void);
void WEAK exti9_5_isr(void);
void WEAK tim1_brk_isr(void);
void WEAK tim1_up_isr(void);
void WEAK tim1_trg_com_isr(void);
void WEAK tim1_cc_isr(void);
extern void tim2_isr();
void WEAK tim3_isr(void);
void WEAK tim4_isr(void);
void WEAK i2c1_ev_isr(void);
void WEAK i2c1_er_isr(void);
void WEAK i2c2_ev_isr(void);
void WEAK i2c2_er_isr(void);
void WEAK spi1_isr(void);
void WEAK spi2_isr(void);
void WEAK usart1_isr(void);
void WEAK usart2_isr(void);
void WEAK usart3_isr(void);
void WEAK exti15_10_isr(void);
void WEAK rtc_alarm_isr(void);
void WEAK usb_wakeup_isr(void);
void WEAK tim8_brk_isr(void);
void WEAK tim8_up_isr(void);
void WEAK tim8_trg_com_isr(void);
void WEAK tim8_cc_isr(void);
void WEAK adc3_isr(void);
void WEAK fsmc_isr(void);
void WEAK sdio_isr(void);
void WEAK tim5_isr(void);
void WEAK spi3_isr(void);
void WEAK usart4_isr(void);
void WEAK usart5_isr(void);
void WEAK tim6_isr(void);
void WEAK tim7_isr(void);
void WEAK dma2_channel1_isr(void);
void WEAK dma2_channel2_isr(void);
void WEAK dma2_channel3_isr(void);
void WEAK dma2_channel4_5_isr(void);

__attribute__ ((section(".vectors")))
void (*const vector_table[]) (void) =
{
	(void *)((unsigned long)stack + sizeof(stack)), reset_handler, nmi_handler, hard_fault_handler, mem_manage_handler, bus_fault_handler, usage_fault_handler, 0, 0, 0, 0,	/* Reserved */
	    sv_call_handler, debug_monitor_handler, 0,	/* Reserved */
	    pend_sv_handler,
	    sys_tick_handler,
	    wwdg_isr,
	    pvd_isr,
	    tamper_isr,
	    rtc_isr,
	    flash_isr,
	    rcc_isr,
	    exti0_isr,
	    exti1_isr,
	    exti2_isr,
	    exti3_isr,
	    exti4_isr,
	    dma1_channel1_isr,
	    dma1_channel2_isr,
	    dma1_channel3_isr,
	    dma1_channel4_isr,
	    dma1_channel5_isr,
	    dma1_channel6_isr,
	    dma1_channel7_isr,
	    adc1_2_isr,
	    usb_hp_can_tx_isr,
	    usb_lp_can_rx0_isr,
	    can_rx1_isr,
	    can_sce_isr,
	    exti9_5_isr,
	    tim1_brk_isr,
	    tim1_up_isr,
	    tim1_trg_com_isr,
	    tim1_cc_isr,
	    tim2_isr,
	    tim3_isr,
	    tim4_isr,
	    i2c1_ev_isr,
	    i2c1_er_isr,
	    i2c2_ev_isr,
	    i2c2_er_isr,
	    spi1_isr,
	    spi2_isr,
	    usart1_isr,
	    usart2_isr,
	    usart3_isr,
	    exti15_10_isr,
	    rtc_alarm_isr,
	    usb_wakeup_isr,
	    tim8_brk_isr,
	    tim8_up_isr,
	    tim8_trg_com_isr,
	    tim8_cc_isr,
	    adc3_isr,
	    fsmc_isr,
	    sdio_isr,
	    tim5_isr,
	    spi3_isr,
	    usart4_isr,
	    usart5_isr,
	    tim6_isr,
	    tim7_isr,
	    dma2_channel1_isr,
	    dma2_channel2_isr, dma2_channel3_isr, dma2_channel4_5_isr,};

void reset_handler(void)
{
	volatile unsigned *src, *dest;
 asm("MSR msp, %0": :"r"(vector_table[0]));

#ifdef DEBUG
	SCB->SHCSR |= 0x00070000;
	/* Enable usage, bus and memory faults */
#endif

	for (src = &_etext, dest = &_data; dest < &_edata; src++, dest++)
		*dest = *src;

	while (dest < &_ebss)
		*dest++ = 0;

	/* Call the application's entry point. */
	SystemInit();
	main();
}

void blocking_handler(void)
{
	while (1) ;
}

void null_handler(void)
{
	/* Do nothing. */
}

#pragma weak nmi_handler = null_handler
#pragma weak hard_fault_handler = blocking_handler
#pragma weak mem_manage_handler = blocking_handler
#pragma weak bus_fault_handler = blocking_handler
#pragma weak usage_fault_handler = blocking_handler
#pragma weak sv_call_handler = null_handler
#pragma weak debug_monitor_handler = null_handler
#pragma weak pend_sv_handler = null_handler
#pragma weak sys_tick_handler = null_handler
#pragma weak wwdg_isr = null_handler
#pragma weak pvd_isr = null_handler
#pragma weak tamper_isr = null_handler
#pragma weak rtc_isr = null_handler
#pragma weak flash_isr = null_handler
#pragma weak rcc_isr = null_handler
#pragma weak exti0_isr = null_handler
#pragma weak exti1_isr = null_handler
#pragma weak exti2_isr = null_handler
#pragma weak exti3_isr = null_handler
#pragma weak exti4_isr = null_handler
#pragma weak dma1_channel1_isr = null_handler
#pragma weak dma1_channel2_isr = null_handler
#pragma weak dma1_channel3_isr = null_handler
#pragma weak dma1_channel4_isr = null_handler
#pragma weak dma1_channel5_isr = null_handler
#pragma weak dma1_channel6_isr = null_handler
#pragma weak dma1_channel7_isr = null_handler
#pragma weak adc1_2_isr = null_handler
#pragma weak usb_hp_can_tx_isr = null_handler
#pragma weak usb_lp_can_rx0_isr = null_handler
#pragma weak can_rx1_isr = null_handler
#pragma weak can_sce_isr = null_handler
#pragma weak exti9_5_isr = null_handler
#pragma weak tim1_brk_isr = null_handler
#pragma weak tim1_up_isr = null_handler
#pragma weak tim1_trg_com_isr = null_handler
#pragma weak tim1_cc_isr = null_handler
#pragma weak tim2_isr = null_handler
#pragma weak tim3_isr = null_handler
#pragma weak tim4_isr = null_handler
#pragma weak i2c1_ev_isr = null_handler
#pragma weak i2c1_er_isr = null_handler
#pragma weak i2c2_ev_isr = null_handler
#pragma weak i2c2_er_isr = null_handler
#pragma weak spi1_isr = null_handler
#pragma weak spi2_isr = null_handler
#pragma weak usart1_isr = null_handler
#pragma weak usart2_isr = null_handler
#pragma weak usart3_isr = null_handler
#pragma weak exti15_10_isr = null_handler
#pragma weak rtc_alarm_isr = null_handler
#pragma weak usb_wakeup_isr = null_handler
#pragma weak tim8_brk_isr = null_handler
#pragma weak tim8_up_isr = null_handler
#pragma weak tim8_trg_com_isr = null_handler
#pragma weak tim8_cc_isr = null_handler
#pragma weak adc3_isr = null_handler
#pragma weak fsmc_isr = null_handler
#pragma weak sdio_isr = null_handler
#pragma weak tim5_isr = null_handler
#pragma weak spi3_isr = null_handler
#pragma weak usart4_isr = null_handler
#pragma weak usart5_isr = null_handler
#pragma weak tim6_isr = null_handler
#pragma weak tim7_isr = null_handler
#pragma weak dma2_channel1_isr = null_handler
#pragma weak dma2_channel2_isr = null_handler
#pragma weak dma2_channel3_isr = null_handler
#pragma weak dma2_channel4_5_isr = null_handler
