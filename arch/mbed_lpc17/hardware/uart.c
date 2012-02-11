#include "uart.h"
#include <math.h>

#define NR_OF_UARTS 4

// UART register bits
#define UART_LCR_DLAB		(0x80)
#define UART_LCR_NOPAR		(0x00)
#define UART_LCR_1STOP		(0x00)
#define UART_LCR_8BITS		(0x03)
#define UART_IER_EI		(0x07)
#define UART_FCR_EN		(0x01)
#define UART_FCR_CLR		(0x06)
#define UART_FCR_DMA		(1UL<<3)
#define UART_LSR_TEMT		(0x40)

// datasheet : "UARTs have 16 B Receive and Transmit FIFOs."

struct FractionalBaudEntry {
	float FRest;
	uint8_t divAddVal, mulVal;
};

struct uart_device{
	LPC_UART_TypeDef *base;

	void * rx_dma;
//	DmaP2M<char> *m_RxDMA;
//	DmaM2P<char> *m_TxDMA;
	void * tx_dma;

};

struct uart_device uarts[NR_OF_UARTS];

size_t GetTxBuffLen(int device_num) 
{ 
	size_t ret = 0;//uarts[device_num].tx_dma ? uarts[device_num].tx_dma->getBuffLen() : 0; 
	return ret;
}

size_t GetRxBuffLen(int device_num) 
{ 
	size_t ret = 0;// uarts[device_num].rx_dma ?	uarts[device_num].rx_dma->getBuffLen() : 0; 
	return ret;
}

void uart_init(int device_num, size_t txFifoLen, size_t rxFifoLen, bool useDMA)
{
	if(device_num > NR_OF_UARTS)
		return;

	uarts[device_num].base = 0;
	uarts[device_num].rx_dma = 0;
	uarts[device_num].tx_dma = 0;
	
	switch (device_num)
	{
		case 0:
			LPC_SC->PCONP |= 0x1<<3;
			LPC_SC->PCLKSEL0 &= ~(0x3<<6);
			uarts[device_num].base = (LPC_UART_TypeDef*)LPC_UART0_BASE;
			break;
		case 1:
			LPC_SC->PCONP |= 0x1<<4;
			LPC_SC->PCLKSEL0 &= ~(0x3<<8);
			uarts[device_num].base =  (LPC_UART_TypeDef*)LPC_UART1_BASE;
			break;
		case 2:
			LPC_SC->PCONP |= 0x1<<24;
			LPC_SC->PCLKSEL1 &= ~(0x3<<16);
			uarts[device_num].base = (LPC_UART_TypeDef*)LPC_UART2_BASE;
			break;
		case 3:
			LPC_SC->PCONP |= 0x1<<25;
			LPC_SC->PCLKSEL1 &= ~(0x3<<18);
			uarts[device_num].base = (LPC_UART_TypeDef*)LPC_UART3_BASE;
			break;
		default:
			device_num = -1;
	}

	if (useDMA)
	{
/*		gdma_init; // FIXME
		GPDMA *dmaTx = gdma_get_channel();
		GPDMA *dmaRx = GPDMA::getChannel();
		uarts[device_num].tx_dma = new DmaM2P<char>(GPDMA::UART0Tx_Mat00, (char*)&(uarts[device_num].base->THR), txFifoLen, dmaTx);
		uarts[device_num].rx_dma = new DmaP2M<char>(GPDMA::UART0Rx_Mat01, (char*)&(uarts[device_num].base->RBR), rxFifoLen, dmaRx);
	*/}

	// Turn on the FIFO's and clear the buffers
	uarts[device_num].base->FCR = UART_FCR_EN | UART_FCR_CLR;// | ((uarts[device_num].tx_dma || uarts[device_num].rx_dma) ? UART_FCR_DMA : 0);

	if (uarts[device_num].rx_dma || uarts[device_num].tx_dma)
		uarts[device_num].base->FCR |= UART_FCR_DMA;
}

const struct FractionalBaudEntry FractionalBaudTable[] =
{
	{1.000,0,1},
	{1.067,1,15},
	{1.071,1,14},
	{1.077,1,13},
	{1.083,1,12},
	{1.091,1,11},
	{1.100,1,10},
	{1.111,1,9},
	{1.125,1,8},
	{1.133,2,15},
	{1.143,1,7},
	{1.154,2,13},
	{1.167,1,6},
	{1.182,2,11},
	{1.200,1,5},
	{1.214,3,14},
	{1.222,2,9},
	{1.231,3,13},
	{1.250,1,4},
	{1.267,4,15},
	{1.273,3,11},
	{1.286,2,7},
	{1.300,3,10},
	{1.308,4,13},
	{1.333,1,3},
	{1.357,5,14},
	{1.364,4,11},
	{1.375,3,8},
	{1.385,5,13},
	{1.400,2,5},
	{1.417,5,12},
	{1.429,3,7},
	{1.444,4,9},
	{1.455,5,11},
	{1.462,6,13},
	{1.467,7,15},
	{1.500,1,2},
	{1.533,8,15},
	{1.538,7,13},
	{1.545,6,11},
	{1.556,5,9},
	{1.571,4,7},
	{1.583,7,12},
	{1.600,3,5},
	{1.615,8,13},
	{1.625,5,8},
	{1.636,7,11},
	{1.643,9,14},
	{1.667,2,3},
	{1.692,9,13},
	{1.700,7,10},
	{1.714,5,7},
	{1.727,8,11},
	{1.733,11,15},
	{1.750,3,4},
	{1.769,10,13},
	{1.778,7,9},
	{1.786,11,14},
	{1.800,4,5},
	{1.818,9,11},
	{1.833,5,6},
	{1.846,11,13},
	{1.857,6,7},
	{1.867,13,15},
	{1.875,7,8},
	{1.889,8,9},
	{1.900,9,10},
	{1.909,10,11},
	{1.917,11,12},
	{1.923,12,13},
	{1.929,13,14},
	{1.933,14,15}
};

void uart_FindBaudWithFractional(uint32_t wantedBaud, uint32_t *divisor, uint32_t *fracDiv)
{
	float FRest = 1.5;
	int divAddVal = 0, mulVal = 1;

	// Setup the baud rate:  Calculate the divisor value.
	// Note: PCLK is CCLK/4, so the 16 in the equations becomes 64.
	*divisor = F_CPU / (wantedBaud * 64);

	// Check for integer divisor, otherwise compute fractional divisors
	if (F_CPU % (wantedBaud * 64) != 0)
	{
		*divisor = (uint32_t)floorf(F_CPU / (wantedBaud * 64 * FRest));
		FRest = F_CPU / (64 * wantedBaud * (float)(*divisor));
		if (FRest > 1.1 && FRest < 1.9)
		{
			for (unsigned char j = 0; j < 71; j++)
			{
				if (FractionalBaudTable[j].FRest > FRest
						&& FRest < FractionalBaudTable[j+1].FRest)
				{
					mulVal = FractionalBaudTable[j].mulVal;
					divAddVal = FractionalBaudTable[j].divAddVal;
					break;
				}
			}
		}
	}
	*fracDiv = (divAddVal & 0x0F) | ((mulVal & 0x0F) << 4);
}

void uart_SetBaud(int device_num,uint32_t baud)
{
	uint32_t ulDivisor=8;
	uint32_t ulFracDiv = 217;

	// Setup a fractional baud rate
	uart_FindBaudWithFractional(baud, &ulDivisor, &ulFracDiv);
//
	uarts[device_num].base->FDR = ulFracDiv;

	// Set the DLAB bit so we can access the divisor
	uarts[device_num].base->LCR = UART_LCR_DLAB;

	// Setup the divisor
	uarts[device_num].base->DLL = (unsigned char)(ulDivisor & (uint32_t)0xff);
	ulDivisor >>= 8;
	uarts[device_num].base->DLM = (unsigned char)(ulDivisor & (uint32_t)0xff);

	// Setup transmission format and clear the DLAB bit to enable transmission
	uarts[device_num].base->LCR = UART_LCR_NOPAR | UART_LCR_1STOP | UART_LCR_8BITS;

//	if (uarts[device_num].rx_dma)
//		uarts[device_num].rx_dma->startReading();

}

int uart_write(int device_num, const char * buf, size_t len)
{
	if (len < 1)
		return 0;
	if (uarts[device_num].tx_dma) {
	//	return uarts[device_num].tx_dma->write(buf, len);
	}
	return uart_writeUnbuffered(device_num, buf, len);
}

int uart_writeUnbuffered(int device_num, const char * buf, size_t len)
{
	if (len < 1)
		return 0;
	for (size_t i = 0; i < len; i++)
	{
		while (!(uarts[device_num].base->LSR & (1 << 5)));
		uarts[device_num].base->THR = *(buf++);
	}
	return len;
}

int uart_read(int device_num, char * buf, size_t len)
{
	size_t num;
	if (len < 1)
		return 0;
/*	if (uarts[device_num].rx_dma) {
		return uarts[device_num].rx_dma->read(buf, len);
		return 0;
	}
*/
	for (num = 0; num < len; num++) {
		if (LPC_UART0->LSR & 0x1) {// Receive data ready
			*buf++ = LPC_UART0->RBR & 0xFF;
		}
	}
	return num;
}

char uart_getc()
{
	char ret = 0x00;
	uart_read(STD_UART,&ret,1);
	return ret;
}

void uart_putc(const char c)
{
		uart_writeUnbuffered(STD_UART,(const char *)&c,1);
}

void uart_print_binary(unsigned long n)
{ 
	unsigned long base = 2;
	unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
	unsigned long i = 0;

	if (n == 0) {
		uart_putc('0');
		return;
	} 

	while (n > 0) {
		buf[i++] = n % base;
		n /= base;
	}

	for (; i > 0; i--)
		uart_putc(buf[i - 1] < 10 ?
			'0' + buf[i - 1] :
			'A' + buf[i - 1] - 10);
}

