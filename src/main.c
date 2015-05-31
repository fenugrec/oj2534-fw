/*
*/

#include <stdint.h>
#include <stddef.h>	//for #define NULL !
//#include <stdatomic.h>

#include "oj2534.h"
#include <stm32f0xx.h>
#include "stypes.h"
#include "utils.h"

#include "fifos.h"
#include "iso.h"
#include "timers.h"
#include "pmsg.h"
u32 SystemCoreClock;

void SystemInit(void) {
#ifdef TESTING
	//use 8MHz HSI
	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Set HSION bit */
	RCC->CR |= RCC_CR_HSION;

	RCC->CFGR &= ~(RCC_CFGR_PLLNODIV | RCC_CFGR_MCO_PRE | RCC_CFGR_MCO |
				RCC_CFGR_ADCPRE | RCC_CFGR_PPRE | RCC_CFGR_HPRE | RCC_CFGR_SW);

	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);

	RCC->CR &= ~RCC_CR_HSEBYP;

	/* Reset PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
	RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC);

	RCC->CFGR2 &= ~(RCC_CFGR2_PREDIV1);

	RCC->CFGR3 &= ~(RCC_CFGR3_USART2SW | RCC_CFGR3_ADCSW | RCC_CFGR3_CECSW |
				RCC_CFGR3_I2C1SW | RCC_CFGR3_USART1SW);

	RCC->CR2 &= ~RCC_CR2_HSI14ON;

	SystemCoreClock = 8*1000*1000;
#else
	//setup USB PLL + CRS etc
#endif // TESTING
	return;
}

int main(void) {

	timers_init();
	fifo_init();
	isotx_init();
	pmsg_init();

	while (1) {
		isotx_qwork();	//also call qwork on txb reception etc
	}
	return 0;
}
