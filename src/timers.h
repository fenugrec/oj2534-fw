#ifndef TIMERS_H
#define TIMERS_H

#include "stypes.h"
#include <stm32f0xx_tim.h>

/** Timer resource allocation **/

/* Master, free-running clock (u32, autowrap, 10kHz, no ints) */
#define frclock	TIM2->CNT
#define frclock_conv	10	//so time(ms) = frclock / frclock_conv

/* Periodic MSG timer */
// basic 16 bit timer, interrupt on expiry.
#define PMSG_TMR	TIM16
#define PMSG_IRQH	TIM16_IRQHandler

/* tx worker timers */
// free-running 16bit timer with 4 compare ints, one per txworker
#define TXWORK_TMR	TIM3
#define TXWORK_TMR_IRQH	TIM3_IRQHandler

#define ISO_TMR_CCR	TXWORK_TMR->CCR1
//#define CAN_TMR_CCR	TXWORK_TMR->CCR2
//#define J1850_TMR_CCR	TXWORK_TMR->CCR3


/** timer mgmt funcs **/

/* tx worker funcs */
//txwork_setint() : interrupt in <ms> ms
//TODO :
void txwork_setint(u16 ms, volatile u32 * CCR);


#endif	//TIMERS_H
