#ifndef TIMERS_H
#define TIMERS_H

#include "stypes.h"
#include <stm32f0xx_tim.h>

/** Timer resource allocation **/

/* Master, free-running clock (u32, autowrap, no ints)
 Timestamps : either SOF (CAN), or "end of last bit of msg" (ISO)
 - 10kHz == 100us resol; max msgfreq = 10kHz on CAN
 - 32bit @ 10kHz = 119h span.
 - 24bit (Systick) @ 10kHz = 28min span... enough for most stuff ?
 TODO : init CNT to (0 - 1000) to debug wraparound handling
	If viable, use systick (+ inc separate counter on wrap?), free up TIM2
*/
#define frclock	TIM2->CNT
#define frclock_conv	10	//so time(ms) = frclock / frclock_conv

/* Periodic MSG timer */
// basic 16 bit timer, interrupt on expiry.
#define PMSG_TMR	TIM16
#define PMSG_IRQH	TIM16_IRQHandler

/* PWM timer for J1850 ? */
//TIM1 (16bit adv)

/* tx worker timers */
// free-running 16bit timer with 4 compare ints, one per txworker
// (config with no preload, no pin I/O, "PWM mode 1",...)
// XXX what happens with wraparound vs CCxIF ?
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
