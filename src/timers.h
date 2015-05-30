#ifndef TIMERS_H
#define TIMERS_H

#include "stypes.h"
#include <stm32f0xx.h>
#include <stm32f0xx_tim.h>

/******* Timer resource allocation *******/

/** Master, free-running clock (u32, autowrap, no ints) **/
/*
Timestamps : either SOF (CAN), or "end of last bit of msg" (ISO)
 - 10kHz == 100us resol; max msgfreq = 10kHz on CAN
 - 32bit @ 10kHz = 119h span.
 - 24bit (Systick) @ 10kHz = 28min span... enough for most stuff ?
 TODO : init CNT to (-100) to debug wraparound handling?
	If viable, use systick (+ inc separate counter on wrap?), free up TIM2
*/
#define frclock_conv	10	//so time(ms) = frclock / frclock_conv
#define FRCLOCK_TMR	TIM2
#define IRQN_FRCLOCK	TIM2_IRQn
#define frclock	FRCLOCK_TMR->CNT

/** Periodic MSG timer **/
// basic 16 bit * 1ms downcount timer, interrupt on expiry.
#define PMSG_TMR	TIM16
#define PMSG_IRQH	TIM16_IRQHandler
#define PMSG_APBC	RCC_APB2Periph_TIM16
#define IRQN_PMSG	TIM16_IRQn

/* PWM timer for J1850 ? */
//#define J1850_TMR	TIM1	//16bit adv

/** tx worker timers **/
// free-running 16bit * 1ms timer with 4 compare ints, one per txworker
// (config with no preload, no pin I/O, "PWM mode 1",...)
#define TXWORK_TMR	TIM3
#define TXWORK_APBC	RCC_APB1Periph_TIM3
#define TXWORK_TMR_IRQH	TIM3_IRQHandler
#define IRQN_TXWORK	TIM3_IRQn

#define ISO_TMR_CCR	TXWORK_TMR->CCR1
#define ISO_TMR_CCIF	TIM_FLAG_CC1
#define ISO_TMR_CC_IT	TIM_IT_CC1
#define ISO_TMR_CCG	TIM_EGR_CC1G
//#define CAN_TMR_CCR	TXWORK_TMR->CCR2
//#define CAN_TMR_CCIF	TIM_FLAG_CC2
//#define CAN_TMR_CCG	TIM_EGR_CC2G
//#define J1850_TMR_CCR	TXWORK_TMR->CCR3


/** timer mgmt funcs **/

//partial, low-level init of timers.
void timers_init(void);

/* tx worker funcs */
//txwork_setint() : interrupt in <ms> ms
void txwork_setint(u16 ms, volatile u32 * CCR);

/* pmsg funcs */
//pmsg_setint() : interrupt once, in <next> ms
void pmsg_setint(u16 next);

#endif	//TIMERS_H
