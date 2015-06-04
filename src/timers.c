/* Common code for timer resources */

#include <stddef.h>

#include "oj2534.h"
#ifdef TESTING
	#include <stm32f0xx_dbgmcu.h>
#endif // TESTING

#include <stm32f0xx.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_rcc.h>
#include "timers.h"

#include "iso.h"	//iso tx worker

static void frclock_init(void);
static void txwork_timer_init(void);
static void pmsg_timer_init(void);

//fr_div : convert frclock-based time to millisecs
u16 fr_div(u32 dts) {
	if (dts >= ((1<<16) * frclock_conv)) {
		//clip large vals
		return -1;
	}
	return dts / frclock_conv;
}

void timers_init(void) {
#ifdef TESTING
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_DBGMCU, ENABLE);
	DBGMCU_APB1PeriphConfig(DBGMCU_APB1_FZ_DBG_TIM2_STOP, ENABLE);	//frclock
	DBGMCU_APB1PeriphConfig(DBGMCU_APB1_FZ_DBG_TIM3_STOP, ENABLE);	//txworker
	DBGMCU_APB2PeriphConfig(DBGMCU_APB2_FZ_DBG_TIM16_STOP, ENABLE);	//pmsg

#endif // TESTING
	frclock_init();
	//do these really need to be accessible outside timers.c ?
	txwork_timer_init();
	pmsg_timer_init();

	return;
}

/* frclock : init + start */
static void frclock_init(void) {
	TIM_TimeBaseInitTypeDef tbi;

	NVIC_DisableIRQ(IRQN_FRCLOCK);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	tbi.TIM_ClockDivision = TIM_CKD_DIV1;
	tbi.TIM_CounterMode = TIM_CounterMode_Up;
	tbi.TIM_Period = -1;
	tbi.TIM_Prescaler = (SystemCoreClock / (1000 * frclock_conv)) - 1;	//100us incs
	//tbi.TIM_RepetitionCounter	//irrelevant on TIM2
	TIM_TimeBaseInit(FRCLOCK_TMR, &tbi);
	FRCLOCK_TMR->CNT = 0;
	TIM_Cmd(FRCLOCK_TMR, ENABLE);

	return;
}

/** TXworker stuff **/
static void txwork_timer_init(void) {
	TIM_TimeBaseInitTypeDef tbi;
	NVIC_DisableIRQ(IRQN_TXWORK);
	RCC_APB1PeriphClockCmd(TXWORK_APBC, ENABLE);
	TXWORK_TMR->DIER = 0;	//disable all ints
	tbi.TIM_ClockDivision = TIM_CKD_DIV1;
	tbi.TIM_CounterMode = TIM_CounterMode_Up;
	tbi.TIM_Period = -1;
	tbi.TIM_Prescaler = (SystemCoreClock / 1000) -1;	//1ms incs
	//tbi.TIM_RepetitionCounter	//irrelevant on TIM3
	TIM_TimeBaseInit(TXWORK_TMR, &tbi);
	TXWORK_TMR->CNT = 0;
	TIM_Cmd(TXWORK_TMR, ENABLE);
	NVIC_EnableIRQ(IRQN_TXWORK);	//always enabled; CCx ints are enabled individually with DIER
	return;
}

//Interrupt handler for tx worker interrupts
void TXWORK_TMR_IRQH(void) {
	if (TXWORK_TMR->SR & ISO_TMR_CCIF) {
		TIM_ITConfig(TXWORK_TMR, ISO_TMR_CC_IT, DISABLE);
#ifndef DISABLE_ISO
		iso_work();
#endif
	} else {
		DBGM("bad TXW int", TXWORK_TMR->SR);
	}
	return;

}

// set specified CCRx to match in (ms) millisecs.
// Caution: TIM_ITconfig does an unlocked R-M-W on DIER, so this should only be called from the TMR ISR !
void isowork_setint(u16 ms, volatile u32 * CCR) {
	u32 now;
	assert((ms > 0) && (CCR != NULL));

	TIM_ClearFlag(TXWORK_TMR, ISO_TMR_CCIF);

	now = TXWORK_TMR->CNT;
	*CCR = now + ms;

	TIM_ITConfig(TXWORK_TMR, ISO_TMR_CC_IT, ENABLE);
	return;
}


/** PMSG stuff **/

static void pmsg_timer_init(void) {
	TIM_TimeBaseInitTypeDef tbi;

	NVIC_DisableIRQ(IRQN_PMSG);
	RCC_APB2PeriphClockCmd(PMSG_APBC, ENABLE);
	TIM_Cmd(PMSG_TMR, DISABLE);

	tbi.TIM_ClockDivision = TIM_CKD_DIV1;
	tbi.TIM_CounterMode = TIM_CounterMode_Up;
	tbi.TIM_Prescaler = (SystemCoreClock / 1000) -1;	//1ms incs
	tbi.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(PMSG_TMR, &tbi);

	PMSG_TMR->CR1 |= TIM_CR1_OPM;	//one-pulse mode ! every Update disables the timer.
	PMSG_TMR->CNT = 0;

	TIM_ClearFlag(PMSG_TMR, TIM_FLAG_Update);
	TIM_ITConfig(PMSG_TMR, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(IRQN_PMSG);
	return;
}

//setint : set PMSG_TMR to expire once, in 'next' ms; set pending if next==0
void pmsg_setint(u16 next) {

	if (next == 0) {
		//directly set pending
		TIM_Cmd(PMSG_TMR, DISABLE);	//cancel current countdown
		NVIC_SetPendingIRQ(IRQN_PMSG);
		return;
	} else {
		PMSG_TMR->ARR = next;
		TIM_Cmd(PMSG_TMR, ENABLE);
	}
	return;
}

//better HF handler : stop frclock; TODO : other stuff ? (put in safe state)
void HardFault_Handler(void) {
	TIM_Cmd(FRCLOCK_TMR, DISABLE);
	while (1) {}
}
