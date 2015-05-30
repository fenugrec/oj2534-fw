/* Common code for timer resources */
//XXX todo : (auto)calculate prescaler values
#include <stddef.h>

#include <stm32f0xx.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_rcc.h>
#include "timers.h"

#include "iso.h"	//iso tx worker

static void frclock_init(void);
static void txwork_timer_init(void);
static void pmsg_timer_init(void);

void timers_init(void) {
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
	tbi.TIM_ClockDivision = TIM_CKD_DIV1;	//XXX a calcular
	tbi.TIM_CounterMode = TIM_CounterMode_Up;
	tbi.TIM_Period = -1;
	tbi.TIM_Prescaler = 0;	//XXX a calcular
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
	tbi.TIM_ClockDivision = TIM_CKD_DIV1;	//XXX calc
	tbi.TIM_CounterMode = TIM_CounterMode_Up;
	tbi.TIM_Period = -1;
	tbi.TIM_Prescaler = 0;	//XXX a calcular
	//tbi.TIM_RepetitionCounter	//irrelevant on TIM3
	TIM_TimeBaseInit(TXWORK_TMR, &tbi);
	TXWORK_TMR->CNT = 0;
	TIM_Cmd(TXWORK_TMR, ENABLE);
	NVIC_EnableIRQ(IRQN_TXWORK);
	return;
}

//Interrupt handler for tx worker interrupts
void TXWORK_TMR_IRQH(void) {
	if (TXWORK_TMR->SR & ISO_TMR_CCIF) {
		TIM_ITConfig(TXWORK_TMR, ISO_TMR_CC_IT, DISABLE);
		TIM_ClearFlag(TXWORK_TMR, ISO_TMR_CCIF);
		isotx_work();
	} else {
		DBGM("bad TXW int", TXWORK_TMR->SR);
	}
	return;

}

// set specified CCRx to match in (ms) millisecs.
// Caution: TIM_ITconfig does an unlocked R-M-W on DIER, so this should only be called from the TMR ISR !
void txwork_setint(u16 ms, volatile u32 * CCR) {
	u32 now;
	assert((ms > 0) && (CCR != NULL));

	now = TXWORK_TMR->CNT;

	TIM_ClearFlag(TXWORK_TMR, ISO_TMR_CCIF);
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

	tbi.TIM_ClockDivision = TIM_CKD_DIV1;	//XXX calc
	tbi.TIM_CounterMode = TIM_CounterMode_Down;
	tbi.TIM_Period = -1;
	tbi.TIM_Prescaler = 0;	//XXX a calcular
	tbi.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(PMSG_TMR, &tbi);

	PMSG_TMR->CR1 |= TIM_CR1_OPM;	//one-pulse mode ! every Update disables the timer.
	TIM_ITConfig(PMSG_TMR, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(PMSG_TMR, TIM_FLAG_Update);
	NVIC_EnableIRQ(IRQN_PMSG);
	return;
}

//setint : set PMSG_TMR to expire once, in 'next' ms
void pmsg_setint(u16 next) {
	assert(next > 0);
	PMSG_TMR->CNT = next;
	TIM_Cmd(PMSG_TMR, ENABLE);
	return;
}
