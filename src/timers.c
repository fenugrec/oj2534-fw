/* Common code for timer resources */

#include <stddef.h>

#include <stm32f0xx_tim.h>
#include "timers.h"

//Interrupt handler for tx worker interrupts
void TXWORK_TMR_IRQH(void) {
	//XXX determine which compare interrupt; dispatch
	return;

}

void txwork_setint(u16 ms, volatile u32 * CCR) {
	u32 now;
	//XXX TODO, and make sure no int is lost
	assert((ms > 0) && (CCR != NULL));
	//XXX disable tmr while doing this?
	now = TXWORK_TMR->CNT;
	*CCR = now + ms;
	return;
}
