/* ISO RX worker
*/

#include <stm32f0xx.h>

#include "stypes.h"
#include "utils.h"
#include "params.h"
#include "timers.h"

#include "iso_shr.h"
#include "iso.h"

void rxw(u8 rxb, u32 ts);

//ISO_IRQH : handler for all USART interrupts
void ISO_IRQH(void) {
	u32 ts;
	ts = frclock;
	//XXX check other ints?
	//XXX signal error if RX while state && DUP_IDLE
	if (USART_GetITStatus(ISO_UART, USART_IT_RXNE)) {
		u8 rxb;
		u32 lock;
		//RXNE: just got some data.
		//TODO : merge this "generic RX" state with "TX_IDLE" state ?
		rxb = ISO_UART->RDR;	//this clears RXNE flag

		iso_ts.last_act = ts;

		//make sure we didn't overflow between IRQ and RDR read
		if (USART_GetFlagStatus(ISO_UART, USART_FLAG_ORE)) {
			DBGM("ISO RX ovf", rxb);
			//XXX abort / break message build
			USART_ClearFlag(ISO_UART, USART_FLAG_ORE);
			return;
		}
		//check noise flag
		if (USART_GetFlagStatus(ISO_UART, USART_FLAG_NE)) {
			//XXX do we care?
			DBGM("ISO noise", 0);
			USART_ClearFlag(ISO_UART, USART_FLAG_NE);
		}
		// check framing error (bad stop bit, etc)
		if (USART_GetFlagStatus(ISO_UART, USART_FLAG_FE)) {
			DBGM("ISO frame err", 0);
			//XXX abort / break message build
			return;
		}

		lock=sys_SDI();
		if (dup_state == WAITDUPLEX) {
			if (rxb == duplex_req) {
				dup_state = DUP_IDLE;
			} else {
				dup_state = DUP_ERR;
			}
			sys_RI(lock);
			isotx_work();
			return;
		}
		sys_RI(lock);

		iso_ts.last_RX = ts;
		rxw(rxb, ts);
		return;
	}
	return;
}
//RX worker; received byte rxb @ time ts (units of frclock)
void rxw(u8 rxb, u32 ts) {

	//XXX build message etc
	DBGM("rxw ts:rxb", (ts & ~0xFF) | rxb);
	return;
}
