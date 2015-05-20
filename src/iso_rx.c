/* ISO RX worker
*/

#include <stm32f0xx.h>
#include <stm32f0xx_usart.h>

#include "stypes.h"
#include "utils.h"
#include "params.h"
#include "iso_shr.h"

void rxw(u8 rxb, u32 ts);

//ISO_IRQH : handler for ISO USART handler
void ISO_IRQH(void) {
	u32 ts;
	ts = frclock;
	//XXX check ints
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

		lock=sys_SDI();
		if (dup_state == WAITDUPLEX) {
			if (rxb == duplex_req) {
				dup_state = DUP_IDLE;
			} else {
				dup_state = DUP_ERR;
			}
			sys_RI(lock);
			//XXX tail-chain into TX worker
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
	return;
}
