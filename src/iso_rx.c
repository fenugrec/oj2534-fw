/* ISO RX worker
*/

#include <stm32f0xx.h>
#include <stm32f0xx_usart.h>

#include "stypes.h"
#include "utils.h"
#include "params.h"
#include "iso_shr.h"

void rxw(void);

void ISO_IRQH(void) {
	//XXX classer ints
	rxw();
	return;
}
//RX IRQH (int on data RX)
void rxw(void) {
	u32 lock;
	u32 ts;
	u8 rxb;
	
	ts = frclock;
	rxb = ISO_UART->RDR;
	
	iso_ts.last_RX = ts;
	iso_ts.last_act = ts;
	
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
	
	//XXX build message etc
	return;
}
