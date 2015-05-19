#ifndef ISO_SHR_H
#define ISO_SHR_H

/** common defs shared between iso_tx and iso_rx **/

#include <stdatomic.h>
#include <stm32f0xx.h>

#define ISO_UART	USART3
#define ISO_IRQH	USART3_4_IRQHandler

/** timestamps **/

//struct iso_ts : timestamps for iso txworker + rxworker
//atomic read/writes ! (no r-m-w should be ok)
//timebase = 100us ! source= frclock
//NOTE : u32 wraparound is tricky for timeouts !
static struct {
	u32	last_act;	//last bus activity
	u32	last_RX;
	u32	last_TX;
	u32	tx_started;	//frclock at beginning of tx or init
	u32	tx_timeout;	//timeout if (frclock - tx_started) >= tx_timeout.
} iso_ts = {0};


/** Duplex removal **/
/*
 * For simple && safe access:
 * 1) rx: lock; if WAITDUPLEX && duplex match { state=DUP_IDLE;}; unlock; tailchain tx
 * 2) tx: lock; check state && write duplex_req; write state=WAITDUP, unlock
*/

//dup_state: WAITDUPLEX : next RX int checks duplex_req && set DUP_ERR or IDLE as required
extern enum dupstate_t { DUP_IDLE, WAITDUPLEX, DUP_ERR} dup_state;
extern u8 duplex_req;	//next byte expected if WAITDUPLEX


#endif
