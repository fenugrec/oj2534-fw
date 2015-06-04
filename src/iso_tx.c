//iso_tx.c
//TXworker & stuff for iso9141 / 14230

/* TODO:
	-reduce # of frclock_conv divs ! (no hw div on M0 ! fuck !)
	-finish UART code
	-double check isot_ts.last_act usage
	-add init ioctl callbacks/etc
*/

#include <stddef.h>

#include <stm32f0xx_usart.h>

#include "stypes.h"
#include "msg.h"
#include "fifos.h"
#include "pmsg.h"
#include "utils.h"
#include "params.h"
#include "timers.h"

#include "txwork.h"
#include "rxwork.h"
#include "iso.h"

//if specified timeout==0, and for PMSG timeouts : use DEFTIMEOUT ms per byte
#define DEFTIMEOUT	2
//duplex error if echo not received within DUPTIMEOUT ms
#define DUPTIMEOUT	2
//"bus too busy" error if init doesn't complete within ISO_TMAXINIT ms
#define ISO_TMAXINIT	5000

/* private funcs (maybe the _ could be dropped since they're static) */
static void _isotx_startinit(u16 len, u8 type);
static void _isotx_slowi(void);
static void _isotx_fasti(void);
static int _isotx_findpmsg(void);
static void _isotx_start(uint len, u16 timeout);
static void _isotx_continue(void);
static void _isotx_done(void);
static void iso_rxw(int reason);
#define RXW_REASON_MOAR	0	//new data received
#define RXW_REASON_TMR	1	//timer expiry


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
	u32	tx_timeout;	//timed out if (frclock - tx_started) >= tx_timeout.
} iso_ts = {0};

/** state machine **/
/* struct "its" (iso tx state) : internal data for tx worker state machine
 * */

// .iso_state :
//		TX while sending a msg;
//		FI* during 14230 fast init;
//		SI* during 5baud init (9141 or 14230)
//only read / modified by iso worker or its support functions

static struct  {
	enum txs { ISO_IDLE, FI, SI, ISO_TX, ISO_RX} iso_state;	//ISO worker state machine
	//following members are valid only if iso_state == TX
	enum txm {TXM_FIFO, TXM_PMSG} tx_mode;	//determines how to fetch data
	uint pm_id;	//if _mode==PMSG : current msgid (set in _findpmsg)
	u8 pm_data[PMSG_MAX_SIZE];	//if _mode==PMSG : copy of pmsg data
	uint curpos;	//# of bytes sent so far
	uint curlen;	//# of bytes in current msg
}	its = {
		.iso_state = ISO_IDLE
	};

/*** iso duplex removal ***/
//when dup_state=DUP_WAIT : next RX int checks duplex_req && set DUP_ERR or IDLE as required

/*
 * For simple && safe access:
 * 1) rx: lock; if DUP_WAIT && duplex match { state=DUP_IDLE;}; unlock; tailchain tx
 * 2) tx: lock; check state && write duplex_req; write state=WAITDUP, unlock
*/

/* dup_state:
	-DUP_WAIT : next RX int checks duplex_req && set DUP_ERR or DUP_OK as required
	-CHEAT :next RX int writes byte to duplex_req to be parsed by txworker (instead of RX message builder)
			--> this is for slow init
*/
enum dupstate_t { DUP_IDLE, DUP_WAIT, DUP_OK, DUP_CHEAT, DUP_ERR} dup_state = DUP_IDLE;
u8 duplex_req;	//next byte expected if DUP_WAIT


/*** iso RX stuff ***/
uint	rx_pending;	//XXX atomic ! : used by USART IRQ to signal new data for iso_worker
u8	rbyte;
u32 rxts;	//timestamp of rbyte

/*** iso init data ***/
static u16 iso_initlen;	//1 for slow init, [datasize] for fastinit
static u16 iso_initwait;	//required idle before init (ms)
//iso_initstate: statemachine for slow/fast inits. FI* : fast init states, SI* = slowinit states
static enum initstate_t {INIT_IDLE, FI0, FI1, FI2, SI0, SI1, SI2, SI3, SI4, SI5, SI6, SI7} iso_initstate=INIT_IDLE;


/* iso_work() : tx/rx worker for ISO9141/ISO14230
  Tolerates being polled continuously;
  Called ONLY from: the self-settable TMR match IRQ.
	iso_qwork() can be used to force the interrupt pending bit
*/
void iso_work(void) {
	struct txblock txb;	//de-serialize blocks from fifo
	u16 bsize, tout;	//msg length, timeout(ms)

	if (rx_pending) {
		if ((its.iso_state != ISO_RX) ||
			(its.iso_state != ISO_IDLE)) {
			//"why are we receiving data ?"
			DBGM("spuribyte/badstate!", 0);
			//XXX how severe is this? parse anyway.
		}
		iso_rxw(RXW_REASON_MOAR);
		return;
	}

	switch (its.iso_state) {
	case ISO_TX:
		_isotx_continue();	//takes care of everything
		return;
		break;
	case ISO_RX:
		iso_rxw(RXW_REASON_TMR);	//we had no data, but call anyway.
		return;
		break;
	case FI:
		_isotx_fasti();	//does everything
		return;
		break;
	case SI:
		_isotx_slowi();	//does everything
		return;
		break;
	case ISO_IDLE:
		//prioritize periodic msgs.
		if (_isotx_findpmsg() == 0) {
			//_findpmsg takes care of everything
			return;
		}
		// no PMSG : try regular msgs

		//get hdr + blocksize
		if (fifo_cblock(TXW, TXW_RP_ISO, (u8 *) &txb, 3) != 3) {
			//probably an incomplete txblock, or nothing. Don't auto-poll.
			return;
		}
		if ((txb.hdr & TXB_SENDABLE) ==0) {
			//txblock not ready, retry soon
			isowork_setint(20, &ISO_TMR_CCR);	//XXX auto-poll... maybe not efficient
			return;
		}
		//valid, sendable block : skip, or parse completely.
		bsize = (txb.sH <<8) | txb.sL;
		if (bsize >= (u16) -TXB_DATAPOS) {
			//fatal : bad block size (can't skip block !)
			DBGM("huge txb", bsize);
			big_error();
		}

		if (((txb.hdr & TXB_PROTOMASK)>>TXB_PROTOSHIFT) == MP_ISO) {
			//get complete hdr
			if (fifo_rblock(TXW, TXW_RP_ISO, (u8 *) &txb, TXB_DATAPOS) != TXB_DATAPOS) {
				//fatal? block was marked TXB_SENDABLE, but incomplete...
				big_error();
			}
			//(next fifo reads will get actual data)
			tout = (txb.tH <<8) | txb.tL;
			if (txb.hdr & TXB_SPECIAL) {
				//special flag means fast/slow init for ISO
				u8 t_init;	//txb.data[0] = type
				if (fifo_rblock(TXW, TXW_RP_ISO, &t_init, 1) != 1) {
					//fatal: incomplete block
					big_error();
				}
				_isotx_startinit(bsize, t_init);	//takes care of everything
				return;
			}
			its.tx_mode = TXM_FIFO;
			_isotx_start(bsize, tout);
			return;
		} else {
			//bad proto ==> skip block.
			(void) fifo_skip(TXW, TXW_RP_ISO, bsize + TXB_DATAPOS);
				//this isn't an error : there could be no additional blocks, so fifo_skip would fail...
			// XXX set next int (auto-poll ?), etc
			return;
		}
		break;	//case ISO_IDLE
	default:
		break;
	}	//switch txstate

	return;
}

//iso_qwork() : queue TX worker interrupt. XXX TODO : merge with setinit(0) ?
void iso_qwork(void) {
	TXWORK_TMR->EGR = ISO_TMR_CCG;	//force CC1IF flag
	TIM_ITConfig(TXWORK_TMR, ISO_TMR_CC_IT, ENABLE);
	return;
}

/*********** IRQ STUFF *********/
//ISO_IRQH : handler for all USART interrupts; high prio
void ISO_IRQH(void) {
	u32 ts;
	ts = frclock;
	//XXX check other ints?
	//XXX signal error if RX while state && DUP_IDLE
	if (USART_GetITStatus(ISO_UART, USART_IT_RXNE)) {
		u8 rxb;
		u32 lock;
		//RXNE: just got some data.
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
		switch (dup_state) {
		case DUP_CHEAT:
			//skip duplex check, give byte to txworker
			duplex_req = rxb;
			dup_state = DUP_OK;
			sys_RI(lock);
			iso_qwork();
			return;
			break;
		case DUP_WAIT:
			//verify duplex match
			if (rxb == duplex_req) {
				dup_state = DUP_OK;
			} else {
				dup_state = DUP_ERR;
			}
			sys_RI(lock);
			iso_qwork();
			return;
			break;
		case DUP_IDLE:
			//normal RX byte; forward to rx worker
			sys_RI(lock);
			iso_ts.last_RX = ts;
			rxts = ts;
			rbyte = rxb;
			rx_pending = 1;
			iso_qwork();
			return;
			break;
		default:
			//bad sequence; unexpected RX
			sys_RI(lock);
			DBGM("unexpected RX", rxb);
			//XXX do nothing else ?
			return;
			break;
		}	//switch dup_state

	}	//if RXNE
	return;
}	//UART IRQH



/*********** ISO TX STUFF *********/

//_isotx_findpmsg : start PMSG if possible
//calls _isotx_start() && ret 0 if ok
//ret -1 if failed
static int _isotx_findpmsg(void) {
	uint pm_len;

	if ( !pmsg_get(MP_ISO, its.pm_data, &pm_len, &its.pm_id)) {
		its.tx_mode = TXM_PMSG;
		_isotx_start(pm_len, pm_len * DEFTIMEOUT);
		return 0;
	}
	return -1;	//no pmsg claimed & started
}

//init stuff for txing a new msg; set iso_state; ensure P3_min ; set next txw int.
//its.tx_mode needs to be set before calling this
static void _isotx_start(uint len, u16 timeout) {
	u32 guardtime;

	if (len == 0) {
		big_error();
		return;
	}

	its.iso_state = ISO_TX;
	its.curpos = 0;
	its.curlen = len;

	if (timeout == 0) timeout = len * DEFTIMEOUT;
	//XXX j2534 timeout is defined as API blocking timeout; counting from now is not quite compliant.
	iso_ts.tx_started = frclock;
	iso_ts.tx_timeout = timeout * frclock_conv;

	guardtime = (iso_ts.tx_started - iso_ts.last_act) / frclock_conv;
	if (guardtime < tparams.p3min) {
		isowork_setint(tparams.p3min - guardtime, &ISO_TMR_CCR);
	} else {
		iso_qwork();	//re-enter
	}
	return;
}

/* _isotx_continue : takes care of
 * 	- setting next int
 *  - de-duplex
 *  - check if done
 *  - check if TX timeout
 * */
static void _isotx_continue(void) {
	u32 guardtime;
	u8 nextbyte;
	u32 lock;

	//check if tx finished
	if (its.curpos >= its.curlen) {
		_isotx_done();
		return;
	}
	//or timed out
	if ((frclock - iso_ts.tx_started) >= iso_ts.tx_timeout) {
		DBGM("isotx timeout", its.curpos);
		//XXX flag tx timeout err + indication for msgid?
		(void) fifo_skip(TXW, TXW_RP_ISO, its.curlen - its.curpos);
		its.curpos = its.curlen;
		_isotx_done();
		return;
	}

	guardtime = (frclock - iso_ts.last_act) / frclock_conv;

	if (its.curpos == 0) {
		//if nothing sent yet :
		//make sure guard time is OK for first transmit (p3min in case iso_ts.last_act changed recently)
		if (guardtime < tparams.p3min) {
			isowork_setint(tparams.p3min - guardtime, &ISO_TMR_CCR);
			return;
		}
	} else {
		//1) check duplex
		switch (dup_state) {
		case DUP_WAIT:
			//no echo yet
			if (guardtime >= DUPTIMEOUT ) {
				//duplex timeout: unrecoverable
				DBGM("No duplex", duplex_req);
				isotx_abort();
				return;
			}
			//finish DUPTIMEOUT
			isowork_setint(DUPTIMEOUT - guardtime, &ISO_TMR_CCR);
			return;
			break;
		case DUP_OK:
			//good echo
			break;
		case DUP_ERR:
			//duplex error: unrecoverable
			DBGM("bad duplex", duplex_req);
			isotx_abort();
			return;
			break;
		default:
			DBGM("bad dupstate", dup_state);
			big_error();
			return;
			break;
		}
		//2) enforce P4
		if (guardtime < tparams.p4min) {
			isowork_setint(tparams.p4min - guardtime, &ISO_TMR_CCR);
			return;
		}
	}

	//get next byte
	switch (its.tx_mode) {
	case TXM_FIFO:
		if (fifo_rblock(TXW, TXW_RP_ISO, &nextbyte, 1) != 1) {
			DBGM("TXM_FIFO err", its.curpos);
			big_error();
			return;
		}
		break;
	case TXM_PMSG:
		nextbyte = its.pm_data[its.curpos];
		break;
	default:
		DBGM("bad txmode", its.tx_mode);
		big_error();
		return;
		break;
	}

	lock=sys_SDI();
	duplex_req = nextbyte;
	dup_state = DUP_WAIT;
	sys_RI(lock);

	ISO_UART->TDR = nextbyte;

	its.curpos += 1;
	//XXX update timestamps after TC int ?
	iso_ts.last_act = frclock;
	iso_ts.last_TX = iso_ts.last_act;

	guardtime = (DUPTIMEOUT > tparams.p4min)? DUPTIMEOUT : tparams.p4min ;
	isowork_setint(guardtime, &ISO_TMR_CCR);
	//if operating with P4<DUPTIMEOUT, either RX IRQH will tailchain into txw,
	// or guardtime IRQ will run txw
	return;

}


//_isotx_done : should be called after msg tx (matches _isotx_start).
// reset iso_state; set next txw int
// TODO : merge with _abort ?
static void _isotx_done(void) {
	its.iso_state = ISO_IDLE;
	dup_state = DUP_IDLE;
	iso_ts.last_TX = frclock;
	iso_ts.last_act = iso_ts.last_TX;
	its.curlen = 0;
	isowork_setint(tparams.p3min, &ISO_TMR_CCR);
	return;
}

//isotx_abort : aborts current transmission; reset state machines;
//skip current txblock.
//XXX TODO : ensure this can work if called async while in txw...
void isotx_abort(void) {
	USART_Cmd(ISO_UART, DISABLE);	//clears UE
	//XXX reset speed? (in case slowinit clobbered it)
	//XXX clear TX break
	if ((its.iso_state == ISO_TX) && (its.tx_mode == TXM_FIFO)) {
		uint skiplen = its.curlen - its.curpos;
		if (fifo_skip(TXW, TXW_RP_ISO, skiplen) != skiplen ) {
			//can't skip current block ?
			DBGM("txabort can't skip ", skiplen);
			big_error();
			return;
		}
		//XXX generate indication for msgid ?
	}
	its.iso_state = ISO_IDLE;
	return;
}

//isotx_init : TODO
//call only on reset or after isotx_abort.
void isotx_init(void) {
	//XXX set USART params,
	//Baud rates : divisor = 16bits, etc
	USART_Cmd(ISO_UART, ENABLE);
	its.iso_state = ISO_IDLE;
	its.curpos = 0;
	its.curlen = 0;
	iso_initstate = INIT_IDLE;
	dup_state = DUP_IDLE;
	return;
}

/**** ISO init funcs ****/

//_isotx_startinit : set next int, iso_state, _initstate
static void _isotx_startinit(u16 len, u8 type) {
	u32 now;

	assert(len != 0);

	now = frclock;
	iso_ts.tx_timeout = (ISO_TMAXINIT * frclock_conv);	//die in case init never ends

	//safe, although suboptimal: wait for longest of W5 or Tidle
	iso_initwait = (tparams.w5 < tparams.tidle)? tparams.tidle:tparams.w5;

	iso_initlen = len;
	iso_ts.tx_started = now;

	switch (type) {
	case ISO_SLOWINIT:
		assert(len == 1);
		its.curlen = 1;
		its.curpos = 0;	//1 addr byte
		its.iso_state = SI;
		iso_initstate = SI0;
		break;
	case ISO_FASTINIT:
		its.iso_state = FI;
		iso_initstate = FI0;
		break;
	default:
		DBGM("bad init type", type);
		big_error();
		break;
	}
	iso_qwork();
	return;
}

//_slowi : slow init; called from txw
//maintain current state of iso_initstate; when init is done we need to return 2 keybytes through ioctl...
//build special RX message (dll translates to ioctl resp?) ?
static void _isotx_slowi(void) {
	static u8 addr, kb1, kb2;
	u32 guardtime;
	//check if init time was exceeded :
	if ((frclock - iso_ts.tx_started) >= iso_ts.tx_timeout) {
		DBGM("SI timeout", iso_initstate);
		//XXX reset UART?
		iso_initstate = INIT_IDLE;
		isotx_abort();
		return;
	}

	switch (iso_initstate) {
	case SI0: {
		static uint bcount=0;	//counter for manual TX
		static u8 tempbyte;

		//Can't set USART to 5bps because of 16bit divisor. bangbit instead

		if (bcount == 0) {
			//make sure W5 / Tidle is respected:
			guardtime = (frclock - iso_ts.last_act) / frclock_conv;
			if (guardtime < iso_initwait) {
				isowork_setint(iso_initwait - guardtime, &ISO_TMR_CCR);
				return;
			}
			USART_Cmd(ISO_UART, DISABLE);
			//XXX adj pin funcs
			// Only once : get addr
			if (fifo_rblock(TXW, TXW_RP_ISO, &addr, 1) != 1) {
				DBGM("fifo_rb", frclock);
				big_error();
			}
			tempbyte = addr;
			//XXX TX=0 (bit 0 = startbit)
			iso_ts.last_TX = frclock;	//abuse : timestamp start of startbit
			bcount++;
			isowork_setint(200, &ISO_TMR_CCR);
			return;
		}
		//here, bcount >= 1; (1: finishing startbit)

		//adjust bit times : continue only if (bcount * 200ms) has elapsed.
		guardtime = (frclock - iso_ts.last_TX) / frclock_conv;
		if (guardtime < (bcount * 200)) {
			isowork_setint((bcount * 200) - guardtime, &ISO_TMR_CCR);
			return;
		}

		if (bcount <= 9) {
			if (bcount == 9)
				tempbyte = 1;	//bit 9 = stopbit
			//LSB first.
			if (tempbyte & 1) {
				//XXX TX=1
			} else {
				//XXX TX=0
			}
			tempbyte = tempbyte >>1;
			bcount++;
			isowork_setint(200, &ISO_TMR_CCR);
			return;
		}	//bitloop
		//Here, bcount==10 and we finished stopbit
		iso_ts.last_TX = frclock;
		bcount = 0;	//reset for next time !!
		} //codeblock

		//XXX reset pin funcs, etc
		USART_Cmd(ISO_UART, ENABLE);
		//Here, we sent the address @ 5bps. No duplex echo to receive.

		iso_initstate = SI2;	//skip SI1, due to bad planning
		dup_state = DUP_CHEAT;	//steal next RX byte (0x55 sync within W1)
		isowork_setint(tparams.w1, &ISO_TMR_CCR);
		return;
		break;
	case SI1:
		//XXX TODO : split SI0 in two chunks
		return;
		break;
	case SI2:
		//waiting for sync byte
		guardtime = (frclock - iso_ts.last_TX)/frclock_conv;
		if (dup_state == DUP_OK) {
			//we got a byte !
			iso_ts.last_RX = frclock;
			if (duplex_req != 0x55) {
				DBGM("bad sync", duplex_req);
				//XXX proceed anyway?
			}
		} else {
			//no sync byte
			if (guardtime < tparams.w1) {
				//still some time left
				isowork_setint(tparams.w1 - guardtime, &ISO_TMR_CCR);
				return;
			}
			DBGM("no sync",0);
			isotx_abort();
			return;
		}
		dup_state = DUP_CHEAT;	//wait for KB1
		iso_initstate = SI3;
		isowork_setint(tparams.w2, &ISO_TMR_CCR);
		return;
		break;
	case SI3:
		//waiting for KB1
		guardtime = (frclock - iso_ts.last_RX)/frclock_conv;
		if (dup_state == DUP_OK) {
			iso_ts.last_RX = frclock;
			kb1 = duplex_req;
		} else {
			//no KB1 !
			if (guardtime < tparams.w2) {
				//still some time left
				isowork_setint(tparams.w2 - guardtime, &ISO_TMR_CCR);
				return;
			}
			DBGM("no KB",1);
			isotx_abort();
			return;
		}

		dup_state = DUP_CHEAT;	//for KB2
		iso_initstate = SI4;
		isowork_setint(tparams.w3, &ISO_TMR_CCR);
		return;
		break;
	case SI4:
		//waiting for KB2
		guardtime = (frclock - iso_ts.last_RX)/frclock_conv;
		if (dup_state == DUP_OK) {
			iso_ts.last_RX = frclock;
			kb2 = duplex_req;
		} else {
			//no KB2 !
			if (guardtime < tparams.w3) {
				//still some time left
				isowork_setint(tparams.w3 - guardtime, &ISO_TMR_CCR);
				return;
			}
			DBGM("no KB",2);
			isotx_abort();
			return;
		}

		//wait W4 before sending ~KB2
		iso_initstate = SI5;
		isowork_setint(tparams.w4 - (frclock - iso_ts.last_RX)/frclock_conv, &ISO_TMR_CCR);
		break;
	case SI5:
		//send ~KB2
		guardtime = (frclock - iso_ts.last_RX) / frclock_conv;	//enforce W4
		if (guardtime < tparams.w4) {
			isowork_setint(tparams.w4 - guardtime, &ISO_TMR_CCR);
			return;
		}
		ISO_UART->TDR = ~kb2;
		iso_ts.last_TX = frclock;
		dup_state = DUP_WAIT;
		iso_initstate = SI6;
		isowork_setint(DUPTIMEOUT, &ISO_TMR_CCR);
		return;
		break;
	case SI6:
		guardtime = (frclock - iso_ts.last_TX) / frclock_conv;
		//check ~KB2 duplex:
		switch (dup_state) {
		case DUP_WAIT:
			//no echo yet
			if (guardtime >= DUPTIMEOUT ) {
				//duplex timeout: unrecoverable
				DBGM("No duplex", duplex_req);
				isotx_abort();
				return;
			}
			isowork_setint(DUPTIMEOUT - guardtime, &ISO_TMR_CCR);
			return;
			break;
		case DUP_OK:
			//good echo
			break;
		case DUP_ERR:
			//duplex error: unrecoverable
			DBGM("bad duplex", duplex_req);
			isotx_abort();
			return;
			break;
		default:
			DBGM("bad dupstate", dup_state);
			big_error();
			return;
			break;
		}

		dup_state = DUP_CHEAT;	//wait for ~addr
		iso_initstate = SI7;
		isowork_setint(tparams.w4, &ISO_TMR_CCR);
		break;
	case SI7:
		guardtime = (frclock - iso_ts.last_TX) / frclock_conv;
		if (dup_state == DUP_OK) {
			iso_ts.last_RX = frclock;
			if (duplex_req != addr) {
				DBGM("bad ~addr", duplex_req);
				//take a chance and continue?
			}
		} else {
			//no ~addr !
			if (guardtime < tparams.w4) {
				//still some time left
				isowork_setint(tparams.w4 - guardtime, &ISO_TMR_CCR);
				return;
			}
			DBGM("no ~addr",0);
			isotx_abort();
			return;
		}

		//XXX success: signal kb1,kb2
		DBGM("kb1:kb2", kb1<<8 | kb2);
		iso_initstate = INIT_IDLE;
		break;
	default:
		DBGM("bad SI state", iso_initstate);
		big_error();
		break;
	}	//switch iso_initstate
	its.iso_state = ISO_IDLE;
	return;
}	//_slowi

//_fasti : fast init; called by iso txworker
//maintain current state of fast init; when WUP is done start normal TX for StartComm request
static void _isotx_fasti(void) {
	u32 guardtime;
	static u32 wupc;	//tune tWUP

	//check if init time was exceeded :
	if ((frclock - iso_ts.tx_started) >= iso_ts.tx_timeout) {
		iso_initstate = INIT_IDLE;
		//XXX clear TX break; enable UART
		DBGM("FI timeout", iso_initstate);
		iso_initstate=INIT_IDLE;
		isotx_abort();
		return;
	}

	switch (iso_initstate) {
	case FI0:
		//make sure guard time is respected:
		guardtime = (frclock - iso_ts.last_act) / frclock_conv;
		if (guardtime < iso_initwait) {
			isowork_setint(iso_initwait - guardtime, &ISO_TMR_CCR);
			return;
		}
		//start WUP (W5 or Tidle already elapsed)
		//XXX disable UART;
		//XXX set tx_break;
		wupc = frclock;	//timestamp WUPstart
		isowork_setint(tparams.tinil, &ISO_TMR_CCR);
		iso_initstate = FI1;
		break;
	case FI1:
		//validate tINI_L (break on TXD for fast init)
		guardtime = (frclock - wupc) / frclock_conv;
		if (guardtime < tparams.tinil) {
			isowork_setint(tparams.tinil - guardtime, &ISO_TMR_CCR);
			return;
		}
		//XXX clear TX_break;
		guardtime = (frclock - wupc)/frclock_conv;	//true tiniL (ms)
		isowork_setint(tparams.twup - guardtime, &ISO_TMR_CCR);
		iso_initstate = FI2;
		break;
	case FI2:
		//after tWUP : TX request using standard mechanism
		//typically we're sending a StartComm req; the response is treated as a normal RX msg
		//validate tWUP
		guardtime = (frclock - wupc) / frclock_conv;	//true tWUP (ms)
		if (guardtime < tparams.twup) {
			isowork_setint(tparams.twup - guardtime, &ISO_TMR_CCR);
			return;
		}
		//XXX purge RX bufs
		USART_Cmd(ISO_UART, ENABLE);
		iso_initstate = INIT_IDLE;
		_isotx_start(iso_initlen, iso_initlen * DEFTIMEOUT);
		break;
	default:
		DBGM("bad FI state", iso_initstate);
		big_error();
		break;
	}
	//XXX signal ioctl complete
	its.iso_state = ISO_IDLE;
	return;
}	//_isotx_fasti


/***************** RX WORKER *****************/
//iso_rxpush : timestamp rxblock and write to RXW fifo
static void iso_rxpush(struct rxblock * rxb, u32 ts) {
	rxb->ts[3] = (u8) ts;
	ts >>= 8;
	rxb->ts[2] = (u8) ts;
	ts >>= 8;
	rxb->ts[1] = (u8) ts;
	ts >>= 8;
	rxb->ts[0] = (u8) ts;
	if (fifo_wblock(RXW, (u8 *) rxb, RXBS_MINSIZE + rxb->len) != rxb->len) {
		DBGM("RXW fifull!", 0);
		//XXX medium error ... retry later or dump current block?
	}
	return;
}
//iso_rxw() :  "subset" of iso worker statemachine; called only from inside iso_work
//
static void iso_rxw(int reason) {
	#define RXW_CHUNKSIZ	64	//most ISO messages will have <= 64 bytes of data
	static u8 rbuf[RXBS_MINSIZE + RXW_CHUNKSIZ];
	struct rxblock * rxb = (struct rxblock *) rbuf;
	u8 *len = &rxb->len;	//shorthand

	if ((its.iso_state == ISO_RX) &&
		(rxts - iso_ts.last_RX) >= tparams.p1max) {
			//"close" current message
			rxb->hdr |= RXB_SEQEND;
			iso_rxpush(rxb, rxts);
			its.iso_state = ISO_IDLE;
	}

	if (reason == RXW_REASON_TMR) {
		return;	//nothing else to do
	}

	iso_ts.last_RX = rxts;

	if (its.iso_state == ISO_IDLE) {
		//start new msg : hdr = Normal, ISO, sequence start.
		rxb->hdr =  RXB_TYPE | (MP_ISO << RXB_NPSHIFT) | RXB_SEQSTART;
		rxb->len = 0;
		its.iso_state = ISO_RX;
	}

	// Write data, possibly send current chunk
	rxb->data[*len] = rbyte;
	(*len)++;
	if (*len == RXW_CHUNKSIZ) {
		//block full !
		iso_rxpush(rxb, rxts);
		*len = 0;
		rxb->hdr &= RXB_SEQSTART;
		rxb->hdr |= RXB_SEQCONT;
	}

	DBGM("rxw ts:rb", (rxts & ~0xFF) | rbyte);
	rx_pending = 0;	//clear 'pending' flag
	isowork_setint(tparams.p1max, &ISO_TMR_CCR);

	return;
}	//rxw() ; rx worker

