//iso_tx.c
//TXworker & stuff for iso9141 / 14230

#include <stddef.h>
#include "txwork.h"
#include "iso_shr.h"
#include "stypes.h"
#include "msg.h"
#include "fifos.h"
#include "pmsg.h"
#include "utils.h"
#include "params.h"

//if specified timeout==0, and for PMSG timeouts : use DEFTIMEOUT ms per byte
#define DEFTIMEOUT	2
//duplex error if echo not received within DUPTIMEOUT ms
#define DUPTIMEOUT	2
//"bus too busy" error if init doesn't complete within ISO_TMAXINIT ms
#define ISO_TMAXINIT	5000

static void _isotx_startinit(u16 len, u8 type);
static void _isotx_slowi(void);
static void _isotx_fasti(void);
static int isotx_findpmsg(void);
static void _isotx_start(uint len, u16 timeout, u8 * optdata);
static void _isotx_continue(void);
static void _isotx_done(void);
static void isotx_abort(void);

/* struct "its" (iso tx state) : internal data for tx worker state machine
 * */
// .tx_state :
//		TX while sending a msg;
//		FI* during 14230 fast init;
//		SI* during 5baud init (9141 or 14230)
//only read / modified by iso_txwork() or its support functions

static struct  {
	enum txs { TX_IDLE, FI, SI, TX} tx_state;
	//following members are valid only if tx_state == TX
	enum txm {TXM_FIFO, TXM_PMSG} tx_mode;	//determines how to fetch data
	uint pm_id;	//if _mode==PMSG : current msgid (set in _findpmsg)
	u8 *pm_data;	//if _mode==PMSG : temp ptr to pmsg data
	uint curpos;	//# of bytes sent so far
	uint curlen;	//# of bytes in current msg
}	its = {
		.tx_state = TX_IDLE
	};

/*** iso duplex removal (see iso_shr.h) ***/
//dup_state=WAITDUPLEX : next RX int checks duplex_req && set DUP_ERR or IDLE as required
enum dupstate_t dup_state=DUP_IDLE;
u8 duplex_req;	//next byte expected if WAITDUPLEX

/*** iso init data ***/
static u16 iso_initlen;	//1 for slow init, [datasize] for fastinit
static u32 iso_initwait;	//required idle before init (ms*frclock_conv)
//iso_initstate: statemachine for slow/fast inits. FI* : fast init states, SI* = slowinit states
static enum initstate_t {INIT_IDLE, FI0, FI1, FI2, SI0, SI1, SI2} iso_initstate=INIT_IDLE;

//iso_txwork() : should tolerate being polled continuously?, but
//should be called by a self-settable TMR expiry IRQ;
//XXX also needs to be called / tail-chained into, from the RX IRQ (for de-duplexing)
//XXX int de UART_TX_DONE doit pouvoir tailchainer aussi, ou pas ? depend de slowinit
void iso_txwork(void) {
	struct txblock txb;	//de-serialize blocks from fifo
	u16 bsize, tout;	//msg length, timeout(ms)

	switch (its.tx_state) {
	case TX:
		_isotx_continue();	//takes care of everything
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
	case TX_IDLE:
		//prioritize periodic msgs.
		if (isotx_findpmsg() == 0) {
			//_findpmsg takes care of everything
			return;
		}
		// no PMSG : try regular msgs

		//get hdr + blocksize
		if (fifo_cblock(TXW, TXW_RP_ISO, (u8 *) &txb, 3) != 3) {
			//probably an incomplete txblock
			//XXX set next int, etc
			return;
		}
		if ((txb.hdr & TXB_SENDABLE) ==0) {
			//txblock not ready
			//XXX set next int, etc
			return;
		}
		//valid, sendable block : skip, or parse completely.
		bsize = (txb.sH <<8) | txb.sL;
		if (bsize >= (u16) -TXB_DATAPOS) {
			//fatal : bad block size (can't skip block !)
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

			_isotx_start(bsize, tout, NULL);
			return;
		} else {
			//bad proto ==> skip block.
			(void) fifo_skip(TXW, TXW_RP_ISO, bsize + TXB_DATAPOS);
				//this isn't an error : there could be no additional blocks, so fifo_skip would fail...
			// XXX set next int, etc
			return;
		}
		break;	//case TX_IDLE
	default:
		break;
	}
	return;
}

/*********** ISO TX STUFF *********/
//ptet fitter dans un autre fihier ?

//isotx_findpmsg : cycle through pmsg IDs, try to claim an enabled ISO pmsg.
//TODO : generaliz pour CAN ?
//calls _isotx_start() && ret 0 if ok
//ret -1 if failed
static int isotx_findpmsg(void) {
	uint pmsg_id;
	for (pmsg_id=0; pmsg_id < PMSG_MAXNUM; pmsg_id++) {
		if (!pmsg_claim(pmsg_id)) {
			u8 *pmsg_data;
			uint pmsg_len;
			//found & claimed a queued message; can we handle it?
			if (pmsg_getproto(pmsg_id) != MP_ISO) {
				pmsg_release(pmsg_id);
				continue;
			}
			pmsg_data = pmsg_getmsg(pmsg_id, &pmsg_len);
			if (pmsg_data == NULL) {
				pmsg_release(pmsg_id);
				DBGM("bad pmsg?", pmsg_id);
				continue;
			}
			_isotx_start(pmsg_len, pmsg_len * DEFTIMEOUT, pmsg_data);
			its.pm_id = pmsg_id;
			return 0;
		}
	}	//for (pmsg)
	return -1;	//no pmsg claimed & started
}

//init stuff for txing a new msg; set tx_state; ensure P3_min ; set next txw int
//optdata is optional; NULL for regular fifo messages, or points to PMSG data
static void _isotx_start(uint len, u16 timeout, u8 * optdata) {
	u32 guardtime;

	if (len == 0) {
		big_error();
		return;
	}

	if (optdata != NULL) {
		its.tx_mode = TXM_PMSG;
		its.pm_data = optdata;
	} else {
		its.tx_mode = TXM_FIFO;
	}
	its.tx_state = TX;
	its.curpos = 0;
	its.curlen = len;

	if (timeout == 0) timeout = len * DEFTIMEOUT;
	//XXX j2534 timeout is defined as API blocking timeout; counting from now is not quite compliant.
	iso_ts.tx_started = frclock;
	iso_ts.tx_timeout = timeout * frclock_conv;

	guardtime = (frclock - iso_ts.last_act);
	if (guardtime < (tparams.p3min * frclock_conv)) {
		//XXX set next int(tparams.p3min*frclock_conv - guardtime);
	} else {
		//XXX set pending int to re-enter ? or 1ms
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
		//XXX flag tx timeout err
		(void) fifo_skip(TXW, TXW_RP_ISO, its.curlen - its.curpos);
		its.curpos = its.curlen;
		_isotx_done();
		return;
	}

	guardtime = (frclock - iso_ts.last_act);

	if (its.curpos == 0) {
		//if nothing sent yet :
		//make sure guard time is OK for first transmit (p3min in case iso_ts.last_act changed recently)
		if (guardtime < (tparams.p3min * frclock_conv)) {
			//XXX set next int(tparams.p3min*frclock_conv - guardtime);
			return;
		}
	} else {
		//1) check duplex
		switch (dup_state) {
		case WAITDUPLEX:
			//no echo yet
			if (guardtime >= DUPTIMEOUT * frclock_conv) {
				//duplex timeout: unrecoverable
				DBGM("No duplex", duplex_req);
				isotx_abort();
				return;
			}
			//just ret: RX IRQH will tailchain into txworker
			return;
			break;
		case DUP_IDLE:
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
		if (guardtime < (tparams.p4min * frclock_conv)) {
			//XXX set next int (tparams.p4min * frclock_conv - guardtime)
			return;
		}
	}

	//get next byte
	switch (its.tx_mode) {
	case TXM_FIFO:
		if (fifo_rblock(TXW, TXW_RP_ISO, &nextbyte, 1) != 1) {
			DBGM("TXM_FIFO err@", its.curlen);
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
	dup_state = WAITDUPLEX;
	sys_RI(lock);

	ISO_UART->TDR = nextbyte;

	its.curpos += 1;

	iso_ts.last_act = frclock;
	iso_ts.last_TX = iso_ts.last_act;

	guardtime = (DUPTIMEOUT > tparams.p4min)? DUPTIMEOUT : tparams.p4min ;
	//XXX set int(guardtime);
	//if operating with P4<DUPTIMEOUT, either RX IRQH will tailchain into txw,
	// or guardtime IRQ will run txw
	return;

}


//_isotx_done : should be called after msg tx (matches _isotx_start).
// release PMSG if applicable, reset tx_state; set next txw int
// TODO : merge with _abort ?
static void _isotx_done(void) {
	if (its.tx_mode == TXM_PMSG) {
		pmsg_release(its.pm_id);
		pmsg_unq(its.pm_id);
	}
	its.tx_state = TX_IDLE;
	iso_ts.last_TX = frclock;
	iso_ts.last_act = iso_ts.last_TX;
	its.curlen = 0;
	//XXX set next int?
	return;
}

//isotx_abort : aborts current transmission; reset state machines;
//skip current txblock.
//XXX TODO : ensure this can work if called async while in txw...
static void isotx_abort(void) {
	//XXX reset speed (in case slowinit clobbered it)
	//XXX clear TX break
	if (its.tx_state == TX) {
		uint skiplen = its.curlen - its.curpos;
		if (fifo_skip(TXW, TXW_RP_ISO, skiplen) != skiplen ) {
			//can't skip current block ?
			DBGM("txabort can't skip ", skiplen);
			big_error();
			return;
		}
		//XXX generate indication for msgid ?
	}
	its.tx_state = TX_IDLE;
	return;
}


/**** ISO init funcs ****/

//_isotx_startinit : set next int, tx_state, _initstate
static void _isotx_startinit(u16 len, u8 type) {
	u32 now;

	assert(len != 0);

	now = frclock;
	iso_ts.tx_timeout = (ISO_TMAXINIT * frclock_conv);	//die in case init never ends

	//safe, although suboptimal: wait for longest of W5 or Tidle
	iso_initwait = (tparams.w5 < tparams.tidle)? tparams.tidle:tparams.w5;
	iso_initwait *= frclock_conv;

	iso_initlen = len;
	iso_ts.tx_started = now;

	switch (type) {
	case ISO_SLOWINIT:
		assert(len == 1);
		its.tx_state = SI;
		iso_initstate = SI0;
		break;
	case ISO_FASTINIT:
		its.tx_state = FI;
		iso_initstate = FI0;
		break;
	default:
		DBGM("bad init type", type);
		big_error();
		break;
	}
	//XXX set txw to re-enter
	return;
}

//_slowi : slow init; called by isotx IRQH
//maintain current state of iso_initstate; when init is done we need to return 2 keybytes through ioctl...
//build special RX message (dll translates to ioctl resp?) ?
static void _isotx_slowi(void) {
	static u8 kb1, kb2;
	//check if init time was exceeded :
	if ((frclock - iso_ts.tx_started) >= iso_ts.tx_timeout) {
		DBGM("SI timeout", iso_initstate);
		//XXX reset UART;
		iso_initstate = INIT_IDLE;
		isotx_abort();
		return;
	}

	switch (iso_initstate) {
	case SI0: {
		u32 guardtime;
		//make sure guard time is respected:
		guardtime = (frclock - iso_ts.last_act);
		if (guardtime < iso_initwait) {
			//XXX set next int(iso_initwait - guardtime);
			return;
		}
		//XXX setup UART @ 5bps ? + start TX, etc.
		return;
		}
		break;
	case SI1:
		break;
	case SI2:
		break;
	default:
		DBGM("bad SI state", iso_initstate);
		big_error();
		break;
	}
	return;
}	//_slowi
//_fasti : fast init; called by iso txworker
//maintain current state of fast init; when WUP is done start normal TX for StartComm request
static void _isotx_fasti(void) {
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
	case FI0: {
		u32 guardtime;
		//make sure guard time is respected:
		guardtime = (frclock - iso_ts.last_act);
		if (guardtime < iso_initwait) {
			//XXX set next int(iso_initwait - guardtime);
			return;
		}
		//start WUP (W5 or Tidle already elapsed)
		//XXX disable UART;
		//XXX set tx_break;
		//XXX set next int(tparams.tinil);
		iso_initstate = FI1;
		}
		break;
	case FI1:
		//get here after tINI_L (break on TX for fast init)
		//XXX clear TX_break;
		//XXX set next int(tparams.twup - tparams.tinil);
		iso_initstate = FI2;
		break;
	case FI2:
		//after tWUP : TX request using standard mechanism
		//typically we're sending a StartComm req; the response is treated as a normal RX msg
		//XXX purge RX bufs
		//XXX enable USART;
		iso_initstate = INIT_IDLE;
		_isotx_start(iso_initlen, iso_initlen * DEFTIMEOUT, NULL);
		break;
	default:
		DBGM("bad FI state", iso_initstate);
		big_error();
		break;
	}
	return;
}	//_isotx_fasti
