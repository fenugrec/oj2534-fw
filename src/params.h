#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>
#include "stypes.h"

/* timings : j2534-1 p.43+
 * TODO : access funcs;
 Technically these should be part of chanconf_t, but they're only applicable to ISO
 and we only have 1 ISO channel.
 */

//struct isoparams : parameters for ISO chan
struct isoparams_t {
	_Bool	k_only;	//K or K+L for init; (set in Connect)

	u32	DATA_RATE;	//bps
	_Bool LOOPBACK;

	u16	p1max;	//(ms) max interbyte for ECU resps
	u16	p3min;	//(ms) min time between end of ECU resp(s) and new tester req
	u16	p4min;	//(ms) min interbyte for tester reqs
	u16	w0;		//(ms) bus idle before slowinit
	u16	w1;		//(ms) tmax pre syncpattern
	u16	w2;		//(ms) tmax pre KB1
	u16	w3;		//(ms) tmax KB1->KB2
	u16	w4;		//(ms) tmax KB2->~KB2
	u16	w5;		//(ms) idle pre slowinit
	u16	tidle;	//(ms) bus idle pre fastinit
	u16	tinil;	//(ms) duration of TX_break of WUP
	u16	twup;	//(ms) total duration of WUP

	u8	PARITY;	//0 : none, 1: O, 2: E
	_Bool	b8;	//8 databit, else 7
	u8		FIVE_BAUD_MOD;

};
extern struct isoparams_t isoparams;

//params for CAN / iso15765 channel
struct canparams_t {
	u32	DATA_RATE;	//bps
	_Bool LOOPBACK;
	u8	BIT_SPL_POINT;
	u8	SYNC_JMP_W;
	u8	ISO15765_BS;
	u8	ISO15765_STMIN;
	u16	BS_TX;
	u16	STMIN_TX;
	u8	ISO15765_WFT_MAX;
};

/* params for J1850 channel, not impl
struct j1850params_t {
	u32	DATA_RATE;	//bps
	_Bool LOOPBACK;
	u8	NODE_ADDR;
	u8	NET_LINE;

};
*/

/* params for SCI chan, not impl
struct sciparams_t {
	u32	DATA_RATE;	//bps
	_Bool LOOPBACK;
	//T1 ... T5
};
*/

#endif
