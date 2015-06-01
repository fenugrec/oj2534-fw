#ifndef TXWORK_H
#define TXWORK_H

// txwork.h
// public stuff for feeding TX workers

#include "stypes.h"

//implem of txblocks within fifo; use u8 because of unaligned reads
struct txblock {
	u8	hdr;
		#define TXB_SENDABLE	(1<<7)	//block is complete & ready to send (write last!)
		#define TXB_PROTOSHIFT	5
		#define TXB_PROTOMASK	0x3	//enum msgproto = hdr&PROTOMASK >> PROTOSHIFT
		#define TXB_SPECIAL	(1<<4)	//flag special blocks (fast/slow init)
		#define TXB_IDMASK 0x7	//2..0 : msgid (pour fb avec dll)
	u8	sH;
	u8	sL;	//u16 sizeof (*data) == (sH<<8) | sL
	u8	tH;
	u8	tL;	//u16 timeout (ms)
	#define TXB_DATAPOS	5	// &txblock + TXB_datapos == txblock.data
	u8	data[];	//"flexible length array"
};

/*** format of special txblocks (i.e. (txblock.hdr & TXB_SPECIAL) ***/
/*** if mproto ==ISO : ***/
//TODO : j2534-1 7.3.5, p.47 : "FIVE_BAUD_MOD" config
// timeout is ignored
// data[0] = type :
enum txb_spectype {ISO_SLOWINIT=0, ISO_FASTINIT=1};
// case ISO_SLOWINIT:	data[1] = tgt addr (ex.: 0x33)
// case ISO_FASTINIT:	data[1] = msg data (ex.: StartComm, 0xC1 0x33 0xFC 0x81 cks)

#endif
