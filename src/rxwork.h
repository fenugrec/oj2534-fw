#ifndef RXWORK_H
#define RXWORK_H
/* RX worker stuff

RX workers parse whatever's on their bus, filter (optional), and
feed "rxb"s (rx blocks) through the RXW fifo so USB can forward to host.
ADC worker will also use these
*/

#include "stypes.h"
#include "msg.h"

#define RXBS_MINSIZE	6	//size of 'struct rxblock' excluding data
struct rxblock {
	u8	hdr;
		#define RXB_TYPE	(1<<7)
		//if type==1 : "normal", from rx builders :
			#define RXB_NPSHIFT	5	//enum msgproto == (flags & PROTOMASK) >> PROTOSHIFT
			#define RXB_NPMASK	(0x3 << RXB_NPSHIFT)
		//if type==0 : reserved

		#define RXB_SEQCONT		0x0	//continues a multi-block msg
		#define RXB_SEQSTART	0x1	//start of a multi-block msg
		#define	RXB_SEQEND		0x2	//last block (possibly 0 data)
		#define RXB_SEQTOTAL	0x3	// == (SEQSTART | SEQEND) : message entirely contained in this block
		#define RXB_SEQMASK		0x3	//rxb sequence == (hdr & RXB_SEQMASK)

	u8	len;	//total block size (== 2 + 4 + datalen)
	u8	ts[4];		//32-bit timestamp from frclock : end of last bit of the message; == ts[0]<<24 + ts[1]<<16 + ts[2]<<8 + ts[3]
	u8	data[];
};
#endif // RXWORK_H
