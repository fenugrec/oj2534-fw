/* general util funcs for txworkers */

#include "stypes.h"
#include "txwork.h"
#include "msg.h"

#if 0
//semi-useless for the firmware
//start filling a non-special txblock : set hdr, size, timeout fields
//ret 0 if ok
int txw_filltxb(enum msgproto mp, struct txblock *ptxb, u16 siz, u16 to) {
	ptxb->sH = (u8) (siz >> 8);
	ptxb->sL = (u8) siz;
	ptxb->tH = (u8) (to >> 8);
	ptxb->tL = (u8) to;
	ptxb->hdr = (mp << TXB_PROTOSHIFT);
	return 0;
}
#endif
