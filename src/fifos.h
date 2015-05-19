#ifndef FIFOS_H
#define FIFOS_H

//fifos.h
//public stuff for fifos (txworker, rxworker)

#include "stypes.h"

#define FIFO_NUM 2
enum fifo_id {TXW=0, RXW=1};	//index into worker_fifos[]

#define FIFO_MAX_RPTRS 2
// TXW_RP_* : read ptr IDs for worker_fifos[fifo_id].rp[]
enum fifo_rp {TXW_RP_ISO=0, RXW_RP=0,
	TXW_RP_CAN=1};

//fifo_init() : call once on init ! TODO : move to declarator?
void fifo_init(void);

//write block : write len bytes; advance ptr && ret len if success.
//ret 0 if incomplete
uint fifo_wblock(enum fifo_id fid, u8 *src, const uint len);

//wblockf : wblock without disable ints; but slower loop
uint fifo_wblockf(enum fifo_id fid, u8 *src, const uint len);

//uint fifo_rblock : copy len bytes, advance ptr if success. Locks during whole read
//ret copied len if success, else 0
uint fifo_rblock(enum fifo_id fid, enum fifo_rp rpid, u8 * dest, const uint len);

//_cblock : copy up to (len) bytes without adjusting rptr !
//ret copied len if 100% success, 0 if incomplete.
uint fifo_cblock(enum fifo_id fid, enum fifo_rp rpid, u8 * dest, const uint len);

//fifo_skip : advance readptr if possible. ret len if success, else 0
uint fifo_skip(enum fifo_id fid, enum fifo_rp rpid, const uint len);

#endif