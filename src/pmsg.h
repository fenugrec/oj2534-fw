#ifndef PMSG_H
#define PMSG_H

// pmsg.h
// stuff for periodicmsgs; {txworker*, USB commands} need these

/*
7.2.7 periodic msgs : max len=12B including hdr "or CAN id"; prioritize PTwm; respect
bus idle (P3min etc), "generate txdone (15765) indications and loopback msgs if enabled)" ??.
For 15765, periodic could tx during multi-frame tx/rx.
Support >= 10 periodic msgs : ~ 10*(4B (timing+proto/flags) + 12B)
Period = [5, 65535]ms.
Techniques :
	- 3: int every X ms, check every msgtimer expiry, queue msg... prob=overhead
	- 2: sort msgs, set tmr for next due - less overhead. ==> flawed !
	-> 1: maintain "countdown" for each pmsg, int on next "most soonest"

*/

#include "stypes.h"
#include "msg.h"

// PMSG ids = 0 to (PMSG_MAXNUM -1)
#define PMSG_MAXNUM 10	//J2534-1 , 7.2.7 : "support at least 10 pmsgs"

#define PMSG_MAX_SIZE	12	//J2534-1, 7.2. : "pmsg limited in length to a single msg <= 12B,
							//including header / CAN ID"

/* func protos */

//pmsg_init : clear data, setup tmr
void pmsg_init(void);

//attempt to add a msg in slot 'id'
//ret 0 if ok
int pmsg_add(uint id, enum msgproto mp, u16 per, uint len, u8 *data);

//delete pmsg <id>. always succeed
//for use by USB command dispatch
void pmsg_del(uint id);

//pmsg_get: find next queued pmsg with matching proto;
//ret 0 and fill caller's *buf, *pmlen, *pmid if ok.
int pmsg_get(enum msgproto mp, u8 *buf, uint * pmlen, uint *pmid);

#endif	//PMSG_H
