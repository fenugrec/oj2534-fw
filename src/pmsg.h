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

TODO : copy pmsg data instead of claim/release crap ?
TODO : pmsg_del + on a claimed message... async completion notif ?
TODO : merge pmsg_release && pmsg_unq ?
*/

#include "stypes.h"
#include "msg.h"

// PMSG ids = 0 to (PMSG_MAXNUM -1)
#define PMSG_MAXNUM 10	//J2534-1 , 7.2.7

/* func protos */

//pmsg_init : clear data, setup tmr
void pmsg_init(void);

//attempt to add a msg in slot 'id'
//ret 0 if ok
int pmsg_add(uint id, enum msgproto mp, u16 per, uint len, u8 *data);

//delete or queue for deletion a pmsg; return -1 if queued, 0 if ok
//for use by USB command dispatch
int pmsg_del(uint id);

//find,claim, get info for a queued pmsg with proto <mprot>;
// rets ptr to data + sets len and pmid if success, NULL if no msg claimed.
u8 * pmsg_claim(enum msgproto mprot, uint *len, uint *pmid);

//clear BUSY flag, delete message if queued for del; always succed
void pmsg_release(uint id);

//clear TXQ flag: always succeed
void pmsg_unq(uint id);

#endif	//PMSG_H
