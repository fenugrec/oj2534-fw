#ifndef PMSG_H
#define PMSG_H

// pmsg.h
// stuff pour periodicmsgs; {txworker*, USB commands} ont besoin de ca
// tentative de rien publier en-dehors de pmsg.c

/*
7.2.7 periodic msgs : max len=12B incluant hdr "or CAN id"; prio sur messages avec PTwm; respecte
bus idle (P3min etc), "generate txdone (15765) indications and loopback msgs if enabled). Sur 15765, le periodic peut txer pendant un multi-frame tx/rx.
Supporter au moins 10 periodic msgs = 10*(12B + timing(2B) + proto/flags(1B) + ?), mettons 160B.
Periode = [5, 65535]ms.
Techniques : voir pmsg.{h,c}
	- 3: int chaque 1ms, check si chaque msgtimer expired, queue-er msgs... overhead a voir
	- 2: sorter msgs, setting tmr pour le "next due" ? mieux, IRQH est plus rapide XXXX FLAWED
	-> 1: garder "countdown" pour chaque tmr, prendre le "most soonest" et inter.
*/

#include "stypes.h"
#include "msg.h"

// PMSG ids = 0 to (PMSG_MAXNUM -1)
#define PMSG_MAXNUM 10	//J2534-1 , 7.2.7

/* func protos */

//interrupt handler (TMR. expiry..)
void pmsg_IRQH(void);

//delete or queue for deletion a pmsg; return -1 if queued, 0 if ok
//for use by USB command dispatch
int pmsg_del(uint id);

//set BUSY flag; ret 0 if ok, -1 if error (PMSG not enabled, or not queued etc)
//caller can loop through valid IDs to find next queued msg
//once a pmsg is claimed, it's protected from modif + deletion
int pmsg_claim(uint id);

//clear BUSY flag, delete message if queued for del; always succed
void pmsg_release(uint id);

//clear TXQ flag: always succeed
void pmsg_unq(uint id);

//get proto of req'd message.
//caller MUST have claimed the msg first
enum msgproto pmsg_getproto(uint id);

//get data ptr + datalen for req'd message.
// caller MUST have claimed the msg first
//ret NULL if error || disabled
u8 * pmsg_getmsg(uint id, uint *len);

#endif	//PMSG_H
