// pmsg.c
/************ periodic msg shits ************/
//TODO : pmsg_add() stuff

#include <stddef.h>

#include "pmsg.h"
#include "msg.h"
#include "stypes.h"
#include "utils.h"

#define PMSG_TMR	TIM16	//XXX
#define PMSG_IRQH	TIM16_IRQHandler
#define PMSG_MAX_SIZE	12	//must fit in PMSG_LENMASK

//access to ->flags NEEDS to be atomic
struct pmsg_desc {
	u32 flags;
		#define PMSG_ENABLED	1<<31	//data valid & sendable. set by pmsg_add(id), cleared by pmsg_del(id)
		#define PMSG_TXQ	1<<26	//msg is queued for TX ( pmsg_unq() after tx)
		#define PMSG_DELQ	1<<25	//msg is queued for deletion (pmsg_del(id))
		#define	PMSG_BUSY	1<<24	//msg is in use (pmsg_claim(), pmsg_release())
		#define PMSG_PROTOSHIFT	20	//enum msgproto = (flags & PROTOMASK) >> PROTOSHIFT
		#define PMSG_PROTOMASK	0x3 << PMSG_PROTOSHIFT
		#define PMSG_LENSHIFT	16	//msg len = (flags & LENMASK)>>LENSHIFT
		#define PMSG_LENMASK	0xF << PMSG_LENSHIFT
		#define PMSG_PERMASK	0xffff	//period (ms) = flags & PMSG_PERMASK
	u8 data[PMSG_MAX_SIZE];
};


// PMSG ids = 0 to (PMSG_MAXNUM -1)

static struct pmsg_desc pmsg_table[PMSG_MAXNUM];	//all possible periodic msgs.

//rearm : set PMSG_TMR to expire in 'next' ms
//(internal use only?)
static void _pmsg_rearm(u16 next) {
	PMSG_TMR->CNT = next;
	//XXX set reload repeats to 1 + start
	return;
}

/*************/
/* technique 1 : maintain countdowns per PMSG, int on next soonest, and set PMSG_TXQ flag.
 *  pmsg_IRQH : semi-low prio INT on TMRx overflow/expiry.
 */
void PMSG_IRQH(void) {
	//XXX if (tmr expired)
	pmsg_IRQH();
	return;
}
void pmsg_IRQH(void) {
	static u16 pmsg_countdown[ARRAY_SIZE(pmsg_table)];	//countdown=0 means disabled
	static int idx_next;	//id of expired countdown
	u16 min;
	u16 newmin;	//finding next "soonest" msg
	u32 pflags;
	u32 lock;
	
	min = pmsg_countdown[idx_next];	//this has just elapsed;
	newmin = (u16) -1;
	
	lock=sys_SDI();	//yuck, but no choice
	for (uint i=0; i< ARRAY_SIZE(pmsg_table); i++) {
		pflags = pmsg_table[i].flags;
		if ((pflags & PMSG_ENABLED) == 0) {
			//this makes sure new msgs will be queued as soon as added
			pmsg_countdown[i]=0;
			continue;
		}
		if (pmsg_countdown[i] <= min) {
			//queue + reload all due msgs
			pflags |= PMSG_TXQ;
			pmsg_table[i].flags = pflags;
			pmsg_countdown[i] = (u16) (pflags & PMSG_PERMASK);
			
		} else {
			//decrement others
			pmsg_countdown[i] -= min;
		}
		// find next soonest && enabled msgid.
		if (pmsg_countdown[i] < newmin) {
			newmin = pmsg_countdown[i];
			idx_next = i;
		}
	}
	sys_RI(lock);

	_pmsg_rearm(pmsg_countdown[idx_next]);	//reload + restart tmr
	return;
}

//delete or queue for deletion a pmsg; return -1 if queued, 0 if ok
//for use by USB command dispatch
int pmsg_del(uint id) {
	u32 lock;
	u32 pflags;
	int rv;
	if (id >= ARRAY_SIZE(pmsg_table)) return 0;
	lock=sys_SDI();
	pflags = pmsg_table[id].flags;
	if ((pflags & PMSG_ENABLED) ==0) {
		//already disabled == deleted
		rv = 0;
	} else if (pflags & PMSG_BUSY) {
		//queue deletion and signal business
		pmsg_table[id].flags = pflags | PMSG_DELQ;
		rv = -1;
	} else {
		//enabled and not busy : delete !
		pmsg_table[id].flags = 0;
		rv = 0;
	}
	sys_RI(lock);
	return rv;	
}

//set BUSY flag; ret 0 if ok, -1 if error (PMSG not enabled, or not queued etc)
int pmsg_claim(uint id) {
	u32 lock;
	u32 pflags;
	int rv;
	if (id >= ARRAY_SIZE(pmsg_table)) return -1;
	lock=sys_SDI();
	pflags = pmsg_table[id].flags;
	if (((pflags & PMSG_ENABLED) ==0) ||
		((pflags & PMSG_TXQ) == 0)) {
		rv = -1;
	} else {
		pmsg_table[id].flags = pflags | PMSG_BUSY;
		rv = 0;
	}
	sys_RI(lock);
	return rv;
}

//clear BUSY flag, delete message if queued for del; always succed
void pmsg_release(uint id) {
	u32 lock;
	u32 pflags;
	if (id >= ARRAY_SIZE(pmsg_table)) return;
	lock=sys_SDI();
	pflags = pmsg_table[id].flags;
	if (pflags & PMSG_DELQ) pflags = 0;
	pflags &= ~PMSG_BUSY;
	sys_RI(lock);
	return;
}

//clear TXQ flag: always succeed
void pmsg_unq(uint id) {
	u32 lock;
	if (id >= ARRAY_SIZE(pmsg_table)) return;
	lock=sys_SDI();
	pmsg_table[id].flags &= ~PMSG_TXQ;
	sys_RI(lock);
	return;
}

//get proto of req'd message.
//caller MUST have claimed the msg first
enum msgproto pmsg_getproto(uint id) {
	u32 pflags;
	if (id >= ARRAY_SIZE(pmsg_table)) return MP_INVALID;
	pflags = pmsg_table[id].flags;
	if ((pflags & PMSG_ENABLED) ==0 ||
		(pflags & PMSG_BUSY) ==0 ) return MP_INVALID;
	return (enum msgproto) ((pflags & PMSG_PROTOMASK) >> PMSG_PROTOSHIFT);
}

//get data ptr + datalen for req'd message.
// caller MUST have claimed the msg first
//ret NULL if error || disabled || unclaimed
u8 * pmsg_getmsg(uint id, uint *len) {
	u32 pflags;
	u32 lock;
	if (id >= ARRAY_SIZE(pmsg_table) ||
		len==NULL) return NULL;
	lock=sys_SDI();
	pflags = pmsg_table[id].flags;
	if ((pflags & PMSG_ENABLED) ==0 ||
		(pflags & PMSG_BUSY) ==0 ) {
		sys_RI(lock);
		return NULL;
	}
	sys_RI(lock);	//we can unlock since we confirmed msg is claimed
	
	*len = (pflags & PMSG_LENMASK) >> PMSG_LENSHIFT;
	return pmsg_table[id].data;
}
