// pmsg.c
/************ periodic msg shits ************/

#include <stddef.h>
#include <string.h>

#include "timers.h"
#include "pmsg.h"
#include "msg.h"
#include "stypes.h"
#include "utils.h"

//access to ->flags NEEDS to be atomic
struct pmsg_desc {
	u32 flags;
		#define PMSG_ENABLED	(1<<31)	//data valid & sendable. set by pmsg_add(id), cleared by pmsg_del(id)
		#define PMSG_NEW	(1<<30)	//freshly added (set in pmsg_add; cleared in pmsg_work)
		#define PMSG_TXQ	(1<<26)	//msg is queued for TX ( pmsg_unq() after tx)
		#define PMSG_DELQ	(1<<25)	//msg is queued for deletion (pmsg_del(id))
		#define PMSG_PROTOSHIFT	20	//enum msgproto = (flags & PROTOMASK) >> PROTOSHIFT
		#define PMSG_PROTOMASK	(0x3 << PMSG_PROTOSHIFT)
		#define PMSG_LENSHIFT	16	//msg len = (flags & LENMASK)>>LENSHIFT
		#define PMSG_LENMASK	(0xF << PMSG_LENSHIFT)
		#if ((PMSG_LENMASK >> PMSG_LENSHIFT) < PMSG_MAX_SIZE)
			#error	bad PMSG MAXLEN !
		#endif
		#define PMSG_PERMASK	0xffff	//period (ms) = flags & PMSG_PERMASK
	u8 data[PMSG_MAX_SIZE];
};

void pmsg_work(void);

// PMSG ids = 0 to (PMSG_MAXNUM -1)
static struct pmsg_desc pmsg_table[PMSG_MAXNUM];	//all possible periodic msgs.


/*************/
/* technique 1 : maintain countdowns per PMSG, int on next soonest, and set PMSG_TXQ flag.
 *
 */

//clear pmsg table
void pmsg_init(void) {

	for (uint i=0; i< ARRAY_SIZE(pmsg_table); i++) {
		pmsg_table[i].flags = 0;
	}
	return;
}

//PMSG_IRQH : semi-low prio INT on TMRx overflow/expiry.
void PMSG_IRQH(void) {
	TIM_Cmd(PMSG_TMR, DISABLE);	//cancel in case it was a forced int
	TIM_ClearFlag(PMSG_TMR, TIM_FLAG_Update);
	NVIC_ClearPendingIRQ(IRQN_PMSG);
	pmsg_work();
	return;
}

//worker, called ONLY from IRQH handler (TMR expiry). Tolerates polling but not recursion
void pmsg_work(void) {
	static u16 pmsg_countdown[ARRAY_SIZE(pmsg_table)];	//countdown=0 means disabled
	static u32 last_run=0;	//frclock @ last countdown update.
	u16 delta_ms;		//ms elapsed since last countdown update
	u16 newmin;	//finding next "soonest" msg
	u32 pflags;
	u32 lock;

	delta_ms = (frclock - last_run) / frclock_conv;	//OK cast if <65536 ms !

	newmin = (u16) -1;

	lock=sys_SDI();	//yuck, but no choice
	for (uint i=0; i< ARRAY_SIZE(pmsg_table); i++) {
		pflags = pmsg_table[i].flags;
		if ((pflags & PMSG_ENABLED) == 0) continue;
		if (pflags & PMSG_NEW) {
			//this makes sure new msgs will be queued as soon as added
			pmsg_countdown[i]=0;
			pmsg_table[i].flags &= ~PMSG_NEW;
		}
		if (pmsg_countdown[i] <= delta_ms) {
			//queue + reload all due msgs
			pmsg_table[i].flags |= PMSG_TXQ;
			pmsg_countdown[i] = (u16) (pflags & PMSG_PERMASK);
		} else {
			//decrement others
			pmsg_countdown[i] -= delta_ms;
		}
		// find next soonest && enabled msgid.
		if (pmsg_countdown[i] <= newmin) {
			newmin = pmsg_countdown[i];
		}
	}	//for pmsg_table
	last_run = frclock;
	sys_RI(lock);

	pmsg_setint(newmin);	//reload + restart tmr
	return;
}

//attempt to add a msg in slot 'id'. NOT re-entrant!
//ret 0 if ok
int pmsg_add(uint id, enum msgproto mp, u16 per, uint len, u8 *data) {
	u32 pflags;

	assert((len > 0) && (mp < MP_INVALID) &&
			(len <= PMSG_MAX_SIZE) && (data != NULL));

	pflags = pmsg_table[id].flags;
	if (pflags & PMSG_ENABLED)  {
		DBGM("msgid exists", id);
		return -1;	//msgid exists !?
	}

	memcpy(pmsg_table[id].data, data, len);
	pflags = PMSG_ENABLED | PMSG_NEW | (mp << PMSG_PROTOSHIFT) |
			(len << PMSG_LENSHIFT) | per ;
	pmsg_table[id].flags = pflags;
	pmsg_setint(0);	//force reparse
	return 0;
}

//delete pmsg <id>. always succeed
//for use by USB command dispatch
void pmsg_del(uint id) {
	assert(id < ARRAY_SIZE(pmsg_table));

	pmsg_table[id].flags = 0;	//clear PMSG_ENABLED == delete  !
	return;
}


//pmsg_get: find next queued pmsg with matching proto;
//ret 0 and fill *buf, *pmlen, *pmid if ok.
int pmsg_get(enum msgproto mp, u8 *buf, uint * pmlen, uint *pmid) {

	u32 lock;
	uint id;
	u32 pflags;

	assert((buf != NULL) && (pmlen != NULL) && (pmid != NULL));

	lock=sys_SDI();

	for (id=0; id < ARRAY_SIZE(pmsg_table); id++) {
		pflags = pmsg_table[id].flags;
		if (((pflags & PMSG_ENABLED) ==0) ||
			((pflags & PMSG_TXQ) == 0)) {
			continue;
		}
		if ( ((pflags & PMSG_PROTOMASK) >> PMSG_PROTOSHIFT) == mp) {
			*pmlen = (pflags & PMSG_LENMASK) >> PMSG_LENSHIFT;
			*pmid = id;
			memcpy(buf, pmsg_table[id].data, *pmlen);
			pmsg_table[id].flags &= ~PMSG_TXQ;	//auto-unqu !
			sys_RI(lock);
			return 0;
		}
	}	//for (pmsg)
	sys_RI(lock);

	return -1;	//no pmsg found
}
