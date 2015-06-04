//fifos.c
// tx/rx worker fifos

//TODO : atomic types for rp, wp members
//ensure no simult calls to fifo_wblock* on the same fifo

// next read is @  *rp, next write *wp;
// if rp==wp : no data ready to read
// if wp+1 == rp : full, can't write
//

#include <stddef.h>
#include "fifos.h"
#include "stypes.h"
#include "utils.h"

/* TXW fifo; see txwork.h */
/* RXW fifo
	0- J2534 says msg len <= 4128 !
	1- dynamic mem alloc is bad
	2- not enough RAM to give a 4k buffer to every RX builder
	Solution : rx code builds rxb "chunks"; see rxwork.h
*/


struct fifo {
	uint nrp;	//number of used read ptrs in rp[]
	volatile uint rp[FIFO_MAX_RPTRS];	//enum fifo_rp indexes into this
	volatile uint wp;	//always 1 write ptr
	uint siz;
	u8 *data;
};

#define TXW_SIZE	400	//TODO : tx message splitting
#define RXW_SIZE	4200	//was 4200 :
static struct fifo worker_fifos[FIFO_NUM];
static u8 txwf_data[TXW_SIZE];
static u8 rxwf_data[RXW_SIZE];

//fifo_init() : call once on init ! TODO : move to declarator?
void fifo_init(void) {
	//init TXW fifo
	worker_fifos[TXW].nrp = 2;
	worker_fifos[TXW].rp[TXW_RP_ISO]=0;
	worker_fifos[TXW].rp[TXW_RP_CAN]=0;
	worker_fifos[TXW].wp=0;
	worker_fifos[TXW].siz = ARRAY_SIZE(txwf_data);
	worker_fifos[RXW].data = txwf_data;
	//init RXW fifo
	worker_fifos[RXW].nrp = 1;
	worker_fifos[RXW].rp[RXW_RP]=0;
	worker_fifos[RXW].wp=0;
	worker_fifos[RXW].siz = ARRAY_SIZE(rxwf_data);
	worker_fifos[RXW].data = rxwf_data;
	return;
}

//write block : write len bytes; advance ptr && ret len if success. Locks a lot
//ret 0 if incomplete
uint fifo_wblock(enum fifo_id fid, u8 *src, const uint len) {
	u32 lock;
	uint done;
	uint nrp, min_rp;	//to parse through rptrs
	u8 *data;
	volatile uint *wpp;
	uint wp;	//temp indexer
	uint siz;

	if (fid >= ARRAY_SIZE(worker_fifos) ||
		src == NULL) return 0;

	siz = worker_fifos[fid].siz;
	data = worker_fifos[fid].data;
	wpp = &worker_fifos[fid].wp;
	nrp = worker_fifos[fid].nrp;

	lock=sys_SDI();
	wp = *wpp;

	//find "earliest" read ptr : calc "used_data" = dist:
	//	if rp>wp : dist = siz -rp + wp
	//	if rp<=wp : dist = wp - rp

	for (min_rp = 0; nrp > 0; nrp--) {
		static uint max_used = 0;
		uint used,rp;
		rp = worker_fifos[fid].rp[nrp-1];
		used = (rp > wp)? (siz - rp + wp):(wp - rp);
		if (used > max_used) {
			max_used = used;
			min_rp = rp;
		}
	}
	//here : min_rp == read index that will be encountered soonest
	for (done=0; done < len; done++) {
		data[wp] = *src;	//tentative write
		wp++;
		src++;
		if (wp == siz) wp=0;
		if (wp == min_rp) {
			//full !  => cancel
			sys_RI(lock);
			return 0;
		}

	}
	*wpp = wp;	//update wp
	sys_RI(lock);
	return done;
}

//_wblockf: write len bytes; advance ptr && ret len if success. Doesn't disable ints (XXX make rp, wp _Atomic)
//ret 0 if incomplete; not re-entrant if called with the same fifo (XXX add write ptr lock ??)
uint fifo_wblockf(enum fifo_id fid, u8 *src, const uint len) {
	uint done;
	uint rpi;	//rp index
	uint nrp, siz;
	u8 *data;
	volatile uint *wpp;
	uint wp;	//temp indexer

	if (fid >= ARRAY_SIZE(worker_fifos) ||
		src == NULL) return 0;

	siz = worker_fifos[fid].siz;
	data = worker_fifos[fid].data;
	wpp = &worker_fifos[fid].wp;
	nrp = worker_fifos[fid].nrp;

	wp = *wpp;

	//Copy data, check against all read ptrs
	for (done=0; done < len; done++) {
		data[wp] = *src;	//tentative write
		wp++;
		src++;
		if (wp == siz) wp=0;
		for (rpi=0; rpi < nrp; rpi++) {
			if (wp == worker_fifos[fid].rp[rpi]) {
				//full => cancel
				return 0;
			}
		}
	}
	*wpp = wp;	//update wp
	return done;
}


//uint fifo_rblock : copy len bytes, advance ptr if success. Locks during whole read
//ret copied len if success, else 0
uint fifo_rblock(enum fifo_id fid, enum fifo_rp rpid, u8 * dest, const uint len) {
	u32 lock;
	uint siz, done, rp, wp;
	volatile uint *rpp;	//to modify rp once done
	u8 *data;

	if (fid >= ARRAY_SIZE(worker_fifos) ||
		rpid >= worker_fifos[fid].nrp ||
		dest == NULL) return 0;

	siz = worker_fifos[fid].siz;
	data = worker_fifos[fid].data;
	rpp = &worker_fifos[fid].rp[rpid];

	lock=sys_SDI();
	wp = worker_fifos[fid].wp;
	rp= *rpp;	//get current index

	for (done=0; done < len; done++) {
		if (rp == wp) {
			sys_RI(lock);
			return 0;
		}
		*dest = data[rp];
		dest++;
		rp++;
		if (rp == siz) rp=0;
	}
	*rpp = rp;	//update readptr
	sys_RI(lock);
	return done;
}

//uint fifo_rblockf : copy len bytes, advance ptr if success. No IRQ disable ==> can't be called simultaneously on the same fifo/rp !
//ret copied len if success, else 0
uint fifo_rblockf(enum fifo_id fid, enum fifo_rp rpid, u8 * dest, const uint len) {
	uint siz, done, rp;
	volatile uint *rpp;	//to modify rp once done
	volatile uint *wpp;
	u8 *data;

	if (fid >= ARRAY_SIZE(worker_fifos) ||
		rpid >= worker_fifos[fid].nrp ||
		dest == NULL) return 0;

	siz = worker_fifos[fid].siz;
	data = worker_fifos[fid].data;
	rpp = &worker_fifos[fid].rp[rpid];
	wpp = &worker_fifos[fid].wp;
	rp= *rpp;	//get current index

	for (done=0; done < len; done++) {
		if (rp == *wpp) {
			return 0;
		}
		*dest = data[rp];
		dest++;
		rp++;
		if (rp == siz) rp=0;
	}
	*rpp = rp;	//update readptr
	return done;
}


//_cblock : copy up to (len) bytes without adjusting rptr !
//ret copied len if 100% success, 0 if incomplete.
uint fifo_cblock(enum fifo_id fid, enum fifo_rp rpid, u8 * dest, const uint len) {
	u32 lock;
	uint done, siz;
	uint rp, wp;
	u8 *data;

	if (fid >= ARRAY_SIZE(worker_fifos) ||
		rpid >= worker_fifos[fid].nrp ||
		dest == NULL) return 0;

	siz = worker_fifos[fid].siz;
	data = worker_fifos[fid].data;
	wp = worker_fifos[fid].wp;

	lock=sys_SDI(); //lock to prevent _rblock from clearing data
	rp = worker_fifos[fid].rp[rpid];

	for (done=0; done < len; done++) {
		if (rp == wp) {
			done=0;	//"caught up"; not enough data
			break;
		}
		*dest = data[rp];
		dest++;
		rp++;	//doesn't update fifo ptr
		if (rp == siz) rp=0;
	}
	sys_RI(lock);
	return done;
}

//_rlen : ret # of bytes available
//internal use only; no bounds checking. Maybe inline? or delete
uint fifo_rlen(enum fifo_id fid, enum fifo_rp rpid) {
	uint rp,wp;
	uint siz;

	siz = worker_fifos[fid].siz;

	rp = worker_fifos[fid].rp[rpid];
	wp = worker_fifos[fid].wp;

	return (rp > wp)? (siz - rp + wp):(wp - rp);
}

//fifo_skip : advance readptr if possible. ret len if success, else 0
uint fifo_skip(enum fifo_id fid, enum fifo_rp rpid, const uint len) {
	u32 lock;
	uint rp,wp;
	volatile uint *rpp;
	uint rlen, siz, done;

	if (fid >= ARRAY_SIZE(worker_fifos) ||
		rpid >= worker_fifos[fid].nrp) return 0;

	siz = worker_fifos[fid].siz;
	if (len >= siz) return 0;

	rpp = &worker_fifos[fid].rp[rpid];
	wp = worker_fifos[fid].wp;

	lock=sys_SDI();
	rp = *rpp;

	if (wp >= rp) {
		//no wrap
		rlen = (wp-rp);
	} else {
		//wrap
		rlen = (siz - rp + wp);
	}
	if (len <= rlen) {
		*rpp += len;
		done = len;
	} else {
		done = 0;
	}
	sys_RI(lock);
	return done;
}
