#ifndef PARAMS_H
#define PARAMS_H

/* timings : j2534-1 p.43+
 * TODO : ajouter funcs pour modif
 */

struct tparams_t {
	u16	p1max;	//(ms) max interbyte for ECU resps
	u16	p3min;	//(ms) min time between end of ECU resp(s) and new tester req
	u16	p4min;	//(ms) min interbyte for tester reqs
	u16	w0;		//(ms) bus idle before slowinit
	u16	w1;		//(ms) tmax pre syncpattern
	u16	w2;		//(ms) tmax pre KB1
	u16	w3;		//(ms) tmax KB1->KB2
	u16	w4;		//(ms) tmax KB2->~KB2
	u16	w5;		//(ms) idle pre slowinit
	u16	tidle;	//(ms) bus idle pre fastinit
	u16	tinil;	//(ms) duration of TX_break of WUP
	u16	twup;	//(ms) total duration of WUP
};

//struct tparams : timing parameters
extern struct tparams_t tparams;

#endif
