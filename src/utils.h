#ifndef UTILS_H
#define UTILS_H
/********* utils.h *****/

#include <stm32f0xx.h>
#include "stypes.h"

/************ int disable + restore************/
//locking : 2 sauces,  https://forum.pjrc.com/threads/24098-I-don-t-think-critical-sections-work-%28interrupt-locking%29

//ACHTUNG re-ordering ! fuck !  __disable_irq() and __set_PRIMASK "should" be ok because of "memory" in clobberlist...
//Do we need a #define barrier() asm("" ::: "memory") ?

//sys_SDI: save + disable ints
__attribute__((always_inline)) static inline u32 sys_SDI(void) {
	u32 retval = __get_PRIMASK();
	__disable_irq();
	return retval;
	/* ~ equiv of
	uint32_t temp;
	asm volatile("mrs %0, primask\n" : "=r" (temp)::); //CMSIS __get_PRIMASK
	asm volatile("cpsid i":::"memory"); //__disable_irq
	*/
}

//restore ints :
#define sys_RI __set_PRIMASK
//void __set_PRIMASK(u32 pm);
	// fait 		asm volatile ("MSR primask, %0" : : "r" (temp) : "memory"); //__set_PRIMASK

/************ gadgets/helpers ************/
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#define	DBGM(x,n)	dbg_log( __FILE__ ": " x, n)

/************ misc ************/
//big_error() : TODO : reset all periphs except USB, set error status
void big_error(void);

//dbg_log() : TODO, copy message to static error buf, readable by USB or debugger
void dbg_log(const char * dm, int eno);

//assert() : test assert, if failed log message && call big_error
void _assertlog(const char *f1, const char *f2);
#define assert(x) if (!(x)) _assertlog(__FILE__, __func__)

#endif	//UTILS_H
