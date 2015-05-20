#ifndef UTILS_H
#define UTILS_H
/********* utils.h *****/

#include <stm32f0xx.h>

/************ int disable + restore************/
//lockage ints : 2 sauces,  https://forum.pjrc.com/threads/24098-I-don-t-think-critical-sections-work-%28interrupt-locking%29
//ACHTUNG re-ordering ! fuck !  __disable_irq() et __set_PRIMASK "devraient" etre ok parcqu'ils ont "memory" dans clobberlist.
//faudrait ptet un #define barrier() asm("" ::: "memory")
//sys_SDI: save + disable ints
__attribute__((always_inline)) static inline u32 sys_SDI(void) {
	u32 retval = __get_PRIMASK();
	__disable_irq();
	return retval;
	/* ~ equiv a
	uint32_t temp;
	asm volatile("mrs %0, primask\n" : "=r" (temp)::); //CMSIS __get_PRIMASK
	asm volatile("cpsid i":::"memory"); //__disable_irq
	*/
}

//restorer ints :
#define sys_RI __set_PRIMASK
//void __set_PRIMASK(u32 pm);
	// fait 		asm volatile ("MSR primask, %0" : : "r" (temp) : "memory"); //__set_PRIMASK

/************ timing ************/
// master, free-running clock (u32, autowrap, 10kHz, no ints)
#define frclock	TIM2->CNT
#define frclock_conv	10	//so time(ms) = frclock / frclock_conv

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
