/*
TODO : split testmode in a separate file
*/

#include <stdint.h>
#include <stddef.h>	//for #define NULL !
//#include <stdatomic.h>

#include "oj2534.h"
#include <stm32f0xx.h>
#include "stypes.h"
#include "utils.h"

#include "fifos.h"
#include "iso.h"
#include "timers.h"
#include "pmsg.h"

u32 SystemCoreClock;	//current clock freq (Hz)

#ifdef TESTING
#include "txwork.h"
#include "fifos.h"
void test_fifo(void);
void test_pmsg1(void);
void test_pmsg2(void);
void test_isotx1(void);

#endif // TESTING

void SystemInit(void) {
#ifdef TESTING
	//use 8MHz HSI
	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Set HSION bit */
	RCC->CR |= RCC_CR_HSION;

	RCC->CFGR &= ~(RCC_CFGR_PLLNODIV | RCC_CFGR_MCO_PRE | RCC_CFGR_MCO |
				RCC_CFGR_ADCPRE | RCC_CFGR_PPRE | RCC_CFGR_HPRE | RCC_CFGR_SW);

	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);

	RCC->CR &= ~RCC_CR_HSEBYP;

	/* Reset PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
	RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC);

	RCC->CFGR2 &= ~(RCC_CFGR2_PREDIV1);

	RCC->CFGR3 &= ~(RCC_CFGR3_USART2SW | RCC_CFGR3_ADCSW | RCC_CFGR3_CECSW |
				RCC_CFGR3_I2C1SW | RCC_CFGR3_USART1SW);

	RCC->CR2 &= ~RCC_CR2_HSI14ON;

	SystemCoreClock = 8*1000*1000;
#else
	//setup USB PLL + CRS etc
#endif // TESTING
	return;
}

#ifdef TESTING
int main(void) {
	u32 tstart;

	timers_init();
	fifo_init();
	isotx_init();
	pmsg_init();

	test_fifo();

	tstart = frclock;
	test_pmsg1();

	while ((frclock - tstart) < (1200 * frclock_conv)) {
		//let PMSG worker loop a few times
		//verify countdown mechanism + TXQ flagging
	}
	test_pmsg2();

	while (1) {
		static int t1done=0;
		isotx_qwork();

		//txwork test 1 : run once
		if (!t1done) {
			if ((frclock - tstart) > (1300 * frclock_conv)) {
				test_isotx1();
				t1done = 1;;
			}
		}
	}	//mainloop
}

//test_fifo : simple high-level API test; doesn't not test robustness VS interrupts
void test_fifo(void) {
	u8 src[]={0x55,0xAA,0x69};
	u8 dest[5]={0};
//	uint rlen;

//	rlen = fifo_rlen(TXW, TXW_RP_ISO);
	if (fifo_wblock(TXW, (u8 *) 0, (uint) -1) != 0)	//attempt to over-fill buf; should fail.
		big_error();
//	if (rlen != fifo_rlen(TXW, TXW_RP_ISO))		//free len should not change
//		big_error();
	if (fifo_wblock(TXW, src, 3) != 3)		//test valid wblock
		big_error();
//	if ((fifo_rlen(TXW, TXW_RP_ISO) - rlen) != 3)	//test free len decrease
//		big_error();
	if (fifo_wblockf(TXW, src, 3) != 3)		//test wblockf
		big_error();
//	if ((fifo_rlen(TXW, TXW_RP_ISO) - rlen) != 6)	//test free len decrease #2
//		big_error();

	if (fifo_skip(TXW, TXW_RP_ISO, (uint) -1) != 0)	//attempt to over-skip #1
		big_error();
//	if (fifo_skip(TXW, TXW_RP_ISO, rlen - 1) != 0)	//attempt to over-skip #2
//		big_error();

	if (fifo_rblock(TXW, TXW_RP_ISO, dest, 2) != 2)	//test rblock
		big_error();
	if (dest[1] != src[1])
		big_error();
	if (fifo_rblockf(TXW, TXW_RP_ISO, &dest[2], 1) != 1)	//test rblockf
		big_error();
	if (dest[2] != src[2])
		big_error();
	if (fifo_cblock(TXW, TXW_RP_ISO, dest, 1) != 1)	//test cblock #1
		big_error();
	if (dest[0] != src[0])
		big_error();
	if (fifo_cblock(TXW, TXW_RP_ISO, dest, (uint) -1) != 0)	//test bad cblock
		big_error();

	return;
}

//test periodic msg code; call from main loop ?
void test_pmsg1(void) {
	u8 pmt[]={0x69,0x55};

	#define TEST_PM_PER	500

	if (pmsg_add(0, MP_ISO, TEST_PM_PER, ARRAY_SIZE(pmt), pmt) != 0) {
		DBGM("pmsgadd err", 0);
	}
	return;
}

void test_pmsg2(void) {
	pmsg_del(0);
	return;
}

//iso txworker test 1: manual slow init (no ioctl)
void test_isotx1(void) {
	#define IDSIZ1 2
	u8 tbraw[TXB_DATAPOS+IDSIZ1];
	struct txblock *tb = (struct txblock *)tbraw;
	tb->sH = 0;
	tb->sL = IDSIZ1;
	tb->tH = 0;
	tb->tL = 0;
	tb->data[0]=(u8) ISO_SLOWINIT;
	tb->data[1]=0x33;
	tb->hdr = (MP_ISO << TXB_PROTOSHIFT) | TXB_SPECIAL | TXB_SENDABLE;	//ensure ordering ?

	fifo_wblock(TXW, tbraw, ARRAY_SIZE(tbraw));
	return;
}

//iso txworker test 2: manual fast init (no ioctl)
void test_isotx2(void) {
	#define IDSIZ2 6
	u8 tbraw[TXB_DATAPOS+IDSIZ2];
	uint i=0;
	struct txblock *tb = (struct txblock *)tbraw;
	tb->sH = 0;
	tb->sL = IDSIZ2;
	tb->tH = 0;
	tb->tL = 0;
	tb->data[i++]=(u8) ISO_FASTINIT;
	tb->data[i++]= 0x81;	//startcomm request
	tb->data[i++]= 0x10;
	tb->data[i++]= 0xFC;
	tb->data[i++]= 0x81;
	tb->data[i++]= 0x0E;
	tb->hdr = (MP_ISO << TXB_PROTOSHIFT) | TXB_SPECIAL | TXB_SENDABLE;	//ensure ordering ?

	fifo_wblock(TXW, tbraw, ARRAY_SIZE(tbraw));
	return;
}

#else	//normal, non-TESTING

int main(void) {

	timers_init();
	fifo_init();
	isotx_init();
	pmsg_init();

	while (1) {
		isotx_qwork();	//also call qwork on txb reception etc
	}
	return 0;
}
#endif // TESTING
