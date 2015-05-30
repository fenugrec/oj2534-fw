/*
*/

#include <stdint.h>
#include <stddef.h>	//for #define NULL !
//#include <stdatomic.h>

#include <stm32f0xx.h>
#include "stypes.h"
#include "utils.h"

#include "fifos.h"
#include "iso.h"
#include "timers.h"
#include "pmsg.h"

void SystemInit(void) {}

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
