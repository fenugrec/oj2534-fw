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

void SystemInit(void) {}

int main(void) {

	fifo_init();
	isotx_init();

	while (1) {
		isotx_work();
	}
	return 0;
}
