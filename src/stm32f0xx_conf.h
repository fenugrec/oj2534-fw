#ifndef STCONF_H
#define STCONF_H

#include "oj2534.h"
#include "utils.h"

/* SPL code needs assert_param(x) to be defined; it's only to sanity-check arguments
 passed to SPL functions.
 */

#if (ASSERTPARAM_LEVEL == 0)
	//do nothing
	#define assert_param(x) (void) 0
#elif (ASSERTPARAM_LEVEL == 1)
	//fallback to regular assert
	#define assert_param(x) if (!(x)) assert(x)
#else
	//XXX refine this:
	#define assert_param(x) big_error()
#endif

#include <stm32f0xx_rcc.h>
#include <stm32f0xx_usart.h>

#endif
