#ifndef STCONF_H
#define STCONF_H

/* SPL code needs assert_param(x) to be defined */
#include "utils.h"
#define assert_param(x) if (!(x)) assert(x);

#include <stm32f0xx_rcc.h>
#include <stm32f0xx_usart.h>

#endif
