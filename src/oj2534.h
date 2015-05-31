#ifndef OJ2534_H
#define OJ2534_H

/** Public / shared stuff for oj2534 firmware **/

//adjust assert_param() behavior (popular in SPL code); see utils.h
#define ASSERTPARAM_LEVEL 2

//debugging:
#define TESTING	//HSI clock, no USB

/* defines to facilitate RAMbuilds */
//#define DISABLE_ISO	//prevents calls to isotx_work

#endif // OJ2534_H
