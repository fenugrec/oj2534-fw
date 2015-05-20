/* Va avec utils.h; shits generales */

#include <stm32f0xx.h>
#include "stypes.h"
#include <string.h>	//yuck - just for strncpy

//big_error : hangs the firmware, reset all periphs except USB, set error status
void big_error(void) {
	//TODO : TODO.
	return;
}

//dbg_log() : TODO, copy message to static error buf, readable by USB or debugger
void dbg_log(const char * dm, int eno) {
	//TODO: TODO
	return;
}

//_assertlog: generate message, log with dbg_log, call big_error
void _assertlog(const char *f1, const char *f2) {
	#define ASSERTLEN 80
	char assertm[ASSERTLEN+1];
	size_t f1len;

	f1len = strlen(f1);
	if (f1len >= ASSERTLEN*0.75) f1len = 0.75*ASSERTLEN;	//use max 75% of buffer for f1
	strncpy(assertm, f1, f1len);
	strncpy(assertm + f1len, f2, ASSERTLEN - f1len);
	assertm[ASSERTLEN]=0;
	dbg_log(assertm, 0);
	big_error();
	return;
}
