/* Va avec utils.h; shits generales */

#include <stm32f0xx.h>
#include "stypes.h"
#include "timers.h"

#include <string.h>	//yuck - just for strncpy

//big_error : hangs the firmware, reset all periphs except USB, set error status
void big_error(void) {
	static u32 killtime;
	killtime = frclock;
	//TODO : TODO.
	while (1) {}
	return;
}

//dbg_log() : TODO, copy message to static error buf, readable by USB or debugger
void dbg_log(const char * dm, int eno) {
	//TODO: TODO
	return;
}

//_assertlog: generate message, log with dbg_log, call big_error
void _assertlog(const char *f1) {
	#define ASSERTLEN 80
	char assertm[ASSERTLEN+1];
	size_t f1len;

	f1len = strlen(f1);
	f1len = (f1len > ASSERTLEN)? ASSERTLEN : f1len;
	strncpy(assertm, f1, f1len);
	assertm[f1len]=0;
	dbg_log(assertm, 0);
	big_error();
	return;
}
