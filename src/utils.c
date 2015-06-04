/* Va avec utils.h; shits generales */

#include <stm32f0xx.h>
#include "stypes.h"
#include "timers.h"

#include <string.h>	//yuck - just for strlen

//big_error : hangs the firmware, reset all periphs except USB, set error status
volatile u32 killtime;
void big_error(void) {

	killtime = frclock;
	//TODO : TODO.
	while (1) {}
	return;
}

char last_err[20];
int last_eno;
//dbg_log() : copy message to static error buf, readable by USB (TODO) or debugger
void dbg_log(const char * dm, int eno) {
	memcpy(last_err, dm, strlen(dm));
	last_eno = eno;
	return;
}

//_assertlog: generate message, log with dbg_log, call big_error
void _assertlog(const char *f1) {
	#define ASSERTLEN 80
	char assertm[ASSERTLEN+1];
	size_t f1len;

	f1len = strlen(f1);
	f1len = (f1len > ASSERTLEN)? ASSERTLEN : f1len;
	memcpy(assertm, f1, f1len);
	assertm[f1len]=0;
	dbg_log(assertm, 0);
	big_error();
	return;
}
