#include <sys/time.h>
#include "xparameters.h"
#include "xtime_l.h"

#if 0 // TODO
extern "C" int _gettimeofday(struct timeval *tp, struct timezone *tzp) {
	(void) tzp;
    XTime tStart;
    XTime_GetTime(&tStart);
    tp->tv_usec = (tStart / (COUNTS_PER_SECOND / 1'000'000)) % 1'000'000;
    tp->tv_sec  = tStart / COUNTS_PER_SECOND;
    return 0;
}
#endif
