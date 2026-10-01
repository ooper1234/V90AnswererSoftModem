/* SLiRP 1.0.17 casts its -1 timeout sentinel through unsigned int.
 * On 64-bit systems select() receives 4294967295 and fails with EINVAL.
 * Normalize only that sentinel to the intended five-second poll.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <sys/select.h>
int select(int n, fd_set *r, fd_set *w, fd_set *e, struct timeval *t)
{
    static int (*real_select)(int, fd_set *, fd_set *, fd_set *, struct timeval *);
    if (!real_select)
        *(void **) (&real_select) = dlsym(RTLD_NEXT, "select");
    if (t && t->tv_usec == UINT32_MAX) {
        t->tv_sec = 5;
        t->tv_usec = 0;
    }
    return real_select(n, r, w, e, t);
}
