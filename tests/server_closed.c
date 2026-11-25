/* sm_call_server_closed_step: hang up once the last bytes are sent after
   the BBS or pppd closed the pty, or 20 s after at the latest. */
#include <assert.h>
#include <stdio.h>
#include "sm_call.h"

#define SR 8000
static int run(int drain_after_ms, int chunk, int *hung_at_ms)
{
    long long close = 0, drained = 0;
    int t;
    for (t = 0; t < 60000; t += chunk * 1000 / SR) {
        if (sm_call_server_closed_step(&close, &drained, chunk, t >= drain_after_ms)) {
            *hung_at_ms = t;
            return 1;
        }
    }
    return 0;
}

int main(void)
{
    int at;
    /* Everything sent at once: 2 s of grace, then hang up. */
    assert(run(0, 160, &at) && at >= 1960 && at <= 2020);
    /* The goodbye takes 1.5 s to go: hang up 2 s after that. */
    assert(run(1500, 160, &at) && at >= 3460 && at <= 3520);
    /* Never drains: hang up at 20 s anyway. */
    assert(run(1000000, 160, &at) && at >= 19960 && at <= 20020);
    /* More bytes queued again restarts the grace. */
    {
        long long close = 0, drained = 0;
        int i;
        for (i = 0; i < 50; i++) assert(!sm_call_server_closed_step(&close, &drained, 160, 1)); /* 1 s */
        assert(!sm_call_server_closed_step(&close, &drained, 160, 0));
        assert(drained == 0);
        for (i = 0; i < 99; i++) assert(!sm_call_server_closed_step(&close, &drained, 160, 1));
        assert(sm_call_server_closed_step(&close, &drained, 160, 1));
    }
    puts("server_closed: ok");
    return 0;
}
