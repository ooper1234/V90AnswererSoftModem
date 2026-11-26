/* Character framing on the transmit bit queue: one stop bit by default, two
   when asked for (V.23), so a receiver that joined back-to-back text on the
   wrong bit gets idle mark to resync on. */
#include "serial/sm_async.h"
#include <stdio.h>

static int popped(sm_bitq_t *q, int *bits, int max)
{
    int n = 0, b;
    while (n < max && (b = sm_bitq_pop(q)) >= 0)
        bits[n++] = b;
    return n;
}

int main(void)
{
    sm_bitq_t q;
    int bits[32], n, i, fail = 0;

    sm_bitq_init(&q);
    sm_bitq_push_byte(&q, 'C');
    n = popped(&q, bits, 32);
    if (n != 10 || bits[0] != 0 || bits[9] != 1) { puts("FAIL default: one stop bit"); fail = 1; }

    sm_bitq_init(&q);
    q.stop_bits = 2;
    sm_bitq_push_byte(&q, 'C');
    n = popped(&q, bits, 32);
    if (n != 11 || bits[0] != 0 || bits[9] != 1 || bits[10] != 1) { puts("FAIL two stop bits"); fail = 1; }
    for (i = 0; i < 8; i++)
        if (bits[1 + i] != (('C' >> i) & 1)) { puts("FAIL data bits"); fail = 1; break; }

    if (!fail) puts("sm_async stop bits: ok");
    return fail;
}
