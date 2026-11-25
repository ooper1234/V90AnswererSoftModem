/* Bell 103 vs V.22 caller detection, with the echo of our own V.22bis
   answer signal on the line as it is on a real call. */
#include "call/sm_legacy_detect.h"
#include <stdio.h>
#include <stdlib.h>

static int run(const char *name, int bell, int v22psk, double echo, int want)
{
    sm_legacy_detect_t d;
    double ph = 0, eph = 0, pph = 0;
    int got = 0, sym = 0, phase = 0;
    sm_legacy_detect_init(&d);
    srand(1);
    for (int n = 0; n < 8000 * 2; n++) {
        double x = 0;
        if (bell) { x += 0.1 * sin(ph); ph += 2 * M_PI * 1270 / 8000; }
        if (v22psk) {                       /* 600 baud DPSK on 1200 Hz, random data */
            if (n % (8000 / 600) == 0) { phase = (phase + (rand() & 3)) & 3; sym++; }
            x += 0.1 * sin(pph + phase * M_PI / 2); pph += 2 * M_PI * 1200 / 8000;
        }
        /* our USB1 echo: unscrambled ones on the 2400 Hz carrier, lines at 1800/3000 */
        x += echo * (sin(eph * 0.75) + sin(eph * 1.25)); eph += 2 * M_PI * 2400 / 8000;
        int r = sm_legacy_detect_step(&d, x);
        if (r && !got) got = r;
    }
    printf("%-40s got %d want %d %s\n", name, got, want, got == want ? "ok" : "FAIL");
    (void) sym;
    return got != want;
}

int main(void)
{
    int f = 0;
    f |= run("Bell 103 mark, no echo", 1, 0, 0.0, 1);
    f |= run("Bell 103 mark under strong V.22bis echo", 1, 0, 0.15, 1);
    f |= run("V.22 caller data, no echo", 0, 1, 0.0, 0);
    f |= run("V.22 caller data under echo", 0, 1, 0.15, 0);
    f |= run("echo alone", 0, 0, 0.15, 0);
    return f;
}
