/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef AUDIOSOCKET_MODEM_INPUT_CLOCK_H
#define AUDIOSOCKET_MODEM_INPUT_CLOCK_H
#include <stdint.h>

struct modem_input_clock { long next_ms; int valid; long missing_samples; };

/* AudioSocket omits RTP timestamps. Keep small missing-packet intervals in
 * the 8 kHz sample stream; the modem's error control handles damaged data. */
static int modem_send_silence(int svc, int samples)
{
 int16_t zeros[160] = { 0 };
 struct ast_frame silence = { 0 };
 silence.frametype = AST_FRAME_VOICE;
 silence.subclass.format = ast_format_slin;
 silence.data.ptr = zeros;
 while (samples > 0) {
  int count = samples > 160 ? 160 : samples;
  silence.samples = count;
  silence.datalen = count * (int)sizeof(zeros[0]);
  if (ast_audiosocket_send_frame(svc, &silence)) return -1;
  samples -= count;
 }
 return 0;
}

static int modem_send_input(int svc, const struct ast_frame *f,
 struct modem_input_clock *clock)
{
 int timed = ast_test_flag(f, AST_FRFLAG_HAS_TIMING_INFO);
 if (!f->samples && !f->datalen) return 0;
 if (f->samples <= 0 || f->samples > 8000 || f->datalen < 0) return -1;
 if (timed) {
  long gap = clock->valid ? f->ts - clock->next_ms : 0;
  /* Large jumps can be a new RTP origin; never synthesize unbounded audio. */
  if (gap > 0 && gap <= 200) {
   if (modem_send_silence(svc, (int)gap * 8)) return -1;
   clock->missing_samples += gap * 8;
  }
  clock->next_ms = f->ts + f->samples / 8;
  clock->valid = 1;
 } else if (clock->valid) {
  clock->next_ms += f->samples / 8;
 }
 if (!f->datalen) {
  if (modem_send_silence(svc, f->samples)) return -1;
  clock->missing_samples += f->samples;
  return 0;
 }
 if (!f->data.ptr) return -1;
 return ast_audiosocket_send_frame(svc, f);
}
#endif
