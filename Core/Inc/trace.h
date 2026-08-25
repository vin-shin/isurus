/**
  ******************************************************************************
  * @file    trace.h
  * @brief   ISR-rate capture buffer for control-loop transients.
  *
  *          The integer mirrors in foc.c are decimated to 1 kHz because no
  *          debugger polls faster than that. A current loop closed at 1 kHz
  *          settles a step in about a millisecond, so those mirrors give one
  *          or two points on the very transient that has to be judged - and
  *          the host-polled sparkline in dial.py is coarser still, by design.
  *
  *          Anything that has to be SEEN rather than averaged therefore needs
  *          capturing at the ISR rate and reading out afterwards. That is all
  *          this is: arm it, let it fill, read it over SWD.
  *
  *          Cost when idle is a load, a compare and a not-taken branch. The
  *          float-to-int conversions sit AFTER the arm check on purpose, so a
  *          disarmed trace costs the control loop essentially nothing - which
  *          is what lets it stay compiled in rather than being a debug build.
  ******************************************************************************
  */
#ifndef TRACE_H
#define TRACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 512 samples at PWM_FREQ_HZ is 17 ms at 30 kHz - long enough to hold the arm
 * -> step latency of an SWD write (about 1 ms) plus the whole transient, and
 * 4 KB of a 128 KB part. Rounding this up is cheap; the reason not to is that
 * a longer window is a longer wait for every readout. */
#define TRACE_LEN   512U
#define TRACE_CH    4U

typedef struct {
  /* ---- control: write these over SWD ------------------------ offset ---- */
  /* Write 1 to start a capture. That is the WHOLE protocol: the ISR sees the
   * 1, resets count/div/done itself and moves this to 2 to mark the capture
   * running, then clears it to 0 when the buffer is full.
   *
   * It self-initialises for a reason. The obvious way to use this from a
   * script is a single `mww arm 1`, and an earlier version trusted the host
   * to zero count first - so a second arm resumed at index 512 and wrote
   * eight bytes off the end of the buffer into whatever BSS followed, while
   * reporting a complete capture that was actually the previous one. It
   * looked like two identical measurements rather than like memory
   * corruption, which is the worst way for that to present. */
  uint32_t arm;      /*  0  0 = idle, 1 = start (host), 2 = running (ISR)   */
  uint32_t done;     /*  4  1 = buffer holds a complete capture            */
  uint32_t count;    /*  8  samples captured so far                        */
  uint32_t decim;    /* 12  keep 1 sample in N. 0 or 1 = every ISR tick     */

  uint32_t div;      /* 16  internal decimation counter                     */

  /* 20: id_ma, iq_ma, vd_pm, vq_pm per sample, interleaved. int16 because
   * the buffer is read back over SWD one word at a time and halving it halves
   * the readout time; every channel here fits with room to spare (currents
   * are bounded by LIM_IQ_MAX_MA, the voltages are per-mille of the bus). */
  int16_t  buf[TRACE_LEN][TRACE_CH];
} TraceState_t;

/* Defined in main.c alongside the other control-loop globals. */
extern volatile TraceState_t g_trace;

static inline void Trace_Arm(volatile TraceState_t *t, uint32_t decim)
{
  t->decim = decim;
  t->arm   = 1U;     /* the ISR does the rest - see the comment on `arm` */
}

/* Capture one sample. Call from the control ISR.
 *
 * The arguments are floats and are converted INSIDE, after the arm test, so
 * that a disarmed trace does not pay for four float-to-int conversions on
 * every switching period. always_inline is what makes that actually happen -
 * as an out-of-line call the arguments would be evaluated at the call site
 * whether or not the body wanted them. */
__attribute__((always_inline))
static inline void Trace_Capture(volatile TraceState_t *t,
                                 float a, float b, float c, float d)
{
  uint32_t state = t->arm;
  if (state == 0U) { return; }

  if (state == 1U)
  {
    /* Freshly armed. Own the reset here rather than trusting whoever wrote
     * the 1 to have zeroed count first. */
    t->count = 0U;
    t->div   = 0U;
    t->done  = 0U;
    t->arm   = 2U;
  }

  if (t->decim > 1U)
  {
    if (++t->div < t->decim) { return; }
    t->div = 0U;
  }

  uint32_t i = t->count;

  /* Belt and braces against an index that should be impossible. count is only
   * ever advanced below and only ever from zero, but this buffer sits in the
   * control ISR and an out-of-range store here corrupts whatever BSS follows
   * - a failure that would show up somewhere else entirely, long afterwards. */
  if (i >= TRACE_LEN)
  {
    t->arm  = 0U;
    t->done = 1U;
    return;
  }
  t->buf[i][0] = (int16_t)(a * 1000.0f);
  t->buf[i][1] = (int16_t)(b * 1000.0f);
  t->buf[i][2] = (int16_t)(c * 1000.0f);
  t->buf[i][3] = (int16_t)(d * 1000.0f);

  i++;
  t->count = i;
  if (i >= TRACE_LEN)
  {
    /* Stop rather than wrap. A one-shot window that ends where the data ends
     * needs no head pointer to interpret, and every use of this so far is a
     * response to a stimulus the host just applied. */
    t->arm  = 0U;
    t->done = 1U;
  }
}

#ifdef __cplusplus
}
#endif

#endif /* TRACE_H */
