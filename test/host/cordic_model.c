/**
  ******************************************************************************
  * @file    test/host/cordic_model.c
  * @brief   Behavioural model of the STM32G4 CORDIC, cos/sin mode.
  *
  *          MODELLED, not faked. It reproduces the hardware's own conventions,
  *          because the entire reason for running foc.c on the host is to
  *          catch angle-convention mistakes - and a model that quietly used a
  *          different convention would happily pass a broken implementation.
  *
  *          The conventions that matter, from RM0440:
  *
  *            - The angle argument is Q31 covering [-pi, pi). The value v
  *              means (v / 2^31) * pi radians. So 0x80000000 is -pi, not +pi,
  *              and the wrap is at a half turn.
  *            - With NRES = 2, one operation yields TWO results read from the
  *              same RDATA address, COSINE FIRST, then sine.
  *            - Results are Q31 in [-1, 1), so +1.0 saturates to 0x7FFFFFFF.
  *
  *          That first point is the one under test. foc.c converts an encoder
  *          count to the argument with a bare `counts << 17`, which is exact
  *          precisely because the 15-bit count walks into the Q31 sign bit at
  *          the same place the angle passes pi. The predecessor project
  *          centres the count first and is off by exactly pi - both sin and
  *          cos negated - which no closed-loop behaviour reveals because the
  *          inverse Park uses the same pair. This model is what makes that
  *          visible without a motor.
  ******************************************************************************
  */

#include "main.h"
#include <math.h>

unsigned long g_cordic_ops = 0;

RccHostRegs_t g_rcc_host = { 0 };

static CordicHostRegs_t s_regs;

/* Where we are in FOC_SinCos's fixed four-access sequence:
 *   0  the WDATA store        (WDATA takes its new value AFTER this call)
 *   1  the CSR read           (compute here; WDATA is now current)
 *   2  the first RDATA read   (cosine)
 *   3  the second RDATA read  (sine)
 */
static unsigned s_phase = 0;
static uint32_t s_sin_q31 = 0;

void Cordic_Host_Reset(void)
{
  s_phase = 0;
  s_regs.CSR   = 0;
  s_regs.WDATA = 0;
  s_regs.RDATA = 0;
}

static uint32_t to_q31(double v)
{
  /* Q31 saturates at just under +1.0; -1.0 is exactly representable. */
  double s = v * 2147483648.0;
  if (s >  2147483647.0) { s =  2147483647.0; }
  if (s < -2147483648.0) { s = -2147483648.0; }
  return (uint32_t)(int32_t)llround(s);
}

CordicHostRegs_t *Cordic_Host_Access(void)
{
  /* RRDY is held set at all times. On silicon it clears once both results
   * have been read and sets again when an operation completes; the poll in
   * foc.c only ever waits for it to be set, so a model that keeps it set is
   * indistinguishable from one that toggles it - and it keeps FOC_Init's
   * longer access sequence from deadlocking here. */
  s_regs.CSR |= (uint32_t)CORDIC_CSR_RRDY;

  switch (s_phase)
  {
    case 1:
    {
      /* The angle argument is Q31 over [-pi, pi). Reading WDATA as SIGNED is
       * the whole convention: 0x80000000 must come out as -pi. */
      int32_t q   = (int32_t)s_regs.WDATA;
      double  ang = ((double)q / 2147483648.0) * M_PI;

      s_regs.RDATA = to_q31(cos(ang));   /* cosine first, per NRES = 2 */
      s_sin_q31    = to_q31(sin(ang));
      g_cordic_ops++;
      break;
    }
    case 3:
      s_regs.RDATA = s_sin_q31;
      break;
    default:
      break;
  }

  s_phase = (s_phase + 1u) & 3u;
  return &s_regs;
}
