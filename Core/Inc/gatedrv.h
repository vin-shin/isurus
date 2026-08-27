/**
  ******************************************************************************
  * @file    gatedrv.h
  * @brief   The six gate drivers' fault and ready lines.
  *
  *          Twelve pins, one READY and one FAULT per switch, from six
  *          UCC21756-Q1 isolated drivers. They were the "unexplained inputs"
  *          in board.h section 9 until the schematic arrived, and nothing read
  *          them until this module.
  *
  *            switch   RDY    FLT
  *            U high   PB4    PD2
  *            U low    PA9    PA12
  *            V high   PC9    PA8
  *            V low    PB14   PB15
  *            W high   PB0    PB11
  *            W low    PB10   PB2
  *
  *          Both are OPEN DRAIN, so both need pulls; GateDrv_Init configures
  *          all twelve as inputs with pull-ups. Until it ran, they floated.
  *
  *            FLT   active LOW.  DESAT / overcurrent alarm, and it LATCHES in
  *                               the driver.
  *            RDY   active HIGH. Power-good on that driver's isolated supply.
  *
  *          So a healthy switch reads both HIGH, and either going low is bad.
  *
  * ---------------------------------------------------------------------------
  * Why this is POLLED and not twelve interrupts
  * ---------------------------------------------------------------------------
  *          The first instinct is EXTI on all twelve, and it is wrong twice.
  *
  *          It does not fit. EXTI lines are numbered by PIN, not by port, so
  *          two pins with the same number cannot both have one. PA9 and PC9
  *          collide, and so do PD2 and PB2 - and those two are not even both
  *          READY lines that could be dropped, since PD2 and PB2 are the U-high
  *          and W-low FAULTS. There is no subset of the useful signals that
  *          fits on distinct EXTI lines.
  *
  *          More importantly it is not needed, and this is the part worth
  *          understanding before anyone 'improves' it. The UCC21756 protects
  *          itself: DESAT detection turns that switch off in 200 ns, entirely
  *          without the MCU, and FLT is the driver TELLING us afterwards. The
  *          fast protection has already happened by the time the pin moves.
  *          What the MCU owes is to stop commanding, latch the reason and say
  *          which switch - none of which is measured in nanoseconds.
  *
  *          So it polls from the control ISR, at PWM_FREQ_HZ. Four port reads
  *          and some masking, once per period, with no interrupt priority
  *          interactions to reason about against a hard 50 us deadline.
  *
  * ---------------------------------------------------------------------------
  * There is no per-switch clear
  * ---------------------------------------------------------------------------
  *          FLT latches inside the driver and is cleared by pulsing RST/EN -
  *          which is PC8, and which all six drivers share. Clearing one
  *          driver's fault therefore resets the whole bridge. Any recovery
  *          path has to be a whole-bridge affair; there is no way to bring one
  *          switch back.
  ******************************************************************************
  */
#ifndef GATEDRV_H
#define GATEDRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/* Bit positions, used for every mask in this module. Ordered so the bit
 * number reads as (phase * 2 + low), which keeps the decode table honest. */
#define GD_UH               0U
#define GD_UL               1U
#define GD_VH               2U
#define GD_VL               3U
#define GD_WH               4U
#define GD_WL               5U
#define GD_SWITCHES         6U
#define GD_ALL_MASK         0x3FU

/* Consecutive identical polls before a line is believed.
 *
 * 2 at PWM_FREQ_HZ is 100 us. FLT latches in the driver, so nothing is lost
 * by waiting - the assertion does not go away on its own - and a single
 * sample on a 588 V bridge is not something to fault a drive on. Two is
 * enough to reject an isolated glitch and is still forty times faster than
 * anything thermal.
 *
 * It is NOT a substitute for the driver's own protection, which has already
 * turned the switch off 200 ns after the event. */
#define GD_DEBOUNCE_POLLS   2U

typedef struct {
  uint32_t flt_mask;      /*  0  bit per switch, 1 = FLT asserted now      */
  uint32_t notready_mask; /*  4  bit per switch, 1 = RDY deasserted now    */
  uint32_t flt_latched;   /*  8  sticky FLT since the last GateDrv_Clear   */
  uint32_t nrdy_latched;  /* 12  sticky not-ready, same                    */
  uint32_t flt_events;    /* 16  transitions into faulted, since boot      */
  uint32_t polls;         /* 20                                            */
} GateDrvTelem_t;

/* Configures all twelve as inputs with pull-ups. Must run before the control
 * ISR starts polling, and before anything trusts a reading. */
void GateDrv_Init(void);

/* Sample all twelve and update the masks. Called from the control ISR.
 * Cheap by construction: four IDR reads and a fixed table walk. */
void GateDrv_Poll(void);

/* Debounced state. Both are bit-per-switch, GD_* bit positions. */
uint32_t GateDrv_FaultMask(void);
uint32_t GateDrv_NotReadyMask(void);

/* Drop the sticky masks. Only meaningful after the drivers have actually been
 * reset, which is a whole-bridge operation - see the header. */
void GateDrv_ClearLatched(void);

void GateDrv_GetTelem(GateDrvTelem_t *t);

/* "UH", "VL", ... for the lowest set bit in a mask, or "--" if none. For
 * printing and for telemetry decode; returns a static string. */
const char *GateDrv_SwitchName(uint32_t mask);

#ifdef __cplusplus
}
#endif

#endif /* GATEDRV_H */
