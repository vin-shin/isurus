/**
  ******************************************************************************
  * @file    openloop.h
  * @brief   Open-loop (V/f) three-phase drive - a rotating voltage vector.
  *
  *          No current control and no rotor feedback: this commands a voltage
  *          vector that rotates at a fixed electrical frequency and hopes the
  *          rotor follows. Phase current is therefore set by winding impedance
  *          and back-EMF alone, which at standstill is just V_bus/R_phase.
  *          Start with a very small modulation index and raise it slowly.
  ******************************************************************************
  */
#ifndef OPENLOOP_H
#define OPENLOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* The vector is recomputed at this rate, derived from the 1 kHz SysTick so the
 * commanded frequency is exact rather than dependent on loop timing. */
#define OL_UPDATE_HZ        1000U

/* Refuse anything above this; a modulation index near 1.0 into a stalled motor
 * is how you find out what your bus supply is really capable of. */
#define OL_MOD_MAX_PERMILLE 500U

typedef struct {
  uint32_t phase;        /* electrical angle, full uint32 range = 1 rev */
  uint32_t inc;          /* phase increment per update                  */
  uint32_t freq_x100;    /* commanded electrical frequency, Hz * 100    */
  uint32_t mod_permille; /* modulation index, 0..OL_MOD_MAX_PERMILLE    */
  uint32_t duty_u;       /* last applied duty in counts                 */
  uint32_t duty_v;
  uint32_t duty_w;
  uint32_t updates;      /* update counter                              */
} OpenLoopState_t;

void OpenLoop_Init(OpenLoopState_t *s, uint32_t pwm_period);

/* Set electrical frequency (Hz*100) and modulation index (per-mille). */
void OpenLoop_SetCommand(OpenLoopState_t *s, uint32_t freq_x100,
                         uint32_t mod_permille);

/* Advance the angle by one update period and apply the new duties.
 * Call at exactly OL_UPDATE_HZ. */
void OpenLoop_Update(OpenLoopState_t *s);

/* Zero the vector: all three phases to 0% and the angle reset. */
void OpenLoop_Stop(OpenLoopState_t *s);

#ifdef __cplusplus
}
#endif

#endif /* OPENLOOP_H */
