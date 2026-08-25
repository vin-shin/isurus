/**
  ******************************************************************************
  * @file    test/host/shim/main.h
  * @brief   Host stand-in for Core/Inc/main.h, so foc.c compiles and runs
  *          natively without being modified.
  *
  *          foc.c includes main.h to reach two peripherals and touches a very
  *          small part of each:
  *
  *              CORDIC->CSR / WDATA / RDATA   and five CORDIC_CSR_* macros
  *              RCC->AHB1ENR                  and RCC_AHB1ENR_CORDICEN
  *
  *          The rest of the real main.h is HAL the host build has no use for.
  *          This header goes ahead of Core/Inc on the include path, so
  *          `#include "main.h"` lands here instead.
  *
  *          foc.c is compiled EXACTLY as it ships. A harness that needs the
  *          file under test edited is testing a different program - which
  *          matters especially here, because the bug class this exists to
  *          catch is angle-convention errors, and those live in precisely the
  *          lines a "port" would be tempted to touch.
  *
  *          How the CORDIC is intercepted: `CORDIC` is a macro expanding to a
  *          FUNCTION CALL returning the register pointer, so the model runs
  *          once per register access and can update the struct before each
  *          read. Plain struct stores cannot be intercepted in C, which is why
  *          this indirection exists.
  ******************************************************************************
  */
#ifndef HOST_MAIN_H
#define HOST_MAIN_H

#include <stdint.h>

/* ---- CORDIC ------------------------------------------------------------- */

#define CORDIC_CSR_FUNC_Pos        0U
#define CORDIC_CSR_PRECISION_Pos   4U
#define CORDIC_CSR_NARGS           (1UL << 17)
#define CORDIC_CSR_NRES            (1UL << 18)
#define CORDIC_CSR_RRDY            (1UL << 31)

typedef struct {
  volatile uint32_t CSR;
  volatile uint32_t WDATA;
  volatile uint32_t RDATA;
} CordicHostRegs_t;

/* Called on every `CORDIC->...` access - see the note above. */
CordicHostRegs_t *Cordic_Host_Access(void);

/* Re-synchronise the access-phase counter. The model tracks where it is in
 * FOC_SinCos's fixed four-access sequence (store WDATA, read CSR, read RDATA,
 * read RDATA), which is what lets it return cos and sin from two reads of the
 * same address. FOC_Init uses a different, longer sequence, so the harness
 * calls this once after FOC_Init to line the counter back up.
 *
 * If foc.c's CORDIC access pattern ever changes, this model desynchronises and
 * the sign-convention test fails loudly rather than quietly returning the
 * wrong trig - which is the correct way for it to break. */
void Cordic_Host_Reset(void);

extern unsigned long g_cordic_ops;   /* completed operations since reset */

#define CORDIC   (Cordic_Host_Access())

/* ---- RCC ---------------------------------------------------------------- */

#define RCC_AHB1ENR_CORDICEN   (1UL << 3)

typedef struct { volatile uint32_t AHB1ENR; } RccHostRegs_t;
extern RccHostRegs_t g_rcc_host;

#define RCC   (&g_rcc_host)

#endif /* HOST_MAIN_H */
