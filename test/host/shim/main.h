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

/* drive.c's windowed watchdog touches these. They are modelled as plain
 * memory rather than emulated: the host tests assert on the STATE MACHINE -
 * which cause latches, whether the bridge was safed, what a clear does - not
 * on WWDG timing, which is a property of the silicon and cannot be checked
 * here anyway. Drive_WatchdogTripped is driven by seeding RCC->CSR. */
#define RCC_APB1ENR1_WWDGEN    (1UL << 11)
#define RCC_CSR_WWDGRSTF       (1UL << 30)
#define RCC_CSR_RMVF           (1UL << 23)

typedef struct {
  volatile uint32_t AHB1ENR;
  volatile uint32_t APB1ENR1;
  volatile uint32_t CSR;
} RccHostRegs_t;
extern RccHostRegs_t g_rcc_host;

#define RCC   (&g_rcc_host)

/* ---- WWDG and the debug freeze ------------------------------------------ */

#define WWDG_CR_WDGA           (1UL << 7)
#define WWDG_CFR_WDGTB_Pos     11U

typedef struct { volatile uint32_t CR; volatile uint32_t CFR; } WwdgHostRegs_t;
extern WwdgHostRegs_t g_wwdg_host;
#define WWDG  (&g_wwdg_host)

#define DBGMCU_APB1FZR1_DBG_WWDG_STOP  (1UL << 11)

typedef struct { volatile uint32_t APB1FZR1; } DbgmcuHostRegs_t;
extern DbgmcuHostRegs_t g_dbgmcu_host;
#define DBGMCU  (&g_dbgmcu_host)

/* csense.h declares `extern ADC_HandleTypeDef hadc5;` at file scope, so the
 * name has to parse even though the host never touches it - CSense_ReadVbus
 * and the rest are stubbed. Opaque is deliberate: anything that tries to use
 * a field of it here should fail to build rather than silently pretend. */
typedef struct { int host_opaque; } ADC_HandleTypeDef;

/* Host clock. Tests step it explicitly so the SELFTEST retry timer and
 * ms_in_state are deterministic rather than wall-clock dependent. */
uint32_t HAL_GetTick(void);
void     Host_SetTick(uint32_t ms);

#endif /* HOST_MAIN_H */
