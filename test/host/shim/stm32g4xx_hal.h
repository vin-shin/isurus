/**
  ******************************************************************************
  * @file    test/host/shim/stm32g4xx_hal.h
  * @brief   Lets the REAL Core/Inc/main.h compile on the host.
  *
  *          shim/main.h replaces Core/Inc/main.h for sources in Core/Src,
  *          because a quote-include there misses Core/Inc and falls through to
  *          the include path. That does not work for headers that LIVE in
  *          Core/Inc - csense.h's `#include "main.h"` searches its own
  *          directory first and finds the real one, which then asks for
  *          stm32g4xx_hal.h and stops the build.
  *
  *          So supply that name. It resolves to the shim from here, and the
  *          include guard makes the second arrival a no-op, which means both
  *          routes end up with exactly one set of definitions.
  ******************************************************************************
  */
#ifndef HOST_STM32G4XX_HAL_H
#define HOST_STM32G4XX_HAL_H

#include "main.h"

#endif /* HOST_STM32G4XX_HAL_H */
