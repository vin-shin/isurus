/**
  ******************************************************************************
  * @file    gatedrv.c
  * @brief   The six gate drivers' fault and ready lines.
  ******************************************************************************
  */

#include "gatedrv.h"
#include "board.h"

/* Port index into the IDR snapshot below. */
#define GP_A    0U
#define GP_B    1U
#define GP_C    2U
#define GP_D    3U
#define GP_N    4U

typedef struct {
  uint8_t  port;
  uint16_t pin;
} GdPin_t;

/* Indexed by GD_* switch bit. From schematic sheet 2. */
static const GdPin_t s_rdy[GD_SWITCHES] = {
  { GP_B, GPIO_PIN_4  },   /* UH */
  { GP_A, GPIO_PIN_9  },   /* UL */
  { GP_C, GPIO_PIN_9  },   /* VH */
  { GP_B, GPIO_PIN_14 },   /* VL */
  { GP_B, GPIO_PIN_0  },   /* WH */
  { GP_B, GPIO_PIN_10 },   /* WL */
};

static const GdPin_t s_flt[GD_SWITCHES] = {
  { GP_D, GPIO_PIN_2  },   /* UH */
  { GP_A, GPIO_PIN_12 },   /* UL */
  { GP_A, GPIO_PIN_8  },   /* VH */
  { GP_B, GPIO_PIN_15 },   /* VL */
  { GP_B, GPIO_PIN_11 },   /* WH */
  { GP_B, GPIO_PIN_2  },   /* WL */
};

static const char *const s_names[GD_SWITCHES] = {
  "UH", "UL", "VH", "VL", "WH", "WL"
};

static volatile uint32_t s_flt_mask;
static volatile uint32_t s_nrdy_mask;
static volatile uint32_t s_flt_latched;
static volatile uint32_t s_nrdy_latched;
static volatile uint32_t s_flt_events;
static volatile uint32_t s_polls;

/* Debounce state: the last raw sample and how many times it has repeated. */
static uint32_t s_raw_flt_prev;
static uint32_t s_raw_nrdy_prev;
static uint32_t s_flt_run;
static uint32_t s_nrdy_run;

void GateDrv_Init(void)
{
  GPIO_InitTypeDef g = {0};
  uint32_t i;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* PULL-UPS are not optional. Both lines are open drain, so without a pull
   * an idle driver leaves its pin floating - and a floating FLT reads as
   * whatever the last charge on the pin says, which is indistinguishable from
   * "healthy" often enough to be useless and from "faulted" often enough to
   * be maddening. The pull is what makes the idle state mean something. */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;

  for (i = 0U; i < GD_SWITCHES; i++)
  {
    GPIO_TypeDef *port;

    g.Pin = s_rdy[i].pin;
    switch (s_rdy[i].port)
    {
      case GP_A: port = GPIOA; break;
      case GP_B: port = GPIOB; break;
      case GP_C: port = GPIOC; break;
      default:   port = GPIOD; break;
    }
    HAL_GPIO_Init(port, &g);

    g.Pin = s_flt[i].pin;
    switch (s_flt[i].port)
    {
      case GP_A: port = GPIOA; break;
      case GP_B: port = GPIOB; break;
      case GP_C: port = GPIOC; break;
      default:   port = GPIOD; break;
    }
    HAL_GPIO_Init(port, &g);
  }

  s_flt_mask      = 0U;
  s_nrdy_mask     = GD_ALL_MASK;   /* assume not ready until a poll says otherwise */
  s_flt_latched   = 0U;
  s_nrdy_latched  = GD_ALL_MASK;
  s_flt_events    = 0U;
  s_polls         = 0U;
  s_raw_flt_prev  = 0U;
  s_raw_nrdy_prev = GD_ALL_MASK;
  s_flt_run       = 0U;
  s_nrdy_run      = 0U;
}

void GateDrv_Poll(void)
{
  uint32_t idr[GP_N];
  uint32_t raw_flt  = 0U;
  uint32_t raw_nrdy = 0U;
  uint32_t i;

  /* One read per port, then everything else is register arithmetic. Four
   * loads rather than twelve HAL_GPIO_ReadPin calls, and all twelve lines
   * come from the same instant - which matters, because a DESAT event tends
   * to take more than one switch with it and the mask should show that as one
   * event rather than as a sequence. */
  idr[GP_A] = GPIOA->IDR;
  idr[GP_B] = GPIOB->IDR;
  idr[GP_C] = GPIOC->IDR;
  idr[GP_D] = GPIOD->IDR;

  for (i = 0U; i < GD_SWITCHES; i++)
  {
    /* FLT is active LOW: pin low means faulted. */
    if ((idr[s_flt[i].port] & s_flt[i].pin) == 0U)
    {
      raw_flt |= (1UL << i);
    }
    /* RDY is active HIGH: pin low means NOT ready. */
    if ((idr[s_rdy[i].port] & s_rdy[i].pin) == 0U)
    {
      raw_nrdy |= (1UL << i);
    }
  }

  s_polls++;

  /* Debounce each mask independently. A mask that repeats GD_DEBOUNCE_POLLS
   * times is believed; anything else leaves the previous belief standing. */
  if (raw_flt == s_raw_flt_prev)
  {
    if (s_flt_run < GD_DEBOUNCE_POLLS) { s_flt_run++; }
    if (s_flt_run >= GD_DEBOUNCE_POLLS)
    {
      uint32_t newly = raw_flt & ~s_flt_mask;
      if (newly != 0U) { s_flt_events++; }
      s_flt_mask     = raw_flt;
      s_flt_latched |= raw_flt;
    }
  }
  else
  {
    s_raw_flt_prev = raw_flt;
    s_flt_run      = 1U;
  }

  if (raw_nrdy == s_raw_nrdy_prev)
  {
    if (s_nrdy_run < GD_DEBOUNCE_POLLS) { s_nrdy_run++; }
    if (s_nrdy_run >= GD_DEBOUNCE_POLLS)
    {
      s_nrdy_mask     = raw_nrdy;
      s_nrdy_latched |= raw_nrdy;
    }
  }
  else
  {
    s_raw_nrdy_prev = raw_nrdy;
    s_nrdy_run      = 1U;
  }
}

uint32_t GateDrv_FaultMask(void)    { return s_flt_mask; }
uint32_t GateDrv_NotReadyMask(void) { return s_nrdy_mask; }

void GateDrv_ClearLatched(void)
{
  s_flt_latched  = s_flt_mask;
  s_nrdy_latched = s_nrdy_mask;
}

void GateDrv_GetTelem(GateDrvTelem_t *t)
{
  t->flt_mask      = s_flt_mask;
  t->notready_mask = s_nrdy_mask;
  t->flt_latched   = s_flt_latched;
  t->nrdy_latched  = s_nrdy_latched;
  t->flt_events    = s_flt_events;
  t->polls         = s_polls;
}

const char *GateDrv_SwitchName(uint32_t mask)
{
  uint32_t i;

  for (i = 0U; i < GD_SWITCHES; i++)
  {
    if ((mask & (1UL << i)) != 0U) { return s_names[i]; }
  }
  return "--";
}
