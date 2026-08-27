/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   RM44SI magnetic angle sensor over SPI3.
  ******************************************************************************
  */

#include "encoder.h"
#include "spi.h"
#include "main.h"

/* Chip select: manual GPIO, SPI3 is configured for software NSS. */
#define ENC_CS_PORT             BOARD_ENC_XDIR_PORT
#define ENC_CS_PIN              BOARD_ENC_XDIR_PIN

/* The part answers whatever it is clocked with, so the transmitted word only
 * has to be something harmless. The board's own bring-up code sends zeros. */
#define RM44SI_READ_CMD         0x0000U

/* A 14-bit transfer reads all-ones when MISO is stuck high - a dead link, an
 * unpowered sensor, a missing pull. Two bits narrower than the 16-bit test
 * this replaces, which is the whole point: 0xFFFF can never arrive on a
 * 14-bit frame, so the old constant would have made the detector blind. */
#define RM44SI_FRAME_MASK       0x3FFFU
#define RM44SI_FRAME_DEAD       0x3FFFU

#define ENC_SPI_TIMEOUT_MS      2U

static uint16_t s_last_good = 0;
volatile uint32_t g_enc_sub_consec = 0;
volatile uint32_t g_enc_sub_total  = 0;

static uint16_t s_debug_rx = 0;

/* Sensor count -> the 15-bit convention every consumer is written around.
 * See the long note in encoder.h before changing this. */
static inline uint16_t Encoder_Widen(uint16_t native)
{
    return (uint16_t)((native & ENC_NATIVE_MASK) << ENC_UPSHIFT);
}

void Encoder_Init(void)
{
    HAL_GPIO_WritePin(ENC_CS_PORT, ENC_CS_PIN, GPIO_PIN_SET);
}

/* One 14-bit full-duplex frame with CS asserted around it. */
static HAL_StatusTypeDef Encoder_Frame(uint16_t tx, uint16_t *rx)
{
    HAL_StatusTypeDef st;

    HAL_GPIO_WritePin(ENC_CS_PORT, ENC_CS_PIN, GPIO_PIN_RESET);
    st = HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx, (uint8_t *)rx, 1,
                                 ENC_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(ENC_CS_PORT, ENC_CS_PIN, GPIO_PIN_SET);

    return st;
}

/* One frame is the whole read. The A1333's command-then-response pipeline,
 * and the >350 ns CS idle gap it needed between the two, are both gone. */
Encoder_Status_t Encoder_ReadAngle(uint16_t *raw_counts)
{
    uint16_t rx = 0;

    if (Encoder_Frame(RM44SI_READ_CMD, &rx) != HAL_OK)
    {
        *raw_counts = s_last_good;
        return ENC_ERR_SPI;
    }

    s_debug_rx  = (uint16_t)(rx & RM44SI_FRAME_MASK);
    s_last_good = Encoder_Widen(rx);
    *raw_counts = s_last_good;

    return ENC_OK;
}

void Encoder_GetDebugData(uint16_t *rx1, uint16_t *rx2)
{
    *rx1 = s_debug_rx;
    *rx2 = 0U;      /* only ever one frame on this part - see encoder.h */
}

uint32_t Encoder_RawToDegX100(uint16_t raw_counts)
{
    /* 32767 * 36000 fits in a uint32_t, so this needs no 64-bit math. The
     * argument is already in the 15-bit convention. */
    return ((uint32_t)(raw_counts & (ENC_REPORT_COUNTS - 1U)) * 36000U)
           / ENC_REPORT_COUNTS;
}

/* Direct-register transfer. HAL_SPI_TransmitReceive carries far too much
 * overhead for the control ISR's budget.
 *
 * SPI3's data size is 14 bits, so DR is still accessed 16 bits wide and the
 * peripheral takes care of the frame length. The access MUST stay 16-bit: a
 * byte write to DR would start an 8-bit transfer regardless of the configured
 * data size, which is a hardware behaviour and not a compiler one. */
static inline uint16_t Spi3Xfer16Fast(uint16_t tx)
{
    uint32_t guard = 2000U;

    while (((SPI3->SR & SPI_SR_TXE) == 0U) && (--guard != 0U)) { }
    *(volatile uint16_t *)&SPI3->DR = tx;

    guard = 2000U;
    while (((SPI3->SR & SPI_SR_RXNE) == 0U) && (--guard != 0U)) { }
    return *(volatile uint16_t *)&SPI3->DR;
}

uint16_t Encoder_ReadAngleFast(void)
{
    uint32_t guard;
    uint16_t rx;

    ENC_CS_PORT->BSRR = (uint32_t)ENC_CS_PIN << 16;
    rx = Spi3Xfer16Fast(RM44SI_READ_CMD);
    guard = 2000U;
    while (((SPI3->SR & SPI_SR_BSY) != 0U) && (--guard != 0U)) { }
    ENC_CS_PORT->BSRR = (uint32_t)ENC_CS_PIN;

    rx &= RM44SI_FRAME_MASK;

    if (rx != RM44SI_FRAME_DEAD)
    {
        s_debug_rx       = rx;
        s_last_good      = Encoder_Widen(rx);
        g_enc_sub_consec = 0U;
    }
    else
    {
        /* Substituting the last good angle, and SAYING SO. See the comment on
         * g_enc_sub_consec in encoder.h - this path used to be silent, which
         * made a dead encoder indistinguishable from a stationary one. */
        g_enc_sub_consec++;
        g_enc_sub_total++;
    }

    return s_last_good;
}
