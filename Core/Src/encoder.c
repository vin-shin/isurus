/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Allegro A1333 magnetic angle sensor over SPI1.
  ******************************************************************************
  */

#include "encoder.h"
#include "spi.h"
#include "main.h"

/* Chip select: manual GPIO, SPI1 is configured for software NSS. */
#define ENC_CS_PORT             GPIOA
#define ENC_CS_PIN              GPIO_PIN_4

/* A1333 commands */
#define A1333_ANG15_CMD         0x3200U   /* read 15-bit angle register */
#define A1333_NOP_CMD           0x0000U   /* clocks out the response */

#define ENC_SPI_TIMEOUT_MS      2U

static uint16_t s_last_good = 0;
static uint16_t s_debug_rx1 = 0;
static uint16_t s_debug_rx2 = 0;

void Encoder_Init(void)
{
    HAL_GPIO_WritePin(ENC_CS_PORT, ENC_CS_PIN, GPIO_PIN_SET);
}

/* One 16-bit full-duplex frame with CS asserted around it. */
static HAL_StatusTypeDef Encoder_Frame(uint16_t tx, uint16_t *rx)
{
    HAL_StatusTypeDef st;

    HAL_GPIO_WritePin(ENC_CS_PORT, ENC_CS_PIN, GPIO_PIN_RESET);
    st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)rx, 1,
                                 ENC_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(ENC_CS_PORT, ENC_CS_PIN, GPIO_PIN_SET);

    return st;
}

/* The A1333 needs CS to stay idle >350 ns between the command frame and the
 * response frame. At 128 MHz this loop is comfortably longer than that. */
static inline void Encoder_CsIdleDelay(void)
{
    volatile uint32_t d = 20;
    while (d--) {}
}

/* Read the 15-bit angle using the A1333's two-frame pipelined protocol:
 * frame 1 issues the command, frame 2 clocks out the answer. */
Encoder_Status_t Encoder_ReadAngle(uint16_t *raw_counts)
{
    uint16_t rx1 = 0;
    uint16_t rx2 = 0;

    if (Encoder_Frame(A1333_ANG15_CMD, &rx1) != HAL_OK)
    {
        *raw_counts = s_last_good;
        return ENC_ERR_SPI;
    }

    Encoder_CsIdleDelay();

    if (Encoder_Frame(A1333_NOP_CMD, &rx2) != HAL_OK)
    {
        *raw_counts = s_last_good;
        return ENC_ERR_SPI;
    }

    s_debug_rx1 = rx1;
    s_debug_rx2 = rx2;

    s_last_good = (uint16_t)(rx2 & 0x7FFFU);
    *raw_counts = s_last_good;

    return ENC_OK;
}

void Encoder_GetDebugData(uint16_t *rx1, uint16_t *rx2)
{
    *rx1 = s_debug_rx1;
    *rx2 = s_debug_rx2;
}

uint32_t Encoder_RawToDegX100(uint16_t raw_counts)
{
    /* 32767 * 36000 fits in a uint32_t, so this needs no 64-bit math. */
    return ((uint32_t)(raw_counts & 0x7FFFU) * 36000U) / 32768U;
}
