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
HAL_StatusTypeDef Encoder_Frame(uint16_t tx, uint16_t *rx)
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
void Encoder_CsIdleDelay(void)
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

/* Direct-register 16-bit transfer. HAL_SPI_TransmitReceive carries far too
 * much overhead for a 50 us budget. */
static inline uint16_t Spi1Xfer16Fast(uint16_t tx)
{
    uint32_t guard = 2000U;

    while (((SPI1->SR & SPI_SR_TXE) == 0U) && (--guard != 0U)) { }
    *(volatile uint16_t *)&SPI1->DR = tx;

    guard = 2000U;
    while (((SPI1->SR & SPI_SR_RXNE) == 0U) && (--guard != 0U)) { }
    return *(volatile uint16_t *)&SPI1->DR;
}

uint16_t Encoder_ReadAngleFast(void)
{
    uint32_t guard;

    /* Frame 1: issue the ANG15 command. */
    ENC_CS_PORT->BSRR = (uint32_t)ENC_CS_PIN << 16;
    (void)Spi1Xfer16Fast(A1333_ANG15_CMD);
    guard = 2000U;
    while (((SPI1->SR & SPI_SR_BSY) != 0U) && (--guard != 0U)) { }
    ENC_CS_PORT->BSRR = (uint32_t)ENC_CS_PIN;

    /* >350 ns of CS idle, per the A1333. */
    for (volatile uint32_t d = 0; d < 12U; d++) { }

    /* Frame 2: clock the answer back. */
    ENC_CS_PORT->BSRR = (uint32_t)ENC_CS_PIN << 16;
    uint16_t rx = Spi1Xfer16Fast(A1333_NOP_CMD);
    guard = 2000U;
    while (((SPI1->SR & SPI_SR_BSY) != 0U) && (--guard != 0U)) { }
    ENC_CS_PORT->BSRR = (uint32_t)ENC_CS_PIN;

    if (rx != 0xFFFFU) { s_last_good = (uint16_t)(rx & 0x7FFFU); }

    return s_last_good;
}

/* ===================== A1333 register access ============================ *
 *
 * Frame: bit15=0, bit14=W1R0, bits13:8=address, bits7:0=data.
 * Reads are pipelined - the response to a command arrives in the NEXT frame,
 * which is why every read below costs two transfers.
 */

#define A1333_WRITE_BIT     (1U << 14)

/* Direct register addresses (byte-addressed; 16-bit regs span addr:addr+1). */
#define A1333_REG_EWA       0x03U   /* extended write address, low byte */
#define A1333_REG_EWDH      0x04U   /* write data, upper 16 bits        */
#define A1333_REG_EWDL      0x06U   /* write data, lower 16 bits        */
#define A1333_REG_EWCS      0x08U   /* EXW[15] start                    */
#define A1333_REG_EWCS_ST   0x09U   /* WDN[0] done                      */
#define A1333_REG_ERA       0x0BU   /* extended read address, low byte  */
#define A1333_REG_ERCS      0x0CU   /* EXR[15] start                    */
#define A1333_REG_ERCS_ST   0x0DU   /* RDN[0] done                      */
#define A1333_REG_ERDH      0x0EU   /* read data, upper 16 bits         */
#define A1333_REG_ERDL      0x10U   /* read data, lower 16 bits         */
#define A1333_REG_IKEY      0x3CU   /* keycode                          */

#define A1333_ZERO_OFFSET_MASK  0x0FFFU   /* ANG bits [11:0] */

/* EEPROM writes take ~24 ms; shadow completes in one clock. */
#define A1333_WRITE_POLL_MAX    200U

Encoder_Status_t Encoder_RegWrite(uint8_t addr, uint8_t data)
{
  uint16_t rx = 0;
  uint16_t cmd = (uint16_t)(A1333_WRITE_BIT |
                            ((uint16_t)(addr & 0x3FU) << 8) |
                            (uint16_t)data);

  if (Encoder_Frame(cmd, &rx) != HAL_OK) { return ENC_ERR_SPI; }
  Encoder_CsIdleDelay();

  return ENC_OK;
}

Encoder_Status_t Encoder_RegRead(uint8_t addr, uint16_t *out)
{
  uint16_t rx = 0;
  uint16_t cmd = (uint16_t)(((uint16_t)(addr & 0x3FU) << 8));

  /* Frame 1 issues the read; frame 2 clocks the answer back. */
  if (Encoder_Frame(cmd, &rx) != HAL_OK) { return ENC_ERR_SPI; }
  Encoder_CsIdleDelay();
  if (Encoder_Frame(0x0000U, &rx) != HAL_OK) { return ENC_ERR_SPI; }
  Encoder_CsIdleDelay();

  *out = rx;
  return ENC_OK;
}

Encoder_Status_t Encoder_Unlock(void)
{
  /* KeyCode 0x0027811F77, entered as five separate byte writes into the
   * keycode field. Unlock persists until the part is powered down. */
  static const uint8_t key[5] = { 0x00U, 0x27U, 0x81U, 0x1FU, 0x77U };

  for (uint32_t i = 0; i < 5U; i++)
  {
    if (Encoder_RegWrite(A1333_REG_IKEY, key[i]) != ENC_OK)
    {
      return ENC_ERR_SPI;
    }
  }

  return ENC_OK;
}

Encoder_Status_t Encoder_ExtRead(uint8_t ext_addr, uint32_t *out)
{
  uint16_t hi = 0, lo = 0, st = 0;

  if (Encoder_RegWrite(A1333_REG_ERA, ext_addr) != ENC_OK) { return ENC_ERR_SPI; }
  if (Encoder_RegWrite(A1333_REG_ERCS, 0x80U)   != ENC_OK) { return ENC_ERR_SPI; }

  for (uint32_t i = 0; i < A1333_WRITE_POLL_MAX; i++)
  {
    if (Encoder_RegRead(A1333_REG_ERCS_ST, &st) != ENC_OK) { return ENC_ERR_SPI; }
    if ((st & 0x0001U) != 0U) { break; }          /* RDN */
    HAL_Delay(1);
  }

  if (Encoder_RegRead(A1333_REG_ERDH, &hi) != ENC_OK) { return ENC_ERR_SPI; }
  if (Encoder_RegRead(A1333_REG_ERDL, &lo) != ENC_OK) { return ENC_ERR_SPI; }

  *out = ((uint32_t)hi << 16) | (uint32_t)lo;
  return ENC_OK;
}

Encoder_Status_t Encoder_ExtWrite(uint8_t ext_addr, uint32_t data)
{
  uint16_t st = 0;

  if (Encoder_RegWrite(A1333_REG_EWA,      ext_addr)                  != ENC_OK) { return ENC_ERR_SPI; }
  if (Encoder_RegWrite(A1333_REG_EWDH,     (uint8_t)(data >> 24))     != ENC_OK) { return ENC_ERR_SPI; }
  if (Encoder_RegWrite(A1333_REG_EWDH + 1U,(uint8_t)(data >> 16))     != ENC_OK) { return ENC_ERR_SPI; }
  if (Encoder_RegWrite(A1333_REG_EWDL,     (uint8_t)(data >> 8))      != ENC_OK) { return ENC_ERR_SPI; }
  if (Encoder_RegWrite(A1333_REG_EWDL + 1U,(uint8_t)(data))           != ENC_OK) { return ENC_ERR_SPI; }
  if (Encoder_RegWrite(A1333_REG_EWCS,     0x80U)                     != ENC_OK) { return ENC_ERR_SPI; }

  for (uint32_t i = 0; i < A1333_WRITE_POLL_MAX; i++)
  {
    if (Encoder_RegRead(A1333_REG_EWCS_ST, &st) != ENC_OK) { return ENC_ERR_SPI; }
    if ((st & 0x0001U) != 0U) { return ENC_OK; }  /* WDN */
    HAL_Delay(1);
  }

  return ENC_ERR_SPI;   /* never reported done */
}

Encoder_Status_t Encoder_GetZeroOffset(uint16_t *offset12, uint8_t from_eeprom)
{
  uint32_t ang = 0;
  uint8_t  addr = from_eeprom ? A1333_EE_ANG : A1333_SHADOW_ANG;

  if (Encoder_ExtRead(addr, &ang) != ENC_OK) { return ENC_ERR_SPI; }

  *offset12 = (uint16_t)(ang & A1333_ZERO_OFFSET_MASK);
  return ENC_OK;
}

Encoder_Status_t Encoder_SetZeroOffset(uint16_t offset12, uint8_t to_eeprom)
{
  uint32_t ang  = 0;
  uint8_t  addr = to_eeprom ? A1333_EE_ANG : A1333_SHADOW_ANG;

  /* Read-modify-write: ORATE, RD, RO and HYSTERESIS share this register and
   * must survive untouched. */
  if (Encoder_ExtRead(addr, &ang) != ENC_OK) { return ENC_ERR_SPI; }

  ang &= ~(uint32_t)A1333_ZERO_OFFSET_MASK;
  ang |= (uint32_t)(offset12 & A1333_ZERO_OFFSET_MASK);

  if (Encoder_Unlock() != ENC_OK) { return ENC_ERR_SPI; }

  return Encoder_ExtWrite(addr, ang);
}

Encoder_Status_t Encoder_ZeroHere(uint16_t *offset12, uint8_t to_eeprom)
{
  uint16_t raw = 0;

  if (Encoder_ReadAngle(&raw) != ENC_OK) { return ENC_ERR_SPI; }

  /* ZERO_OFFSET is 12-bit while the angle we read is 15-bit, so the reading
   * is scaled down by 8. Verify this empirically before trusting it: write a
   * known offset and measure how far the reported angle actually moves. */
  uint16_t off = (uint16_t)((raw >> 3) & A1333_ZERO_OFFSET_MASK);

  if (Encoder_SetZeroOffset(off, to_eeprom) != ENC_OK) { return ENC_ERR_SPI; }

  *offset12 = off;
  return ENC_OK;
}
