/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   Allegro A1333 magnetic angle sensor over SPI1.
  *
  *          Ported from the makoshortfin project, which drives the same part on
  *          SPI3 using the LL drivers. This board wires the encoder to SPI1
  *          (PA5 SCK / PB4 MISO / PB5 MOSI) with a manual chip select on PA4,
  *          and the rest of this project is HAL-based, so the transfer is done
  *          with HAL_SPI_TransmitReceive instead.
  ******************************************************************************
  */
#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    ENC_OK = 0,
    ENC_ERR_SPI,
} Encoder_Status_t;

/* Drives CS high (idle). Call after MX_SPI1_Init(). */
void Encoder_Init(void);

/* Returns raw 15-bit mechanical count [0, 32767].
 * On SPI error the last good value is returned and ENC_ERR_SPI reported. */
Encoder_Status_t Encoder_ReadAngle(uint16_t *raw_counts);

/* Both raw SPI frames from the most recent read, for bring-up debugging. */
void Encoder_GetDebugData(uint16_t *rx1, uint16_t *rx2);

/* Raw 15-bit count -> hundredths of a degree [0, 35999]. Integer only, so
 * this stays usable from an ISR and needs no float-enabled printf. */
uint32_t Encoder_RawToDegX100(uint16_t raw_counts);

/* Direct-register angle read for the control ISR. The HAL path costs most of
 * a PWM period, which is unaffordable at 30 kHz. No error reporting: on a
 * timeout it returns the previous good value. */
uint16_t Encoder_ReadAngleFast(void);

/* ---- A1333 register access -------------------------------------------- *
 *
 * SPI frame is  bit15=0 | bit14=W1R0 | bits13:8=address | bits7:0=data,
 * and reads are pipelined: the response to a command arrives in the NEXT
 * frame. All of this is 16-bit mode 3, same as the angle read.
 *
 * ZERO_OFFSET lives in bits [11:0] of the ANG register, which is reachable
 * two ways:
 *   EEPROM  0x1C  - permanent, but rated for only ~100 write cycles and
 *                   takes ~24 ms per write
 *   SHADOW  0x5C  - volatile (lost at power-off), immediate, unlimited
 *
 * Always trial a value in SHADOW first. Only commit to EEPROM once the
 * number is final; the write budget is small and non-renewable.
 */

#define A1333_EE_ANG        0x1CU   /* EEPROM ANG register  */
#define A1333_SHADOW_ANG    0x5CU   /* Shadow ANG register  */

/* Single-byte direct register write / 16-bit direct register read. */
Encoder_Status_t Encoder_RegWrite(uint8_t addr, uint8_t data);
Encoder_Status_t Encoder_RegRead(uint8_t addr, uint16_t *out);

/* Writes are refused until the keycode is entered. Lasts until power-off. */
Encoder_Status_t Encoder_Unlock(void);

/* Extended (EEPROM / shadow) 32-bit access. */
Encoder_Status_t Encoder_ExtRead(uint8_t ext_addr, uint32_t *out);
Encoder_Status_t Encoder_ExtWrite(uint8_t ext_addr, uint32_t data);

/* Set ZERO_OFFSET (12-bit) in shadow or EEPROM, preserving the other fields
 * of the ANG register. Angle_out = Angle_RAW - ZERO_OFFSET. */
Encoder_Status_t Encoder_SetZeroOffset(uint16_t offset12, uint8_t to_eeprom);
Encoder_Status_t Encoder_GetZeroOffset(uint16_t *offset12, uint8_t from_eeprom);

/* Zero the encoder at the current rotor position: reads the angle now and
 * programs it as the offset, so this position becomes 0 deg. Writes SHADOW
 * unless to_eeprom is set. Returns the offset written via offset12. */
Encoder_Status_t Encoder_ZeroHere(uint16_t *offset12, uint8_t to_eeprom);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
