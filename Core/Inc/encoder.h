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

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
