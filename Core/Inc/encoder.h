/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   RM44SI magnetic angle sensor over SPI3.
  *
  *          Replaces the Allegro A1333 that Mako Longfin used. Both are
  *          absolute magnetic encoders read over SPI, and that is where the
  *          resemblance stops:
  *
  *                          A1333                  this board
  *            bus           SPI1 PA5/PB4/PB5       SPI3 PC10/PC11/PC12
  *            PA15          -                      XDIR, NOT a chip select
  *            resolution    15-bit, 32768 counts   13-bit, 8192 counts
  *            transaction   two 16-bit frames      one 14-bit frame
  *            zeroing       ZERO_OFFSET register   none - see below
  *
  *          The A1333 answered a command in the FOLLOWING frame, so every read
  *          cost two transfers and a CS idle gap between them. This part
  *          answers within the single frame it is clocked by: transmit zeros,
  *          take the response. The angle is the low 13 bits of the 14 clocked
  *          back; what the 14th bit means is not documented in the material
  *          this port was written from, and it is masked off.
  *
  * ---------------------------------------------------------------------------
  * !! PA15 IS A TRANSCEIVER DIRECTION PIN, NOT A CHIP SELECT !!
  * ---------------------------------------------------------------------------
  *          Schematic sheet 3 ("Emrax Position Encoder"). The link is not
  *          plain SPI at all - it is DIFFERENTIAL, through a pair of SN65176B
  *          RS485 transceivers and an NXU0304BQ 3V3/5V level shifter:
  *
  *              SCK  -> ENCLK_P / ENCLK_N     clock pair
  *              MOSI \
  *              MISO / -> ENDAT_P / ENDAT_N   bidirectional data pair
  *              PA15 -> XDIR, driving DE/RE on the data transceiver
  *
  *          So this is an SSI / BiSS / EnDat class interface, where the master
  *          clocks a differential pair and the encoder answers on a shared
  *          data pair - not a four-wire SPI device with a select line.
  *
  *          This code drives PA15 low for the duration of a frame and calls it
  *          a chip select. With DE and RE tied together that happens to hold
  *          the transceiver in RECEIVE for the transfer, which is the right
  *          state for reading, so it may well work as written. It works for
  *          the wrong reason, and it can never TRANSMIT - so any protocol
  *          needing a command sent to the encoder first is not reachable
  *          without driving XDIR the other way around the write.
  *
  *          BOARD_UNKNOWN, and worth settling before trusting the angle: the
  *          actual encoder part and its protocol, whether the 14-bit frame
  *          size is right for it, and the idle sense of XDIR.
  *
  * ---------------------------------------------------------------------------
  * Counts are reported in the 15-bit convention, not the sensor's 13
  * ---------------------------------------------------------------------------
  *          Every consumer of this module - foc.c's electrical angle, the
  *          CORDIC shift that turns counts into a Q31 angle, position.c's
  *          multi-turn unwrapper, the degree conversion here - is written
  *          around a full mechanical turn being 32768 counts. That is not an
  *          arbitrary choice on foc.c's part: the `counts << 17` conversion is
  *          exact precisely because a half turn lands on the Q31 sign bit, and
  *          the comment there explains at length how the off-by-pi version of
  *          that same expression stays self-consistent and silently inverts
  *          torque.
  *
  *          So the sensor's 13-bit reading is shifted up by 2 here, once, and
  *          everything downstream is untouched and still means what it says.
  *          The alternative - rescaling the angle path for a different modulus
  *          - would have put the most delicate and least loudly-failing code
  *          in the project up for renegotiation to save one shift.
  *
  *          The honest consequence: the bottom 2 bits of every reported count
  *          are always zero. Mechanical resolution really is 4x coarser than
  *          on Mako Longfin. That is the hardware, stated rather than hidden.
  *
  * ---------------------------------------------------------------------------
  * There is no zeroing register any more
  * ---------------------------------------------------------------------------
  *          The A1333 carried a 12-bit ZERO_OFFSET in an EEPROM/shadow
  *          register pair, so encoder zero could be made to BE electrical
  *          zero and foc.c needed no offset term at all. Nothing equivalent is
  *          known on this part, so electrical zero moves into firmware:
  *          `g_foc.elec_offset`, which foc.c already adds and which the SWD
  *          tools already read and write.
  *
  *          The whole A1333 register-access surface - unlock, direct and
  *          extended register access, SetZeroOffset, ZeroHere - is gone with
  *          it rather than left compiling against a part that is not there.
  ******************************************************************************
  */
#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "board.h"

typedef enum {
    ENC_OK = 0,
    ENC_ERR_SPI,
} Encoder_Status_t;

/* The sensor's own resolution, and the shift that brings it up to the 15-bit
 * convention every consumer is written around. Derived rather than written as
 * 2, so a different part changes one number in board.h. */
#define ENC_NATIVE_COUNTS       BOARD_ENC_COUNTS      /* 8192  */
#define ENC_NATIVE_MASK         (BOARD_ENC_COUNTS - 1U)
#define ENC_REPORT_COUNTS       32768U
#define ENC_UPSHIFT             2U                    /* 8192 << 2 == 32768 */

/* Drives CS high (idle). Call after MX_SPI3_Init(). */
void Encoder_Init(void);

/* Returns the mechanical count in the 15-bit convention, [0, 32767], with the
 * low 2 bits always zero. On SPI error the last good value is returned and
 * ENC_ERR_SPI reported. */
Encoder_Status_t Encoder_ReadAngle(uint16_t *raw_counts);

/* Encoder link health, maintained by Encoder_ReadAngleFast.
 *
 * That function substitutes the LAST GOOD angle whenever a frame comes back
 * all-ones, which is what a dead MISO line reads as. On its own that is the
 * right thing to do - one bad frame should not put a garbage angle into the
 * Park transform - but it used to be completely silent, so a disconnected
 * encoder produced a plausible, constant, entirely fictional angle for as
 * long as anyone cared to run. The control loop cannot tell that apart from a
 * stalled rotor, and will happily pour current into a fixed electrical angle.
 *
 * So the substitution is counted. g_enc_sub_consec is the run length, which is
 * what matters: isolated substitutions are noise, a sustained run is a broken
 * link. g_enc_sub_total is for spotting a marginal connection that is not yet
 * failing outright.
 *
 * The all-ones test is now against a 14-bit frame rather than a 16-bit one.
 * Testing for 0xFFFF here would never have matched, because the two top bits
 * of a 14-bit transfer never arrive - the detector would have been present,
 * compiled, and permanently blind. */
extern volatile uint32_t g_enc_sub_consec;
extern volatile uint32_t g_enc_sub_total;

/* Consecutive substitutions before the drive treats the encoder as failed.
 *
 * 8 ticks is 400 us at 20 kHz. Long enough that a single disturbed frame -
 * or a short burst of them - is ridden out, short enough that a severed link
 * is caught in well under a millisecond, which is far faster than the rotor
 * can move anywhere interesting. */
#define ENC_MAX_SUBSTITUTIONS   8U

/* The most recent raw SPI frame, for bring-up debugging. Mako Longfin's
 * version handed back two, because the A1333 needed two transfers per read;
 * there is only ever one here. The second output is retained and always zero
 * so the SWD readers and their fixed struct layout do not have to change. */
void Encoder_GetDebugData(uint16_t *rx1, uint16_t *rx2);

/* Count in the 15-bit convention -> hundredths of a degree [0, 35999].
 * Integer only, so this stays usable from an ISR and needs no
 * float-enabled printf. */
uint32_t Encoder_RawToDegX100(uint16_t raw_counts);

/* Direct-register read for the control ISR. The HAL path costs most of a PWM
 * period, which is unaffordable at the control rate. No error reporting: on a
 * timeout it returns the previous good value. */
uint16_t Encoder_ReadAngleFast(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
