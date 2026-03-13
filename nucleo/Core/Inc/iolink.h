/**
 * @file  iolink.h
 * @brief Minimal IO-Link master driver for NUCLEO-F746ZG.
 *
 * Hardware connections (from pin_mapping.md §2):
 *   PA0  — UART4_TX  → TIOL221EVM J6 Pin 1 (Data In / C/Q)
 *   PA1  — UART4_RX  → TIOL221EVM J6 Pin 2 (Data Out / C/Q echo)
 *   PD2  — GPIO OUT  → TIOL221EVM J6 Pin 3 (Enable)
 *   PG2  — GPIO OUT  → TIOL221EVM J6 Pin 4 (Wake-Up)
 *
 * IO-Link speed used: COM2 (38.4 kbps).
 *
 * Frame layout (OPERATE mode, 1-byte PDout, 1-byte PDin):
 *   Master → Device :  [MC=0xA2] [PDout] [CKT_M]
 *   Device → Master :  [PDin]    [CKT_D]
 *
 *   where CKT = CRC8(polynomial=0x37, init=0x00) over all preceding bytes
 *                 combined with the 2-bit CHK type field in the LSBs.
 */

#ifndef IOLINK_H
#define IOLINK_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Return codes
 * -------------------------------------------------------------------------- */
#define IOLINK_OK           (0)
#define IOLINK_ERR_TIMEOUT  (-1)
#define IOLINK_ERR_CRC      (-2)

/* --------------------------------------------------------------------------
 * Gripper process-data commands (PDout byte sent to Zimmer LWR50L-02)
 * -------------------------------------------------------------------------- */
#define IOLINK_PD_GRIP    (0x01u)   /* Bit 0 = 1 → close jaws (grip) */
#define IOLINK_PD_RELEASE (0x00u)   /* Bit 0 = 0 → open jaws (release) */

/* --------------------------------------------------------------------------
 * Gripper status bits returned in PDin byte from the device
 * -------------------------------------------------------------------------- */
#define IOLINK_STATUS_GRIPPED  (0x01u)   /* Bit 0: jaws fully closed */
#define IOLINK_STATUS_OPEN     (0x02u)   /* Bit 1: jaws fully open   */

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initialise GPIO (EN, WAKE) and send IO-Link wake-up pulse.
 *         Must be called once before any iolink_send_*() call.
 *         Relies on huart4 already being initialised by HAL/CubeMX.
 */
void iolink_init(void);

/**
 * @brief  Send one IO-Link cycle with the given process-data output byte.
 * @param  pd_out  Process data to send to the device (IOLINK_PD_GRIP or
 *                 IOLINK_PD_RELEASE).
 * @param  pd_in   [out] Process data received from the device (device status).
 * @return IOLINK_OK on success, IOLINK_ERR_TIMEOUT or IOLINK_ERR_CRC on failure.
 */
int iolink_cycle(uint8_t pd_out, uint8_t *pd_in);

#endif /* IOLINK_H */
