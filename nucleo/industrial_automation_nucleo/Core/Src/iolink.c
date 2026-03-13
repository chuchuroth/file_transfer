/**
 * @file  iolink.c
 * @brief Minimal IO-Link master driver — COM2, 1-byte PDout/PDin.
 *
 * IO-Link physical layer is half-duplex single-wire (C/Q line).
 * The TIOL221EVM transceiver converts the 3.3 V UART signal to the
 * 24 V IO-Link line.  We control the transceiver direction via the EN pin:
 *   EN = 1  → driver enabled (we transmit on C/Q)
 *   EN = 0  → driver disabled (we receive from C/Q)
 *
 * The WAKE pin must be pulled HIGH for ≥ 75 µs to initiate communication.
 *
 * M-sequence frame (OPERATE mode, OD=1, OD byte = PDout):
 *   [MC]  [PDout]  [CKT_M]       — master sends 3 bytes (EN=1)
 *
 * D-sequence response (OD=1, OD byte = PDin):
 *   [PDin]  [CKT_D]              — device responds with 2 bytes (EN=0)
 *
 * MC (Master Command octet) = 0xA2:
 *   Bits 7-6 : MSeq type = 10  (Type_1_V, process data channel)
 *   Bits 5-4 : Channel   = 00  (process data)
 *   Bits 3-0 : 0010           (PD-length encoding: 1 byte out, 1 byte in)
 *
 * CRC8:
 *   Polynomial: 0x37   (IO-Link spec, Section A.1.7)
 *   Initial value: 0x00
 *   The CKT byte places the 6-bit CRC in bits 7-2, and the 2-bit Check
 *   Type (CHK=01) in bits 1-0.
 */

#include "iolink.h"
#include "stm32f7xx_hal.h"   /* CubeMX-generated HAL header */

/* --------------------------------------------------------------------------
 * External HAL handle — defined in main.c by CubeMX code generation
 * -------------------------------------------------------------------------- */
extern UART_HandleTypeDef huart4;

/* --------------------------------------------------------------------------
 * GPIO helpers (CubeMX assigns pin labels; adjust if using different names)
 * --------------------------------------------------------------------------
 * In CubeMX, label PD2 as "IOLINK_EN"  and PG2 as "IOLINK_WAKE".
 * The generated gpio.c will create these macros automatically.
 * -------------------------------------------------------------------------- */
#define IOLINK_EN_GPIO_Port    GPIOD
#define IOLINK_EN_Pin          GPIO_PIN_2

#define IOLINK_WAKE_GPIO_Port  GPIOG
#define IOLINK_WAKE_Pin        GPIO_PIN_2

/* --------------------------------------------------------------------------
 * Timing constants
 * -------------------------------------------------------------------------- */
#define WAKE_PULSE_US        100u    /* > 75 µs required by IO-Link spec */
#define UART_RX_TIMEOUT_MS    10u    /* generous timeout for device reply */

/* Master command octet for 1-byte PDout, 1-byte PDin, OPERATE mode */
#define MC_BYTE  0xA2u

/* --------------------------------------------------------------------------
 * CRC-8 (polynomial 0x37, init 0x00)  Per IO-Link spec Annex A.1.7
 * -------------------------------------------------------------------------- */
static uint8_t crc8_iolink(const uint8_t *data, uint8_t len)
{
    static const uint8_t crc_table[256] = {
        0x00,0x37,0x6E,0x59,0xDC,0xEB,0xB2,0x85,0xB8,0x8F,0xD6,0xE1,0x64,0x53,0x0A,0x3D,
        0x70,0x47,0x1E,0x29,0xAC,0x9B,0xC2,0xF5,0xC8,0xFF,0xA6,0x91,0x14,0x23,0x7A,0x4D,
        0xE0,0xD7,0x8E,0xB9,0x3C,0x0B,0x52,0x65,0x58,0x6F,0x36,0x01,0x84,0xB3,0xEA,0xDD,
        0x90,0xA7,0xFE,0xC9,0x4C,0x7B,0x22,0x15,0x28,0x1F,0x46,0x71,0xF4,0xC3,0x9A,0xAD,
        0xC0,0xF7,0xAE,0x99,0x1C,0x2B,0x72,0x45,0x78,0x4F,0x16,0x21,0xA4,0x93,0xCA,0xFD,
        0xB0,0x87,0xDE,0xE9,0x6C,0x5B,0x02,0x35,0x08,0x3F,0x66,0x51,0xD4,0xE3,0xBA,0x8D,
        0x20,0x17,0x4E,0x79,0xFC,0xCB,0x92,0xA5,0x98,0xAF,0xF6,0xC1,0x44,0x73,0x2A,0x1D,
        0x50,0x67,0x3E,0x09,0x8C,0xBB,0xE2,0xD5,0xE8,0xDF,0x86,0xB1,0x34,0x03,0x5A,0x6D,
        0x80,0xB7,0xEE,0xD9,0x5C,0x6B,0x32,0x05,0x38,0x0F,0x56,0x61,0xE4,0xD3,0x8A,0xBD,
        0xF0,0xC7,0x9E,0xA9,0x2C,0x1B,0x42,0x75,0x48,0x7F,0x26,0x11,0x94,0xA3,0xFA,0xCD,
        0x60,0x57,0x0E,0x39,0xBC,0x8B,0xD2,0xE5,0xD8,0xEF,0xB6,0x81,0x04,0x33,0x6A,0x5D,
        0x10,0x27,0x7E,0x49,0xCC,0xFB,0xA2,0x95,0xA8,0x9F,0xC6,0xF1,0x74,0x43,0x1A,0x2D,
        0x40,0x77,0x2E,0x19,0x9C,0xAB,0xF2,0xC5,0xF8,0xCF,0x96,0xA1,0x24,0x13,0x4A,0x7D,
        0x30,0x07,0x5E,0x69,0xEC,0xDB,0x82,0xB5,0x88,0xBF,0xE6,0xD1,0x54,0x63,0x3A,0x0D,
        0xA0,0x97,0xCE,0xF9,0x7C,0x4B,0x12,0x25,0x18,0x2F,0x76,0x41,0xC4,0xF3,0xAA,0x9D,
        0xD0,0xE7,0xBE,0x89,0x0C,0x3B,0x62,0x55,0x68,0x5F,0x06,0x31,0xB4,0x83,0xDA,0xED
    };
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc_table[crc ^ data[i]];
    }
    return crc;
}

/**
 * @brief  Build the 2-bit-padded CKT byte.
 *         CKT[7:2] = CRC8[5:0],  CKT[1:0] = CHK type = 01 (OD type).
 */
static uint8_t make_ckt(const uint8_t *data, uint8_t len)
{
    uint8_t crc = crc8_iolink(data, len);
    return (uint8_t)((crc & 0xFCu) | 0x01u);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void iolink_init(void)
{
    /* Disable transceiver output initially — we are in receive mode */
    HAL_GPIO_WritePin(IOLINK_EN_GPIO_Port, IOLINK_EN_Pin, GPIO_PIN_RESET);

    /* Send wake-up pulse: pull WAKE line HIGH for WAKE_PULSE_US µs */
    HAL_GPIO_WritePin(IOLINK_WAKE_GPIO_Port, IOLINK_WAKE_Pin, GPIO_PIN_SET);
    /* HAL_Delay granularity is 1 ms; use DWT cycle count for µs precision */
    uint32_t t0 = DWT->CYCCNT;
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000u) * WAKE_PULSE_US;
    while ((DWT->CYCCNT - t0) < cycles) { /* spin */ }
    HAL_GPIO_WritePin(IOLINK_WAKE_GPIO_Port, IOLINK_WAKE_Pin, GPIO_PIN_RESET);

    /* Allow device 850 µs startup time (IO-Link spec §6.3.3) */
    HAL_Delay(2); /* 2 ms — rounded up */
}

int iolink_cycle(uint8_t pd_out, uint8_t *pd_in)
{
    /* ------------------------------------------------------------------ */
    /* 1. Build M-sequence: [MC][PDout][CKT_M]                            */
    /* ------------------------------------------------------------------ */
    uint8_t tx[3];
    tx[0] = MC_BYTE;
    tx[1] = pd_out;
    uint8_t ckt_src[2] = { MC_BYTE, pd_out };
    tx[2] = make_ckt(ckt_src, 2);

    /* ------------------------------------------------------------------ */
    /* 2. Transmit M-sequence (EN=1 → driver active)                      */
    /* ------------------------------------------------------------------ */
    HAL_GPIO_WritePin(IOLINK_EN_GPIO_Port, IOLINK_EN_Pin, GPIO_PIN_SET);
    HAL_StatusTypeDef st = HAL_UART_Transmit(&huart4, tx, 3, 10);
    HAL_GPIO_WritePin(IOLINK_EN_GPIO_Port, IOLINK_EN_Pin, GPIO_PIN_RESET);

    if (st != HAL_OK) {
        return IOLINK_ERR_TIMEOUT;
    }

    /* ------------------------------------------------------------------ */
    /* 3. Receive D-sequence: [PDin][CKT_D]  (EN=0 → receiver active)    */
    /* ------------------------------------------------------------------ */
    uint8_t rx[2] = { 0x00, 0x00 };
    st = HAL_UART_Receive(&huart4, rx, 2, UART_RX_TIMEOUT_MS);
    if (st != HAL_OK) {
        return IOLINK_ERR_TIMEOUT;
    }

    /* ------------------------------------------------------------------ */
    /* 4. Verify CKT_D                                                    */
    /* ------------------------------------------------------------------ */
    uint8_t expected_ckt = make_ckt(&rx[0], 1);
    if (rx[1] != expected_ckt) {
        return IOLINK_ERR_CRC;
    }

    *pd_in = rx[0];
    return IOLINK_OK;
}
