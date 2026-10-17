/**
 * @file    hil_target.h
 * @brief   On-target HIL run-time: 100 Hz control ISR + UART frame handling.
 *
 * Integration contract for the STM32 application (see hil_port_stm32f4.c for a
 * concrete HAL example):
 *
 *   1. main()  : HIL_Init();
 *   2. UART RX : call HIL_OnByte(b) for EVERY received byte
 *                (from the DMA "half/full transfer" callback or the RX ISR)
 *   3. TIM6 ISR: call HIL_Step10ms() every 10 ms exactly
 *                (configure TIM6 for 100 Hz, highest reasonable priority)
 *
 * Timing budget on STM32F407IGT6 @168 MHz with FPU:
 *   one control tick is roughly 2-4 k single-precision FLOPs + 12 trig calls
 *   -> ~30-60 us, i.e. < 1 % CPU at 100 Hz.
 *   The UART at 460800 baud needs ~3 ms to move 138 B, which comfortably
 *   fits inside the 10 ms period even with a lot of slack.
 */

#ifndef HIL_TARGET_H
#define HIL_TARGET_H

#include "hil_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Freshness policy --------------------------------------------------- */
/* A setpoint older than this (ms) is considered stale.  At 100 Hz the PC
 * sends every 10 ms, so 50 ms == 5 missed frames.                        */
#define HIL_SETPOINT_TIMEOUT_MS   (50u)

/* Latest command actually written to the thrusters ------------------- */
extern volatile float g_hil_last_u[HIL_N_U];

/* Counters, handy for the PC-side diagnostics ------------------------ */
extern volatile uint32_t g_hil_tick_cnt;      /* control ticks executed     */
extern volatile uint32_t g_hil_tx_cnt;        /* command frames transmitted */
extern volatile uint32_t g_hil_rx_ok;         /* valid setpoint frames      */
extern volatile uint32_t g_hil_rx_crc;        /* CRC failures               */
extern volatile uint32_t g_hil_rx_frame;      /* framing failures           */
extern volatile uint32_t g_hil_rx_drop;       /* setpoints overwritten      */

/* Life cycle -------------------------------------------------------- */
void HIL_Init(void);

/* Feed one received UART byte (safe to call from an ISR).             */
void HIL_OnByte(uint8_t b);

/* Execute exactly one 10 ms control tick.  Call from the TIM ISR.     */
void HIL_Step10ms(void);

/* Optional: fill a status frame on demand (not used by the ISR path).  */
void HIL_GetCommand(hil_command_t *out);

#ifdef __cplusplus
}
#endif

#endif /* HIL_TARGET_H */
