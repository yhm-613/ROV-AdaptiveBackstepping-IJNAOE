/**
 * @file    hil_port.h
 * @brief   Hardware abstraction layer for the HIL run-time.
 *
 * Only four functions have to be provided by the board support package.
 * hil_port_stm32f4.c contains a ready-to-adapt implementation for the
 * YS-F4Pro board (STM32F407IGT6) built on the STM32Cube HAL.
 */

#ifndef HIL_PORT_H
#define HIL_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  One-off hardware setup: UART pins + baud rate, TX DMA, TIM6 100 Hz.
 *         Called from HIL_Init() before the first control tick.
 */
void hil_port_init(void);

/**
 * @brief  Transmit @p len bytes on the HIL UART.
 * @return number of bytes accepted, or -1 on error.
 *
 * Called once per control tick with a 34-byte frame.  Must be non-blocking
 * enough to fit inside the 10 ms ISR: either fire-and-forget DMA, or copy
 * into a software ring buffer drained by the TX interrupt.
 */
int hil_port_uart_write(const uint8_t *buf, uint16_t len);

/**
 * @brief  Hand the 6 clamped thruster voltages to the plant interface.
 *
 * In a pure HIL set-up the "plant" is the PC, so this usually does nothing
 * (or writes to a DAC / a scope pin for debugging).  In a real RCP set-up
 * this drives the ESC/PWM outputs.
 */
void hil_port_apply_u(const volatile float *u, int n);

/**
 * @brief  Start the UART RX path: DMA in circular mode + IDLE-line interrupt.
 *         Called from hil_port_init(); normally not called directly.
 */
void hil_port_rx_start(void);
void hil_port_rx_restart(void);   /* self-heal after ORE/FE/NE error */

/**
 * @brief  Harvest every byte the RX DMA has written since the last call.
 *
 * Call this from USART1_IRQHandler() when the IDLE flag is set, i.e. once per
 * received burst.  Each byte is handed to HIL_OnByte(), which drives the
 * incremental frame parser.
 */
void hil_port_rx_poll_idle(void);
void hil_port_rx_watchdog(void);    /* 100 Hz RX health check, call from tick */

/**
 * @brief  Start the 100 Hz control tick (TIM6).
 *         Called from hil_port_init(); normally not called directly.
 */
void hil_port_tim6_start(void);

/**
 * @brief  Execute one control period.  Call from TIM6_DAC_IRQHandler().
 *
 * Reads TIM6->SR, clears UIF and runs HIL_Step10ms().  It deliberately does
 * NOT go through HAL_TIM_IRQHandler(), so do not keep that call as well.
 */
void hil_port_tim6_isr(void);

/**
 * @brief  Free-running millisecond counter (SysTick based is fine).
 */
uint32_t hil_port_millis(void);

#ifdef __cplusplus
}
#endif

#endif /* HIL_PORT_H */
