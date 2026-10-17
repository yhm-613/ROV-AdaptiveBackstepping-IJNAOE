/**
 * @file    hil_port_stm32f4.c
 * @brief   Board support for YS-F4Pro (STM32F407IGT6) -- STM32Cube HAL.
 *
 * Wiring used by this implementation (verified on the actual YS-F4Pro board
 * on 2026-10-12 by reading the factory firmware's GPIO MODER/AFR registers):
 *
 *   USART1  PB6  (TX, AF7)  ->  onboard CH340G RXD  ->  MINI USB  ->  PC
 *           PB7  (RX, AF7)  <-  onboard CH340G TXD  <-  MINI USB  <-  PC
 *           GND             <->  common via the MINI USB shield/ground
 *
 *   NOTE: PB6/PB7, *not* PA9/PA10.  On this board PA9/PA10 are left as plain
 *   inputs by the factory firmware; the CH340G is hard-wired to PB6/PB7.
 *   No external USB-TTL adapter is needed -- one MINI USB cable powers the
 *   board and carries the serial link.  See firmware/stm32_app/README.md 1.1.
 *
 *   TIM6    APB1 timer, 100 Hz update interrupt -> HIL_Step10ms()
 *
 * Everything else is CubeMX boilerplate: enable USART1 in asynchronous mode,
 * enable both of its DMA channels (TX memory -> peripheral, RX circular),
 * and enable TIM6.  See firmware/stm32_app/README.md for the click-by-click
 * CubeMX table and docs/CubeMX配置清单.md for the wiring checklist.
 *
 * RECOMMENDED INTEGRATION (this is what the defaults assume):
 *
 *   HIL_PORT_INIT_UART = 0   ->  CubeMX generates MX_USART1_UART_Init() and
 *                                HAL_UART_MspInit(), which own huart1 and the
 *                                two hdma_* handles.  This file then only
 *                                supplies the run-time behaviour: TX ring
 *                                buffer, RX idle-line pump, NVIC priorities
 *                                and the 100 Hz tick.
 *
 *   Set it to 1 only if you are NOT using CubeMX for the peripheral set-up;
 *   doing so while CubeMX also generates stm32f4xx_it.c causes a HardFault
 *   (see the long comment above HIL_PORT_INIT_UART).
 *
 * IMPORTANT: level shifting.  The STM32 pins are 3.3 V; most USB-TTL
 * adapters are 3.3 V too, so direct connection is normally fine.
 * Never connect a 5 V TTL adapter output straight to PA10.
 */

#include "hil_port.h"
#include "hil_target.h"

#include "stm32f4xx_hal.h"      /* provided by CubeMX / the board SDK */

#include <string.h>

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

/* 460800 is the smallest rate that carries 138 B / 10 ms with < 60 % load.
 * Use 921600 if your USB-TTL adapter is stable at that speed.           */
#define HIL_UART_BAUD           (460800u)

/* ------------------------------------------------------------------ */
/* Who owns USART1 and its DMA handles?                                */
/*                                                                     */
/*   0  (DEFAULT, SAFE)  the CubeMX-generated MX_USART1_UART_Init()    */
/*                       owns huart1 AND both DMA handles              */
/*   1                   uart1_init() further down does it instead     */
/*                                                                     */
/* KEEP THIS AT 0 unless you hand-write the entire peripheral set-up.  */
/*                                                                     */
/* Why 0 is the safe choice:                                           */
/*   The CubeMX-generated stm32f4xx_it.c ships                        */
/*                                                                     */
/*       void DMA2_Stream7_IRQHandler(void)                            */
/*       { HAL_DMA_IRQHandler(&hdma_usart1_tx); }                      */
/*                                                                     */
/*   where hdma_usart1_tx is the GLOBAL handle CubeMX declares in      */
/*   main.c and links to huart1 inside HAL_UART_MspInit().  If         */
/*   uart1_init() below instead creates its OWN file-local DMA handle, */
/*   the interrupt fires on a handle that was never initialised        */
/*   (Instance == NULL) and HAL_DMA_IRQHandler() dereferences it on    */
/*   its very first line -> HardFault on the first transmitted frame.  */
/*   Letting CubeMX own the handles keeps exactly one DMA object per   */
/*   stream, which is the invariant the HAL assumes.                   */
/* ------------------------------------------------------------------ */
#define HIL_PORT_INIT_UART      (0)

extern UART_HandleTypeDef huart1;   /* defined in the CubeMX main.c      */

/* ------------------------------------------------------------------ */
/* TX: software ring buffer drained by DMA                             */
/* ------------------------------------------------------------------ */

#define HIL_TX_BUF_SZ           (512u)   /* 14 frames of headroom          */

static uint8_t               s_tx_buf[HIL_TX_BUF_SZ];
static volatile uint16_t     s_tx_head = 0u;   /* written by ISR (producer) */
static volatile uint16_t     s_tx_tail = 0u;   /* written by DMA callback   */
static volatile uint8_t      s_tx_busy = 0u;

static uint16_t tx_used(void)
{
    return (uint16_t)((s_tx_head - s_tx_tail) & (HIL_TX_BUF_SZ - 1u));
}

static uint16_t tx_free(void)
{
    return (uint16_t)(HIL_TX_BUF_SZ - 1u - tx_used());
}

static void tx_kick(void)
{
    uint16_t used, chunk;

    if (s_tx_busy != 0u) { return; }
    used = tx_used();
    if (used == 0u)      { return; }

    /* transmit up to the end of the linear region in one DMA shot */
    chunk = (uint16_t)(HIL_TX_BUF_SZ - s_tx_tail);
    if (chunk > used) { chunk = used; }

    s_tx_busy = 1u;
    if (HAL_UART_Transmit_DMA(&huart1, &s_tx_buf[s_tx_tail], chunk) != HAL_OK) {
        s_tx_busy = 0u;
    }
}

/** HAL callback, called from the DMA IRQ when the chunk is on the wire. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) { return; }

    uint16_t used = tx_used();
    uint16_t chunk = (uint16_t)(HIL_TX_BUF_SZ - s_tx_tail);
    if (chunk > used) { chunk = used; }

    s_tx_tail = (uint16_t)((s_tx_tail + chunk) & (HIL_TX_BUF_SZ - 1u));
    s_tx_busy = 0u;
    tx_kick();
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int hil_port_uart_write(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (len > tx_free()) { return -1; }        /* never block the ISR */

    for (i = 0u; i < len; ++i) {
        s_tx_buf[s_tx_head] = buf[i];
        s_tx_head = (uint16_t)((s_tx_head + 1u) & (HIL_TX_BUF_SZ - 1u));
    }
    tx_kick();
    return (int)len;
}

void hil_port_apply_u(const volatile float *u, int n)
{
    /* Pure HIL: the plant lives on the PC, so there is nothing to drive.
     * Uncomment one of the blocks below for a real RCP set-up.          */
    (void)u;
    (void)n;

#if 0
    /* --- Option A: 6 PWM outputs (e.g. TIM1 CH1..CH4 + TIM8 CH1..CH2) --- */
    for (int i = 0; i < n; ++i) {
        /* map -100..+100 V to a 1100..1900 us pulse; 1500 us = stop      */
        float v = u[i];
        if (v >  100.0f) { v =  100.0f; }
        if (v < -100.0f) { v = -100.0f; }
        uint32_t us = (uint32_t)(1500.0f + v * 4.0f);
        esc_set_pulse_us(i, us);
    }
#endif

#if 0
    /* --- Option B: 6 DAC channels, purely for oscilloscope debugging --- */
    for (int i = 0; i < n; ++i) {
        uint16_t code = (uint16_t)(2048.0f + u[i] * 20.0f);   /* +/-100 V */
        debug_dac_write(i, code);
    }
#endif
}

/* ------------------------------------------------------------------ */
/* UART RX: DMA in circular mode + IDLE-line interrupt                 */
/* ------------------------------------------------------------------ */

#define HIL_RX_BUF_SZ          (256u)
static uint8_t s_rx_buf[HIL_RX_BUF_SZ];
static volatile uint16_t s_rx_pos = 0u;

void hil_port_rx_start(void)
{
    s_rx_pos = 0u;
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UART_Receive_DMA(&huart1, s_rx_buf, HIL_RX_BUF_SZ);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/** @brief  Restart RX DMA after any UART error.
 *  @warning  Without this, ONE glitched byte kills reception forever.
 *
 *  Two HAL (F4) quirks make a naive restart insufficient:
 *    1. HAL's error path (ORE/FE/NE in DMA mode) aborts the RX stream and
 *       never calls the error callback again.
 *    2. HAL_DMA_Abort_IT() on F4 leaves hdma->State == HAL_DMA_STATE_ABORT
 *       because no DMA interrupt fires to complete the abort; every later
 *       HAL_UART_Receive_DMA() then returns HAL_BUSY while the UART-side
 *       DMAR/EIE bits have already been re-set -- a half-dead state we
 *       observed live (CR3=0x41, S5CR=0, rx dead after frame 1).
 *  So this function resets the stream, the flags and the handle states BY
 *  HAND before re-arming. */
void hil_port_rx_restart(void)
{
    DMA_HandleTypeDef *hd = huart1.hdmarx;
    extern volatile uint32_t g_dbg_restart;
    g_dbg_restart++;

    /* 1. stop the stream unconditionally */
    __HAL_DMA_DISABLE(hd);
    while (hd->Instance->CR & DMA_SxCR_EN) { }

    /* 2. clear every stream-5 flag (write-1-to-clear) */
    __HAL_DMA_CLEAR_FLAG(hd, DMA_FLAG_FEIF1_5 | DMA_FLAG_DMEIF1_5 |
                             DMA_FLAG_TEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5);

    /* 3. reset the handle states that the stuck abort left behind */
    hd->State     = HAL_DMA_STATE_READY;
    hd->ErrorCode = HAL_DMA_ERROR_NONE;
    huart1.RxState       = HAL_UART_STATE_READY;
    huart1.ReceptionType = HAL_UART_RECEPTION_STANDARD;
    huart1.ErrorCode     = HAL_UART_ERROR_NONE;

    /* 4. clear UART error flags (SR read + DR read does it on F4) */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    /* 5. re-arm */
    s_rx_pos = 0u;
    if (HAL_UART_Receive_DMA(&huart1, s_rx_buf, HIL_RX_BUF_SZ) == HAL_OK) {
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    }
}

/** HAL error callback: any reception error re-arms the RX path instead of
 *  leaving it dead (the parser simply re-hunts for the 0x55 0xAA sync). */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) { return; }
    hil_port_rx_restart();
}

/** @brief  Call this from USART1_IRQHandler() when the IDLE flag is set. */
void hil_port_rx_poll_idle(void)
{
    uint16_t pos = (uint16_t)(HIL_RX_BUF_SZ - __HAL_DMA_GET_COUNTER(huart1.hdmarx));

    while (s_rx_pos != pos) {
        HIL_OnByte(s_rx_buf[s_rx_pos]);
        s_rx_pos = (uint16_t)((s_rx_pos + 1u) % HIL_RX_BUF_SZ);
    }
}

/** @brief  100 Hz RX health watchdog, called from HIL_Step10ms().
 *
 *  Some HAL error path (observed with glitched bytes on a marginal link)
 *  clears CR3.DMAR without ever delivering the abort callback, which leaves
 *  reception dead forever while everything else looks alive.  Checking two
 *  registers per tick costs ~nothing and recovers within 10 ms. */
void hil_port_rx_watchdog(void)
{
    if (((USART1->CR3 & USART_CR3_DMAR) == 0u) ||
        ((huart1.hdmarx->Instance->CR & DMA_SxCR_EN) == 0u)) {
        hil_port_rx_restart();
    }
}

/* ------------------------------------------------------------------ */
/* Millisecond counter                                                 */
/* ------------------------------------------------------------------ */

uint32_t hil_port_millis(void)
{
    return HAL_GetTick();
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

#if HIL_PORT_INIT_UART
static void uart1_init(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* PB6 = TX (AF7), PB7 = RX (AF7) -- the onboard CH340G is wired here.
     * Do NOT move this to PA9/PA10: that pair is unconnected on this board. */
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = HIL_UART_BAUD;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* USART1_TX on DMA2 stream 7, channel 4 */
    {
        static DMA_HandleTypeDef hdma_tx;
        __HAL_RCC_DMA2_CLK_ENABLE();
        hdma_tx.Instance                 = DMA2_Stream7;
        hdma_tx.Init.Channel             = DMA_CHANNEL_4;
        hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_tx.Init.Mode                = DMA_NORMAL;
        hdma_tx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_tx);
        __HAL_LINKDMA(&huart1, hdmatx, hdma_tx);
        /* NVIC priorities are assigned centrally -- see hil_port_nvic_config() */
    }

    /* USART1_RX on DMA2 stream 5, channel 4 -- circular */
    {
        static DMA_HandleTypeDef hdma_rx;
        hdma_rx.Instance                 = DMA2_Stream5;
        hdma_rx.Init.Channel             = DMA_CHANNEL_4;
        hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_rx);
        __HAL_LINKDMA(&huart1, hdmarx, hdma_rx);
        /* NVIC priorities are assigned centrally -- see hil_port_nvic_config() */
    }
}
#endif /* HIL_PORT_INIT_UART */

/* ------------------------------------------------------------------ */
/* TIM6: 100 Hz control tick                                           */
/* ------------------------------------------------------------------ */

/* APB1 timer clock = 84 MHz on a 168 MHz F407 with the standard clock tree */
#define HIL_TIM6_APB1_HZ       (84000000u)

void hil_port_tim6_start(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM6->PSC = (uint16_t)((HIL_TIM6_APB1_HZ / 1000000u) - 1u);  /* 1 MHz  */
    TIM6->ARR = 10000u - 1u;                                     /* 10 ms  */
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR  = 0u;
    TIM6->DIER = TIM_DIER_UIE;
    /* NVIC priority + enable are done in hil_port_nvic_config() */

    TIM6->CR1 |= TIM_CR1_CEN;
}

/** @brief  Call this from TIM6_DAC_IRQHandler(). */
void hil_port_tim6_isr(void)
{
    if ((TIM6->SR & TIM_SR_UIF) != 0u) {
        TIM6->SR = (uint32_t)~TIM_SR_UIF;
        HIL_Step10ms();
    }
}

/* ------------------------------------------------------------------ */
/* NVIC priorities                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief  Assign interrupt priorities.
 *
 * Called unconditionally from hil_port_init(), i.e. also when CubeMX owns
 * the peripheral set-up -- the priorities are part of the run-time contract,
 * not of the peripheral configuration.
 *
 *   group A (preempt 5): USART1, DMA2_Stream5 (RX)
 *       Above the control tick so a byte burst can never be dropped while
 *       HIL_Step10ms() is running.
 *
 *   group B (preempt 6): TIM6 (control tick), DMA2_Stream7 (TX)
 *       Deliberately the SAME preemption level.  Both of them touch the
 *       shared TX ring-buffer state (s_tx_busy / s_tx_head / s_tx_tail) and
 *       tx_kick() tests-then-sets s_tx_busy non-atomically.  Giving them
 *       equal priority means neither can interrupt the other, so those
 *       read-modify-writes are implicitly atomic without a critical section.
 *       Raising DMA2_Stream7 above TIM6 re-introduces a double DMA start.
 *
 * Requires HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4), which
 * HAL_Init() already sets; the sub-priority argument is therefore unused.
 */
static void hil_port_nvic_config(void)
{
    HAL_NVIC_SetPriority(USART1_IRQn,       5u, 0u);
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5u, 0u);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,     6u, 0u);
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6u, 0u);

    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/* ------------------------------------------------------------------ */

void hil_port_init(void)
{
#if HIL_PORT_INIT_UART
    uart1_init();
#endif
    hil_port_nvic_config();
    hil_port_rx_start();
    hil_port_tim6_start();
}
