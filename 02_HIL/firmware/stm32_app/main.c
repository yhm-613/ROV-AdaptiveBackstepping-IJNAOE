/**
 * @file    main.c
 * @brief   REFERENCE implementation for the ROV HIL target (YS-F4Pro).
 *
 * ---------------------------------------------------------------------------
 *  This is a REFERENCE file, not a drop-in replacement.
 *
 *  It depends on artifacts that STM32CubeMX generates for your specific
 *  project: main.h, stm32f4xx_hal_conf.h, stm32f4xx_hal_msp.c (which contains
 *  HAL_UART_MspInit / HAL_TIM_Base_MspInit), the startup file and the linker
 *  script.  Copying this file alone into a fresh folder will not build.
 *
 *  The supported workflow is:
 *      1. configure the chip in CubeMX following firmware/stm32_app/README.md
 *      2. generate code
 *      3. apply the five edits listed in README section 3
 *      4. compare against this file if something looks off
 * ---------------------------------------------------------------------------
 *
 *  Interrupt priorities are NOT set here.  hil_port_nvic_config(), called from
 *  HIL_Init() -> hil_port_init(), owns them, because they are part of the
 *  run-time contract rather than of the peripheral configuration.
 *
 *  Generated module layout expected (STM32CubeIDE / MDK-ARM):
 *      Core/Inc/main.h
 *      Core/Inc/stm32f4xx_hal_conf.h
 *      Core/Src/main.c              <- this file
 *      Core/Src/stm32f4xx_it.c
 *      Core/Src/stm32f4xx_hal_msp.c
 *      Core/Src/stm32f4xx_hal_timebase_tim.c   (optional)
 */

/* USER CODE BEGIN Includes */
#include "hil_target.h"
#include "hil_port.h"
/* USER CODE END Includes */

#include "main.h"

/* ------------------------------------------------------------------ */
/* Handle definitions (CubeMX puts these here)                         */
/* ------------------------------------------------------------------ */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_usart1_tx;
TIM_HandleTypeDef  htim6;

/* ------------------------------------------------------------------ */
/* Prototypes                                                          */
/* ------------------------------------------------------------------ */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);

/* ------------------------------------------------------------------ */

int main(void)
{
    /* MCU Configuration ------------------------------------------------- */

    HAL_Init();

    /* 168 MHz: HSE -> PLL -> SYSCLK, APB1 = 42 MHz, APB2 = 84 MHz.
     * TIM6 hangs off APB1, whose *timer* clock is 2 x 42 = 84 MHz.
     * If your board has an 8 MHz crystal instead of 25 MHz, only PLLM
     * changes (8 instead of 25); PLLN/PLLP stay 336/2 so SYSCLK is still
     * 168 MHz.  CubeMX fills this in for you.                            */
    SystemClock_Config();

    /* Initialize all configured peripherals ---------------------------- */

    MX_GPIO_Init();
    MX_DMA_Init();            /* MUST precede MX_USART1_UART_Init():
                                 HAL_UART_MspInit() links huart1 to the two
                                 DMA handles and therefore needs the DMA
                                 controller clock already enabled.        */
    MX_USART1_UART_Init();    /* owns huart1 + hdma_usart1_tx/rx.
                                 hil_port_stm32f4.c keeps
                                 HIL_PORT_INIT_UART = 0 so that this single
                                 set of handles is the only one in the
                                 image.                                    */
    MX_TIM6_Init();           /* only configures PSC/ARR and the NVIC bit;
                                 hil_port_tim6_start() re-writes PSC/ARR with
                                 the same values and starts the counter.   */

    /* ---- HIL run-time ------------------------------------------------ *
     * HIL_Init() does three things, in this order:
     *   1. rov_ctrl_deploy_initialize()   generated controller state
     *   2. hil_port_init()
     *        -> hil_port_nvic_config()    interrupt priorities
     *        -> hil_port_rx_start()       UART RX DMA (circular) + IDLE IRQ
     *        -> hil_port_tim6_start()     100 Hz tick, 10 ms
     *   3. seeds the controller inputs with zeros
     *
     * Nothing else has to be polled: every byte arrives through the IDLE
     * interrupt and every control step runs in the TIM6 interrupt.        */
    HIL_Init();

    /* Infinite loop ---------------------------------------------------- */

    while (1)
    {
        /* Heartbeat only.  If the LED is dark and the serial port is silent,
         * check whether the debugger has stopped inside HardFault_Handler()
         * -- that is the signature of the classic DMA-handle mix-up
         * described in HIL_PORT_INIT_UART's comment block.                  */
#ifdef LED_Pin
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif
        HAL_Delay(500);
    }
}

/* ------------------------------------------------------------------ */
/* System Clock Configuration                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief  System Clock Configuration
 *         SYSCLK 168 MHz, HCLK 168 MHz, PCLK1 42 MHz, PCLK2 84 MHz
 *
 * @note   HSE_VALUE in stm32f4xx_hal_conf.h must match the crystal on the
 *         board (8000000 or 25000000).  CubeMX sets it from the value you
 *         enter on the Clock Configuration page.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Supply voltage scaling: 168 MHz requires Scale 1 (max 168 MHz) */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* HSE on, PLL on.  PLLM = 25 for a 25 MHz crystal, 8 for an 8 MHz one */
    RCC_OscInitStruct.OscillatorType       = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState             = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState         = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource        = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM             = 25u;   /* -> 1 MHz  */
    RCC_OscInitStruct.PLL.PLLN             = 336u;  /* -> 336 MHz */
    RCC_OscInitStruct.PLL.PLLP             = RCC_PLLP_DIV2;  /* -> 168 MHz */
    RCC_OscInitStruct.PLL.PLLQ             = 7u;    /* -> 48 MHz (unused) */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 168 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;     /* PCLK1 =  42 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;     /* PCLK2 =  84 MHz */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ------------------------------------------------------------------ */
/* Peripherals                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief DMA controller clock + interrupt enables.
 *
 * The priorities written here are the intended layout; hil_port_nvic_config()
 * re-applies exactly these values at run time, so the two can never drift.
 *
 *   preempt 5 : USART1, DMA2_Stream5 (RX)   -- above the control tick
 *   preempt 6 : TIM6,     DMA2_Stream7 (TX) -- MUST be equal, see README 2.5
 */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5u, 0u);   /* USART1_RX */
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6u, 0u);   /* USART1_TX */
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/**
 * @brief USART1 in asynchronous mode at 460800 8N1, no flow control.
 *
 * 138 bytes are exchanged every 10 ms = 13.8 kB/s.  8N1 needs 10 bits per
 * byte, so the wire rate is 138000 bit/s: 115200 would not fit, 230400 is
 * the bare minimum, 460800 gives ~3.3x headroom.
 *
 * The DMA channels themselves are configured by HAL_UART_MspInit() in
 * stm32f4xx_hal_msp.c, which CubeMX generates.
 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 460800u;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM6 as the 100 Hz control tick.
 *
 * APB1 timer clock is 84 MHz (PCLK1 is 42 MHz, doubled because the APB
 * prescaler is not 1).
 *      84 MHz / (83 + 1) = 1 MHz
 *       1 MHz / (9999 + 1) = 100 Hz  ->  10 ms
 *
 * hil_port_tim6_start() writes the same PSC/ARR directly to the registers
 * and then sets CEN, so this function exists mainly to let CubeMX emit the
 * NVIC enable and the HAL_TIM_Base_MspInit() clock setup.
 */
static void MX_TIM6_Init(void)
{
    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 83u;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 9999u;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO — only the optional status LED.
 *
 * If you did not label an LED pin in CubeMX, this collapses to enabling the
 * GPIO clocks and nothing else, which is harmless.
 */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
}

/* ------------------------------------------------------------------ */

/**
 * @brief  Retargeted by CubeMX to a breakpoint / infinite loop.
 *         Not to be confused with the HIL timeouts, which never trap --
 *         they only raise the HIL_ST_TIMEOUT status bit.
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
