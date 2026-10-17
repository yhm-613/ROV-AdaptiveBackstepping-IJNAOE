# STM32 目标端集成（YS-F4Pro / STM32F407IGT6）

把 `wb_hil/firmware/` 里的运行时 + `rov_ctrl_deploy_ert_rtw/` 里的生成代码
装进一个能在板子上跑的工程。工具链二选一：**Keil MDK-ARM 5** 或
**STM32CubeIDE**；两者都先用 **STM32CubeMX** 生成外设初始化骨架。

```
wb_hil/
├─ firmware/            要加进工程的运行时（协议 / 目标机 / 端口层）
├─ rov_ctrl_deploy_ert_rtw/   要加进工程的生成代码
└─ firmware/stm32_app/  ← 本目录：CubeMX 清单 + 两份参考文件
    ├─ README.md        本文件
    ├─ main.c           参考实现：CubeMX 生成后应该长什么样
    └─ stm32f4xx_it.c   参考实现：中断服务函数
```

> ⚠ **推荐路径是「CubeMX 生成 → 按 §3 改 5 处」，而不是直接拿本目录的
> `main.c` / `stm32f4xx_it.c` 覆盖。** 这两个文件是**对照用的参考**：
> 它们依赖 CubeMX 生成的 `main.h` / `stm32f4xx_hal_conf.h` / MSP 文件 /
> 启动文件，单独拿出去是编译不了的。自己从零搭工程只会多花时间。

---

## 1. 板子规格（已核实，照填即可）

来源：上海交通大学机械与动力工程学院《嵌入式系统设计》实验指导书
（该实验课用的正是 YS-F4Pro），以及硬石 YS-F4Pro 开发手册。

| 项 | 值 | 影响 |
|---|---|---|
| CPU | STM32F407IGT6 / LQFP-176 / 1 MB Flash / 192 KB RAM | CubeMX 里选 **STM32F407IGTx** |
| **HSE 晶振** | **8 MHz** 无源石英晶振（接 PH0/PH1） | §2.1 的 `Input frequency` 填 **8**。填错则 168 MHz、所有波特率、10 ms 节拍**同步偏掉** |
| LSE | 32.768 kHz（RTC 用，本项目不用） | Disable |
| 调试口 | **仅 SWD**，不支持 JTAG：PA13 = SWDIO、PA14 = SWCLK | §2.1 的 Debug 必须选 **Serial Wire**，否则下次连不上 |
| **板载 USART1** | **PB6 = USART1_TX、PB7 = USART1_RX**（AF7），板载 **CH340G** 就接在这两个脚上 | 见 §2.2，**必须手改**。不是 PA9/PA10 —— 实测证据见 §1.1 |
| **板载 LED** | **PH9、PE5、PE6**（3 个） | §2.7 的心跳灯用 **PH9**，User Label 设成 `LED` |
| 按键 | PE0–PE4（5 个） | 本项目不用 |
| 供电 | MINI USB 口供 5 V，或外部 7–36 V | — |
| 以太网 | 10/100M PHY 板载 | 本项目不用（走串口） |

**必须用 HSE，不能用 HSI。** HSI 是内部 RC，±1% 误差，460800 波特率会直接崩。

### 1.1 USART1 引脚：已实测确认，**本文档旧版写的 PA9/PA10 是错的**

上一版这里留了个待确认项，问"MINI USB 口到底是 USB-OTG 还是 CH340 转串口"。
**2026-10-12 直接在板子上读寄存器，已定论：**

| 引脚 | MODER | 复用功能 | 实测含义 |
|---|---|---|---|
| **PB6** | 2（复用） | **AF7** | **USART1_TX** ← 板载 CH340G 的 RXD 接这里 |
| **PB7** | 2（复用） | **AF7** | **USART1_RX** ← 板载 CH340G 的 TXD 接这里 |
| PA9 | 0（输入） | AF0 | **未使用** |
| PA10 | 0（输入） | AF0 | **未使用** |

**证据**：板子上电跑着硬石出厂例程，读它的 `GPIOA/GPIOB` 的 `MODER` 与 `AFR`
寄存器得到上表。同时 `USART1->CR1 = 0x0000200C`（UE|TE|RE 全开）、
`BRR = 0x02D9`，在 PCLK2 = 84 MHz 下正好是 **115200 baud** ——
即出厂例程就是在 **USART1 @ 115200** 上打日志，而它只可能走 PB6/PB7。

> 读寄存器用的命令（板子连着就能复现）：
> `pyocd commander -t cortex_m -N -c "read32 0x40020420" -c "read32 0x40020024"`
> `AFR[0]` 的 bit24-27 = PB6 复用号、bit28-31 = PB7 复用号，两者都应为 7。

**这个结论的两个直接后果：**

1. **不需要买外部 USB-TTL 模块。** 板载 CH340G 已经把 USART1 引到 MINI USB 口了，
   一根 MINI USB 线同时供电和通信。
2. **CubeMX 里绝不能沿用默认的 PA9/PA10。** 按默认生成的话代码能编译、能烧写、
   LED 能闪，但**串口一个字节都不会出来**，而且现象是"完全安静"，
   极容易被误判成"板子没跑起来"或"PC 端没发"。这是本工程最昂贵的一个坑。

---

## 1.5 IDE 怎么选（先看这条，能省半天）

**结论：装 STM32CubeIDE 的 1.x 版（推荐 1.19.0），不要装 2.x，不建议用 Keil。**

| | **STM32CubeIDE 1.x** | STM32CubeIDE 2.x | Keil MDK |
|---|---|---|---|
| 图形化配置（.ioc） | **内置**，一个软件搞定 | **已剥离**，要另装 CubeMX | 要另装 CubeMX |
| 编译器 | 自带 `arm-none-eabi-gcc` | 同左 | ARMCC（另装） |
| 调试器 | 自带 GDB + ST-Link 支持 | 同左 | 同左 |
| **代码大小限制** | **无** | 无 | ⚠ **社区版 32 KB**（详见下） |
| 额外依赖 | 无 | 可能触发 JDK 问题 | 要激活许可证 |
| 费用 | 免费 | 免费 | 免费版受限 |

**三个理由，逐个说清：**

1. **1.x vs 2.x 的分界**：ST 在 **2026 年 1 月发布的 2.0** 把 CubeMX 从 CubeIDE 里拆了出去。
   装 2.x 就得再装一个 STM32CubeMX，而 CubeMX 基于 Eclipse；**只兼容 JDK 8**，机器上若装了 JDK 17/21 会直接启动失败。
   **装 1.19.0 则一个软件全包，不碰 JDK。**

2. **Keil 有个你可能踩的坑**：免费版（MDK-Lite / 社区版未激活 AC6 时）**限制 32 KB 代码**，
   超了会报 `Error: L6050U: The code size of this image (xxxxx bytes) exceeds the maximum allowed`。
   本工程 = F4 HAL 库 + 控制器生成代码，**大概率超过 32 KB**。
   新版 MDK-Community 用 Arm Compiler 6 可以解除，但要**在线激活、许可证一年一续**——白折腾。直接上 CubeIDE 最省事。

3. **路径规则**：安装路径和工作区路径**都不要有中文、空格**。建议 `C:\ST\STM32CubeIDE`、
   工作区 `C:\ST\workspace_hil`。

下载页（需注册免费 ST 账号，邮箱验证即可）：

- CubeIDE：<https://www.st.com/en/development-tools/stm32cubeide.html>
- ST-Link 独立驱动（CubeIDE 安装时勾选即可，一般不用单独下）：<https://www.st.com/en/development-tools/st-link-v2.html>

> 下载慢的话可以用中科大镜像：<https://mirrors.ustc.edu.cn/stm32cube/>
> —— 但镜像只同步了固件包（F4 HAL 库），**IDE 安装包还是得走官网**。

安装时**务必勾选 "ST-Link Server / ST-Link driver"**。装完先别建工程，
先插上 ST-Link，到「设备管理器 → 通用串行总线设备」确认出现
**STMicroelectronics STLink**（无黄色感叹号）。这一步过了再往下走。

---

## 2. CubeMX 逐项配置

新建工程 → 选芯片 **STM32F407IGTx**（不要选板子型号，选裸芯片最干净）。

### 2.1 RCC 与时钟树

| 位置 | 项 | 值 |
|---|---|---|
| System Core → RCC | High Speed Clock (HSE) | **Crystal/Ceramic Resonator** |
| System Core → RCC | Low Speed Clock (LSE) | Disable |
| System Core → SYS | Debug | **Serial Wire**（板子只支持 SWD，务必选，否则下次连不上） |

Clock Configuration 页：

```
Input frequency : 8            (MHz)   ← YS-F4Pro 就是 8 MHz，已核实
PLL Source      : HSE
/ M             : 8            -> 1 MHz
* N             : 336          -> 336 MHz
/ P             : 2            -> SYSCLK 168 MHz
/ Q             : 7            -> 48 MHz (USB/SDIO，本项目不使用)
AHB Prescaler   : 1            -> HCLK  168 MHz
APB1 Prescaler  : 4            -> PCLK1  42 MHz
APB2 Prescaler  : 2            -> PCLK2  84 MHz
```

> 8 MHz 晶振下 CubeMX 的 `/M` 正好填 8，VCO 输入 1 MHz ✓。
> 若哪天换成 25 MHz 的板子，`/M` 改 25，其余不变。

> **APB1 定时器时钟 = 2 × 42 = 84 MHz**（预分频 ≠ 1 时自动 ×2）。
> TIM6 挂在 APB1 上，这是 `hil_port_stm32f4.c` 里
> `HIL_TIM6_APB1_HZ = 84000000` 的来源。APB1 预分频若不是 4，10 ms 就不准。

### 2.2 USART1

Connectivity → USART1 → Mode: **Asynchronous**

| Parameter Settings | 值 |
|---|---|
| Baud Rate | **460800** Bits/s |
| Word Length | 8 Bits (including Parity) |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit |
| Over Sampling | 16 Samples |
| Hardware Flow Control | None |

GPIO Settings：**PB6 = USART1_TX**、**PB7 = USART1_RX**。

> ⚠ **CubeMX 默认会把 USART1 分配到 PA9/PA10，必须手动改成 PB6/PB7。**
>
> 改法（二选一）：
>
> 1. 在芯片图上**按住 Ctrl 再点 PB6**，弹出复用功能列表 → 选 `USART1_TX`；
>    同样按住 Ctrl 点 **PB7** → 选 `USART1_RX`。此时 PA9/PA10 上的黄色标记
>    会自动消失。
> 2. 或在 Pinout 视图左上角的下拉框里选 `USART1`，再对每个信号用
>    右键 → `Pin Assignment` 指定到 PB6 / PB7。
>
> **改完务必回 `GPIO Settings` 表里核对一遍**：应显示
> `USART1_TX  PB6` 与 `USART1_RX  PB7`，且**表里不应再出现 PA9 / PA10**。
> 若还残留 PA9/PA10 行，说明没改干净 —— 这时编译烧写都正常、LED 也闪，
> 但串口全程静默，是最容易误判的一种失败。
>
> 硬件上不需要做任何事：板载 CH340G 已经固定接在 PB6/PB7，
> 你只要让固件把 USART1 复用到这两个脚即可。

### 2.3 DMA

USART1 → **DMA Settings** → Add 两次：

| 方向 | DMA 请求 | Stream / Channel | Direction | Mode | Priority | Increment Address | Data Width |
|---|---|---|---|---|---|---|---|
| TX | USART1_TX | **DMA2 Stream7 / Ch4** | Memory To Peripheral | **Normal** | High | Memory ✓ / Peripheral ✗ | Byte / Byte |
| RX | USART1_RX | **DMA2 Stream5 / Ch4** | Peripheral To Memory | **Circular** | High | Memory ✓ / Peripheral ✗ | Byte / Byte |

> RX 必须 **Circular**——`hil_port_rx_poll_idle()` 靠"DMA 当前写指针"
> 与"上次读到哪"的差值取出新字节，Normal 模式收满一次就停了。

### 2.4 TIM6

Timers → TIM6：

| 项 | 值 |
|---|---|
| Clock Source | **Internal Clock** |
| Prescaler (PSC) | **83** |
| Counter Period (ARR) | **9999** |
| auto-reload preload | Disable |
| Counter Mode | Up |

→ 84 MHz / 84 = 1 MHz；1 MHz / 10000 = **100 Hz = 10 ms** ✓

Counter Settings 下方的 **NVIC Settings → TIM6 global interrupt ✓ 勾上**。

> `hil_port_tim6_start()` 会再用寄存器把 PSC/ARR 写一遍（值相同），
> 所以这里填错不会致命——但填对能避免排查时的困惑。

### 2.5 NVIC

System Core → NVIC → NVIC Settings，4 项全部 **Enable**：

| 中断 | CubeMX 里设的抢占优先级 | 运行时实际值 | 说明 |
|---|---|---|---|
| USART1 global interrupt | 5 | 5 | |
| DMA2 stream5 global interrupt | 5 | 5 | RX |
| DMA2 stream7 global interrupt | **6** | **6** | TX，**必须与 TIM6 同级** |
| TIM6 global interrupt | **6** | **6** | |

NVIC Priority Group 用默认的 `4 bits preemption / 0 bits subpriority`（= `NVIC_PRIORITYGROUP_4`）。

> 这里的数值其实**会被 `hil_port_nvic_config()` 覆盖**（它在 `hil_port_init()`
> 里无条件执行）。勾选的意义是让 CubeMX 在 `stm32f4xx_it.c` 里把中断服务
> 函数的骨架生成出来；优先级按上表填只是为了两边一致、便于对照。
>
> 为什么 DMA2_Stream7 必须与 TIM6 同级 → 见
> `docs/HIL部署与串口通信说明.md` §10.4 的「② TX 环形缓冲区的竞态」。

### 2.6 Project Manager

| 页签 | 项 | 值 |
|---|---|---|
| Project | Toolchain/IDE | **MDK-ARM V5** 或 **STM32CubeIDE** |
| Project | Firmware Package | F4 最新包（FW_F4 V1.27.x 起） |
| Code Generator | Copy only necessary library files | ✓ |
| Code Generator | Generate peripheral initialization as a pair of .c/.h files | 可勾可不勾 |
| Code Generator | **Delete previously generated files when not re-generated** | **✗ 必须取消勾选**（否则手加进 `Core/` 的文件会被删掉） |
| Advanced Settings | 各外设的 Generate IRQ handler | 保持默认 ✓ |

点 **GENERATE CODE**。

### 2.7 GPIO：心跳灯（强烈建议，引脚已确定）

YS-F4Pro 板载 3 个 LED，分别在 **PH9、PE5、PE6**。选 **PH9**：

1. 在 CubeMX 的芯片图上点 **PH9** → 选 `GPIO_Output`
2. 到 **GPIO 页签**，找到 PH9，把 **User Label 改成 `LED`**
3. 顺手把 GPIO Output Level 设成 `Low`（低电平灭，上电不刺眼）

这样 `main.h` 里会生成 `LED_Pin` / `LED_GPIO_Port`，§3.2 里那段
`#ifdef LED_Pin` 的心跳代码就生效了。

**这个灯在上板调试时价值很高**：§5 的第 1 步（只接 ST-Link 烧点灯程序）
靠它确认工具链通了；后面链路出问题时，也能一眼区分
"板子卡在 HardFault"（灯不闪）和"串口没通"（灯照闪）。

---

## 3. 生成之后要改的 5 处

### 3.1 加入源文件

把下面 6 个 `.c` 加进工程（Keil：右键组 → Add Existing Files；
CubeIDE：拖进 `Core/Src` 或直接在工程属性里加路径）：

```
wb_hil/firmware/hil_protocol.c
wb_hil/firmware/hil_target.c
wb_hil/firmware/hil_port_stm32f4.c        ← 端口层（UART+DMA+TIM6）
wb_hil/rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy.c
wb_hil/rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy_data.c
```

**不要**加 `hil_selftest.c` 和 `hil_target_hosttest.c`——那是 PC 端的测试程序，
里面各有自己的 `main()`，会跟工程的 `main()` 冲突。

头文件搜索路径加两条：

```
wb_hil/firmware
wb_hil/rov_ctrl_deploy_ert_rtw
```

> ⚠ `hil_port_stm32f4.c` 顶部的 `HIL_PORT_INIT_UART` **保持默认的 0**。
> 改成 1 会让它自己再建一套 DMA 句柄，与 CubeMX 的
> `DMA2_Stream7_IRQHandler()` 用的句柄不是同一个对象 → 第一帧就 HardFault。
> 详见 `docs/HIL部署与串口通信说明.md` §10.4 的「① 两套 DMA 句柄」。

### 3.2 `Core/Src/main.c`

顶部加两个 include：

```c
#include "hil_target.h"
#include "hil_port.h"
```

在 `MX_USART1_UART_Init();` 之后、`while (1)` 之前插入：

```c
HIL_Init();   /* → rov_ctrl_deploy_initialize() + 启动 RX DMA 与 TIM6 100 Hz */
```

**顺序不能变**：`MX_DMA_Init()` → `MX_USART1_UART_Init()` → `HIL_Init()`。
（`HAL_UART_MspInit()` 在 `MX_USART1_UART_Init()` 里链接 DMA 句柄，
必须晚于 `MX_DMA_Init()`。）

主循环里可以加心跳（有 LED 才生效）：

```c
while (1)
{
#ifdef LED_Pin
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif
    HAL_Delay(500);
}
```

### 3.3 `Core/Src/stm32f4xx_it.c` — TIM6

找到 CubeMX 生成的 `TIM6_DAC_IRQHandler()`，**整段替换**成：

```c
void TIM6_DAC_IRQHandler(void)
{
    hil_port_tim6_isr();      /* 10 ms 控制节拍 */
}
```

> **不要**在这里调 `HAL_TIM_IRQHandler(&htim6)`。`hil_port_tim6_isr()`
> 直接读 `TIM6->SR` 并调 `HIL_Step10ms()`，不走 HAL 的回调链；
> 保留 `HAL_TIM_IRQHandler()` 会多进一层 HAL 状态机，且它自己也会清标志。

别忘了在文件顶部 `#include "hil_port.h"`。

### 3.4 `Core/Src/stm32f4xx_it.c` — USART1 IDLE

CubeMX 生成的 `USART1_IRQHandler()` 默认只有 `HAL_UART_IRQHandler(&huart1);`。
改成：

```c
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);   /* 内部会读 SR 再读 DR */
        hil_port_rx_poll_idle();              /* 把 DMA 新收到的字节喂给解析器 */
    }
    HAL_UART_IRQHandler(&huart1);
}
```

`hil_port_rx_start()` 里已经调了 `__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE)`，
所以不需要在 CubeMX 里额外配置。

### 3.5 `DMA2_Stream5/7_IRQHandler` 保持原样

```c
void DMA2_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_rx); }
void DMA2_Stream7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
```

这个**不要改**——保持 CubeMX 生成的样子，句柄就是 §3.1 里说的那套唯一的句柄。

可选：把 CubeMX 的 `HardFault_Handler()` 里那个 `while (1)` 外面加个
断点或点灯，上板时能一眼区分"卡在 HardFault"和"串口没通"。

---

## 4. 编译与链接选项

| 项 | 值 | 说明 |
|---|---|---|
| Optimization | `-O2` | |
| **FPU** | `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` | 让 `sinf` / `cosf` 走硬件 |
| C 标准 | C99 或以上 | `hil_protocol.h` 里用了 for 循环内声明 |
| **Use MicroLIB** | ✓（Keil） | 否则要确保 `memcpy` / `memset` 有实现 |
| Defines | `USE_HAL_DRIVER`、`STM32F407xx` | CubeMX 自动加 |

**关于 `real_T`**：本项目生成代码里 `real_T = double`，
Cortex-M4F 没有硬件双精度 FPU，`rov_ctrl_deploy_step()` 会走软件浮点，
单周期估算 1–3 ms（10 ms 周期的 10–30%），余量够。
想榨性能就按 `docs/HIL部署与串口通信说明.md` §5.5 把模型改成单精度模型。
**但在改之前先按现在的配置把链路跑通**——保持 double 时 PiL 验证结论是
"float32 层级 0 个元素不相等"，这是能直接写进论文的数字。

---

## 5. 首次上电验证顺序

**不要一上来就跑闭环。** 按下面 4 步走，每步都能把故障范围砍一半。

| 步 | 做什么 | 期望 | 不对的话 |
|---|---|---|---|
| 1 | **只接 ST-Link**（串口线不接），烧一个点灯/心跳程序 | LED 闪，或调试器能停下 | 工具链问题，先不管串口 |
| 2 | 烧**本工程**，串口助手接上，**不发任何数据** | **持续不断的数据流：每 10 ms 一帧 34 B，即 3400 B/s**（这就是心跳） | 完全没有数据 → 查 §3.3/§3.4 两处改动、或 USART1 引脚；有乱码 → 波特率或时钟不对 |
| 3 | 串口助手**手工发一帧 104 B 的 setpoint** | 回帧的第 29–30 字节开始跟着你的 seq 走，且 status 出现 `DATA_FRESH` | 无回应 → 查 §3.3/§3.4 两处改动 |
| 4 | 开 `rov_hil_host.slx`（`StopTime` 先给 2 s） | `link = 0`，u 有输出 | 看 `hil_bridge_host.m` 的 COM 口和波特率 |

> ⚠ **更正（2026-10-12，读源码确认）**：本文档旧版写"板子是被动方，没收到
> setpoint 就不回帧"——**这是错的**。`HIL_Step10ms()` 在
> `hil_target.c` 里是**无条件**组帧发送的：TIM6 每 10 ms 进一次中断，就发一帧
> 34 B，和有没有收到 setpoint 无关（没收到只是把 `DATA_FRESH` 清掉、输入保持上一拍）。
>
> 所以第 2 步的正确定判据是**应该看到 3400 B/s 的连续流**，而不是"安静"。
> 这反而更好——不需要 PC 发任何东西，就能一次验完
> 「TIM6 时钟 → 控制任务 → USART1 → DMA → CH340 → PC」整条链，
> 而且 100 Hz 的周期本身就是 TIM6/APB1 时钟树对不对的直接证据（每帧间隔应为 10.0 ms）。

---

## 6. 故障速查

| 现象 | 大概率原因 | 处理 |
|---|---|---|
| **串口一片安静，分不清是"链路坏"还是"固件没写对"** | 出厂例程本来就不主动发数据，被动监听分辨不出来 | 跑 **`tools/swd_link_test/`** 那套免固件验证（用 SWD 直接驱动 USART1 收发），先把"硬件链路"这个变量彻底排除掉 |
| **编译烧写都正常、LED 也闪，但串口全程静默**（连乱码都没有） | **USART1 被配在 PA9/PA10**（CubeMX 默认），而板载 CH340G 接在 **PB6/PB7** | 按 §1.1 / §2.2 改成 PB6/PB7。注意这个故障会伪装成"正常安静"，见 §5 第 2 步的说明 |
| 一上电就不响应，调试器停在 `HAL_DMA_IRQHandler` | `HIL_PORT_INIT_UART` 被改成 1，两套 DMA 句柄 | 改回 0（§3.1） |
| `TIMEOUT` 位常置 | PC 没发帧 / 波特率不符 / TX-RX 接反 | 交叉接线、共地、两边都设 460800 |
| 大量 `RX_CRC_ERR` | 波特率漂移、线太长、未共地 | 先共地；杜邦线缩短；降速到 230400 试 |
| 偶发 `TX_FAIL`，`tx_cnt` 落后于 `tick_cnt` | DMA2_Stream7 优先级被设得高于 TIM6 | 保持同级 6（§2.5） |
| 收到帧但 u 全 0 | TIM6 中断没跑 | 检查 §3.3 是否替换成功；`g_hil_tick_cnt` 是否在涨 |
| PC 侧仿真极慢 | `hil_bridge_host.m` 是阻塞式往返 | 正常现象，`StopTime` 设小 |
| 编译报 `undefined reference to 'HIL_Init'` | `hil_target.c` 没加进工程 | 按 §3.1 加文件 |
| 编译报重复定义 `main` | 误加了 `hil_selftest.c` / `hil_target_hosttest.c` | 从工程里移除 |

---

## 7. 与本目录两份参考文件的关系

| 文件 | 用途 |
|---|---|
| `main.c` | CubeMX 生成并完成 §3.2 改动后，`main.c` 应该长什么样。**对照用**，不要整体覆盖 |
| `stm32f4xx_it.c` | 完成 §3.3 / §3.4 后应该长什么样。**对照用** |

MSP 文件（`stm32f4xx_hal_msp.c`）本目录**不提供**——它必须由 CubeMX 生成，
因为里面 `HAL_UART_MspInit()` 负责建立 `huart1` 与两个 DMA 句柄的链接关系，
手写容易和生成版本冲突。
