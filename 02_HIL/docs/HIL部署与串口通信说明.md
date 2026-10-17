# ROV 自适应反步控制器 —— 半实物仿真（HIL）部署说明

> 目标硬件：**YS-F4Pro（STM32F407IGT6，Cortex-M4F @168 MHz，硬件单精度 FPU）**
> 控制周期：**T_s = 10 ms（100 Hz）**
> 通信：**UART 串口，460800 bps（最低 230400，推荐 921600）**
> 架构形态：**L2 快速控制原型（RCP）—— 控制器跑真实单片机，被控对象跑 PC**

---

## 1. 总体架构

```
┌──────────────────────── PC（Simulink） ────────────────────────┐
│                                                                │
│   轨迹发生器  →  ηd ─┐                                          │
│                       ├→ e = ηd − η                             │
│   ROV 六自由度模型 ◄──┘                                          │
│      η , ν  ─────────────────────────────┐                      │
│                                          │                      │
│   Inverse T ←── u (6路) ◄── [串口接收] ◄─┼── HIL Bridge          │
│   T (6×6)  → 饱和 ±100V                  │   写 104B / 读 34B   │
└──────────────────────────────────────────┼──────────────────────┘
                                           │  USB-TTL
                              ╔════════════▼════════════╗
                              ║  YS-F4Pro / STM32F407    ║
                              ║  ┌────────────────────┐  ║
                              ║  │ TIM6 10ms 中断     │  ║
                              ║  │  ├ 取最新 setpoint │  ║
                              ║  │  ├ 自适应反步 step │  ║
                              ║  │  ├ 饱和 ±100V      │  ║
                              ║  │  └ 回发 34B        │  ║
                              ║  └────────────────────┘  ║
                              ║  USART1 + DMA 双向       ║
                              ╚═════════════════════════╝
```

关键设计点：**PC 端每一步仿真与单片机每一个控制周期严格 1:1 对应**。
HIL Bridge 在发送 setpoint 后会**阻塞等待**回帧，因此"一次 Simulink 步进"= "一次单片机 10 ms 中断"，
仿真时间与控制器内部离散状态始终同步，PC 快慢不影响闭环一致性。

---

## 2. 文件清单

| 路径 | 说明 |
|---|---|
| `rov_ctrl_deploy.slx` | **板上部署控制器模型**（离散化 + 去观测 + 参数外置） |
| `rov_ctrl_deploy_ert_rtw/` | Embedded Coder 生成的 C 源码 |
| `rov_hil_loopback.slx` | PC 端回环验证模型（连续/离散控制器二选一，无串口） |
| `rov_hil_host.slx` | PC 端串口 HIL 主机模型 |
| `firmware/hil_protocol.h/.c` | 串口协议（打包/解包/CRC/增量解析），与硬件无关 |
| `firmware/hil_selftest.c` | 协议层自检（32 项断言），PC 上直接跑 |
| `firmware/hil_target.h/.c` | 目标端运行时（100 Hz 控制中断 + 帧处理） |
| `firmware/hil_port.h` | 硬件抽象层（4 个必做钩子 + 4 个中断钩子） |
| `firmware/hil_port_stm32f4.c` | YS-F4Pro 的 HAL 实现（UART1 + DMA + TIM6） |
| `firmware/hil_target_hosttest.c` | PC 上整链回放测试（读 CSV 喂生成代码），PiL 用 |
| `firmware/hil_target_sim.c` | **串口版虚拟目标**：PC 上扮演 STM32，配虚拟串口对使用 |
| `firmware/stm32_app/` | **CubeMX 逐项清单 + `main.c` / `stm32f4xx_it.c` 参考实现** |
| `pc/hil_pack_setpoint.m` 等 | PC 侧 MATLAB 参考实现 |
| `pc/hil_bridge_host.m` | PC 侧阻塞式帧同步收发 |
| `scripts/build_*.m` | 模型构建 / 代码生成脚本 |
| `docs/虚拟串口对测试.md` | 无板子时把串口链路跑通的完整步骤 |
| `out/*.log` | 每一步的构建日志（排错依据） |

---

## 3. 部署模型的接口

`rov_ctrl_deploy.slx` 已把原模型的连续部分全部离散化，并剔除了所有观测块。

### 输入（4 个，各 6×1）

| 端口 | 名称 | 含义 | 单位 |
|---|---|---|---|
| 1 | `eta` | 当前位姿 η = [x y z φ θ ψ] | m, rad |
| 2 | `nu` | 当前体坐标系速度 ν = [u v w p q r] | m/s, rad/s |
| 3 | `eta_d` | 期望位姿 ηd | m, rad |
| 4 | `nu_d` | **期望速度 η̇d（解析给定）** | m/s, rad/s |

> ⚠ 端口 4 `nu_d` 是本次调试新增的**必需输入**，不是可选量。原因见 §10.2：
> 原模型靠连续 `Derivative` 块对 ηd 数值微分得到 η̇d，离散成差分器后会把参考里
> 的阶跃跳变放大 `1/T_s` 倍，导致闭环发散，且**与 T_s 取值无关**。

### 输出（1 个，6×1）

| 端口 | 名称 | 含义 |
|---|---|---|
| 1 | `u` | 6 路推进器电压指令（已含 `pinv(T)`、`T`、±100 V 饱和） |

### 板内自算的量（不需要 PC 下发）

| 量 | 来源子系统 | 说明 |
|---|---|---|
| `J(η)` | `Kinematics` | Euler 变换矩阵，由 η 实时计算 |
| `Crb` / `Cav` | `Coriolis` | 刚体 / 附加质量科氏力矩阵，由 ν 实时计算 |
| 常数矩阵 | 代码生成时内联 | `M`、`Ma`、`DL`、`T`、`pinv(T)`、`I`、`m` |
| `d(α₁)/dt` | 控制器内部 `Derivative1` | 对**计算得到**的虚拟控制 α₁ 求导，必须保留（实测关掉会失稳） |

### 与原模型的三处必要改造

| 项 | 原模型 | 部署模型 | 原因 |
|---|---|---|---|
| 状态积分器 `c1_hat` / `c2_hat` | 连续 `Integrator` | `DiscreteIntegrator`（前向 Euler，T_s=0.01） | 代码生成只支持定步长离散 |
| 微分器 `Derivative1`（对 α₁） | 连续 `Derivative` | `Discrete Derivative`（K=1，T_s=0.01） | 同上 |
| 微分器 `Derivative`（对 ηd） | 连续 `Derivative` | **删除**，改由端口 4 `nu_d` 显式输入 | 见 §10.2，数值差分器会毁掉闭环 |
| 观测块 | 6×Scope + 2×To Workspace | 全部删除 | 上板无意义，且拖慢代码 |

> 注意：原模型的 `MaxStep = 1e-3` 是 **ode45 求解步长**，不是控制周期。
> `T_s = 10 ms` 是本次部署**新引入的设计参数**，论文中如有需要应单列说明。

---

## 4. 串口协议规范

### 4.1 帧格式（小端序）

```
偏移  长度  字段
----  ----  --------------------------------------------------
   0     1  SYNC0 = 0x55
   1     1  SYNC1 = 0xAA
   2     1  TYPE  (0x01 = SETPOINT，0x02 = COMMAND)
   3     1  LEN   payload 字节数
   4   LEN  PAYLOAD
 4+LEN   2  CRC16-Modbus（覆盖 SYNC0 到 payload 最后一字节）
```

### 4.2 SETPOINT 帧（PC → MCU，104 B）

| 偏移 | 长度 | 字段 | 类型 |
|---|---|---|---|
| 4 | 24 | `eta[6]` | float32 |
| 28 | 24 | `nu[6]` | float32 |
| 52 | 24 | `eta_d[6]` | float32 |
| 76 | 24 | `nu_d[6]` | float32，**期望速度 η̇d（解析给定，必需）** |
| 100 | 2 | `seq` | uint16 递增序号 |

### 4.3 COMMAND 帧（MCU → PC，34 B）

| 偏移 | 长度 | 字段 | 类型 |
|---|---|---|---|
| 4 | 24 | `u[6]` | float32，推进器电压 |
| 28 | 2 | `seq` | uint16，回显被消费的 setpoint 序号 |
| 30 | 2 | `status` | uint16 状态位 |

### 4.4 status 位定义

| 位 | 名称 | 含义 |
|---|---|---|
| 0 | `INIT_DONE` | 控制器初始化完成 |
| 1 | `DATA_FRESH` | 本周期收到新 setpoint |
| 2 | `TIMEOUT` | setpoint 超过 50 ms 未更新（连续丢 5 帧） |
| 3 | `RX_CRC_ERR` | 发生过 CRC 错误 |
| 4 | `RX_OVERFLOW` | 发生过组帧错误/缓冲溢出 |
| 5 | `SATURATED` | 至少一路 `|u|` 触及 100 V |
| 6 | `TX_FAIL` | 发送缓冲满 |

### 4.5 时序与容错

- **主机侧**：发 setpoint → 阻塞等 command（超时 50 ms）→ 把 u 交给被控对象 → 进入下一步
- **目标侧**：TIM6 每 10 ms 触发一次；若期间收到多个 setpoint，只保留最新的（`RX_DROP` 计数）
- **重同步**：解析器逐字节状态机，任何畸形序列都回到重新寻找 `0x55 0xAA`，单帧损坏最多损失一个周期
- **CRC**：CRC16-Modbus，多项式 0x8005（反射 0xA001），初值 0xFFFF
  校验值 `crc16("123456789") = 0x4B37`（与自检程序一致）

### 4.6 带宽核算

| 项 | 数值 |
|---|---|
| 每周期字节数 | 104 + 34 = **138 B** |
| 100 Hz 所需吞吐 | **13.8 kB/s** |
| 8N1 所需波特率 | **138 000 bps** |
| 115200 bps | ❌ 不够（0.83×） |
| **230400 bps** | ✅ 最低可用（占用 60%） |
| **460800 bps** | ✅ **推荐**（占用 30%） |
| 921600 bps | ✅ 余量充足（占用 15%） |

---

## 5. STM32 侧集成步骤

### 5.1 CubeMX 配置

| 外设 | 配置 |
|---|---|
| USART1 | Asynchronous，**460800**，8N1，无流控；**PB6=TX，PB7=RX（AF7）** ⚠ 见下注 |
| DMA | USART1_TX → DMA2 Stream7 Ch4（Normal，Memory→Periph，字节对齐）<br>USART1_RX → DMA2 Stream5 Ch4（**Circular**，Periph→Memory） |
| NVIC | USART1 中断使能；DMA2_Stream5/7 中断使能；TIM6 中断使能 |
| TIM6 | Prescaler = 83（84 MHz → 1 MHz），Counter Period = 9999（**10 ms**） |
| 时钟 | HSE + PLL → SYSCLK 168 MHz（保证 APB1 定时器时钟 84 MHz） |

> ⚠ **USART1 必须手动改到 PB6/PB7。** CubeMX 默认分配到 PA9/PA10，而 YS-F4Pro
> 板载 CH340G 是接在 **PB6/PB7** 上的（2026-10-12 读板子上出厂固件的
> `GPIOB->MODER/AFR` 实测确认：PB6/PB7 = AF7，PA9/PA10 = 普通输入）。
> 照默认配的后果是"能编译、能烧写、LED 会闪，但串口全程静默"。
> 完整证据与 CubeMX 改法见 `../firmware/stm32_app/README.md` §1.1 / §2.2。

### 5.2 加入的文件

```
Core/Inc/       hil_protocol.h  hil_target.h  hil_port.h
Core/Src/       hil_protocol.c  hil_target.c  hil_port_stm32f4.c
                rov_ctrl_deploy.c  rov_ctrl_deploy_data.c   ← 代码生成产物
```

**不要**加入 `hil_selftest.c` 和 `hil_target_hosttest.c`——它们各有自己的
`main()`，是给 PC 用的测试程序，加进工程会与 `Core/Src/main.c` 冲突。

头文件搜索路径加 `wb_hil/firmware` 与 `wb_hil/rov_ctrl_deploy_ert_rtw`。

编译选项：`-O2 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`，C99 或以上，
并**勾选 Use MicroLIB**（或提供 `memcpy` 实现）。

> **CubeMX 逐项勾选表 + 生成后要改的 5 处 + 首次上电顺序**，全部在
> **`firmware/stm32_app/README.md`** 里，配两份对照用的参考实现
> （`stm32_app/main.c`、`stm32_app/stm32f4xx_it.c`）。

### 5.3 调用点

> **⚠ 只能有一套外设初始化。** `hil_port_stm32f4.c` 顶部的
> `HIL_PORT_INIT_UART` 默认是 **0**，意思是"USART1 与它的两个 DMA 句柄
> 由 CubeMX 生成"。**不要**同时把它改成 1——那样 `uart1_init()` 会建出
> 第二套 `DMA_HandleTypeDef`，而 CubeMX 的 `stm32f4xx_it.c` 里的
> `DMA2_Stream7_IRQHandler()` 调的是它自己的全局句柄（从未初始化，
> `Instance == NULL`），`HAL_DMA_IRQHandler()` 第一行解引用就
> **HardFault**，第一帧都发不出去。详见 §10.4。

```c
/* main.c ---------------------------------------------------------- */
#include "hil_target.h"
#include "hil_port.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();           /* 必须早于 MX_USART1_UART_Init：
                                HAL_UART_MspInit 会在里面链接 hdma_* */
    MX_USART1_UART_Init();   /* 由 CubeMX 生成，拥有 huart1 与两个 DMA 句柄 */

    HIL_Init();              /* → rov_ctrl_deploy_initialize()
                                → hil_port_init()
                                    ├─ hil_port_nvic_config()   设中断优先级
                                    ├─ hil_port_rx_start()      启动 RX DMA + IDLE 中断
                                    └─ hil_port_tim6_start()    启动 100 Hz 节拍 */

    for (;;) { /* 主循环空转，全部工作在中断里 */ }
}

/* stm32f4xx_it.c -------------------------------------------------- */
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        hil_port_rx_poll_idle();      /* 把 DMA 收到的字节喂给解析器 */
    }
    HAL_UART_IRQHandler(&huart1);
}

void TIM6_DAC_IRQHandler(void)
{
    hil_port_tim6_isr();              /* 10 ms 控制节拍，注意这里「不」调
                                         HAL_TIM_IRQHandler(&htim6) */
}

/* DMA2_Stream5_IRQHandler / DMA2_Stream7_IRQHandler 保持 CubeMX 生成的：
   HAL_DMA_IRQHandler(&hdma_usart1_rx / &hdma_usart1_tx)，不要手改。 */
```

上面两份文件的完整可拷贝版本在 **`firmware/stm32_app/`**（`main.c` /
`stm32f4xx_it.c` / `stm32f4xx_hal_msp.c`），以及 **`firmware/stm32_app/README.md`**
里的 CubeMX 逐项勾选表。

### 5.4 中断优先级

优先级**不由 CubeMX 决定**，而是 `hil_port_nvic_config()` 在 `hil_port_init()`
里无条件设置的——因为它是运行期契约的一部分（见 `hil_port_stm32f4.c` 注释）。
CubeMX 里只要勾上 NVIC 使能即可，优先级填什么都会被覆盖。

| 中断 | 抢占优先级 | 分组 | 说明 |
|---|---|---|---|
| USART1 | 5 | A | 高于控制节拍，保证一串字节到达时能被及时取走 |
| DMA2_Stream5（RX） | 5 | A | 同上 |
| TIM6（控制节拍） | 6 | B | 10 ms 抖动要求 < 50 µs |
| DMA2_Stream7（TX） | **6** | B | **必须与 TIM6 同级**，理由见下 |

> **为什么 DMA TX 必须和 TIM6 同级（而不是更高）**
>
> `hil_port_uart_write()`（跑在 TIM6 中断里）与 `HAL_UART_TxCpltCallback()`
> （跑在 DMA2_Stream7 中断里）**都会碰同一组 TX 环形缓冲区状态**
> （`s_tx_busy` / `s_tx_head` / `s_tx_tail`），而 `tx_kick()` 里
> `if (s_tx_busy != 0u) return; s_tx_busy = 1u;` 是"先判断后写"的非原子操作。
>
> 只要两者抢占优先级相同，它们就无法互相打断，这组读改写天然互斥，
> 不需要临界区。**如果按直觉把 DMA TX 提到 5（高于 TIM6），
> DMA 中断就能在 TIM6 中断刚通过 `s_tx_busy` 判断、还没写入的瞬间插进来，
> 两边各自启动一次 DMA 传输 → TX 缓冲区错乱。**

### 5.5 算力核算与浮点精度

单周期约 2–4 k 次浮点运算 + 12 次三角函数（`sin`/`cos`）。

**当前生成代码使用 `real_T = double`**（原因见下方"为什么不是 float"），
Cortex-M4F 没有硬件双精度 FPU，会走软件浮点，实测经验倍数约 8–15×。
即便如此，单周期估算仍在 **1–3 ms**，即 10 ms 周期的 **10%–30%**，余量充足。

如果要榨干性能，可以把模型改成真正的单精度模型，切换后
`rtwtypes.h` 里会变成 `typedef float real_T;`，走 M4F 硬件 FPU，单周期降到 30–60 µs（<1%）。

**为什么不是 float**

Embedded Coder 里 `real_T` 跟随**模型信号精度**，而不是硬件位宽：
只有"单精度模型"（所有信号都是 single）才会生成 `real32_T`。
本模型的常数矩阵（`M`、`Ma`、`DL`、`pinv(T)`、`T`）都是 double，
再往下所有运算都会被提升为 double，整链回落成 `real_T`。

已尝试并确认**无效**的配置项：

| 配置项 | 结果 |
|---|---|
| `ProdHWDeviceType = ARM Compatible->ARM Cortex-M` | ❌ `real_T` 仍为 double |
| `TargetHWDeviceType` 同步改为 Cortex-M | ❌ 同上 |
| `DefaultUnderspecifiedDataType = 'single'` | ❌ 同上 |
| 脚本批量把各 Constant/Gain 改成 single | ❌ 嵌套子系统内常数块批量改写会引发类型冲突，Update Diagram 失败 |

**正确的做法**（在 Simulink 里手工做，一次到位）：

1. Configuration Parameters → Math and Data Types → **Default for underspecified data type = single**
2. 把所有 `Constant` 块（`DL` / `MA` / `MRB - open` / `I` / `m` / 各 `Constant1..5`）
   的 **Output data type 改成 `single`**
3. 把所有 `Gain` 块（`InvT` / `Tmat` / `Rg*` 等）的
   **Output data type 设为 `single`**、**Parameter data type 设为 `Inherit: Same as output`**
4. 根 Inport（`eta`/`nu`/`eta_d`）、根 Outport（`u`）、`Saturate`、`DiscreteIntegrator`
   的 **Output data type 也设为 `single`**
5. 编译一次，确认没有 "Data type mismatch"；再生成代码，
   `grep "typedef.*real_T" rtwtypes.h` 应显示 `typedef float real_T;`

> `scripts/build_deploy.m` 里有一个 `FORCE_SINGLE` 开关实现了第 4 步的自动化，
> 但第 2、3 步的嵌套常数块必须在 GUI 里改（脚本批量改会报类型冲突）。


---

## 6. PC 侧使用步骤

### 6.1 无开发板时：离线验证

**① 模型与数值层**

```matlab
cd E:\yanjiusheng\wb_hil\scripts
build_pc_models            % 生成 rov_hil_loopback.slx / rov_hil_host.slx
run_discretization_check   % 4 组对照仿真，量化离散化误差
```

**② 串口链路层（仍然不需要板子）—— 建议在买板子之前就做完**

com0com 造一对虚拟串口，`hil_target_sim.exe` 在 PC 上扮演目标机。它链接的是
**和板子同一份** `hil_target.c` + 生成的控制器，只把 `hil_port_*` 换成
Win32 串口调用：

```bash
cd E:/yanjiusheng/wb_hil/firmware
./hil_target_sim.exe --selftest        # 内存自检 28 项，完全不碰串口
./hil_target_sim.exe COM11 460800      # 起虚拟目标；Simulink 侧连 COM10
```

这一层验的是**协议对不对**：帧长、CRC、逐字节重组、序号回显、超时策略、
Simulink 侧阻塞式收发。跑通之后 §6.2 的真实 HIL 只需把 `HIL_PORT` 换成
**板载 CH340G 枚举出的那个 COM 口**（不是外接 USB-TTL），
**Simulink 侧一行代码都不用改**。

> 它验不了**够不够快**：虚拟串口对默认不做波特率仿真，数据立即透传，
> 所以 460800 能否扛住 138 B/周期、10 ms 抖动多大，都必须上板实测。

完整步骤（com0com 安装、口名改 COM10/COM11、预期输出、排查表）见
**`docs/虚拟串口对测试.md`**。

### 6.2 有开发板时：真实 HIL

```matlab
% 1. 改端口号
edit E:\yanjiusheng\wb_hil\pc\hil_bridge_host.m   % HIL_PORT = 'COMx'

% 2. 查看可用串口
serialportlist("available")

% 3. 打开模型
open_system('E:\yanjiusheng\wb_hil\rov_hil_host.slx')

% 4. 设置求解器：定步长 ode4，步长 0.01
set_param('rov_hil_host','SolverType','Fixed-step','Solver','ode4', ...
          'FixedStep','0.01','StopTime','30');
sim('rov_hil_host');
```

### 6.3 三档对照：把「数值误差」和「离散化误差」拆开

脚本 `scripts/run_discretization_check.m` 跑三档，**全部采用共位步长**
（求解步长 = 控制周期 = 10 ms）：

| 场景 | 求解器 | 控制器 | 用途 |
|---|---|---|---|
| `A_ode45_cont` | ode45 变步长，MaxStep 1e-3 | `Backstepping Controller`（连续） | **论文基线** |
| `C_ode4_10m_cont` | ode4 定步长 **10 ms** | 连续 | 隔离「定步长 + 10 ms 粗积分」的误差 |
| `D_ode4_10m_disc` | ode4 定步长 **10 ms** | `Controller_D`（10 ms 离散） | **半实物仿真的真实配置** |

于是两个差值各自对应一个误差来源：

```
A --(定步长 + 10 ms 粗步长积分误差)--> C --(控制器 10 ms 离散化误差)--> D
```

**为什么不跑「ode4 1e-3 + 离散控制器」**：该配置下 `Controller_D` 的连续输入
（来自 `Mux` / `Sum2` / `Integrator`）与内部 0.01 s 离散块速率不匹配，Simulink 会报
"数据完整性"错误；在反馈回路里插 `Zero-Order Hold` 也解决不了（`Sum` 属直通块，
ZOH 无法切断代数环）。既然 C 档已证明 10 ms ode4 对被控对象积分足够（见下表），
就统一采用共位步长——**既避免速率混合，又正好就是 HIL 的真实配置**。

实测结果（`out/discretization_check.log`，StopTime 40 s，4001 点）：

| 场景 | η max\|Δ\| vs A | η RMS Δ | ν max\|Δ\| | u max\|Δ\| | u RMS\|Δ\| | **跟踪 e RMS（末 10 s）** |
|---|---|---|---|---|---|---|
| C（10 ms 定步长，连续控制器） | 9.92e-02 m | 1.12e-02 m | 9.37e-01 | 200 V | 4.26 V | **7.37e-03 m** |
| D（10 ms 定步长 + 离散控制器） | 1.90e-01 m | 2.33e-02 m | 1.77e+00 | 197.1 V | 6.01 V | **1.64e-02 m** |

读数要点：

1. **换成 10 ms 离散控制器只让末段跟踪误差 RMS 从 7.4e-3 m 增到 1.6e-2 m**
   —— 相对 2 m 幅值的参考圆约 **0.8 %**，完全可接受，说明 10 ms 控制周期够用。
2. ⚠ **`u max|Δ| ≈ 200 V` 不是误差**，而是「**饱和翻转**」：通道一旦贴住 ±100 V，
   两侧广义力上的极小差异就会让一个输出 +100、另一个 −100，单点差值直接到
   200 V（正好 2 倍满量程）。**必须同时看 `u RMS|Δ|`**（4.3 / 6.0 V，即满量程的
   4–6 %）才反映真实指令差异。
3. **跟踪误差口径**：`跟踪 e RMS = RMS(η − ηd)` 在最后 10 s 的三档差值，与论文
   正文里"调节时间 / 稳态 RMS"用的是同一个口径。
   （早期版本把 `ηd` 自己拿去作差，得到 2.5e-17 这种毫无意义的数 —— 因为 A/C/D
   三档的参考轨迹本来就是同一个信号，已修正。）

逐 T_s 的并排扫描见 `out/diag_hil.log`：

| T_s | 控制器 | 结果 | η max | u max |
|---|---|---|---|---|
| 10 ms | 离散 | 收敛 | **2.649** | 100 |
| 10 ms | 连续 | 收敛 | 2.605 | 100 |
| 5 ms | 离散 / 连续 | 收敛 / 收敛 | 2.594 / 2.585 | 100 |
| 2 ms | 离散 / 连续 | 收敛 / 收敛 | 2.564 / 2.574 | 100 |
| 1 ms | 离散 / 连续 | 收敛 / 收敛 | 2.554 / 2.570 | 100 |

10 ms 离散相对连续只差 **+1.7%**（2.649 vs 2.605），且 1 ms 也只收敛到同一水平——
说明这个偏差不是采样率不足，而是离散化本身固有的，再加采样率没有收益。

> ⚠ **注意**：表格里 `u max = 100` 是**正常**的，不是发散。圆形参考轨迹的
> 初始过渡段（η(0)=0 而参考圆在 x/y 幅值 2 m、z=2 m）本身就会把推力打到饱和，
> 论文 §5.2 已用 acquisition transient 说明过这一点。

### 6.4 PiL 等效性验证（处理器在环的离线替代）

**这是最有说服力的一步**：把 Simulink 里跑出来的同一组 setpoint 序列
（`out/hosttest_sp.csv`）喂给**生成的真实 C 代码**，再把 C 代码输出的
6 路推力指令与 Simulink 的输出逐点比对。

```bash
cd firmware
gcc -std=c99 -O2 -I. -I../rov_ctrl_deploy_ert_rtw \
    hil_target_hosttest.c hil_protocol.c hil_target.c \
    ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy.c \
    ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy_data.c -lm -o hil_target_hosttest
./hil_target_hosttest ../out/hosttest_sp.csv ../out/hosttest_u_c.csv
cd ../scripts && python compare_hosttest.py
```

四份 10 ms 网格上的 u[6] 序列：

| 文件 | 含义 |
|---|---|
| `hosttest_u_ref.csv` | 连续控制器的输出（基线） |
| `hosttest_u_sim.csv` | Simulink 部署模型，**double** 输入 |
| `hosttest_u_sim_f32.csv` | Simulink 部署模型，输入先量化到 **float32** |
| `hosttest_u_c.csv` | 生成 C 代码 + 协议层 + 目标运行时 |

**实测结果（4001 点 × 6 通道）**：

| 判据 | 比较 | 结果 | 性质 |
|---|---|---|---|
| ① 代码生成 | `sim_f32` vs `c` | max\|Δ\| = **3.84e-6 V** | **输出舍入**（float32 在 \|u\|≤100 处半 ULP = 3.81e-6） |
| **① 终判** | 两侧都收窄到 float32 | **0 / 24006 个元素不相等，max\|Δ\| = 0.000e+00 V** | **逐位完全一致** |
| ①b 传输格式 | `sim` vs `sim_f32` | 峰值 8.42e-3 V，RMS 1.65e-3 V（满量程的 0.008%） | **固有代价** |
| ② 离散化 | `sim` vs `ref` | t=0 峰值 100.5 V；RMS 0.763 V；去掉前 2 s 后峰值 0.237 V | **固有代价** |

> **①终判是最关键的一行**：target 上 u 是 float32（`hil_command_t.u[]` 就是
> `float`），所以「是否一致」必须在 float32 层级判定。在 double 层级看会剩下
> 3.8e-6 V 的「输出舍入」，那**不是误差**，而是把 double 结果写进 float 变量的
> 必然结果。两侧都收窄到 float32 后 **24006 个元素全部相等**，
> 意味着代码生成链路在浮点意义上是**零误差**的。

```bash
# 自己复核这一行（任意有 numpy 的 python）
python -c "
import numpy as np
a=np.loadtxt(r'out/hosttest_u_sim_f32.csv',delimiter=',',skiprows=0)[:,1:7]
c=np.loadtxt(r'out/hosttest_u_c.csv',delimiter=',',skiprows=1)[:,2:8]
A,B=np.float32(a),np.float32(c)
print('不相等的元素 =', (A!=B).sum(), '/', A.size)"
```

这条链跑通意味着以下全部被验证：

| 被验证的东西 | 说明 |
|---|---|
| 生成 C 代码的可编译性 | 与协议层、目标运行时一起链接通过 |
| `hil_target.c` 的运行时逻辑 | 解析 → 置输入 → step → 取输出 → 组帧 全走了一遍 |
| 串口帧格式 | 4001 帧 setpoint 组帧/解帧、CRC 校验、序号回显，**0 CRC 错、0 序号错、0 丢帧** |
| 离散化正确性 | `DiscreteIntegrator` / `Discrete Derivative` 的 Ts 确实是 0.01 |
| 数值一致性 | **float32 逐位一致**（24006/24006） |
| `nu_d` 通路 | 参考速度经端口 4 → 协议字段 → 生成代码 全程无损耗 |

⚠ **踩坑记录：不要把时间列也量化成 float32**
第一版 `run_pil_check.m` 写成 `sp32 = double(single(sp))`，把第 1 列（时间）也
收窄了。`single(23.8) = 23.799999237...`，Simulink 于是按这张**被扭曲的时间轴**
做线性插值，等效给信号注入约 1e-4 量级的相对扰动（插值权重 θ 最大可达 7.6e-5），
再被控制器里增益 100 的微分项放大，凭空冒出 5.5e-3 V 的"偏差"。
正确写法是**只量化信号列**：

```matlab
sp32 = sp;
sp32(:, 2:end) = double(single(sp(:, 2:end)));   % 第 1 列时间保持 double
```

这个"假偏差"曾经让人误以为代码生成链路有问题，排查过程记录在
`out/diag_pil_residual.log`（输入逐列对账，`t` 列 1.83e-6 的偏移就是铁证）
与 `out/diag_step1.log`（4 步定值输入，两侧逐位吻合，排除算式差异）。

---

## 7. 自检清单

> **顺序建议**：先在 PC 上做完 1、12、13（不需要板子）→ 再做 2–5、9–11
> （模型与代码）→ 最后 6–8（上板）。

| # | 检查项 | 方法 | 期望 |
|---|---|---|---|
| 1 | 协议编解码正确 | `firmware>` `gcc hil_selftest.c hil_protocol.c -o t && ./t` | ALL PASS（32 项） |
| 2 | 部署模型无连续状态 | `scripts/build_deploy.m` 日志 | `连续状态块残留 = 0` |
| 3 | 部署模型可编译 | 同上日志 | `Update Diagram 通过` |
| 4 | 代码生成成功 | `scripts/build_codegen.m` 日志 | 生成 `.c/.h` |
| 5 | `real_T` 为**双**精度 | `grep "typedef.*real_T" rov_ctrl_deploy_ert_rtw/rtwtypes.h` | `typedef double real_T;` |
| 6 | 串口连通 | 板上电后接串口助手，看是否有 34 B 周期帧 | 周期 ≈ 10 ms |
| 7 | 状态位正常 | PC 侧打印 `status` | `INIT_DONE DATA_FRESH` |
| 8 | 闭环收敛 | HIL 运行 30 s，看 η 是否收敛到参考圆 | 与回环 D 组趋势一致 |
| 9 | 10 ms 离散闭环收敛 | `scripts/diag_hil.m` | 每个 T_s 下离散组均为「收敛」 |
| 10 | 代码生成一致性 | `firmware> ./hil_target_hosttest ../out/hosttest_sp.csv ../out/hosttest_u_c.csv` 后 `scripts> python compare_hosttest.py` | float32 层级 **0 个元素不相等** |
| 11 | 协议零错误 | 同上回放程序的汇总 | `TX CRC failures = 0`、`sequence mismatches = 0`、`setpoint drops = 0` |
| 12 | **目标机运行时正确（内存，不碰串口）** | `firmware> ./hil_target_sim.exe --selftest` | `RESULT: ALL PASS`（28 项，含 200 拍连续运行与超时策略） |
| 13 | **串口链路通（虚拟串口对，无板子）** | `hil_target_sim.exe COM11 460800` + `rov_hil_host.slx`（`HIL_PORT='COM10'`） | 两侧 `crc_err/badcrc = 0`、`timeouts = 0`、`drop = 0` |
| 14 | **上板红线 ①：DMA 句柄唯一** | `grep HIL_PORT_INIT_UART firmware/hil_port_stm32f4.c` | `(0)`——外设初始化交给 CubeMX |
| 15 | **上板红线 ②：TX 与 TIM6 同级** | `grep -A6 "hil_port_nvic_config" firmware/hil_port_stm32f4.c` | DMA2\_Stream7 与 TIM6 同为抢占级 6 |

12、13 的做法与预期输出见 **`docs/虚拟串口对测试.md`**；
14、15 的失败机理见 §10.4。

---

## 7.1 一键复现全部验证

```bash
# 1) 重建部署模型 + 生成代码 + PC 模型 + 离散化对照 + PiL 回放（单次 MATLAB 启动）
matlab -batch "run('E:\yanjiusheng\wb_hil\scripts\rebuild_and_check.m')"

# 2) C 侧回放 + 三方比对
cd E:/yanjiusheng/wb_hil/firmware
gcc -std=c99 -O2 -I. -I../rov_ctrl_deploy_ert_rtw \
    hil_target_hosttest.c hil_protocol.c hil_target.c \
    ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy.c \
    ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy_data.c -lm -o hil_target_hosttest
./hil_target_hosttest.exe ../out/hosttest_sp.csv ../out/hosttest_u_c.csv
cd ../scripts && python compare_hosttest.py
```

只看结果不想重跑的话，直接读这几份日志：

| 日志 | 看什么 |
|---|---|
| `out/build_deploy.log` | `连续状态块残留 = 0`、`误差符号一致`、`nu_d/1 -> Controller/4` |
| `out/build_codegen.log` | 生成成功 |
| `out/discretization_check.log` | A/C/D 三档对比表 |
| `out/diag_hil.log` | 逐 T_s 收敛性 |
| `out/pil_check.log` | PiL 回放是否跑通 |
| `out/diag_diff.log` | V0–V4 判决矩阵（数值微分元凶） |

---

## 8. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `TIMEOUT` 持续置位 | PC 端串口没打开 / 波特率不符 / 接线反了 | 确认 TX↔RX 交叉、共地，波特率一致 |
| 大量 `RX_CRC_ERR` | 波特率漂移、线太长、无共地 | 降到 230400 试；缩短杜邦线；务必共地 |
| 板子收到帧但 u 全为 0 | 未触发 TIM6 中断 | 检查 `TIM6_DAC_IRQHandler` 是否调用 `hil_port_tim6_isr()` |
| PC 仿真极慢 | 阻塞式串口往返本身耗时 | 正常现象；`StopTime` 设小一点先跑通 |
| `Simulink` 报求解器不支持连续状态 | 忘切 ode4 | 按 §6.2 设置定步长求解器 |
| 代码生成报 `参数 Value 设置无效` | 工作区缺 `DL_nom` / `I` / `m` 等 | 确认 `rovmodel75kg.mat` 与模型同目录 |
| 模型打开后 u 不收敛 | 参考轨迹与实际初值差太远 | 检查 ηd 是否为圆形轨迹、η(0) 是否为零 |

---

## 9. 与论文的关系（写作用词建议）

- 这套链路属于 **rapid control prototyping (RCP) / hardware-in-the-loop (HIL)**，
  即"控制器在真实处理器上执行，被控对象为高保真仿真模型"。
- **不要**写成"实验验证"或"水池试验"；建议措辞：
  > "…the adaptive backstepping controller was implemented on an STM32F407
  > microcontroller and validated in a hardware-in-the-loop configuration,
  > in which the six-degree-of-freedom vehicle dynamics were simulated in
  > real time on the host computer and exchanged with the target over a
  > 460.8 kbaud serial link at a 10 ms control period."
- 应报告的关键参数：**T_s = 10 ms、460800 bps、138 B/周期、实时因子**。
- 离散化引入的偏差应给出定量结果（见 §6.3 的三档对照），
  这正好回应审稿人"纯仿真"的质疑。
- 代码生成的正确性可直接引用 §6.4 的结论：**float32 层级逐位一致
  （24006/24006 个元素，max|Δ| = 0）**，这比"误差小于 1e-3"之类的说法
  强得多，因为它把链路误差压到了浮点表示极限以下。

---

## 10. 从连续模型搬到 10 ms 离散控制器：两次关键修正

这一节记录**把论文里的连续控制器改造成可上板的 10 ms 离散控制器时，
两个必须知道的坑**。两者都不是"精度不够"，而是会直接把闭环打死。

### 10.1 误差符号：e₁ = η − η_d（不是 η_d − η）

原模型 `Sum2` 的 `Inputs = '-+'`，即

```
in1 ← Mux          （参考 η_d，取负）
in2 ← Euler Transformation1（实测 η，取正）
```

⇒ 控制器内部约定的位置误差是 **e₁ = η − η_d**（实测减期望）。

**交叉验证（不用查原图就能确认）**：控制器里 `alpha1 = d(yd)/dt + ĉ₁·err`，
而热启动值 `ĉ₁(0) = −3`。只有取 `e₁ = η − η_d` 才能得到
`α₁ = η̇_d − 3·e₁` 这种"误差越大、修正越向期望靠"的正常形式；
若把 err 取成 `η_d − η`，整个矫正项会变号，闭环必然反向发散。

**后果**：我在 `build_deploy.m` 里第一版把 `e_calc` 建成 `Inputs='+-'`
（`η_d − η`），生成代码里就成了

```c
rtb_Product2 = rov_ctrl_deploy_U.eta_d[i] - rov_ctrl_deploy_U.eta[i];   /* 错！ */
```

修正后为

```c
rtb_Product1 = rov_ctrl_deploy_U.eta_d[i];
rtb_Product2 = rov_ctrl_deploy_U.eta[i] - rtb_Product1;                  /* 对 */
```

`build_deploy.m` 里已加**自动对账**：读原模型 `Sum2` 的 `Inputs`，
与新建 `e_calc` 的 `Inputs` 比对，不一致就报 `[FAIL]`。

### 10.2 参考速度必须解析给定：不要对 ηd 做数值微分

**这是最花时间才定位到的一条。**

原模型用连续块 `Derivative` 对参考 `yd` 求导得到 `η̇_d`。搬到离散域时，
如果照搬成 `Discrete Derivative`（即 `(u[k] − u[k−1])/T_s`），会出现：

| T_s | 离散控制器 | 连续控制器 |
|---|---|---|
| 10 ms | ❌ 发散（η max = 360） | ✅ 收敛（η max = 2.605） |
| 5 ms | ❌ 发散 | ✅ 收敛 |
| 2 ms | ❌ 发散 | ✅ 收敛 |
| 1 ms | ❌ 发散 | ✅ 收敛 |

**注意：离散控制器在 1 ms 下也照样发散——与采样率完全无关。**
这排除了"离散化精度不够"这一类解释。

**判决实验**（`scripts/diag_diff_hypothesis.m`，日志 `out/diag_diff.log`）：
控制器里只有两个微分器，逐个开关做 2×2 矩阵：

| 变体 | `Derivative`（对 η_d） | `Derivative1`（对 α₁） | 结果 | η max |
|---|---|---|---|---|
| V0 基线 | 开 | 开 | ❌ | 360 |
| **V1** | **关** | 开 | **✅ 收敛** | **2.687** |
| V2 | 开 | 关 | ❌ | 360 |
| V3 | 关 | 关 | ❌ | 360 |
| V4 复现 V0 | 开 | 开 | ❌ | 360 |

⇒ **元凶唯一确定：对参考 η_d 求数值微分的那一个 `Derivative`。**

**根因**：参考信号里有 `Step` 跳变。差分器把它放大 `1/T_s` 倍
（10 ms → **200 倍**），在阶跃时刻附近注入一个巨大的假速度前馈，把闭环打飞。
而 `Derivative1` 对的是**计算出来的** α₁，它是光滑的，所以必须保留
（V2 证明关掉它会失稳）。

**修正方案**：删除 `Derivative`，把 `η̇_d` 改为**显式的第 4 个输入 `nu_d`**，
由 PC 端按解析式给出：

```matlab
% Sin 块 A*sin(w*t+phi) 的导数 = (A*w)*sin(w*t+phi+pi/2)
% Step 块的导数取 0（工程上按冲激处理不合理）
```

修正后：**10 ms 离散控制器 η max = 2.649，连续控制器 2.605，只差 +1.7%**。

> 协议里 `hil_setpoint_t` 本来就预留了 `nu_d` 字段（§4.2），
> 所以这个改动只影响 Simulink 侧，串口协议不用改。

### 10.3 副作用：`real_T` 被推成单精度，需显式钉回 double

代码生成配置里有 `DefaultUnderspecifiedDataType = 'single'`。
新建的 Inport 默认 `OutDataTypeStr = 'Inherit: auto'`，在这个配置下会被
推成 **`real32_T`（float）**，而其它入口是 `real_T`（double）——

```c
real_T   eta[6];   real_T  nu[6];   real_T  eta_d[6];
real32_T nu_d[6];                        /* ← 混进了单精度 */
```

后果是 `nu_d` 少了 7 位有效数字，而它正是被增益 100 的微分项吃掉的量。
**修正**：对**全部**根 Inport 与模型内部入口显式设置

```matlab
set_param([DST '/' nmb], 'OutDataTypeStr', 'double');
```

修正后 `rov_ctrl_deploy.h` 里四个入口统一为 `real_T`。

> 顺便记住一个容易踩的结论：`real_T` 在 Embedded Coder 里**跟随模型信号精度**，
> 改 `ProdHWDeviceType` / `TargetHWDeviceType` / `DefaultUnderspecifiedDataType`
> 都**改不动**它——本项目的 `rtwtypes.h` 里 `real_T` 就是 `double`。

### 10.4 上板前必须改掉的两处固件缺陷

Simulink 侧修好不等于板子能跑。下面两处是**纯固件**问题，
2026-10-09 代码审查发现，都会让第一次上电直接失败。

**① 两套 DMA 句柄 → 第一帧就 HardFault**

| 项 | 内容 |
|---|---|
| 症状 | 上电后串口一个字节都不出；若接了调试器，停在 `HAL_DMA_IRQHandler()` 里，进 HardFault |
| 根因 | `HIL_PORT_INIT_UART = 1` 时 `uart1_init()` 建**函数内 `static DMA_HandleTypeDef hdma_tx / hdma_rx`**；而 CubeMX 的 `stm32f4xx_it.c` 里 `DMA2_Stream7_IRQHandler()` 调用的是 **CubeMX 在 `main.c` 里的全局 `hdma_usart1_tx`**——那是另一个对象，从未初始化，`Instance == NULL`。`HAL_DMA_IRQHandler()` 第一行 `hdma->Instance->ISR` 空指针解引用 |
| 修法 | `HIL_PORT_INIT_UART` 默认已改为 **0**：外设初始化全部交给 CubeMX，全工程每个 DMA 流只有一个句柄对象（这是 HAL 的不变式） |

**② TX 环形缓冲区的 check-then-set 竞态**

| 项 | 内容 |
|---|---|
| 症状 | 偶发 `HIL_ST_TX_FAIL`；`g_hil_tx_cnt` 增长比 `g_hil_tick_cnt` 慢；PC 侧看到漏帧 |
| 根因 | `hil_port_uart_write()`（TIM6 中断）与 `HAL_UART_TxCpltCallback()`（DMA2_Stream7 中断）都改 `s_tx_busy`，而 `tx_kick()` 的 `if (busy) return; busy = 1;` 非原子。DMA TX 抢占级高于 TIM6 时，可以插在判断与写入之间 |
| 修法 | `hil_port_nvic_config()` 把 **DMA2_Stream7 与 TIM6 设为同一抢占优先级（都 6）**，二者无法互相打断，读改写天然互斥。USART1 与 DMA RX 留在 5（高于控制节拍），保证收字节不被延迟 |

> 这两处的修法都不是"调参数"，是消除设计歧义：**一个 DMA 流一个句柄**、
> **一组共享状态一个优先级**。改动已在 `firmware/hil_port_stm32f4.c` 落地，
> 两份分支（`HIL_PORT_INIT_UART` = 0 / 1）都过了 `gcc -Wall -Wextra`
> 语法自检（借助本地 HAL 桩头文件）。
