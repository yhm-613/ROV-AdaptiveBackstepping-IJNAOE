# firmware/ — 目标端（STM32F407）HIL 代码

## 文件

| 文件 | 作用 | 是否与硬件相关 |
|---|---|---|
| `hil_protocol.h` | 协议常量、payload 结构、CRC16-Modbus 内联实现 | 否 |
| `hil_protocol.c` | 帧打包 / 解包 / 逐字节增量解析状态机 | 否 |
| `hil_selftest.c` | 主机端自检（32 项断言），可在 PC 上直接运行 | 否 |
| `hil_target.h/.c` | 目标端运行时：100 Hz 控制节拍 + 帧处理 | 否（走 port 层） |
| `hil_target_hosttest.c` | **PC 上整链回放测试**（CSV → 组帧 → 解析 → step → 取 u），PiL 用 | 否 |
| `hil_target_sim.c` | **串口版虚拟目标**：在 PC 上扮演 STM32，配虚拟串口对使用 | 否（Win32 串口） |
| `hil_port.h` | 硬件抽象层接口（4 个必做钩子 + 4 个中断钩子） | 接口 |
| `hil_port_stm32f4.c` | YS-F4Pro 的 HAL 实现（USART1 + DMA + TIM6） | **是** |
| `stm32_app/` | CubeMX 配置清单 + `main.c` / `stm32f4xx_it.c` 参考实现 | **是** |
| `rov_ctrl_deploy.c/.h` 等 | Simulink 代码生成产物（见 `../rov_ctrl_deploy_ert_rtw/`） | 否 |

## 整链回放验证（PiL，最有说服力的一步）

```bash
cd firmware
gcc -std=c99 -O2 -I. -I../rov_ctrl_deploy_ert_rtw \
    hil_target_hosttest.c hil_protocol.c hil_target.c \
    ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy.c \
    ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy_data.c -lm -o hil_target_hosttest
./hil_target_hosttest ../out/hosttest_sp.csv ../out/hosttest_u_c.csv
cd ../scripts && python compare_hosttest.py
```

期望：`TX CRC failures = 0`、`sequence mismatches = 0`、`setpoint drops = 0`，
且 Python 比对给出 **float32 层级 0 个元素不相等**。

可选第 3 个参数会转储"实际喂给控制器的 float32 setpoint"，用于和主机侧逐列对账：

```bash
./hil_target_hosttest ../out/hosttest_sp.csv ../out/hosttest_u_c.csv \
                      ../out/hosttest_sp_c_dump.csv
```

## 主机端自检（先做这一步）

```bash
cd firmware
gcc -std=c99 -Wall -Wextra -O2 -I. hil_selftest.c hil_protocol.c -o hil_selftest
./hil_selftest
```

期望输出结尾：

```
 RESULT: ALL PASS
```

覆盖内容：帧长几何、CRC16-Modbus 已知答案（`crc16("123456789") = 0x4B37`）、
SETPOINT 往返位精确一致、逐字节解析器（干净流 / 前导垃圾 / 单字节损坏 / 重复 SYNC / 错误 LEN）、
COMMAND 往返、带宽核算。

## 串口链路验证（不需要开发板）

`hil_target_sim.c` 在 PC 上扮演目标机：它**链接真实的 `hil_target.c` 与生成
代码**，只把 `hil_port_*` 换成 Win32 串口调用。配一对虚拟串口（com0com），
就能让 `rov_hil_host.slx` 真的跑一遍串口收发。

```bash
cd firmware
gcc -std=c99 -O2 -I. -I../rov_ctrl_deploy_ert_rtw \
    hil_target_sim.c hil_protocol.c hil_target.c \
    ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy.c \
    ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy_data.c \
    -lm -lwinmm -o hil_target_sim.exe          # -lwinmm 是 timeBeginPeriod 需要的

./hil_target_sim.exe --selftest                # ① 内存自检，28 项，不碰串口
./hil_target_sim.exe COM11 460800              # ② 起虚拟目标（另开一个终端）
```

`--selftest` 覆盖：帧长几何、逐字节喂帧、回帧的 sync/type/len/CRC/seq 回显、
4 个状态位、200 拍连续运行的 CRC 与序号、以及断流 50 ms 后 `TIMEOUT` 置位。

②之后在 MATLAB 里把 `pc/hil_bridge_host.m` 的 `HIL_PORT` 指到虚拟对的
**另一半**（如 `COM10`），`open_system('rov_hil_host')`，定步长 ode4 @ 0.01。

完整步骤（com0com 安装、口名改成 COM10/COM11、预期输出、排查表）见
**`../docs/虚拟串口对测试.md`**。

> **这个测试管"对不对"，不管"够不够快"。** 虚拟串口对默认不做波特率仿真，
> 数据立即透传，所以验证不了 460800 是否扛得住 138 B/周期，也验证不了
> 10 ms 抖动。那些必须上板实测。

## 上板集成

**`stm32_app/README.md`** 里有完整路径：CubeMX 逐项勾选表 → 生成后要改的
5 处 → 编译选项 → 首次上电四步验证。架构层面的说明见
`../docs/HIL部署与串口通信说明.md` 第 5 节、故障归因见 §10.4。

两条**上板即失败**的红线，先记住：

| 红线 | 为什么 |
|---|---|
| `hil_port_stm32f4.c` 里 `HIL_PORT_INIT_UART` **保持 0** | 改成 1 会造出第二套 DMA 句柄，而 CubeMX 的 `DMA2_Stream7_IRQHandler()` 调用的是它自己那个未初始化的句柄 → 第一帧就 HardFault |
| **DMA2_Stream7 与 TIM6 抢占优先级必须相同** | 二者都改 TX 环形缓冲区的 `s_tx_busy`，`tx_kick()` 是 check-then-set；同级才能保证天然互斥 |

## 移植到别的板子

只需要重新实现 `hil_port.h` 的 4 个必做钩子：

```c
void     hil_port_init(void);
int      hil_port_uart_write(const uint8_t *buf, uint16_t len);
void     hil_port_apply_u(const volatile float *u, int n);
uint32_t hil_port_millis(void);
```

外加把 UART 收到的每个字节喂给 `HIL_OnByte()`，并每 10 ms 调用一次 `HIL_Step10ms()`。
`hil_protocol.*` 与 `hil_target.c` 完全不用改。

`hil_port_sim` 之外还有 4 个只在中断世界里才有意义的钩子
（`hil_port_rx_start` / `hil_port_rx_poll_idle` / `hil_port_tim6_start` /
`hil_port_tim6_isr`）。不用中断架构的移植可以留空实现——
`hil_target_sim.c` 就是这么做的（它改成主循环里非阻塞读 + 软节拍）。

## setpoint 的四个数组（注意第 4 个）

`HIL_Step10ms()` 每拍把 `g_sp` 灌进生成代码的入口：

```c
rov_ctrl_deploy_U.eta[i]   = (real_T)g_sp.eta[i];
rov_ctrl_deploy_U.eta_d[i] = (real_T)g_sp.eta_d[i];
rov_ctrl_deploy_U.nu[i]    = (real_T)g_sp.nu[i];
rov_ctrl_deploy_U.nu_d[i]  = (real_T)g_sp.nu_d[i];   /* 参考速度，解析给定 */
```

⚠ **`nu_d` 不是可选量，必须由 PC 端解析给出**（`η̇_d = d(ηd)/dt`）。
板子**绝不能**用差分去算它：参考里有 `Step` 跳变，差分器会把跳变放大 `1/T_s`
（10 ms → 200 倍），闭环在阶跃时刻直接发散，且**与 T_s 取值无关**。
实测判决矩阵见 `../out/diag_diff.log`（V0–V4），完整说明见
`../docs/HIL部署与串口通信说明.md` §10.2。

PC 侧构造规则（参考轨迹 `x_d = 2sin(0.2t+π/2)`、`y_d = 2sin(0.2t)` 等）：
`Sin → A·w·sin(w t + φ + π/2)`（即 `A·w·cos(w t + φ)`）；`Step → 0`。

## 精度说明（实测，勿想当然）

`rtwtypes.h` 里**实测**是：

```c
typedef float  real32_T;
typedef double real_T;        /* ← 本项目就是 double，不是 float */
```

原因：`real_T` 在 Embedded Coder 里**跟随模型信号精度**，改
`ProdHWDeviceType` / `TargetHWDeviceType` / `DefaultUnderspecifiedDataType`
都**改不动**它。所有常量参数（`ConstP`）与状态也都是 `real_T` = double。

**对板上性能的影响（必须知道）**：Cortex-M4F 只有**单精度**硬件 FPU，
`double` 运算会落到软件浮点库。

| 估算项 | 数值 |
|---|---|
| 单周期 double 乘加 | ≈ 2000 次 |
| 单周期 double 三角函数（`sin/cos/tan`，J(η) 等） | ≈ 30 次 |
| 折算 | ≈ 3.7e4 cycles（四则）+ ≈ 2.4e4 cycles（超越函数） |
| 合计 @168 MHz | ≈ **3.7e2 µs** |
| 占 10 ms 周期 | ≈ **3.7 %** |

⇒ **10 ms 周期完全够用，建议保持 `double`**。理由是：

1. 性能余量充足（< 5 %）；
2. **保持 double 才能让 PiL 的"逐位一致"结论成立** —— 实测生成代码在
   float32 层级与 Simulink 模型 24006/24006 个元素完全相同
   （见 `../docs/HIL部署与串口通信说明.md` §6.4）；
3. 改成 `single` 会引入需要重新论证的数值差异，而收益（省 ~3 % CPU）没意义。

若将来真的把控制周期压到 1 ms 以内，再考虑改单精度，届时必须重做
§6.4 的等效性验证。
