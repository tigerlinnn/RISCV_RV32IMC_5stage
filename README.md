# RISCV_1 → 紫光同创（Pango）平台移植工程

源工程：`D:\UESTC\2026\QS\project\RISCV_1_Portable\RISCV_1_Portable\RISCV_1`
（Vivado 工程，器件 `xc7a200tfbg484-2`，顶层 `brv32p_soc`，50 MHz）

本工程：`D:\UESTC\2026\QS\project\RISCV_1_pango`
（PDS 工程，器件 **PG2L200H-6FBB676**，顶层 `riscv1_pango_top`，27 MHz）

移植原则：**核心逻辑零改动，只做接口/约束/工具流程适配。**
`rtl/` 下所有 `.v`/`.vh` 都是从源工程逐字复制的，与源工程 **字节一致**；
唯一的 RTL 新增文件是 `top/riscv1_pango_top.v`（板级封装）。

---

## 1. 目录结构

```
RISCV_1_pango/
├── rtl/                    ← 源工程 RISCV_1.srcs/sources_1/new 的逐字复制
│   ├── brv32p_soc.v        （SoC 顶层，未改）
│   ├── core/   alu / branch_predictor / brv32p_core / compressed_decoder /
│   │           csr / decoder / hazard_unit / muldiv / regfile
│   ├── cache/  icache / dcache
│   ├── bus/    axi_interconnect / axi_periph_bridge / axi_sram
│   ├── periph/ gpio / timer / uart
│   └── pkg/    brv32p_defs.vh
├── top/
│   └── riscv1_pango_top.v  ← 【新增】板级封装（只做端口/复位适配）
├── constr/
│   └── riscv1_pango.fdc    ← 【新增】管脚 + 时钟约束（沿用已验证的板级管脚）
├── firmware/
│   ├── firmware.hex                       ← 27 MHz 演示固件（UART 分频 233）
│   ├── firmware_50mhz_original.hex        ← 源工程原版（分频 433，供对照）
│   ├── firmware_coremark_27mhz.hex        ← CoreMark 跑分版（正常打印）
│   └── firmware_coremark_27mhz_slow.hex   ← CoreMark 跑分版（慢速打印，平台日志用）
├── coremark/               ← 源工程 CoreMark 移植（逐字复制）+ 新增构建脚本
│   ├── build_coremark.ps1  ← 【新增】27 MHz 配置的编译脚本
│   └── sim/run_sim.ps1     ← 【新增】XSim 验证脚本
├── bitstream/              ← 已生成的比特流（演示版 / CoreMark 版）
├── build.tcl               ← 【新增】PDS 无头流程脚本
├── build.sh                ← 【新增】调用 pds_shell 的封装
├── prj/                    ← 构建输出（.pds / 综合 / 布局布线 / 时序 / .sbit）
└── README.md               ← 本文件
```

---

## 2. 具体改了哪些内容（全部改动清单）

### 2.1 RTL —— 核心逻辑 0 处改动

以下 18 个 RTL 文件 + 1 个头文件 **逐字节原样复制**，没有修改任何一行：

`rtl/brv32p_soc.v`、`rtl/core/*.v`（9 个）、`rtl/cache/*.v`（2 个）、
`rtl/bus/*.v`（3 个）、`rtl/periph/*.v`（3 个）、`rtl/pkg/brv32p_defs.vh`。

> 源工程里 `brv32p_soc.v` 位于 `sources_1/new/` 根目录，其余在子目录 —— 本工程
> 保持相同的相对目录结构，只把根目录从 `sources_1/new/` 换成 `rtl/`。

**新增的唯一 RTL 文件：`top/riscv1_pango_top.v`**，它只做三件事：

1. **端口适配**：顶层端口从 SoC 的 `clk / rst_n / gpio_in[31:0] /
   gpio_out[31:0] / uart_rx / uart_tx` 换成板级接口
   `sys_clk / key[4:0] / led[7:0] / uart_rx / uart_tx`；
   `gpio_in = {27'b0, key}`，`led = gpio_out[7:0]`；
2. **内部上电复位**：板上 SW2(RESET) 不接 FPGA（它经核心板 SGM811B 复位监控
   芯片去重新配置器件），没有可用的复位管脚，因此用 18 位计数器产生约 4.9 ms
   的上电复位（与 BRV32P_pango 完全相同的做法）；
3. 例化 `brv32p_soc` 并连接上述信号。

> 效果等价于源工程 XDC 中 `rst_n ← U7`、`gpio_in ← KEY0..3`、
> `gpio_out[0..3] ← LED0..3` 的接口约定，只是把板上实际可用资源换成
> 5 个按键 / 8 个 LED 管脚。

### 2.2 约束 —— 新建 `.fdc`

`constr/riscv1_pango.fdc` 的管脚/时钟与已在真板验证过的
`BRV32P_pango/constr/brv32p_pango.fdc` **逐条一致**（顶层端口名相同），
仅注释按本工程调整：

| 顶层端口 | 管脚 | 说明 |
|---|---|---|
| `sys_clk` | **D18** | 核心板 27 MHz 晶振；`create_clock -period 37.037` |
| `key[0..4]` | **C22 C23 B22 A22 B20** | 底板 SW3..SW7 = KEY0..KEY4，低有效 |
| `led[0..7]` | **A20 C18 C19 E18 A17 A18 C17 B17** | 高有效；底板上实际只有 LED0..LED4 五颗 |
| `uart_tx` | **AB26** | FPGA → 平台串口（实测确认的方向） |
| `uart_rx` | **AC26** | 平台串口 → FPGA |

> ⚠️ 串口用的是**远程实验平台**的 AB26/AC26，不是底板 CP2102 那对 C24/B24。
> 平台的提示 "RX·AC26 / TX·AB26" 是按 FPGA 视角写的，已在 BRV32P_pango 上实测。

### 2.3 固件 —— 仅改 1 条指令（UART 分频）

源工程的 `firmware.hex` 是按 **50 MHz** 生成的，UART 分频值 433
（`50e6 / (433+1) = 115207` baud）。本板是 **27 MHz**，同样用 115200 需要
`27e6 / (233+1) = 115385` baud（误差 0.16%），因此：

| 地址 | 原值（50 MHz） | 新值（27 MHz） | 含义 |
|---|---|---|---|
| `0x0000_0080` | `1B100A93` | `0E900A93` | `addi x21, x0, 433` → `addi x21, x0, 233` |

**整个固件只改了这一个字**，其余 45 条指令逐字节相同
（`diff` 可复现：`firmware_50mhz_original.hex` vs `firmware.hex`）。

固件本身未重新编译——它是 `gen_firmware.py` 生成的机器码演示程序：
GPIO 方向 `0xFF`、`gpio_out = 0x34`、UART 以设定分频持续输出 `'H'`，
最后 `ecall`（无陷阱处理程序，`mtvec=0`，于是从头重新执行，形成循环）。
程序逻辑与源工程完全一致。

> `INIT_FILE` 仍为相对名 `"firmware.hex"`，PDS 按 `-work_dir prj/` 解析，
> 所以 `build.sh` 会把 `firmware/firmware.hex` 复制到 `prj/firmware.hex`。
> CoreMark 固件同理，见第 5 节。

### 2.4 工具流程 —— 新增 2 个脚本

- `build.tcl`：PDS 流程（`add_design → set_arch → compile → add_constraint
  → synthesize → dev_map → pnr → report_timing → gen_bit_stream`），
  与 BRV32P_pango 的已验证流程相同；
  - `compile -include_path rtl/pkg`：`brv32p_defs.vh` 在 `pkg/` 子目录；
  - `compile -loop_limit 10000`：`axi_sram.v` 用 8192 次循环初始化存储器，
    超过 ADS 默认 2000 上限（不改 RTL，只提高工具上限）；
  - `-top_module riscv1_pango_top`。
- `build.sh`：复制 `firmware.hex`、把 `build.tcl` 复制成临时脚本再交给
  `pds_shell`（**不能直接把 build.tcl 交给 pds_shell**：它会把执行过的命令
  追写回脚本，导致脚本自我改写）。

---

## 3. 构建方法

```bash
cd D:/UESTC/2026/QS/project/RISCV_1_pango
./build.sh
```

约 10 分钟，输出：

| 文件 | 内容 |
|---|---|
| `prj/generate_bitstream/riscv1_pango_top.sbit` | 比特流（平台下载用） |
| `prj/report_timing/riscv1_pango_top.rtr` | 布局布线后时序 |
| `prj/riscv1_pango.pds` | PDS GUI 工程文件（自动生成） |
| `prj/pds_run.log` | 完整控制台日志 |

换固件重出比特流：`FIRMWARE=/path/to/xxx.hex ./build.sh`。
换工具链：`PDS=/path/to/pds_shell.exe ./build.sh`。

---

## 4. 预期上板现象

- **LED**：`gpio_out = 0x34`（bit 2、bit 4）→ 底板上第 2、第 5 颗 LED 常亮
  （`led[2]=C19`、`led[4]=A17`；底板只有 LED0..LED4 五颗灯，高电平点亮）。
- **UART**：115200 8N1，连续输出字符 `H`（0x48）。

> 远程平台的串口日志会按 64 字节窗口截断（平台已知行为），单字符 `H`
> 不受影响；若看到日志断续，先怀疑平台而不是设计。

---

## 5. CoreMark 集成与跑分

源工程的 `coremark/`（RISC-V 移植版，原配置为 50 MHz）已整体复制到本工程
`coremark/` 并逐字保留；新增 `build_coremark.ps1`，只把 **时钟相关常量**改成
本板的值：

| 配置 | 源工程（50 MHz 板） | 本工程（Pango 27 MHz） |
|---|---|---|
| `CLOCKS_PER_SEC` | 50,000,000 | **27,000,000** |
| UART 分频 | 433 | **233** |
| `ITERATIONS` | 1265（>10 s） | **700**（≈11.3 s） |
| 编译参数 | `-O2 -march=rv32imc_zicsr -mabi=ilp32` | 相同（另加 `-mno-relax`） |

### 5.1 编译

```powershell
cd D:\UESTC\2026\QS\project\RISCV_1_pango\coremark
.\build_coremark.ps1                   # 跑分版（正常打印）
.\build_coremark.ps1 -SlowPrint 60000  # 慢速打印版（远程平台日志用）
.\build_coremark.ps1 -Iterations 5     # 仿真验证用
```

工具链：`D:\UESTC\2026\tools\xpack-riscv-none-elf-gcc-15.2.0-1`。
产物为 `coremark/build/firmware.hex`；已复制到：

| 文件 | 用途 |
|---|---|
| `firmware/firmware_coremark_27mhz.hex` | 正常打印，本地串口看报告 |
| `firmware/firmware_coremark_27mhz_slow.hex` | 每字符慢 ~12 ms，**远程平台串口日志可完整接收**；分数不受影响 |

编译结果：`text 12,314 + data 12 + bss 20`，占 32 KB 主存的约 12.3 KB，
剩余 20 KB 栈空间（链接脚本要求至少 8 KB）。

### 5.2 分数预期

源工程 README 实测单次迭代 **434,782 周期**（-O2，2.30 CoreMark/MHz）：

- 27 MHz 下 700 次迭代 ≈ **11.27 s**，满足 CoreMark“跑满 10 秒”规则；
- 预期约 **62.1 iterations/s ≈ 2.30 CoreMark/MHz**；
- 实际分数以串口打印的 `Total ticks` 为准：
  `CoreMark/s = 700 × 27,000,000 / Total ticks`。

### 5.3 仿真验证（XSim）

`coremark/sim/run_sim.ps1` 使用 5 次迭代固件、直接监听 UART 发送寄存器，
还原串口文本。运行：`.\run_sim.ps1`（Vivado 2018.3，约 3.5 分钟）。

**仿真结果（2026-10-21，PASS）**：

```
CoreMark Size    : 666
Total ticks      : 2238870          (5 次迭代，冷 Cache 计入)
Iterations       : 5
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0xf24c
```

四个 CRC 与官方参考值**完全一致**。报告末尾的 `Errors detected` 只是因为
5 次迭代在仿真里仅 0.08 秒，触发 CoreMark 的“必须跑满 10 秒”规则
（`coremark/core_main.c:373`），**不是计算错误**；上板版 700 次迭代
（≈11.3 s）即会打印 `Correct operation validated.`。

### 5.4 出 CoreMark 比特流

```bash
cd D:/UESTC/2026/QS/project/RISCV_1_pango
FIRMWARE=D:/UESTC/2026/QS/project/RISCV_1_pango/firmware/firmware_coremark_27mhz_slow.hex ./build.sh
```

程序镜像是综合时写进 BRAM 的，**换固件必须重新综合**。
生成的比特流另存为 `bitstream/riscv1_pango_coremark_27mhz.sbit`。

若使用 `firmware_coremark_27mhz.hex`（**不定义** `UART_SLOW_PRINT`，正常速度打印）
重新综合，则另存为
`bitstream/riscv1_pango_coremark_27mhz_fastprint.sbit`（2026-10-25 构建，
md5 `e8794f3d95d436148c6a9f628b07b601`）。时序与慢速版完全相同
（WNS +11.862 ns，All Constraints Met）。注意：正常打印约 50 ms 发完报告，
远程平台日志会明显缺字，但**分数不受影响**。

### 5.5 对 `coremark/` 源码的改动

只有 `coremark/uart.c` 增加了一个**可选**的慢速打印段
（`#ifdef UART_SLOW_PRINT`，默认不定义）。不定义时预处理后与源文件完全一致；
定义后只影响 `stop_time()` 之后的报告打印速度，不影响跑分。
其余 CoreMark 文件逐字节未改。

---

## 6. 构建结果（2026-10-21，PDS 2022.2-SP6.4 无头流程）

**日志 `E:` 错误 0 条；约束检查通过；`Design Summary : All Constraints Met.`**

| 项目 | 结果 |
|---|---|
| 器件 | PG2L200H-6FBB676（`FBB676` 是 Pango 的封装拼写） |
| 综合（ADS） | LUT **17750** / 159800（12%），FF **18573** / 319600（6%） |
|  | APM **11** / 740（`muldiv` 的乘法器进 DSP 硬核） |
|  | Distributed RAM **4352**（I-Cache 8KB + D-Cache 4KB 阵列）, DRM **8**（32KB 主存） |
| 布局布线后时序 | **WNS +11.862 ns**（37.037 ns 周期，0/70166 违例），**TNS 0** |
|  | 估计 Fmax **39.72 MHz** > 实际 27 MHz |
| Hold | WHS **+0.143 ns**（slow）/ **+0.100 ns**（fast），0 违例 |
| 比特流 | `prj/generate_bitstream/riscv1_pango_top.sbit`，**9,188,552 字节** |

复现命令：

```bash
cd D:/UESTC/2026/QS/project/RISCV_1_pango && ./build.sh
```

> 上板验证已于 **2026-10-25 完成**：demo 版第 2/5 颗 LED 常亮 + 串口连续 `H`；
> CoreMark 版全流程通过（见 §7.1）。远程平台串口方向、LED 物理位置等结论
> 与 BRV32P_pango 的实测一致。

### 构建时的两条已知无害提示

- `C: Bitstream-2001: SCBV has not been set` —— 与 BRV32P_pango 相同，不影响下载。
- `W: Timing-4087/4086: Port ... is not constrained` —— 只约束了时钟，没有给
  LED/按键/串口设 IO 延迟约束；report_timing 把它们当组合 IO 处理，
  不影响 `All Constraints Met.`。

### 6.2 CoreMark 版比特流（2026-10-22）

| 项目 | 结果 |
|---|---|
| 固件 | `firmware/firmware_coremark_27mhz_slow.hex`（ITERATIONS=700） |
| 时序 | 与演示版相同：**WNS +11.862 ns**，估计 Fmax 39.72 MHz，0 违例 |
| 约束 | `All Constraints Met.` |
| 比特流 | `bitstream/riscv1_pango_coremark_27mhz.sbit`（9,188,552 B，md5 `34fe645d…`） |
| 与演示版的区别 | 仅 BRAM 初始化内容不同（md5 不同）；RTL、约束、时序完全一致 |

---

## 7. 上板跑分步骤

1. 在远程平台上传并运行 `bitstream/riscv1_pango_coremark_27mhz.sbit`。
2. 串口参数 **115200 8N1**。
3. 约 **11.3 秒**后开始输出 CoreMark 报告；慢速打印版把 570 字节报告摊到
   约 7 秒（每字符 ~12 ms），以适配平台“每次只取 64 字节”的日志行为。
4. 分数计算：

   ```
   CoreMark/s   = 700 × 27,000,000 / Total ticks
   CoreMark/MHz = CoreMark/s / 27
   ```

   预期 `Total ticks ≈ 3.04×10^8` → **≈62 CoreMark/s ≈ 2.30 CoreMark/MHz**。

> 若平台日志仍缺字（平台已知问题），可改用
> `firmware_coremark_27mhz.hex` 重新综合后接本地串口查看完整报告；
> 两份固件的计分逻辑完全相同，只是打印速度不同。

### 7.1 上板实测结果（2026-10-25，PASS）

`bitstream/riscv1_pango_coremark_27mhz.sbit`（md5 `34fe645d571828cef7c90c4a566b7ab4`，
慢速打印固件）在远程平台上完整输出：

```
Total ticks      : 313058796
Total time (secs): 11
Iterations/Sec   : 63
Iterations       : 700
Compiler version : GCC15.2.0
Compiler flags   : -O2 -march=rv32imc_zicsr -mabi=ilp32
Memory location  : STACK
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x65c5
Correct operation validated.
```

- 实测 313,058,796 ticks / 27 MHz = **11.594 s**；700 / 11.594 =
  **60.37 CoreMark/s**，即 **2.236 CoreMark/MHz**（447,227 cycles/iteration）。
- 与 XSim 5 次迭代测得的 447,774 cycles/iteration 相差 **0.12%**，硬件与仿真一致。
- ⚠️ 报告的 `Iterations/Sec : 63` 是 `HAS_FLOAT=0` 下用整数秒算的
  （700 / 11 = 63），不是真实值；计分请用 `Total ticks` 换算。
- CRC 全部匹配，结论 `Correct operation validated`，为有效跑分。

> 同一天 demo 版（`bitstream/riscv1_pango_demo_27mhz.sbit`）也通过：
> 第 2、第 5 颗 LED 常亮 + 串口连续 `H`。上传 CoreMark 版后 LED 全灭，
> 可作为“新 bitstream 已生效”的即时判据。

> **正常打印版**（`..._fastprint.sbit`，2026-10-25 实测）：报告约 50 ms 发完，
> 平台日志如预期缺字（实测吞掉了 `crcmatrix` 等行），但跑分结果与慢速版一致：
> 可见 `Total ticks` 尾数 `058798`（推定为 313,058,798，与慢速版仅差 2 ticks），
> 仍为 **60.37 CoreMark/s = 2.236 CoreMark/MHz**，收到
> `Correct operation validated`。需要完整日志时仍用上面的慢速版。