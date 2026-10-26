// ============================================================================
// riscv1_pango_top.v — RISCV_1 (BRV32P SoC) board top for the Pango remote-lab
//                      board (盘古 200K MINI / MES2L676, PG2L200H-6IFBG676)
// ----------------------------------------------------------------------------
// 本文件是移植时【新增】的唯一 RTL 文件，只做接口适配，不含任何 CPU/SoC 逻辑：
//
//   1. 顶层端口换成板上真实接口（时钟/按键/LED/串口），SoC 的 32 位 gpio_in
//      只引出低 5 位接板上的 5 个用户按键，其余读 0；
//   2. 板上 SW2(RESET) 并不接 FPGA（它驱动核心板复位监控芯片，见
//      ../../BRV32P_pango 的说明），所以 SoC 复位在这里用内部上电复位产生；
//   3. 其余端口与 RISCV_1 的 brv32p_soc 一一对应，核心 RTL 未做任何修改。
//
// 按键：底板丝印 SW2 SW3 SW4 SW5 SW6 SW7，其中 SW3..SW7 = KEY0..KEY4 =
//       C22 C23 B22 A22 B20，低有效、10K 上拉。
// ============================================================================

module riscv1_pango_top (
  input  wire       sys_clk,     // D18 — 核心板 27 MHz 晶振
  input  wire [4:0] key,         // SW3..SW7 = KEY0..KEY4，低有效
  output wire [7:0] led,         // LED0..LED7（底板上只有 LED0..LED4 五颗灯）
  input  wire       uart_rx,     // AC26 — 平台 UART TXD -> FPGA
  output wire       uart_tx      // AB26 — FPGA -> 平台 UART RXD
);

  // ── 内部上电复位 ────────────────────────────────────────────────────────
  // SW2 按下会拉低 REST，经核心板 SGM811B 复位监控芯片去重新配置 FPGA，
  // 没有任何用户 IO 球连接到它。所以这里用计数器产生复位：配置完成后
  // 保持 2^17 个时钟（27 MHz 下约 4.9 ms），再同步释放。
  reg [17:0] por_cnt = 18'd0;
  wire       sys_rst_n = por_cnt[17];

  always @(posedge sys_clk)
    if (!sys_rst_n)
      por_cnt <= por_cnt + 18'd1;

  // GPIO：SoC 输入是 32 位，板上只接 5 个按键，高 27 位读 0。
  wire [31:0] gpio_in;
  wire [31:0] gpio_out;
  assign gpio_in = {27'b0, key};

  brv32p_soc u_soc (
    .clk      (sys_clk),
    .rst_n    (sys_rst_n),
    .gpio_in  (gpio_in),
    .gpio_out (gpio_out),
    .uart_rx  (uart_rx),
    .uart_tx  (uart_tx)
  );

  // 板上 8 个 LED 管脚只用到低 8 位（实际焊了 LED0..LED4 共 5 颗）。
  assign led = gpio_out[7:0];

endmodule