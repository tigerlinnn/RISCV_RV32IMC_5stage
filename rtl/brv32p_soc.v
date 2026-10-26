// ============================================================================
// brv32p_soc.v — BRV32P SoC Top-Level
// ============================================================================

module brv32p_soc #(
  parameter MEM_DEPTH  = 8192,
  parameter INIT_FILE  = "firmware.hex"
)(
  input  wire        clk,
  input  wire        rst_n,
  // GPIO
  input  wire [31:0] gpio_in,
  output wire [31:0] gpio_out,
  // UART
  input  wire        uart_rx,
  output wire        uart_tx
);

  `include "brv32p_defs.vh"   // RISCV_1: 取 Cache 容量等参数(ICACHE_SETS / DCACHE_SETS ...)

  // ── Core <-> Cache signals ─────────────────────────────────────────────
  wire [31:0] core_imem_addr, core_imem_rdata;
  wire        core_imem_rd, core_imem_ready;
  wire        core_ifence_inv;      // fence.i -> 作废 I-Cache

  wire [31:0] core_dmem_addr, core_dmem_wdata, core_dmem_rdata;
  wire        core_dmem_rd, core_dmem_wr, core_dmem_ready;
  wire [1:0]  core_dmem_width;
  wire        core_dmem_sign_ext;

  // ── Cache <-> Interconnect signals ─────────────────────────────────────
  wire [31:0] ic_mem_addr, ic_mem_rdata;
  wire        ic_mem_rd, ic_mem_valid;
  wire [3:0]  ic_mem_arlen;      // RISCV_1: I-Cache 行填充突发长度

  wire [31:0] dc_mem_addr, dc_mem_rdata, dc_mem_wdata;
  wire        dc_mem_rd, dc_mem_wr, dc_mem_valid;
  wire [3:0]  dc_mem_wstrb;
  wire        dc_mem_wr_done;
  wire [3:0]  dc_mem_arlen;      // RISCV_1: D-Cache 行填充突发长度

  // ── Interconnect <-> Slave signals ─────────────────────────────────────
  wire [31:0] s0_araddr, s0_rdata, s0_awaddr, s0_wdata;
  wire [3:0]  s0_arlen;          // RISCV_1: 转给主存的突发长度
  wire        s0_rlast;          // RISCV_1: 突发最后一拍
  wire        s0_arvalid, s0_arready, s0_rvalid, s0_rready;
  wire        s0_awvalid, s0_awready, s0_wvalid, s0_wready;
  wire [3:0]  s0_wstrb;
  wire        s0_bvalid, s0_bready;

  wire [31:0] s1_araddr, s1_rdata, s1_awaddr, s1_wdata;
  wire        s1_arvalid, s1_arready, s1_rvalid, s1_rready;
  wire        s1_awvalid, s1_awready, s1_wvalid, s1_wready;
  wire [3:0]  s1_wstrb;
  wire        s1_bvalid, s1_bready;

  // ── Peripheral bridge <-> Peripherals ──────────────────────────────────
  wire [7:0]  gpio_addr, uart_addr, timer_addr;
  wire        gpio_rd, gpio_wr, uart_rd, uart_wr, timer_rd, timer_wr;
  wire [31:0] gpio_wdata_b, gpio_rdata_b;
  wire [31:0] uart_wdata_b, uart_rdata_b;
  wire [31:0] timer_wdata_b, timer_rdata_b;
  wire        gpio_irq, uart_irq, timer_irq;

  // ── CNN accelerator (0x2000_0300) ──────────────────────────────────────
  wire [7:0]  accel_addr;
  wire        accel_rd, accel_wr;
  wire [31:0] accel_wdata_b, accel_rdata_b;

  // ══════════════════════════════════════════════════════════════════════
  // CPU Core
  // ══════════════════════════════════════════════════════════════════════
  brv32p_core u_core (
    .clk           (clk),
    .rst_n         (rst_n),
    .imem_addr     (core_imem_addr),
    .imem_rd       (core_imem_rd),
    .imem_rdata    (core_imem_rdata),
    .imem_ready    (core_imem_ready),
    .dmem_addr     (core_dmem_addr),
    .dmem_rd       (core_dmem_rd),
    .dmem_wr       (core_dmem_wr),
    .dmem_width    (core_dmem_width),
    .dmem_sign_ext (core_dmem_sign_ext),
    .dmem_wdata    (core_dmem_wdata),
    .dmem_rdata    (core_dmem_rdata),
    .dmem_ready    (core_dmem_ready),
    .ext_irq       (gpio_irq | uart_irq),
    .timer_irq     (timer_irq),
    .ifence_inv    (core_ifence_inv)
  );

  // ══════════════════════════════════════════════════════════════════════
  // I-Cache
  // ══════════════════════════════════════════════════════════════════════
  // RISCV_1: 显式传参, 容量集中在 pkg/brv32p_defs.vh 里改
  icache #(
    .SETS       (ICACHE_SETS),
    .WAYS       (ICACHE_WAYS),
    .LINE_BYTES (ICACHE_LINE_W/8)
  ) u_icache (
    .clk       (clk),
    .rst_n     (rst_n),
    .addr      (core_imem_addr),
    .rd_en     (core_imem_rd),
    .rdata     (core_imem_rdata),
    .ready     (core_imem_ready),
    .inv       (core_ifence_inv),
    .mem_addr  (ic_mem_addr),
    .mem_rd    (ic_mem_rd),
    .mem_arlen (ic_mem_arlen),
    .mem_rdata (ic_mem_rdata),
    .mem_valid (ic_mem_valid)
  );

  // ══════════════════════════════════════════════════════════════════════
  // D-Cache
  // ══════════════════════════════════════════════════════════════════════
  dcache #(
    .SETS       (DCACHE_SETS),
    .WAYS       (DCACHE_WAYS),
    .LINE_BYTES (DCACHE_LINE_W/8)
  ) u_dcache (
    .clk       (clk),
    .rst_n     (rst_n),
    .addr      (core_dmem_addr),
    .rd_en     (core_dmem_rd),
    .wr_en     (core_dmem_wr),
    .width     (core_dmem_width),
    .sign_ext  (core_dmem_sign_ext),
    .wdata     (core_dmem_wdata),
    .rdata     (core_dmem_rdata),
    .ready     (core_dmem_ready),
    .mem_addr  (dc_mem_addr),
    .mem_rd    (dc_mem_rd),
    .mem_arlen (dc_mem_arlen),
    .mem_wr    (dc_mem_wr),
    .mem_wdata (dc_mem_wdata),
    .mem_wstrb (dc_mem_wstrb),
    .mem_rdata (dc_mem_rdata),
    .mem_valid (dc_mem_valid),
    .mem_wr_done(dc_mem_wr_done)
  );

  // ══════════════════════════════════════════════════════════════════════
  // AXI Interconnect
  // ══════════════════════════════════════════════════════════════════════
  axi_interconnect u_axi (
    .clk       (clk),
    .rst_n     (rst_n),
    .m0_araddr (ic_mem_addr),
    .m0_arvalid(ic_mem_rd),
    .m0_arlen  (ic_mem_arlen),
    .m0_arready(),
    .m0_rdata  (ic_mem_rdata),
    .m0_rvalid (ic_mem_valid),
    .m0_rready (1'b1),
    .m1_araddr (dc_mem_addr),
    .m1_arvalid(dc_mem_rd),
    .m1_arlen  (dc_mem_arlen),
    .m1_arready(),
    .m1_rdata  (dc_mem_rdata),
    .m1_rvalid (dc_mem_valid),
    .m1_rready (1'b1),
    .m1_awaddr (dc_mem_addr),
    .m1_awvalid(dc_mem_wr),
    .m1_awready(),
    .m1_wdata  (dc_mem_wdata),
    .m1_wstrb  (dc_mem_wstrb),
    .m1_wvalid (dc_mem_wr),
    .m1_wready (),
    .m1_bvalid (),
    .m1_bready (1'b1),
    .m1_wr_done(dc_mem_wr_done),
    .s0_araddr (s0_araddr), .s0_arvalid(s0_arvalid), .s0_arready(s0_arready),
    .s0_arlen  (s0_arlen),
    .s0_rdata  (s0_rdata),  .s0_rvalid (s0_rvalid),  .s0_rlast  (s0_rlast),
    .s0_rready (s0_rready),
    .s0_awaddr (s0_awaddr), .s0_awvalid(s0_awvalid), .s0_awready(s0_awready),
    .s0_wdata  (s0_wdata),  .s0_wstrb  (s0_wstrb),   .s0_wvalid (s0_wvalid),
    .s0_wready (s0_wready), .s0_bvalid (s0_bvalid),  .s0_bready (s0_bready),
    .s1_araddr (s1_araddr), .s1_arvalid(s1_arvalid), .s1_arready(s1_arready),
    .s1_rdata  (s1_rdata),  .s1_rvalid (s1_rvalid),  .s1_rready (s1_rready),
    .s1_awaddr (s1_awaddr), .s1_awvalid(s1_awvalid), .s1_awready(s1_awready),
    .s1_wdata  (s1_wdata),  .s1_wstrb  (s1_wstrb),   .s1_wvalid (s1_wvalid),
    .s1_wready (s1_wready), .s1_bvalid (s1_bvalid),  .s1_bready (s1_bready)
  );

  // ══════════════════════════════════════════════════════════════════════
  // Unified SRAM (Slave 0)
  // ══════════════════════════════════════════════════════════════════════
  axi_sram #(
    .DEPTH     (MEM_DEPTH),
    .INIT_FILE (INIT_FILE)
  ) u_sram (
    .clk     (clk),     .rst_n   (rst_n),
    .araddr  (s0_araddr), .arvalid(s0_arvalid), .arready(s0_arready),
    .arlen   (s0_arlen),
    .rdata   (s0_rdata),  .rvalid (s0_rvalid),  .rlast  (s0_rlast),
    .rready  (s0_rready),
    .awaddr  (s0_awaddr), .awvalid(s0_awvalid), .awready(s0_awready),
    .wdata   (s0_wdata),  .wstrb  (s0_wstrb),   .wvalid (s0_wvalid),
    .wready  (s0_wready), .bvalid (s0_bvalid),  .bready (s0_bready)
  );

  // ══════════════════════════════════════════════════════════════════════
  // Peripheral Bridge (Slave 1)
  // ══════════════════════════════════════════════════════════════════════
  axi_periph_bridge u_pbridge (
    .clk     (clk),      .rst_n    (rst_n),
    .araddr  (s1_araddr), .arvalid (s1_arvalid), .arready(s1_arready),
    .rdata   (s1_rdata),  .rvalid  (s1_rvalid),  .rready (s1_rready),
    .awaddr  (s1_awaddr), .awvalid (s1_awvalid), .awready(s1_awready),
    .wdata   (s1_wdata),  .wstrb   (s1_wstrb),   .wvalid (s1_wvalid),
    .wready  (s1_wready), .bvalid  (s1_bvalid),  .bready (s1_bready),
    .gpio_addr(gpio_addr), .gpio_rd(gpio_rd), .gpio_wr(gpio_wr),
    .gpio_wdata(gpio_wdata_b), .gpio_rdata(gpio_rdata_b),
    .uart_addr(uart_addr), .uart_rd(uart_rd), .uart_wr(uart_wr),
    .uart_wdata(uart_wdata_b), .uart_rdata(uart_rdata_b),
    .timer_addr(timer_addr), .timer_rd(timer_rd), .timer_wr(timer_wr),
    .timer_wdata(timer_wdata_b), .timer_rdata(timer_rdata_b),
    .accel_addr(accel_addr), .accel_rd(accel_rd), .accel_wr(accel_wr),
    .accel_wdata(accel_wdata_b), .accel_rdata(accel_rdata_b)
  );

  // ══════════════════════════════════════════════════════════════════════
  // Peripherals
  // ══════════════════════════════════════════════════════════════════════
  gpio u_gpio (
    .clk(clk), .rst_n(rst_n), .addr(gpio_addr), .wr_en(gpio_wr), .rd_en(gpio_rd),
    .wdata(gpio_wdata_b), .rdata(gpio_rdata_b), .gpio_in(gpio_in), .gpio_out(gpio_out),
    .irq(gpio_irq)
  );

  uart u_uart (
    .clk(clk), .rst_n(rst_n), .addr(uart_addr), .wr_en(uart_wr), .rd_en(uart_rd),
    .wdata(uart_wdata_b), .rdata(uart_rdata_b), .uart_tx(uart_tx), .uart_rx(uart_rx),
    .irq(uart_irq)
  );

  timer u_timer (
    .clk(clk), .rst_n(rst_n), .addr(timer_addr), .wr_en(timer_wr), .rd_en(timer_rd),
    .wdata(timer_wdata_b), .rdata(timer_rdata_b), .irq(timer_irq)
  );

  accel_mmio u_accel (
    .clk(clk), .rst_n(rst_n), .addr(accel_addr),
    .wr_en(accel_wr), .rd_en(accel_rd),
    .wdata(accel_wdata_b), .rdata(accel_rdata_b)
  );

endmodule
