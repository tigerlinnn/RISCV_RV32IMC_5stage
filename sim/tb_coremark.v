`timescale 1ns / 1ps
// ============================================================================
// tb_coremark.v — 在仿真里跑 CoreMark，把串口输出的字符打印出来
// ----------------------------------------------------------------------------
// 思路：不去解码 uart_tx 的串行波形（那要按波特率逐位采样，又慢又容易错），
//       而是直接监听【往 UART 发送寄存器写一个字节】这个动作 ——
//       每写一次就是一个字符，直接打印即可。
//
// 用法：把 CoreMark 编译出的 firmware.hex 放到仿真目录，编译本文件即可。
//
// 判定：CoreMark 跑完会打印 "Correct operation validated."，看到这句就是对
//       （它内部把四个 CRC 和官方预期值比对过了）。
// ============================================================================
module tb_coremark;

  reg         clk = 0;
  reg         rst_n;
  reg  [31:0] gpio_in;
  wire [31:0] gpio_out;
  reg         uart_rx;
  wire        uart_tx;

  // RISCV_1: mid-run probes are OFF by default.  They are $display'ed while
  //   the UART character stream is still running, so they get mixed into the
  //   captured text and the report post-processing used to delete real
  //   characters together with the probe lines.  Turn this on only when
  //   debugging a hang.
  localparam integer PROBE_EN = 0;

  integer cycles;
  integer quiet;              // 连续多少个周期没有新字符
  integer i;

  localparam integer MAX_CYCLES  = 8000000;    // 上限：8M 周期
  localparam integer QUIET_LIMIT = 3000000;    // 3M 周期没输出就停

  // 板上是 47.6MHz（周期 21ns），仿真里用 20ns 近似即可 —— 只影响速度，不影响功能
  always #10 clk = ~clk;

  brv32p_soc #(
    .MEM_DEPTH (8192),
    .INIT_FILE ("firmware.hex")
  ) dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .gpio_in  (gpio_in),
    .gpio_out (gpio_out),
    .uart_rx  (uart_rx),
    .uart_tx  (uart_tx)
  );

  initial begin
    rst_n   = 0;
    gpio_in = 32'h0;
    uart_rx = 1'b1;
    #100;
    rst_n = 1;
  end

  // ---- 监听 UART 的每一次"发送字符"动作，直接打印 ----
  always @(posedge clk) begin
    if (rst_n && dut.u_uart.wr_en && (dut.u_uart.addr[3:2] == 2'd0)) begin
      $write("%c", dut.u_uart.wdata[7:0]);
      $fflush;
    end
  end

  // RISCV_1 新增: 串口字符滑动窗口 —— 一看到 "validated" 就说明 CoreMark 已经
  //   打印完 "Correct operation validated."。此时立刻收工, 不用再等 3M 周期的
  //   "静默超时"(那条尾等待要白跑一两分钟)。
  reg [71:0] uart_tail;
  reg        cm_seen_ok;
  initial begin
    uart_tail  = 72'd0;
    cm_seen_ok = 1'b0;
  end
  always @(posedge clk) begin
    if (rst_n && dut.u_uart.wr_en && (dut.u_uart.addr[3:2] == 2'd0)) begin
      uart_tail <= {uart_tail[63:0], dut.u_uart.wdata[7:0]};
      if ({uart_tail[63:0], dut.u_uart.wdata[7:0]} == "validated")
        cm_seen_ok <= 1'b1;
    end
  end

  // ---- 结束判定 ----
  // ---- 卡死时的连续采样探针 ----
  // ---- 关键判别：UART 到底有没有被写过？PC 最远走到哪？ ----
  reg        uart_any_wr;
  reg [31:0] pc_max;
  reg [31:0] uart_first_addr;
  integer    trap_cnt;
  initial begin
    uart_any_wr     = 1'b0;
    pc_max          = 32'h0;
    uart_first_addr = 32'h0;
    trap_cnt        = 0;
  end
  always @(posedge clk) begin
    if (rst_n && dut.u_core.trap_enter) trap_cnt = trap_cnt + 1;
    if (rst_n && dut.u_uart.wr_en) begin
      if (!uart_any_wr) begin
        uart_first_addr = dut.u_uart.addr;
        $display("[U] UART first write! addr=0x%h data=0x%h", dut.u_uart.addr, dut.u_uart.wdata);
      end
      uart_any_wr = 1'b1;
    end
    if (dut.u_core.pc_if > pc_max) pc_max = dut.u_core.pc_if;
  end

  reg [31:0] last_pc;
  integer    stuck, shown;
  initial begin
    last_pc = 32'hFFFFFFFF;
    stuck   = 0;
    shown   = 0;
  end
  always @(posedge clk) begin
    if (rst_n) begin
      if (dut.u_core.pc_if === last_pc)
        stuck = stuck + 1;
      else begin
        stuck   = 0;
        last_pc = dut.u_core.pc_if;
      end
      if (PROBE_EN && (stuck > 30) && (shown < 50)) begin
        $display("[S] arb=%0d s0aw=%b s0wv=%b s0bv=%b s0br=%b | dc=%0d wbuf=%b wb2=%b mrd=%b mwr=%b mdone=%b",
                 dut.u_axi.arb_state,
                 dut.u_axi.s0_awvalid, dut.u_axi.s0_wvalid,
                 dut.u_axi.s0_bvalid,  dut.u_axi.s0_bready,
                 dut.u_dcache.state,   dut.u_dcache.wbuf_valid,
                 dut.u_dcache.wbuf2_valid,
                 dut.u_dcache.mem_rd,  dut.u_dcache.mem_wr,
                 dut.u_dcache.mem_wr_done);
        shown = shown + 1;
      end
    end
  end

  initial begin
    cycles = 0;
    quiet  = 0;
    while (cycles < MAX_CYCLES) begin
      @(posedge clk);
      cycles = cycles + 1;
      if (rst_n && dut.u_uart.wr_en && (dut.u_uart.addr[3:2] == 2'd0))
        quiet = 0;
      else
        quiet = quiet + 1;

      // RISCV_1: 见到 "Correct operation validated." 立刻结束仿真
      if (cm_seen_ok) begin
        $display("");
        $display("[TB] CoreMark output : Correct operation validated.");
        $display("[TB] RESULT: PASS   (共用 %0d 个周期)", cycles);
        $finish;
      end

      // 探针：每 2 万周期打一次 PC + 关键寄存器
      //   x2 = sp（栈指针）, x8 = s0（main 里的帧指针）
      if (PROBE_EN && ((cycles % 20000) == 0))
        $display("[PC] cycle=%0d pc=%h dc_st=%0d wbuf=%b ic_st=%0d arb_st=%0d m1aw=%b m1w=%b s0aw=%b s0w=%b s0b=%b wrdone=%b",
                 cycles, dut.u_core.pc_if,
                 dut.u_dcache.state, dut.u_dcache.wbuf_valid,
                 dut.u_icache.state, dut.u_axi.arb_state,
                 dut.u_axi.m1_awvalid, dut.u_axi.m1_wvalid,
                 dut.u_axi.s0_awvalid, dut.u_axi.s0_wvalid, dut.u_axi.s0_bvalid,
                 dut.u_axi.m1_wr_done);

      if (quiet > QUIET_LIMIT) begin
        $display("");
        $display("");
        $display("[TB] 输出已停止 (共用 %0d 个周期)", cycles);
        $display("[TB] UART 被写过吗 = %b  (首次写 addr=0x%h)", uart_any_wr, uart_first_addr);
        $display("[TB] PC 走到过的最大地址 = 0x%h   (ee_printf 在 0x207C)", pc_max);
        $display("[TB] 陷阱发生次数 = %0d   mcause=0x%h  mepc=0x%h  mtvec=0x%h",
                 trap_cnt, dut.u_core.u_csr.mcause, dut.u_core.u_csr.mepc,
                 dut.u_core.u_csr.mtvec);
        $finish;
      end
    end

    $display("");
    $display("[TB] 超时：%0d 个周期内程序没有结束", MAX_CYCLES);
    $display("[TB] 最后的 PC = %h   （可以据此判断卡在哪里）", dut.u_core.pc_if);
    $finish;
  end

endmodule
