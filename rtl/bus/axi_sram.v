// ============================================================================
// axi_sram.v - AXI4-Lite SRAM Slave (Unified backing memory)
// ----------------------------------------------------------------------------
// RISCV_1 版本，相对例程的改动：
//
//  1) 地址窗口译码
//     旧版直接用 addr[AW+1:2] 当索引、忽略地址高位，于是 0x1000_0000 会折回到第 0
//     个字，把 0x00 处的指令覆盖掉（下载后能跑、一按复位就执行到被破坏的指令）。
//     现在只有落在 [BASE_ADDR, BASE_ADDR+DEPTH*4) 窗口内的访问才真正读写存储器，
//     窗口外返回 0 / 丢弃写入，并照常给出 AXI 应答（CPU 不会挂死）。
//
//  2) 存储器阵列写成"干净同步块"（无复位），同步写 + 同步读 →
//     综合成块 RAM（BRAM）。旧版把存储器写在带 `negedge rst_n` 的块里，
//     Vivado 日志原文：RAM is sensitive to asynchronous reset signal. this RTL
//     style is not supported. —— 整个存储器被拆成触发器。
//
//  3) 不把带异步复位的寄存器接到 BRAM 的地址脚
//     实现阶段的 DRC(REQP-1839) 警告：若 RAMB 的地址/控制脚由"异步复位寄存器"
//     驱动，复位期间可能破坏存储器内容。因此这里：
//       - 本模块所有状态机改用同步复位；
//       - 读地址在本模块内部单独打一拍（无复位寄存器 rd_addr_q），
//         这样 BRAM 的读地址脚只由本地干净寄存器驱动，不直接受上游缓存影响。
//     代价是读延迟由 2 拍变成 3 拍（IDLE -> WAIT -> RESP），功能不变。
// ============================================================================

module axi_sram #(
  parameter        DEPTH     = 8192,           // 32bit 字数 -> 32KB
  parameter [31:0] BASE_ADDR = 32'h0000_0000,  // 本 slave 的地址窗口基址
  parameter        INIT_FILE = "firmware.hex"
)(
  input  wire        clk,
  input  wire        rst_n,

  // AXI4-Lite slave: Read
  input  wire [31:0] araddr,
  input  wire        arvalid,
  input  wire [3:0]  arlen,      // RISCV_1: 突发长度-1 (0=单拍, Cache 行填充时=行内字数-1)
  output wire        arready,
  output wire [31:0] rdata,
  output wire        rvalid,
  output wire        rlast,      // RISCV_1: 突发最后一拍
  input  wire        rready,

  // AXI4-Lite slave: Write
  input  wire [31:0] awaddr,
  input  wire        awvalid,
  output wire        awready,
  input  wire [31:0] wdata,
  input  wire [3:0]  wstrb,
  input  wire        wvalid,
  output wire        wready,
  output wire        bvalid,
  input  wire        bready
);

  localparam AW = $clog2(DEPTH);
  localparam [31:0] LIMIT_ADDR = BASE_ADDR + DEPTH*4;   // 窗口上界(不含)
  localparam [1:0]  RD_IDLE = 2'd0, RD_WAIT = 2'd1, RD_RESP = 2'd2;
  localparam [1:0]  WR_IDLE = 2'd0, WR_RESP = 2'd1;

  function in_window;
    input [31:0] a;
    begin
      in_window = (a >= BASE_ADDR) && (a < LIMIT_ADDR);
    end
  endfunction

  // ---------------------------------------------------------------------------
  // 存储器阵列 + 干净同步块（无复位）
  //   写：同步写，4 位字节使能
  //   读：读地址打一拍、读数据打一拍（标准同步读模板）
  // ---------------------------------------------------------------------------
  reg [31:0]   mem [0:DEPTH-1];
  reg [AW-1:0] rd_addr_q;      // 读地址寄存器 —— 无复位，只由时钟驱动
  reg [31:0]   rd_data_q;      // 读数据寄存器 —— 无复位

  // 写侧信号（由写状态机给出；状态机用同步复位）
  reg  [1:0]  wr_state;
  reg         wr_in_range_q;
  reg  [31:0] wr_addr_q;
  reg  [31:0] wr_data_q;
  reg  [3:0]  wr_strb_q;

  wire [AW-1:0] wr_index_w = wr_addr_q[AW+1:2];
  wire          wr_en_w    = (wr_state == WR_RESP) && wr_in_range_q;

  integer init_i;
  integer wr_i;

  // RISCV_1: 读通道状态/计数提前声明 —— 下面的读地址逻辑要用到它们
  reg [1:0] rd_state;
  reg       rd_in_range_q;
  reg [3:0] rd_cnt;            // 突发剩余拍数

  initial begin
    for (init_i = 0; init_i < DEPTH; init_i = init_i + 1)
      mem[init_i] = 32'h0000_0013; // NOP
    if (INIT_FILE != "")
      $readmemh(INIT_FILE, mem);
  end

  always @(posedge clk) begin
    // RISCV_1: 读地址不再无条件跟随 araddr，而是"IDLE 收请求地址 / 突发中自增"
    //   注意: 数据寄存器 rd_data_q 比 rd_addr_q 又晚一拍(同步读), 所以地址必须
    //   "提前一拍"推进 —— 在 WAIT 那一拍就要把地址指向下一个字, 否则每个 beat
    //   都会晚一个字(第一拍重复、最后一拍丢失), 整行数据错位。
    if (rd_state == RD_IDLE)
      rd_addr_q <= araddr[AW+1:2];
    else if ((rd_state == RD_WAIT) && (rd_cnt != 4'd0))
      rd_addr_q <= rd_addr_q + 1'b1;
    else if ((rd_state == RD_RESP) && rready && (rd_cnt != 4'd0))
      rd_addr_q <= rd_addr_q + 1'b1;
    rd_data_q <= mem[rd_addr_q];
    if (wr_en_w)
      for (wr_i = 0; wr_i < 4; wr_i = wr_i + 1)
        if (wr_strb_q[wr_i])
          mem[wr_index_w][wr_i*8 +: 8] <= wr_data_q[wr_i*8 +: 8];
  end

  // ---------------------------------------------------------------------------
  // 读通道状态机（同步复位）
  //   拍 N  : IDLE 收下 arvalid（组合地址当拍送进 rd_addr_q）
  //   拍 N+1: WAIT（rd_addr_q 已更新，rd_data_q 下一拍有效）
  //   拍 N+2: RESP 给出 rvalid/rdata
  // ---------------------------------------------------------------------------
  assign arready = (rd_state == RD_IDLE);
  assign rvalid  = (rd_state == RD_RESP);
  assign rlast   = (rd_state == RD_RESP) && (rd_cnt == 4'd0);
  assign rdata   = rd_in_range_q ? rd_data_q : 32'b0;

  always @(posedge clk) begin
    if (!rst_n) begin
      rd_state      <= RD_IDLE;
      rd_in_range_q <= 1'b0;
      rd_cnt        <= 4'd0;
    end else begin
      case (rd_state)
        RD_IDLE: begin
          if (arvalid) begin
            rd_in_range_q <= in_window(araddr);
            rd_cnt        <= arlen;
            rd_state      <= RD_WAIT;
          end
        end
        RD_WAIT: rd_state <= RD_RESP;
        RD_RESP: begin
          if (rready) begin
            if (rd_cnt == 4'd0)
              rd_state <= RD_IDLE;          // 单拍, 或突发的最后一拍
            else
              rd_cnt <= rd_cnt - 1'b1;      // 突发: 继续下一拍
          end
        end
        default: rd_state <= RD_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // 写通道状态机（同步复位）
  //   拍 N  : IDLE 收下 awvalid+wvalid，锁存地址/数据/字节使能
  //   拍 N+1: RESP 给出 bvalid，同时把数据写进存储器
  // ---------------------------------------------------------------------------
  assign awready = (wr_state == WR_IDLE) && awvalid && wvalid;
  assign wready  = (wr_state == WR_IDLE) && awvalid && wvalid;
  assign bvalid  = (wr_state == WR_RESP);

  always @(posedge clk) begin
    if (!rst_n) begin
      wr_state      <= WR_IDLE;
      wr_in_range_q <= 1'b0;
      wr_addr_q     <= 32'b0;
      wr_data_q     <= 32'b0;
      wr_strb_q     <= 4'b0;
    end else begin
      case (wr_state)
        WR_IDLE: begin
          if (awvalid && wvalid) begin
            wr_addr_q     <= awaddr;
            wr_data_q     <= wdata;
            wr_strb_q     <= wstrb;
            wr_in_range_q <= in_window(awaddr);
            wr_state      <= WR_RESP;
          end
        end
        WR_RESP: begin
          if (bready) wr_state <= WR_IDLE;
        end
        default: wr_state <= WR_IDLE;
      endcase
    end
  end

endmodule
