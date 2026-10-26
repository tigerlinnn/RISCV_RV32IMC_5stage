// ============================================================================
// regfile.v — 32x32 Register File for BRV32P
// ============================================================================
// Two read ports (combinational), one write port (synchronous).
// x0 hardwired to zero. No internal forwarding (handled by hazard unit).
// ============================================================================

module regfile (
  input  wire        clk,
  input  wire        rst_n,

  // Read ports (combinational)
  input  wire [4:0]  rs1_addr,
  output wire [31:0] rs1_data,
  input  wire [4:0]  rs2_addr,
  output wire [31:0] rs2_data,

  // Write port (synchronous, rising edge)
  input  wire        wr_en,
  input  wire [4:0]  rd_addr,
  input  wire [31:0] rd_data
);

  reg [31:0] regs [1:31];

  // Read port A
  // RISCV_1 修复: 同拍"写穿"(write-through)。
  // 本研究原方案是"ID 级读寄存器 + EX 级前递", 它有一个正好差 3 条指令的
  // RAW 冲突空洞: 消费者在 ID 读寄存器时, 生产者恰好停在 WB 级(数据还没写进
  // 寄存器堆), 等消费者到 EX 级时生产者已退休、前递来不及 —— 于是读到旧值。
  // 让读口在"本拍正在写同一寄存器"时直接旁路写数据, 即可补上这个空洞。
  wire wr_same_rs1 = wr_en && (rd_addr != 5'd0) && (rd_addr == rs1_addr);
  wire wr_same_rs2 = wr_en && (rd_addr != 5'd0) && (rd_addr == rs2_addr);

  assign rs1_data = (rs1_addr == 5'd0) ? 32'd0 :
                    (wr_same_rs1 ? rd_data : regs[rs1_addr]);

  // Read port B
  assign rs2_data = (rs2_addr == 5'd0) ? 32'd0 :
                    (wr_same_rs2 ? rd_data : regs[rs2_addr]);

  // Write
  integer i;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (i = 1; i < 32; i = i + 1)
        regs[i] <= 32'd0;
    end else if (wr_en && rd_addr != 5'd0) begin
      regs[rd_addr] <= rd_data;
    end
  end

endmodule
