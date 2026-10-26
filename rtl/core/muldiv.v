// ============================================================================
// muldiv.v — M 扩展：乘法（单周期） + 除法（迭代）
// ----------------------------------------------------------------------------
// RISCV_1 说明：原版的除法器有两处错误（33 位比较用错切片、迭代 33 次而不是 32 次），
// 导致 div/divu/rem/remu 结果全错（只有乘法是对的）。
// 这里把除法部分按标准"恢复余数法"重写，乘法部分一行未改：
//
//   每拍： R = (R << 1) | (Q >> 31)     // 余数左移，商最高位拉进余数
//          Q = Q << 1
//          若 R >= D 则 R = R - D, Q[0] = 1
//   共 32 拍（每次产生 1 位商）。R 用 33 位以容纳移位后的值。
//
//   符号处理：按规范，有符号除法先取绝对值运算，最后按符号修正：
//     商为负当且仅当两操作数异号；余数符号跟随被除数。
//   除零特例（规范明确）：div/divu 得 -1（0xFFFFFFFF）；rem/remu 得被除数本身。
// ============================================================================

module muldiv (
  input  wire        clk,
  input  wire        rst_n,

  input  wire        start,
  input  wire [2:0]  op,
  input  wire [31:0] a,          // rs1
  input  wire [31:0] b,          // rs2
  output reg  [31:0] result,
  output reg         busy,
  output reg         valid       // Result ready this cycle
);

  `include "brv32p_defs.vh"

  // ── 乘法（单周期，保持原实现）─────────────────────────────────────────────
  // RISCV_1 修改(时序): 乘法由"单周期组合直通"改成"两拍"。
  //   原来 rs1_fwd/rs2_fwd 一路穿过 3 个 32x32 乘法器 + 结果 mux, 而这两个操作数
  //   前面还可能挂着 D-Cache 读数据(load 前递)。这一整条 = 全局最差路径,
  //   综合实现后 WNS = -0.060ns, 5 个违例端点全在乘法器上。
  //   现在: 第 1 拍把操作数锁进 a_q/b_q (busy=1, EX 被冻住),
  //         第 2 拍用寄存器里的操作数做组合乘法 (busy=1, result 有效)。
  //   最差路径只剩 DSP 乘法本身, 余量很大。
  //   代价: 每条乘法多花 1 个时钟周期 —— 复用除法器已有的 muldiv_busy 停顿机制,
  //         CPU 流水线不需要任何改动。
  reg [31:0] a_q, b_q;
  reg        mul_run;              // 1 = 操作数已锁存, 这一拍出结果

  wire signed [63:0] mul_ss;
  wire signed [63:0] mul_su;
  wire        [63:0] mul_uu;

  assign mul_ss = $signed(a_q) * $signed(b_q);
  assign mul_su = $signed(a_q) * $signed({1'b0, b_q});
  assign mul_uu = $unsigned(a_q) * $unsigned(b_q);

  wire is_mul = (op == MD_MUL || op == MD_MULH || op == MD_MULHSU || op == MD_MULHU);
  wire is_div = (op == MD_DIV || op == MD_DIVU || op == MD_REM || op == MD_REMU);
  wire is_rem = (op == MD_REM || op == MD_REMU);
  wire is_sgn = (op == MD_DIV || op == MD_REM);

  // ── 除法状态 ─────────────────────────────────────────────────────────────
  reg        active;
  reg  [5:0] cnt;
  reg [32:0] r_reg;              // 余数（33 位）
  reg [31:0] q_reg;              // 商
  reg [31:0] d_reg;              // 除数绝对值
  reg        neg_q, neg_r;       // 结果符号
  reg        rem_op;             // 1 = 取余
  reg        zero_div;           // 除零
  reg [31:0] orig_a;             // 原被除数（除零时的余数）

  wire [32:0] r_sh = {r_reg[31:0], q_reg[31]};      // 左移一位并拉进商最高位
  wire [32:0] r_sb = r_sh - {1'b0, d_reg};          // 试减
  wire        ge   = ~r_sb[32];                     // 1 = 够减

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      active  <= 1'b0;  cnt    <= 6'd0;
      r_reg   <= 33'd0; q_reg  <= 32'd0; d_reg <= 32'd0;
      neg_q   <= 1'b0;  neg_r  <= 1'b0;
      rem_op  <= 1'b0;  zero_div <= 1'b0; orig_a <= 32'd0;
      a_q     <= 32'd0; b_q      <= 32'd0; mul_run <= 1'b0;
    end else if (start && !active && is_div) begin
      active   <= 1'b1;
      cnt      <= 6'd0;
      r_reg    <= 33'd0;
      q_reg    <= is_sgn ? (a[31] ? (~a + 1'b1) : a) : a;   // 被除数取绝对值
      d_reg    <= is_sgn ? (b[31] ? (~b + 1'b1) : b) : b;   // 除数取绝对值
      neg_q    <= is_sgn && (a[31] ^ b[31]);                // 商为负 <=> 异号
      neg_r    <= is_sgn && a[31];                          // 余数符号跟随被除数
      rem_op   <= is_rem;
      zero_div <= (b == 32'd0);
      orig_a   <= a;
    end else if (start && !active && !mul_run && is_mul) begin
      // RISCV_1: 乘法第 1 拍 —— 先把(转发之后的)操作数锁住, 下一拍再算
      a_q     <= a;
      b_q     <= b;
      mul_run <= 1'b1;
    end else if (active) begin
      if (cnt == 6'd32) begin
        active <= 1'b0;                                     // 32 拍算完，收工
      end else begin
        r_reg <= ge ? r_sb[32:0] : r_sh;
        q_reg <= {q_reg[30:0], ge};
        cnt   <= cnt + 1'b1;
      end
    end else if (mul_run) begin
      mul_run <= 1'b0;
    end
  end

  wire div_done = active && (cnt == 6'd32);

  // ── 结果（含符号修正与除零特例）──────────────────────────────────────────
  wire [31:0] q_fin = neg_q ? (~q_reg + 1'b1) : q_reg;
  wire [31:0] r_fin = neg_r ? (~r_reg[31:0] + 1'b1) : r_reg[31:0];

  reg [31:0] div_result;
  always @(*) begin
    if (zero_div)
      div_result = rem_op ? orig_a : 32'hFFFF_FFFF;         // 除零：余数=被除数，商=-1
    else
      div_result = rem_op ? r_fin : q_fin;
  end

  // ── 输出 MUX ─────────────────────────────────────────────────────────────
  always @(*) begin
    valid  = 1'b0;
    // RISCV_1 修复: busy 必须从"启动那一拍"就有效。
    //   原实现 busy = active && !div_done, 而 active 要等下一拍才置起 —— 于是
    //   启动那一拍 CPU 以为不忙, 除法器还在运行时指令就被放走, 结果写回成了 0。
    // RISCV_1: 乘法 busy 只在"锁操作数"那一拍拉高。
    //   下一拍(mul_run=1) busy 已经拉低, 但乘法结果(t=寄存器里的 a_q/b_q 组合乘出来)
    //   就在这一拍有效 —— 和除法器 div_done 那一拍的写法完全一致:
    //   busy=0 与 result 有效同拍, EX/MEM 正好在这拍把结果锁走。
    busy   = ((start && !active && !mul_run && is_div) || (active && !div_done) ||
              (start && !active && !mul_run && is_mul));
    result = 32'b0;

    if (mul_run) begin
      valid = 1'b1;
      case (op)
        MD_MUL:    result = mul_ss[31:0];
        MD_MULH:   result = mul_ss[63:32];
        MD_MULHSU: result = mul_su[63:32];
        MD_MULHU:  result = mul_uu[63:32];
        default:   result = 32'b0;
      endcase
    end else if (div_done) begin
      valid  = 1'b1;
      result = div_result;
    end
  end

endmodule
