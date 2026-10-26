// ============================================================================
// branch_predictor.v — 2-bit Saturating Counter Branch History Table
// ============================================================================
// 256-entry BHT indexed by PC[9:2]. Predicts taken/not-taken.
// BTB (Branch Target Buffer) stores the last known target for taken branches.
// ============================================================================

module branch_predictor #(
  parameter BHT_ENTRIES = 256,
  parameter BTB_ENTRIES = 64
)(
  input  wire        clk,
  input  wire        rst_n,

  // RISCV_1 新增: fence.i 作废请求 —— 自修改代码改了指令以后,
  //   BTB 里可能还留着"旧分支的地址/跳转目标"。若新指令恰好不再是分支,
  //   流水线里没人能发现这次错误预测, 会直接跳飞。
  //   所以 fence.i 时把 BTB 的 valid 位全部清掉(I-Cache 也是同时作废的)。
  input  wire        inv,

  // Prediction request (IF stage)
  input  wire [31:0] pc,
  output wire        pred_taken,
  output wire [31:0] pred_target,
  output wire        pred_valid,    // BTB hit

  // Update from EX stage
  input  wire        update_en,
  input  wire [31:0] update_pc,
  input  wire        update_taken,
  input  wire [31:0] update_target
);

  localparam BHT_IDX_W = $clog2(BHT_ENTRIES);
  localparam BTB_IDX_W = $clog2(BTB_ENTRIES);

  // ── BHT: 2-bit saturating counters ───────────────────────────────────
  reg [1:0] bht [0:BHT_ENTRIES-1];

  wire [BHT_IDX_W-1:0] pred_idx;
  wire [BHT_IDX_W-1:0] upd_idx;

  assign pred_idx = pc[BHT_IDX_W+1:2];
  assign upd_idx  = update_pc[BHT_IDX_W+1:2];

  assign pred_taken = bht[pred_idx][1];  // MSB = prediction

  // ── BTB: direct-mapped target cache ──────────────────────────────────
  // RISCV_1 修复(关键): 原来 index=pc[7:2] + tag=pc[31:8], 两者合起来少了 pc[1]。
  //   加了 RVC(16 位压缩指令)以后, 0x10 和 0x12 是两条【不同】的指令, 却会算出
  //   完全相同的 (index, tag) —— 于是 0x12 上那条分支的预测会套到 0x10 头上。
  //   0x10 是 c.nop, 流水线里没有任何一条指令能发现"错误预测",
  //   结果是 PC 直接跳飞、跳过了后面的 bne —— CoreMark 就是死在这上面。
  //   把 pc[1] 并进 tag, tag+index 就完整覆盖 pc[31:1], 不会再误命中。
  reg [31:0]              btb_target [0:BTB_ENTRIES-1];
  reg [32-BTB_IDX_W-2:0]  btb_tag   [0:BTB_ENTRIES-1];
  reg                      btb_valid [0:BTB_ENTRIES-1];

  wire [BTB_IDX_W-1:0]     btb_pred_idx;
  wire [BTB_IDX_W-1:0]     btb_upd_idx;
  wire [32-BTB_IDX_W-2:0]  btb_pred_tag;
  wire [32-BTB_IDX_W-2:0]  btb_upd_tag;

  assign btb_pred_idx = pc[BTB_IDX_W+1:2];
  assign btb_upd_idx  = update_pc[BTB_IDX_W+1:2];
  assign btb_pred_tag = {pc[31:BTB_IDX_W+2], pc[1]};
  assign btb_upd_tag  = {update_pc[31:BTB_IDX_W+2], update_pc[1]};

  assign pred_valid  = btb_valid[btb_pred_idx] &&
                       (btb_tag[btb_pred_idx] == btb_pred_tag);
  assign pred_target = btb_target[btb_pred_idx];

  // ── Update logic ─────────────────────────────────────────────────────
  integer i;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (i = 0; i < BHT_ENTRIES; i = i + 1)
        bht[i] <= 2'b01;  // Weakly not-taken
      for (i = 0; i < BTB_ENTRIES; i = i + 1) begin
        btb_valid[i]  <= 1'b0;
        btb_target[i] <= 32'b0;
        btb_tag[i]    <= 0;
      end
    end else if (inv) begin
      // RISCV_1: fence.i -> 丢掉所有 BTB 表项(只清 valid, 最省资源)
      for (i = 0; i < BTB_ENTRIES; i = i + 1)
        btb_valid[i] <= 1'b0;
    end else if (update_en) begin
      // BHT update: saturating counter
      if (update_taken) begin
        if (bht[upd_idx] != 2'b11)
          bht[upd_idx] <= bht[upd_idx] + 1'b1;
      end else begin
        if (bht[upd_idx] != 2'b00)
          bht[upd_idx] <= bht[upd_idx] - 1'b1;
      end

      // BTB update: store target on taken branches
      if (update_taken) begin
        btb_valid[btb_upd_idx]  <= 1'b1;
        btb_target[btb_upd_idx] <= update_target;
        btb_tag[btb_upd_idx]    <= btb_upd_tag;
      end
    end
  end

endmodule
