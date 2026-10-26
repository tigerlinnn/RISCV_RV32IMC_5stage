// ============================================================================
// icache.v — 2-Way Set-Associative Instruction Cache
// ============================================================================

module icache #(
  parameter SETS       = 64,
  parameter WAYS       = 2,
  parameter LINE_BYTES = 16   // 4 words per line
)(
  input  wire        clk,
  input  wire        rst_n,

  // fence.i 作废请求（电平输入，内部做边沿检测）
  input  wire        inv,

  // CPU interface
  input  wire [31:0] addr,
  input  wire        rd_en,
  output reg  [31:0] rdata,
  output reg         ready,

  // Memory interface (simplified: word-at-a-time fill)
  output reg  [31:0] mem_addr,
  output reg         mem_rd,
  output reg  [3:0]  mem_arlen,   // RISCV_1: 突发长度-1 (一次读回整行)
  input  wire [31:0] mem_rdata,
  input  wire        mem_valid
);

  localparam OFFSET_W = $clog2(LINE_BYTES);   // 4 bits
  localparam SET_W    = $clog2(SETS);          // 6 bits
  localparam TAG_W    = 32 - SET_W - OFFSET_W; // 22 bits
  localparam WORDS    = LINE_BYTES / 4;        // 4
  localparam [3:0] BURST_LEN = WORDS - 1;      // RISCV_1: 行填充突发长度-1

  // ── Storage ──────────────────────────────────────────────────────────
  reg [TAG_W-1:0]    tag_mem  [0:SETS-1][0:WAYS-1];
  reg                valid_mem[0:SETS-1][0:WAYS-1];
  reg                lru      [0:SETS-1]; // 0 = way0 is LRU
  // 数据阵列拍平成一维：综合工具不支持把三维数组映射成 RAM
  // （Vivado 日志原文: "3D RAM data_mem_reg ... is not supported"），
  // 拍平后写同步、读组合，会被推断成分布式 RAM(LUTRAM)，只占几百个查找表。
  // 索引 = (set*WAYS + way)*WORDS + word
  localparam IDX_W = $clog2(SETS*WAYS*WORDS);
  reg [31:0]         data_mem [0:SETS*WAYS*WORDS-1];

  // ── Address decomposition ────────────────────────────────────────────
  wire [TAG_W-1:0]    tag;
  wire [SET_W-1:0]    set;
  wire [OFFSET_W-3:0] word_sel;

  assign tag      = addr[31:SET_W+OFFSET_W];
  assign set      = addr[SET_W+OFFSET_W-1:OFFSET_W];
  assign word_sel = addr[OFFSET_W-1:2];

  // ── Hit detection ────────────────────────────────────────────────────
  wire hit_way0, hit_way1, hit;
  assign hit_way0 = valid_mem[set][0] && (tag_mem[set][0] == tag);
  assign hit_way1 = valid_mem[set][1] && (tag_mem[set][1] == tag);
  assign hit      = hit_way0 || hit_way1;

  // ── 状态编码（提前声明，供阵列索引使用）──────────────────────────────
  localparam [1:0] S_IDLE = 2'd0, S_FILL = 2'd1, S_DONE = 2'd2;
  reg [1:0] state;
  reg [1:0] fill_word_cnt;
  reg       fill_way;
  reg       ic_dm_we;        // 数据阵列写使能（组合产生）
  reg [31:0] fill_base_addr; // 填充起始地址（锁存），填充期间不变

  // ── 阵列索引（单一读地址，替代原来"两路都读出来再二选一"）────────────
  wire [IDX_W-1:0] set_base = set * WAYS * WORDS;
  wire [IDX_W-1:0] rd_idx   = set_base + (hit_way0 ? 0 : WORDS) + word_sel;

  // RISCV_1 修复: 填充一行要花好几个周期, 这期间 PC 可能被跳转/分支纠正改掉。
  // 如果填充时用"当前 addr"去算 tag/组号, 就会出现"旧行的数据 + 新地址的 tag"
  // 这种自相矛盾的行, 之后被当成命中把错误指令交给 CPU。
  // 所以填充过程一律用"填充开始时锁存的地址" fill_base_addr 来算组号/tag,
  // 保证写进缓存的那一行是自洽的; 当前地址确实命中才交给 CPU(见 S_DONE)。
  wire [SET_W-1:0]    fill_set  = fill_base_addr[SET_W+OFFSET_W-1:OFFSET_W];
  wire [TAG_W-1:0]    fill_tag  = fill_base_addr[31:SET_W+OFFSET_W];
  wire [IDX_W-1:0]    fill_base = fill_set * WAYS * WORDS;
  wire [IDX_W-1:0]    fill_idx  = fill_base + fill_way * WORDS + fill_word_cnt;
  wire [IDX_W-1:0]    done_idx  = fill_base + fill_way * WORDS + word_sel;

  // ── FSM ──────────────────────────────────────────────────────────────

  integer s, w;

  // ── fence.i 作废：边沿检测 + 代数保护 ───────────────────────────────
  // inv 是电平信号（可能因流水线停顿保持多拍），这里只在上升沿清一次。
  // 代数(gen)用于作废"作废之前就已经在飞"的那次填充：
  //   旧填充完成时若发现代数变了，就不许它把 tag/valid 写回去，
  //   否则会把作废前读到的旧指令又变成"有效"。
  reg        inv_q;
  wire       inv_pulse = inv & ~inv_q;
  reg [7:0]  gen;
  reg [7:0]  fill_gen;

  always @(*) begin
    ready  = 1'b0;
    rdata  = 32'b0;
    mem_rd = 1'b0;
    mem_arlen = 4'd0;
    mem_addr = 32'b0;

    // 数据阵列写使能（组合）：只在填充时写一次
    ic_dm_we = (state == S_FILL) && mem_valid;

    case (state)
      S_IDLE: begin
        if (rd_en && hit) begin
          ready = 1'b1;
          rdata = data_mem[rd_idx];
        end else if (rd_en && !hit) begin
          ready = 1'b0;
        end else begin
          ready = 1'b1;
        end
      end
      S_FILL: begin
        mem_rd   = 1'b1;
        // RISCV_1: 一次突发把整行读回来(原来是一个字一个字的单拍请求)
        mem_addr  = {fill_base_addr[31:OFFSET_W], {OFFSET_W{1'b0}}};
        mem_arlen = BURST_LEN;
        ready    = 1'b0;
      end
      S_DONE: begin
        // RISCV_1 修复: 只有当"当前取指地址"确实命中刚填好的那一行时才给出数据,
        // 否则让取指继续停住, 回到 S_IDLE 按新地址重新填充。
        ready = hit;
        rdata = data_mem[rd_idx];
      end
      default: begin
        ready = 1'b0;
      end
    endcase
  end

  // ── 数据阵列：单独的干净同步块（没有复位）─────────────────────────────
  //     写同步、读组合 —— 这样综合工具才会把它推断成分布式 RAM，
  //     旧版把写语句放在带 `negedge rst_n` 的块里，工具直接拆成触发器。
  always @(posedge clk) begin
    if (ic_dm_we)
      data_mem[fill_idx] <= mem_rdata;
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      inv_q         <= 1'b0;
      gen           <= 8'b0;
      state         <= S_IDLE;
      fill_word_cnt <= 2'b0;
      fill_way      <= 1'b0;
      fill_base_addr <= 32'b0;
      for (s = 0; s < SETS; s = s + 1) begin
        for (w = 0; w < WAYS; w = w + 1) begin
          valid_mem[s][w] <= 1'b0;
          tag_mem[s][w]   <= 0;
        end
        lru[s] <= 1'b0;
      end
    end else begin
      inv_q <= inv;
      if (inv_pulse) begin
        gen <= gen + 8'd1;
        for (s = 0; s < SETS; s = s + 1) begin
          for (w = 0; w < WAYS; w = w + 1) begin
            valid_mem[s][w] <= 1'b0;
          end
        end
      end
      case (state)
        S_IDLE: begin
          if (rd_en && hit) begin
            lru[set] <= hit_way0 ? 1'b1 : 1'b0;
          end else if (rd_en && !hit) begin
            fill_way       <= lru[set] ? 1'b1 : 1'b0;
            fill_word_cnt  <= 2'b0;
            fill_base_addr <= addr;
            fill_gen       <= gen;
            state          <= S_FILL;
          end
        end

        S_FILL: begin
          if (mem_valid) begin
            if (fill_word_cnt == 2'd3) begin
              // 只有"没被 fence.i 作废过"的填充才许生效
              if (fill_gen == gen) begin
                tag_mem[fill_set][fill_way]   <= fill_tag;   // 用锁存的地址算 tag
                valid_mem[fill_set][fill_way] <= 1'b1;
                lru[fill_set]                 <= fill_way ? 1'b0 : 1'b1;
              end
              state                    <= S_DONE;
            end else begin
              fill_word_cnt <= fill_word_cnt + 1'b1;
            end
          end
        end

        S_DONE: begin
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
