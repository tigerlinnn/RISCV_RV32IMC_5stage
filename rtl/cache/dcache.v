// ============================================================================
// dcache.v — 2-Way Set-Associative Data Cache (Write-Through)
// ============================================================================

module dcache #(
  parameter SETS       = 64,
  parameter WAYS       = 2,
  parameter LINE_BYTES = 16
)(
  input  wire        clk,
  input  wire        rst_n,

  // CPU interface
  input  wire [31:0] addr,
  input  wire        rd_en,
  input  wire        wr_en,
  input  wire [1:0]  width,
  input  wire        sign_ext,
  input  wire [31:0] wdata,
  output reg  [31:0] rdata,
  output reg         ready,

  // Memory interface
  output reg  [31:0] mem_addr,
  output reg         mem_rd,
  output reg  [3:0]  mem_arlen,   // RISCV_1: 突发长度-1 (行填充用, 单字访问=0)
  output reg         mem_wr,
  output reg  [31:0] mem_wdata,
  output reg  [3:0]  mem_wstrb,
  input  wire [31:0] mem_rdata,
  input  wire        mem_valid,
  input  wire        mem_wr_done
);

  `include "brv32p_defs.vh"

  localparam OFFSET_W = $clog2(LINE_BYTES);
  localparam SET_W    = $clog2(SETS);
  localparam TAG_W    = 32 - SET_W - OFFSET_W;
  localparam WORDS    = LINE_BYTES / 4;
  localparam [3:0] BURST_LEN = WORDS - 1;   // RISCV_1: 行填充突发长度-1

  // ── Storage ──────────────────────────────────────────────────────────
  reg [TAG_W-1:0] tag_mem  [0:SETS-1][0:WAYS-1];
  reg             valid_mem[0:SETS-1][0:WAYS-1];
  reg             lru      [0:SETS-1];
  // 数据阵列拍平成一维：三维数组综合工具不支持映射成 RAM，会全部拆成触发器。
  // 索引 = (set*WAYS + way)*WORDS + word
  localparam IDX_W = $clog2(SETS*WAYS*WORDS);
  reg [31:0]      data_mem [0:SETS*WAYS*WORDS-1];

  // ── Address decomposition ────────────────────────────────────────────
  wire [TAG_W-1:0]    tag;
  wire [SET_W-1:0]    set_idx;
  wire [OFFSET_W-3:0] word_sel;
  wire [1:0]          byte_off;

  assign tag      = addr[31:SET_W+OFFSET_W];
  assign set_idx  = addr[SET_W+OFFSET_W-1:OFFSET_W];
  assign word_sel = addr[OFFSET_W-1:2];
  assign byte_off = addr[1:0];

  // ── Hit detection ────────────────────────────────────────────────────
  wire hit0, hit1, hit;
  wire hit_way;
  assign hit0    = valid_mem[set_idx][0] && (tag_mem[set_idx][0] == tag);
  assign hit1    = valid_mem[set_idx][1] && (tag_mem[set_idx][1] == tag);
  assign hit     = hit0 || hit1;
  assign hit_way = hit1;

  // ── 状态编码（提前声明，供下面的读数据选择使用）───────────────────────
  localparam [2:0] DC_IDLE = 3'd0, DC_FILL = 3'd1, DC_FILL_DONE = 3'd2, DC_WRITE_THROUGH = 3'd3,
                   DC_BYPASS_RD = 3'd4, DC_BYPASS_RD2 = 3'd5, DC_BYPASS_WR = 3'd6;
  reg [2:0] state;
  reg [1:0] fill_cnt;
  reg       fill_way;

  // ── 数据阵列写口（组合产生，交给下面那个"无复位的干净块"执行）──────────
  reg        dc_dm_we;
  reg [IDX_W-1:0] dc_dm_idx;
  reg [31:0] dc_dm_data;
  reg [3:0]  dc_dm_strb;
  integer    ii2;

  // ── 阵列索引（单一读地址，替代原来"两路都读出来再二选一"）────────────
  wire [IDX_W-1:0] set_base = set_idx * WAYS * WORDS;
  wire [IDX_W-1:0] rd_idx   = set_base + (hit0 ? 0 : WORDS) + word_sel;
  wire [IDX_W-1:0] wr_idx   = set_base + hit_way * WORDS + word_sel;
  wire [IDX_W-1:0] fill_idx = set_base + fill_way * WORDS + fill_cnt;
  wire [IDX_W-1:0] fillw_idx= set_base + fill_way * WORDS + word_sel;

  // 外设区(0x2xxx_xxxx)不进缓存：直通访问，避免为读写一个外设寄存器而去读一整行
  wire uncached = (addr[31:28] == 4'h2);

  // ── 非对齐访存（RISC-V 允许硬件直接支持）────────────────────────────
  // 真正需要"跨字拼接"的只有两种：
  //   * lh/lhu 落在字节偏移 3  —— 字节 3、4 分属相邻两个字
  //   * lw/sw  落在字节偏移 1/2/3
  // 字节访问(lb/sb)永远是对齐的，不需要管。
  // 做法：非对齐访问一律**绕开缓存**，直接对内存做两次访问再拼接。
  // 好处是本设计的缓存行只有 16 字节，偏移 29/30/31、61/62/63 都会跨行；
  // 绕开缓存之后，"跨行"这个问题根本不存在，不用额外处理。
  wire misaligned = !uncached &&
                    (((width == MEM_HALF) && byte_off[0]) ||      // 半字落在奇数偏移(1 或 3)
                     ((width == MEM_WORD) && (byte_off != 2'd0)));

  // ── 非对齐拆分的辅助量（必须在下面的 rdata 逻辑之前声明）─────────────
  // astrb/wdata_aligned 是"元素对齐到自身宽度"时的字节使能与数据；
  // 再按 byte_off 平移一次，就得到落在前一个字 / 后一个字上的两半。
  reg [3:0] astrb;
  always @(*) begin
    case (width)
      MEM_BYTE: astrb = 4'b0001;
      MEM_HALF: astrb = 4'b0011;
      default:  astrb = 4'b1111;
    endcase
  end

  reg [3:0] mstrb1, mstrb2;   // 前一个字 / 后一个字的字节使能
  reg [7:0] sh_lo, sh_hi;     // 前一个字左移量 / 后一个字右移量
  always @(*) begin
    case (byte_off)
      2'd0: begin mstrb1 = astrb;         mstrb2 = 4'b0000;    sh_lo = 8'd0;  sh_hi = 8'd32; end
      2'd1: begin mstrb1 = astrb << 1;    mstrb2 = astrb >> 3; sh_lo = 8'd8;  sh_hi = 8'd24; end
      2'd2: begin mstrb1 = astrb << 2;    mstrb2 = astrb >> 2; sh_lo = 8'd16; sh_hi = 8'd16; end
      default: begin mstrb1 = astrb << 3; mstrb2 = astrb >> 1; sh_lo = 8'd24; sh_hi = 8'd8;  end
    endcase
  end

  // 非对齐读：先读低字，再读高字，拼成 64 位窗口后按偏移取出
  reg        mis_ph;
  reg [31:0] mis_low, mis_high;
  // 第二个字回来的那一拍 mem_rdata 还没打进 mis_high，所以这里直通用
  wire [31:0] mis_high_eff = ((state == DC_BYPASS_RD2) && mis_ph && mem_valid) ? mem_rdata : mis_high;
  wire [63:0] mis_win = {mis_high_eff, mis_low};
  wire [31:0] mis_out = (mis_win >> sh_lo);

  // ── Sub-word read logic ──────────────────────────────────────────────
  wire [31:0] raw_word;
  assign raw_word = (state == DC_BYPASS_RD) ? mem_rdata : data_mem[rd_idx];

  // Hoisted local variables for sub-word read
  reg [7:0]  byte_val;
  reg [15:0] half_val;

  always @(*) begin
    rdata = 32'b0;
    byte_val = 8'b0;
    half_val = 16'b0;
    if ((state == DC_BYPASS_RD2) && mis_ph) begin
      // 非对齐：两个 32 位字拼成 64 位窗口，再按字节偏移取出需要的宽度
      case (width)
        MEM_HALF: begin
          half_val = mis_out[15:0];
          rdata = sign_ext ? {{16{half_val[15]}}, half_val} : {16'b0, half_val};
        end
        default: rdata = mis_out[31:0];
      endcase
    end else begin
      case (width)
        MEM_BYTE: begin
          case (byte_off)
            2'd0: byte_val = raw_word[7:0];
            2'd1: byte_val = raw_word[15:8];
            2'd2: byte_val = raw_word[23:16];
            2'd3: byte_val = raw_word[31:24];
          endcase
          rdata = sign_ext ? {{24{byte_val[7]}}, byte_val} : {24'b0, byte_val};
        end
        MEM_HALF: begin
          half_val = byte_off[1] ? raw_word[31:16] : raw_word[15:0];
          rdata = sign_ext ? {{16{half_val[15]}}, half_val} : {16'b0, half_val};
        end
        MEM_WORD: rdata = raw_word;
        default:  rdata = raw_word;
      endcase
    end
  end

  // ── Write strobe generation ──────────────────────────────────────────
  reg [3:0] wstrb;
  always @(*) begin
    case (width)
      MEM_BYTE: wstrb = 4'b0001 << byte_off;
      MEM_HALF: wstrb = byte_off[1] ? 4'b1100 : 4'b0011;
      MEM_WORD: wstrb = 4'b1111;
      default:  wstrb = 4'b1111;
    endcase
  end

  reg [31:0] wdata_aligned;
  always @(*) begin
    case (width)
      MEM_BYTE: wdata_aligned = {4{wdata[7:0]}};
      MEM_HALF: wdata_aligned = {2{wdata[15:0]}};
      MEM_WORD: wdata_aligned = wdata;
      default:  wdata_aligned = wdata;
    endcase
  end

  // ── Write buffer ─────────────────────────────────────────────────────
  reg        wbuf_valid;
  reg [31:0] wbuf_addr;
  reg [31:0] wbuf_data;
  reg [3:0]  wbuf_strb;

  // 第二个写缓冲槽：非对齐 store 要拆成两次写，第一笔还没排空时
  // 第二笔先存在这里，等第一笔 mem_wr_done 之后自动顶上。
  reg        wbuf2_valid;
  reg [31:0] wbuf2_addr;
  reg [31:0] wbuf2_data;
  reg [3:0]  wbuf2_strb;
  wire       wbuf_free = !wbuf_valid && !wbuf2_valid;

  // ── FSM ──────────────────────────────────────────────────────────────
  reg [31:0] pending_addr;
  reg [31:0] pending_wdata;
  reg [3:0]  pending_wstrb;
  reg        pending_is_write;

  // 非对齐 store 之后要作废的两条缓存行所属的组号
  wire [31:0] mis_addr_b = {addr[31:2], 2'b00} + 32'd4;
  wire [SET_W-1:0] mis_set_a = addr[SET_W+OFFSET_W-1:OFFSET_W];
  wire [SET_W-1:0] mis_set_b = mis_addr_b[SET_W+OFFSET_W-1:OFFSET_W];

  integer s, w2, ii;

  // ── 数据阵列：单独的干净同步块（没有复位、没有其它控制逻辑）────────────
  //     写同步、读组合 —— 综合工具才会把它推断成分布式 RAM。
  //     旧版把写语句放在带 `negedge rst_n` 的块里，工具直接拆成触发器。
  always @(posedge clk) begin
    for (ii2 = 0; ii2 < 4; ii2 = ii2 + 1)
      if (dc_dm_we && dc_dm_strb[ii2])
        data_mem[dc_dm_idx][ii2*8 +: 8] <= dc_dm_data[ii2*8 +: 8];
  end

  always @(*) begin
    ready     = 1'b0;
    mem_rd    = 1'b0;
    mem_arlen = 4'd0;
    mem_wr    = wbuf_valid;
    mem_addr  = wbuf_valid ? wbuf_addr : 32'b0;
    mem_wdata = wbuf_data;
    mem_wstrb = wbuf_valid ? wbuf_strb : 4'b0;

    // 数据阵列写口：三种情况下要写（命中写 / 填充 / 填充后补写）
    dc_dm_we   = ((state == DC_IDLE) && (rd_en || wr_en) && hit && wr_en && wbuf_free) ||
                 ((state == DC_FILL) && wbuf_free && mem_valid) ||
                 ((state == DC_FILL_DONE) && pending_is_write);
    dc_dm_idx  = ((state == DC_FILL) && wbuf_free && mem_valid) ? fill_idx   :
                 ((state == DC_FILL_DONE) && pending_is_write)      ? fillw_idx  : wr_idx;
    dc_dm_data = ((state == DC_FILL) && wbuf_free && mem_valid) ? mem_rdata  :
                 ((state == DC_FILL_DONE) && pending_is_write)      ? pending_wdata : wdata_aligned;
    dc_dm_strb = ((state == DC_FILL) && wbuf_free && mem_valid) ? 4'b1111    :
                 ((state == DC_FILL_DONE) && pending_is_write)      ? pending_wstrb : wstrb;

    case (state)
      DC_IDLE: begin
        if (rd_en && misaligned) begin
          ready = 1'b0;                        // 非对齐读：要读两次内存
        end else if (wr_en && misaligned) begin
          ready = wbuf_free ? 1'b1 : 1'b0;     // 非对齐写：两笔一起入队
        end else if (rd_en && uncached) begin
          ready = 1'b0;                       // 直通读：等内存数据回来
        end else if (wr_en && uncached) begin
          ready = wbuf_free ? 1'b1 : 1'b0;     // 直通写：写缓冲空就直接收下
        end else if ((rd_en || wr_en) && hit) begin
          if (wr_en && wbuf_valid)
            ready = 1'b0;
          else
            ready = 1'b1;
        end else if ((rd_en || wr_en) && !hit) begin
          ready = 1'b0;
        end else begin
          ready = 1'b1;
        end
      end
      DC_FILL: begin
        if (wbuf_free) begin
          mem_rd   = 1'b1;
          // RISCV_1: 一次突发读整行(不再逐字发请求), 地址固定为行首
          mem_addr  = {pending_addr[31:OFFSET_W], {OFFSET_W{1'b0}}};
          mem_arlen = BURST_LEN;
        end
      end
      DC_FILL_DONE: begin
        ready = 1'b1;
      end
      DC_BYPASS_RD: begin
        mem_rd   = 1'b1;
        mem_addr = pending_addr;
        ready    = mem_valid;
      end
      DC_BYPASS_RD2: begin
        // 必须等写缓冲彻底排空再读内存，否则会读到还没落地的旧数据
        if (wbuf_free) begin
          mem_rd   = 1'b1;
          mem_addr = {pending_addr[31:2], 2'b00} + (mis_ph ? 32'd4 : 32'd0);
        end
        ready    = mis_ph && mem_valid;
      end
      DC_WRITE_THROUGH: begin
        ready = 1'b0;
      end
      DC_BYPASS_WR: begin
        ready = 1'b0;   // 保留状态编号(不再使用), 外设写走写缓冲
      end
      default: ;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state            <= DC_IDLE;
      fill_cnt         <= 2'b0;
      fill_way         <= 1'b0;
      pending_is_write <= 1'b0;
      wbuf_valid       <= 1'b0;
      wbuf_addr        <= 32'b0;
      wbuf_data        <= 32'b0;
      wbuf_strb        <= 4'b0;
      wbuf2_valid      <= 1'b0;
      wbuf2_addr       <= 32'b0;
      wbuf2_data       <= 32'b0;
      wbuf2_strb       <= 4'b0;
      mis_ph           <= 1'b0;
      mis_low          <= 32'b0;
      mis_high         <= 32'b0;
      for (s = 0; s < SETS; s = s + 1) begin
        for (w2 = 0; w2 < WAYS; w2 = w2 + 1)
          valid_mem[s][w2] <= 1'b0;
        lru[s] <= 1'b0;
      end
    end else begin
      // Write buffer drain: 第一笔被 AXI 收掉后，第二笔自动顶上
      if (wbuf_valid && mem_wr_done) begin
        wbuf_valid  <= wbuf2_valid;
        wbuf_addr   <= wbuf2_addr;
        wbuf_data   <= wbuf2_data;
        wbuf_strb   <= wbuf2_strb;
        wbuf2_valid <= 1'b0;
      end

      case (state)
        DC_IDLE: begin
          if (rd_en && misaligned) begin
            pending_addr <= addr;
            mis_ph       <= 1'b0;
            state        <= DC_BYPASS_RD2;
          end else if (wr_en && misaligned) begin
            if (wbuf_free) begin
              wbuf_valid  <= 1'b1;
              wbuf_addr   <= {addr[31:2], 2'b00};
              wbuf_data   <= wdata_aligned << sh_lo;
              wbuf_strb   <= mstrb1;
              wbuf2_valid <= 1'b1;
              wbuf2_addr  <= mis_addr_b;
              wbuf2_data  <= wdata_aligned >> sh_hi;
              wbuf2_strb  <= mstrb2;
              // 缓存里可能还留着这两个字的旧内容：把涉及的两条行作废
              valid_mem[mis_set_a][0] <= 1'b0;
              valid_mem[mis_set_a][1] <= 1'b0;
              valid_mem[mis_set_b][0] <= 1'b0;
              valid_mem[mis_set_b][1] <= 1'b0;
            end
          end else if (rd_en && uncached) begin
            pending_addr <= addr;
            state        <= DC_BYPASS_RD;
          end else if (wr_en && uncached) begin
            // RISCV_1: 外设写走写缓冲(与主存写同一条通路)。
            //   注意: 写是"缓存后放行"(posted), 驱动写完要读回一次确认已经送到,
            //   否则 UART 这类寄存器会被下一条写覆盖 —— 见 coremark/uart.c 的 putc。
            if (!wbuf_valid) begin
              wbuf_valid <= 1'b1;
              wbuf_addr  <= addr;
              wbuf_data  <= wdata_aligned;
              wbuf_strb  <= wstrb;
            end
          end else if ((rd_en || wr_en) && hit) begin
            if (wr_en && wbuf_free) begin
              lru[set_idx] <= hit_way ? 1'b0 : 1'b1;
              wbuf_valid <= 1'b1;
              wbuf_addr  <= addr;
              wbuf_data  <= wdata_aligned;
              wbuf_strb  <= wstrb;
            end else if (!wr_en) begin
              lru[set_idx] <= hit_way ? 1'b0 : 1'b1;
            end
          end else if ((rd_en || wr_en) && !hit && wbuf_free) begin
            fill_way         <= lru[set_idx] ? 1'b1 : 1'b0;
            fill_cnt         <= 2'b0;
            pending_addr     <= addr;
            pending_wdata    <= wdata_aligned;
            pending_wstrb    <= wstrb;
            pending_is_write <= wr_en;
            state            <= DC_FILL;
          end
        end

        DC_FILL: begin
          if (wbuf_free && mem_valid) begin
            if (fill_cnt == 2'd3) begin
              tag_mem[set_idx][fill_way]   <= tag;
              valid_mem[set_idx][fill_way] <= 1'b1;
              lru[set_idx]                 <= fill_way ? 1'b0 : 1'b1;
              state                        <= DC_FILL_DONE;
            end else begin
              fill_cnt <= fill_cnt + 1'b1;
            end
          end
        end

        DC_FILL_DONE: begin
          if (pending_is_write) begin
            wbuf_valid <= 1'b1;
            wbuf_addr  <= pending_addr;
            wbuf_data  <= pending_wdata;
            wbuf_strb  <= pending_wstrb;
          end
          state <= DC_IDLE;
        end

        DC_BYPASS_RD: begin
          if (mem_valid)
            state <= DC_IDLE;
        end

        DC_BYPASS_RD2: begin
          if (mem_valid) begin
            if (!mis_ph) begin
              mis_low <= mem_rdata;
              mis_ph  <= 1'b1;
            end else begin
              mis_high <= mem_rdata;
              mis_ph   <= 1'b0;
              state    <= DC_IDLE;
            end
          end
        end

        DC_WRITE_THROUGH: begin
          if (!wbuf_valid)
            state <= DC_IDLE;
        end

        DC_BYPASS_WR: begin
          // RISCV_1: 外设直通写，等总线应答
          if (mem_wr_done) state <= DC_IDLE;
        end

        default: state <= DC_IDLE;
      endcase
    end
  end

endmodule
