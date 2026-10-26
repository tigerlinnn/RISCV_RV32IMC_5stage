// ============================================================================
// brv32p_core.v — 5-Stage Pipelined RV32IMC CPU Core
// ============================================================================

module brv32p_core (
  input  wire        clk,
  input  wire        rst_n,

  // Instruction memory interface
  output wire [31:0] imem_addr,
  output wire        imem_rd,
  input  wire [31:0] imem_rdata,
  input  wire        imem_ready,

  // Data memory interface
  output wire [31:0] dmem_addr,
  output wire        dmem_rd,
  output wire        dmem_wr,
  output wire [1:0]  dmem_width,
  output wire        dmem_sign_ext,
  output wire [31:0] dmem_wdata,
  input  wire [31:0] dmem_rdata,
  input  wire        dmem_ready,

  // Interrupts
  input  wire        ext_irq,
  input  wire        timer_irq,

  // fence.i -> 作废 I-Cache（自修改代码后必须重新取指）
  output wire        ifence_inv
);

  `include "brv32p_defs.vh"

  // ════════════════════════════════════════════════════════════════════
  // Pipeline control signals
  // ════════════════════════════════════════════════════════════════════
  wire stall_if, stall_id, stall_ex;
  wire flush_id, flush_ex, flush_mem;
  wire [1:0] fwd_rs1, fwd_rs2;
  wire mem_stall;
  wire dmem_stall;
  wire imem_stall;

  assign dmem_stall = (dmem_rd | dmem_wr) & ~dmem_ready;
  // 注意: imem_stall / mem_stall 的赋值放在下面 IF 段里 ——
  //   "跨字取指"(RVC 的 32 位指令落在高半字上)要多花一拍, 也必须算作停顿。

  // ════════════════════════════════════════════════════════════════════
  // IF — Instruction Fetch
  // ════════════════════════════════════════════════════════════════════
  reg  [31:0] pc_if;
  reg  [31:0] pc_next;
  wire [31:0] instr_raw_if;
  wire        bp_pred_taken;
  wire [31:0] bp_pred_target;
  wire        bp_pred_valid;
  wire        is_compressed_if;
  wire        redirect_req;       // 跳转/分支纠正、陷阱、mret（见下）

  // ── 取指窗口对齐（RVC 压缩指令支持）──────────────────────────────────
  // I-Cache 一次只给一个 32 位字，但指令是 2 字节对齐的：
  //   * 指令起点在字的高半字(地址 bit1=1)时，字的低半字属于上一条指令，不能拿来译码；
  //   * 若此时还是一条 32 位指令，它会跨到下一个字，两个半字要拼起来才算一条完整指令。
  // 所以这里把取回的窗口"对齐到指令起点"，必要时补取一次下一个字。
  reg         fetch_hi;      // 1 = 正在补取跨字指令的后半截
  reg  [15:0] instr_hi_q;    // 第一个字里的高半字（跨字指令的前半截）
  wire [15:0] hw_lo_if   = imem_rdata[15:0];
  wire [15:0] hw_hi_if   = imem_rdata[31:16];
  wire        need_second = pc_if[1] && (hw_hi_if[1:0] == 2'b11);
  wire [31:0] fetch_addr  = fetch_hi ? ({pc_if[31:2], 2'b00} + 32'd4) : pc_if;
  wire [31:0] instr_first = pc_if[1] ? {16'b0, hw_hi_if} : imem_rdata;
  // 交付条件：I-Cache 已就绪，且（不需要补取 或 补取已完成）
  wire        split_busy  = fetch_hi ? ~imem_ready : (imem_ready & need_second);
  assign imem_stall = (imem_rd & ~imem_ready) | split_busy;
  assign mem_stall  = dmem_stall | imem_stall;

  // PC Register
  // RISCV_1 修复: 纠正(跳转/分支/陷阱/mret)必须**优先于访存/取指停顿**立刻生效。
  //   - 原版是 `if (!stall_if && !mem_stall) pc_if <= pc_next;` —— 停顿期间纠正会被
  //     丢掉, 表现为跳转不生效(例如 jal x0,-4 的循环跑飞)。
  //   - 但也不能把纠正"暂存起来延后应用": 延迟期间错误路径的指令会继续往前流,
  //     事后只刷新最年轻的两级已经来不及, 那些指令会真的执行掉(踩坏寄存器/PC)。
  //   所以这里让纠正直接插队, 同一拍就改 PC。
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pc_if <= RESET_VECTOR;
    else if (redirect_req)
      pc_if <= pc_next;
    else if (!stall_if && !mem_stall)
      pc_if <= pc_next;
  end

  // 跨字取指的两拍节奏：第一拍记住高半字，第二拍拿到下一个字后拼接交付
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      fetch_hi <= 1'b0;
    // 注意: 纠正信号若"目标就是当前 PC"(pc_next == pc_if), PC 其实没有换地方,
    //   这时不能把补取进度扔掉 —— 否则会死锁:
    //     补取没完成 -> mem_stall=1 -> EX/MEM 冻结 -> 纠正信号一直挂着
    //     -> 又去清补取进度 ... 循环不止。
    else if (redirect_req && (pc_next != pc_if))
      fetch_hi <= 1'b0;
    else if (imem_ready) begin
      if (fetch_hi) begin
        // RISCV_1 修复: 只有"这一拍真的把指令交付进流水线"时才撤 fetch_hi。
        //   否则会活锁: 若流水线因为别的原因被冻结(例如这条 store 本身还在等
        //   D-Cache), IF/ID 不会锁存指令, PC 也不前进 —— 而 fetch_hi 却每拍
        //   清零又重新置 1, 同一条指令被反复交付, PC 永远推不过去。
        //   CoreMark 就是在这条"奇数半字地址上的 store"上卡死的。
        if (!dmem_stall && !stall_if && !stall_id)
          fetch_hi <= 1'b0;
      end
      else if (need_second) begin
        fetch_hi <= 1'b1;
      end
    end
  end

  // RISCV_1 修复(时序): instr_hi_q 的时钟使能只依赖 fetch_hi, 不再依赖 imem_ready。
  //   原来的使能是 `imem_ready && !fetch_hi && need_second`, 而 imem_ready 由
  //   I-Cache 产生、要绕经 AXI 互连才回来, 组合路径很长 —— 综合实现后成了
  //   111 条建立时间违例的终点(最差 -1.254ns)。
  //   改成 `!fetch_hi` 之后: 补取开始那一拍把高半字锁进来, 之后 fetch_hi=1
  //   期间一直保持不变(哪怕第二字要做整行填充也稳), 拿到第二个字时再拼接。
  //   行为与原设计一致, 但时钟使能只看 fetch_hi 这个寄存器, 路径极短。
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      instr_hi_q <= 16'b0;
    else if (!fetch_hi)
      instr_hi_q <= hw_hi_if;
  end

  assign imem_addr = fetch_addr;
  assign imem_rd   = 1'b1;
  assign instr_raw_if = fetch_hi ? {hw_lo_if, instr_hi_q} : instr_first;

  // Compressed decoder
  wire [31:0] instr_expanded_if;
  wire        illegal_c_if;

  compressed_decoder u_cdec (
    .instr_in      (instr_raw_if),
    .instr_out     (instr_expanded_if),
    .is_compressed (is_compressed_if),
    .illegal_c     (illegal_c_if)
  );

  // Branch predictor
  wire        bp_update_en;
  wire [31:0] bp_update_pc, bp_update_target;
  wire        bp_update_taken;

  branch_predictor u_bp (
    .clk         (clk),
    .rst_n       (rst_n),
    .inv         (ifence_inv),
    .pc          (pc_if),
    .pred_taken  (bp_pred_taken),
    .pred_target (bp_pred_target),
    .pred_valid  (bp_pred_valid),
    .update_en   (bp_update_en),
    .update_pc   (bp_update_pc),
    .update_taken(bp_update_taken),
    .update_target(bp_update_target)
  );

  // Next PC MUX
  wire        branch_mispredict_ex;
  wire [31:0] branch_target_ex;
  wire        trap_enter;
  wire        trap_kill_mem;   // RISCV_1: 只有异常才需要杀掉 MEM 级那条指令
  wire [31:0] mtvec_out, mepc_out;
  wire [31:0] irq_cause;      // RISCV_1: 中断原因编码(来自 csr 单元)
  wire [31:0] mret_target;   // RISCV_1: mret 的返回地址(带 CSR 写前递, 见下)
  wire        mret_ex;

  wire [31:0] pc_inc;
  assign pc_inc = is_compressed_if ? 32'd2 : 32'd4;

  always @(*) begin
    if (trap_enter)
      pc_next = mtvec_out;
    else if (mret_ex)
      pc_next = mret_target;
    else if (branch_mispredict_ex)
      pc_next = branch_target_ex;
    else if (bp_pred_taken && bp_pred_valid)
      pc_next = bp_pred_target;
    else
      pc_next = pc_if + pc_inc;
  end

  // ════════════════════════════════════════════════════════════════════
  // IF/ID Pipeline Register
  // ════════════════════════════════════════════════════════════════════
  reg [31:0] pc_id, instr_id;
  reg [31:0] pc_inc_id;
  reg        pred_taken_id;
  reg [31:0] pred_target_id;   // RISCV_1: 预测跳转目标必须跟着指令一起流到 EX
  reg        illegal_c_id;
  // RISCV_1: "这条是真指令" 标志 —— 冲刷产生的气泡必须是 invalid,
  //   否则中断会在气泡(pc=0 的 NOP)上进入, mepc 被写成垃圾(实测表现为程序重启)
  reg        valid_id;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || flush_id) begin
      pc_id         <= 32'b0;
      instr_id      <= 32'h0000_0013; // NOP
      pc_inc_id     <= 32'd4;
      pred_taken_id <= 1'b0;
      pred_target_id <= 32'b0;
      illegal_c_id  <= 1'b0;
      valid_id      <= 1'b0;
    end else if (!stall_id && !mem_stall) begin
      pc_id         <= pc_if;
      instr_id      <= instr_expanded_if;
      pc_inc_id     <= pc_inc;
      pred_taken_id <= bp_pred_taken && bp_pred_valid;
      pred_target_id <= bp_pred_target;
      illegal_c_id  <= illegal_c_if;
      valid_id      <= 1'b1;
    end
  end

  // ════════════════════════════════════════════════════════════════════
  // ID — Instruction Decode
  // ════════════════════════════════════════════════════════════════════
  wire [4:0]          rs1_addr_id, rs2_addr_id, rd_addr_id;
  wire [31:0]         imm_id;
  wire [`CTRL_W-1:0]  ctrl_id;
  wire [31:0]         rs1_data_raw, rs2_data_raw;

  decoder u_decoder (
    .instr    (instr_id),
    .rs1_addr (rs1_addr_id),
    .rs2_addr (rs2_addr_id),
    .rd_addr  (rd_addr_id),
    .imm      (imm_id),
    .ctrl     (ctrl_id)
  );

  // Register file
  wire        wb_wr_en;
  wire [4:0]  wb_rd_addr;
  reg  [31:0] wb_rd_data;

  regfile u_regfile (
    .clk      (clk),
    .rst_n    (rst_n),
    .rs1_addr (rs1_addr_id),
    .rs1_data (rs1_data_raw),
    .rs2_addr (rs2_addr_id),
    .rs2_data (rs2_data_raw),
    .wr_en    (wb_wr_en),
    .rd_addr  (wb_rd_addr),
    .rd_data  (wb_rd_data)
  );

  // ════════════════════════════════════════════════════════════════════
  // ID/EX Pipeline Register
  // ════════════════════════════════════════════════════════════════════
  reg [31:0]         pc_ex, rs1_data_ex, rs2_data_ex, imm_ex;
  reg [31:0]         pc_inc_ex;
  reg [4:0]          rs1_addr_ex, rs2_addr_ex, rd_addr_ex;
  reg [`CTRL_W-1:0]  ctrl_ex;
  reg                pred_taken_ex;
  reg [31:0]         pred_target_ex;
  reg                valid_ex;

  // RISCV_1 修复: 压缩译码器发现的"非法压缩编码"必须也接进"非法指令"陷阱通路。
  //   原来 illegal_c_id 只是被锁存下来, 全程没有任何地方使用它,
  //   于是非法指令会被当成 NOP 静默执行一路跑下去 ——
  //   这正是 riscv-tests 的 illegal.S / shamt.S 挂掉的原因。
  wire [`CTRL_W-1:0] illegal_c_bit = {{(`CTRL_W-1){1'b0}}, illegal_c_id} << `CTRL_ILLEGAL;
  wire [`CTRL_W-1:0] ctrl_id_eff   = ctrl_id | illegal_c_bit;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || flush_ex) begin
      ctrl_ex       <= {`CTRL_W{1'b0}};
      pc_ex         <= 32'b0;
      rs1_data_ex   <= 32'b0;
      rs2_data_ex   <= 32'b0;
      imm_ex        <= 32'b0;
      pc_inc_ex     <= 32'd4;
      rs1_addr_ex   <= 5'b0;
      rs2_addr_ex   <= 5'b0;
      rd_addr_ex    <= 5'b0;
      pred_taken_ex <= 1'b0;
      pred_target_ex <= 32'b0;
      valid_ex      <= 1'b0;
    end else if (!stall_ex && !mem_stall) begin
      ctrl_ex       <= ctrl_id_eff;
      pc_ex         <= pc_id;
      rs1_data_ex   <= rs1_data_raw;
      rs2_data_ex   <= rs2_data_raw;
      imm_ex        <= imm_id;
      pc_inc_ex     <= pc_inc_id;
      rs1_addr_ex   <= rs1_addr_id;
      rs2_addr_ex   <= rs2_addr_id;
      rd_addr_ex    <= rd_addr_id;
      pred_taken_ex <= pred_taken_id;
      pred_target_ex <= pred_target_id;
      valid_ex      <= valid_id;
    end
  end

  // ════════════════════════════════════════════════════════════════════
  // EX — Execute
  // ════════════════════════════════════════════════════════════════════

  // Forwarding MUXes
  reg  [31:0] rs1_fwd, rs2_fwd;
  wire [31:0] alu_result_mem;  // From EX/MEM register
  wire [31:0] wb_data_fwd;     // From MEM/WB

  always @(*) begin
    case (fwd_rs1)
      FWD_EX_MEM:  rs1_fwd = alu_result_mem;
      FWD_MEM_WB:  rs1_fwd = wb_data_fwd;
      default:     rs1_fwd = rs1_data_ex;
    endcase
    case (fwd_rs2)
      FWD_EX_MEM:  rs2_fwd = alu_result_mem;
      FWD_MEM_WB:  rs2_fwd = wb_data_fwd;
      default:     rs2_fwd = rs2_data_ex;
    endcase
  end

  // ALU
  wire [31:0] alu_b, alu_result_ex;
  wire        alu_zero;

  assign alu_b = ctrl_ex[`CTRL_ALU_SRC] ? imm_ex : rs2_fwd;

  alu u_alu (
    .a      (rs1_fwd),
    .b      (alu_b),
    .op     (ctrl_ex[`CTRL_ALU_OP]),
    .result (alu_result_ex),
    .zero   (alu_zero)
  );

  // AUIPC result
  wire [31:0] auipc_result;
  assign auipc_result = pc_ex + imm_ex;

  // MulDiv
  wire [31:0] muldiv_result;
  wire        muldiv_busy, muldiv_valid;

  muldiv u_muldiv (
    .clk    (clk),
    .rst_n  (rst_n),
    .start  (ctrl_ex[`CTRL_MULDIV_EN]),
    .op     (ctrl_ex[`CTRL_MULDIV_OP]),
    .a      (rs1_fwd),
    .b      (rs2_fwd),
    .result (muldiv_result),
    .busy   (muldiv_busy),
    .valid  (muldiv_valid)
  );

  // Branch resolution
  reg  branch_taken_ex;
  wire [31:0] branch_target_computed;

  always @(*) begin
    branch_taken_ex = 1'b0;
    case (ctrl_ex[`CTRL_BR_TYPE])
      BR_EQ:  branch_taken_ex = (rs1_fwd == rs2_fwd);
      BR_NE:  branch_taken_ex = (rs1_fwd != rs2_fwd);
      BR_LT:  branch_taken_ex = ($signed(rs1_fwd) < $signed(rs2_fwd));
      BR_GE:  branch_taken_ex = ($signed(rs1_fwd) >= $signed(rs2_fwd));
      BR_LTU: branch_taken_ex = (rs1_fwd < rs2_fwd);
      BR_GEU: branch_taken_ex = (rs1_fwd >= rs2_fwd);
      default: branch_taken_ex = 1'b0;
    endcase
  end

  assign branch_target_computed = ctrl_ex[`CTRL_JALR] ?
    {alu_result_ex[31:1], 1'b0} : (pc_ex + imm_ex);

  // Mispredict detection
  wire is_branch_or_jump;
  assign is_branch_or_jump = (ctrl_ex[`CTRL_BR_TYPE] != BR_NONE) ||
                              ctrl_ex[`CTRL_JAL] || ctrl_ex[`CTRL_JALR];

  wire actual_taken;
  assign actual_taken = branch_taken_ex || ctrl_ex[`CTRL_JAL] || ctrl_ex[`CTRL_JALR];

  // RISCV_1 修复(性能): 原来第二个条件是拿"实际目标"和 pc+4 比, 对**任何** taken 分支
  //   都成立 —— 于是每条 taken 分支都要冲刷 ID/EX 两级流水线, 预测器等于白装。
  //   现在改成和"当时预测的目标"比: 预测命中的 taken 分支不再冲刷。
  assign branch_mispredict_ex = is_branch_or_jump &&
    ((actual_taken != pred_taken_ex) ||
     (actual_taken && (branch_target_computed != pred_target_ex)));

  assign branch_target_ex = actual_taken ? branch_target_computed :
                            (pc_ex + pc_inc_ex);

  // RISCV_1 FIX(mepc of interrupt): "architectural next PC" of the instr in EX.
  //   taken branch / JAL / JALR -> redirect target ; MRET -> mepc ;
  //   everything else -> pc + instruction length.
  //   This value is pipelined into MEM so that an interrupt taken while the
  //   MEM-stage instruction is a jump/branch/MRET still saves the correct
  //   return address (previously mepc = pc_mem + len, which is wrong for
  //   control transfers: it returned into the middle of the trap handler and
  //   the demo livelocked in trap_entry).
  wire [31:0] next_pc_arch_ex = mret_ex ? mret_target : branch_target_ex;

  // Branch predictor update
  assign bp_update_en     = (ctrl_ex[`CTRL_BR_TYPE] != BR_NONE);
  assign bp_update_pc     = pc_ex;
  assign bp_update_taken  = branch_taken_ex;
  assign bp_update_target = branch_target_computed;

  // MRET
  assign mret_ex = ctrl_ex[`CTRL_MRET];

  // ── 纠正请求（跳转/分支误预测、陷阱、mret）────────────────────────────
  // 这个信号在 PC 更新逻辑里拥有最高优先级（见上面 PC Register 段）。
  assign redirect_req = trap_enter || mret_ex || branch_mispredict_ex;

  // FENCE.I: 在 EX 级发出"作废 I-Cache"请求。
  //   放在 EX（而不是更晚的 MEM/WB），是为了让 fence.i 之后取到的指令
  //   尽可能都经过作废，减小"提前取指命中旧行"的窗口。
  //   I-Cache 内部会对这个电平做边沿检测，保证每条 fence.i 只清一次。
  assign ifence_inv = ctrl_ex[`CTRL_FENCE_I];

  // EX result select
  reg [31:0] ex_result;
  always @(*) begin
    case (ctrl_ex[`CTRL_WB_SEL])
      WB_ALU:    ex_result = alu_result_ex;
      WB_PC4:    ex_result = pc_ex + pc_inc_ex;
      WB_MULDIV: ex_result = muldiv_result;
      WB_AUIPC:  ex_result = auipc_result;   // RISCV_1 修复: pc + imm
      default:   ex_result = alu_result_ex;
    endcase
  end

  // ════════════════════════════════════════════════════════════════════
  // EX/MEM Pipeline Register
  // ════════════════════════════════════════════════════════════════════
  reg [31:0]         pc_mem;
  reg [31:0]         pc_inc_mem;      // RISCV_1: MEM 级指令长度(中断返回地址要用)
  reg [31:0]         ex_result_mem, rs2_data_mem;
  // RISCV_1 修复: CSR 指令的写数据必须是 rs1 的值 (立即数形式用 rs1 字段当 5 位立即数),
  // 原版错误地拿 ALU 计算结果当写数据, 导致 CSR 写入的值被算错。
  reg [31:0]         rs1_data_mem;
  reg [4:0]          rs1_addr_mem;
  reg [4:0]          rd_addr_mem;
  reg [`CTRL_W-1:0]  ctrl_mem;
  reg                valid_mem;
  reg [31:0]         next_pc_mem;     // RISCV_1: architectural next PC of the MEM-stage instr

  // RISCV_1 修复: MEM 级前递抽头必须用"MEM 级真正的结果"。
  //   load 指令在 MEM 级的结果是读回来的数据(dmem_rdata), 不是 ALU 结果(那是地址)。
  //   原版用 ex_result_mem 当抽头, 于是"装载结果立刻被下一条指令使用"时,
  //   前递过去的是地址而不是数据(b/lh/lbu 等子字装载的旁路测试因此失败)。
  // RISCV_1 修复: 前递抽头必须覆盖"所有写回来源"，不只是 load。
  //   `csrr rd, csr` 之后紧跟一条用 rd 的指令时, EX/MEM 前递拿到的原来
  //   是 ALU 结果(rs1+imm), 而不是 CSR 读出来的值 —— 于是读到错数。
  //   (这和当年 load 的抽头问题完全同源, 只是 CSR 这一路一直没补上)
  wire [31:0] csr_rdata;   // 提前声明: CSR 读数据, 由下面的 csr 实例驱动
  wire [31:0] csr_rdata_ex;   // RISCV_1 新增: EX 级的 CSR 提前读
  wire [31:0] csr_rdata_ex_f; // RISCV_1 新增: 再把"MEM 级正在写同一个 CSR"的值前递进来
  reg  [31:0] csr_rdata_mem;  // RISCV_1 新增: 进 MEM 那一拍锁住的 CSR 旧值
  wire [31:0] csr_wr_data;    // RISCV_1 新增: csr.v 给出的"真正要写入的值"(含掩码)
  wire        csr_wr_ok;      // RISCV_1 新增: 该 CSR 地址是否真的会被写
  // RISCV_1 修改(时序): 去掉这里的 "MEM 级 load 数据" 分支。
  //   load 的结果不需要在 MEM 级前递给 EX —— 因为 load-use 冒险会让后面那条
  //   指令在 ID 多停一拍, 它进入 EX 时 load 正好在 WB 级, 由 WB 前递取到数据
  //   (hazard_unit 的 load_use_hazard + fwd MEM_WB 那一路)。
  //   而 D-Cache 的读数据是"地址 -> LUTRAM -> 对齐 mux"这一长串组合逻辑,
  //   挂在 EX 前递口上就成了全局最差路径(ex_result_mem -> D-Cache -> alu_result_mem
  //   -> ALU -> 分支目标比较 -> 取指控制)。删掉它以后最差路径大幅缩短。
  assign alu_result_mem = (ctrl_mem[`CTRL_WB_SEL] == WB_CSR) ? csr_rdata_mem :
                                                             ex_result_mem;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || flush_mem) begin
      ctrl_mem       <= {`CTRL_W{1'b0}};
      pc_mem         <= 32'b0;
      next_pc_mem    <= 32'b0;
      pc_inc_mem     <= 32'd4;
      ex_result_mem  <= 32'b0;
      rs2_data_mem   <= 32'b0;
      rs1_data_mem   <= 32'b0;
      rs1_addr_mem   <= 5'b0;
      rd_addr_mem    <= 5'b0;
      csr_rdata_mem  <= 32'b0;
      valid_mem      <= 1'b0;
    end else if (!mem_stall) begin
      ctrl_mem       <= ctrl_ex;
      pc_mem         <= pc_ex;
      next_pc_mem    <= next_pc_arch_ex;
      pc_inc_mem     <= pc_inc_ex;
      ex_result_mem  <= ex_result;
      rs2_data_mem   <= rs2_fwd;
      rs1_data_mem   <= rs1_fwd;
      rs1_addr_mem   <= rs1_addr_ex;
      rd_addr_mem    <= rd_addr_ex;
      csr_rdata_mem  <= csr_rdata_ex_f;
      valid_mem      <= valid_ex;
    end
  end

  // ════════════════════════════════════════════════════════════════════
  // MEM — Memory Access
  // ════════════════════════════════════════════════════════════════════
  assign dmem_addr     = ex_result_mem;
  assign dmem_rd       = ctrl_mem[`CTRL_MEM_RD];
  assign dmem_wr       = ctrl_mem[`CTRL_MEM_WR];
  assign dmem_width    = ctrl_mem[`CTRL_MEM_WIDTH];
  assign dmem_sign_ext = ctrl_mem[`CTRL_MEM_SIGN];
  assign dmem_wdata    = rs2_data_mem;

  // MEM result
  wire [31:0] mem_result;
  assign mem_result = ctrl_mem[`CTRL_MEM_RD] ? dmem_rdata : ex_result_mem;

  // Trap logic
  wire        irq_pending;

  // RISCV_1 修复(精确中断):
  //   原来只要 irq_pending 就立刻进陷阱 —— 但 MEM 级有可能正好是个"气泡"
  //   (被冲刷掉的指令, ctrl_mem=0, pc_mem=0), 这时 mepc 会被写成 0,
  //   mret 之后整个程序跳回地址 0 重启(实测表现: 串口反复只打横幅第一个字符)。
  //   现在区分两种陷阱:
  //     · 异常(ecall/ebreak/非法指令): 出错指令不执行, mepc = 它的 PC, 并杀掉 MEM 级
  //     · 中断: 等 MEM 级是一条真指令时再进, 让它**执行完**, mepc = 它的下一条地址
  //       (这样中断是"两条指令之间"发生的, 精确, 也不会把 store 重复执行一次)
  wire excep_enter = ctrl_mem[`CTRL_ECALL] || ctrl_mem[`CTRL_EBREAK] ||
                     ctrl_mem[`CTRL_ILLEGAL];
  //   门控条件:  MEM 级真的有一条指令(valid_mem, 气泡不算) 且 这一拍没有被访存停顿
  //   (停顿期间指令还没执行完, 此时中断会把它"跳过" —— 必须等它完成那一拍)
  //   Also never take an interrupt on a CSR instruction: the write of that CSR
  //   is squashed while trap_enter is high, so e.g. "csrsi mstatus,8" would be
  //   silently dropped.  Waiting one cycle costs nothing and keeps the CSR
  //   (and therefore mstatus.MIE / mie / mtvec) state consistent.
  wire irq_enter   = irq_pending && valid_mem && !mem_stall && !excep_enter &&
                     !ctrl_mem[`CTRL_CSR_EN];

  assign trap_enter  = excep_enter || irq_enter;
  assign trap_kill_mem = excep_enter;    // 只有异常才需要"干掉 MEM 级那条"
  wire [31:0] trap_pc_eff = excep_enter ? pc_mem : next_pc_mem;

  reg [31:0] trap_cause, trap_val;
  always @(*) begin
    trap_cause = 32'b0;
    trap_val   = 32'b0;
    if (ctrl_mem[`CTRL_ILLEGAL]) begin
      trap_cause = 32'd2;
    end else if (ctrl_mem[`CTRL_ECALL]) begin
      trap_cause = 32'd11;
    end else if (ctrl_mem[`CTRL_EBREAK]) begin
      trap_cause = 32'd3;
    end else if (irq_enter) begin
      // RISCV_1 修复: 按真正触发的那一路中断编码(GPIO/UART=11, 定时器=7),
      //   原来一律写 11, 导致定时器中断的 ISR 无法用 mcause 分辨来源。
      trap_cause = irq_cause;
    end
  end

  // ════════════════════════════════════════════════════════════════════
  // MEM/WB Pipeline Register
  // ════════════════════════════════════════════════════════════════════
  reg [31:0]         mem_result_wb;
  reg [31:0]         csr_rdata_wb;
  reg [4:0]          rd_addr_wb;
  reg [`CTRL_W-1:0]  ctrl_wb;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ctrl_wb      <= {`CTRL_W{1'b0}};
      mem_result_wb <= 32'b0;
      csr_rdata_wb <= 32'b0;
      rd_addr_wb   <= 5'b0;
    end else if (!mem_stall) begin
      ctrl_wb      <= ctrl_mem;
      mem_result_wb <= mem_result;
      csr_rdata_wb <= csr_rdata_mem;
      rd_addr_wb   <= rd_addr_mem;
    end
  end

  // ════════════════════════════════════════════════════════════════════
  // WB — Writeback
  // ════════════════════════════════════════════════════════════════════
  always @(*) begin
    case (ctrl_wb[`CTRL_WB_SEL])
      WB_MEM:    wb_rd_data = mem_result_wb;
      WB_CSR:    wb_rd_data = csr_rdata_wb;
      default:   wb_rd_data = mem_result_wb;
    endcase
  end

  assign wb_wr_en   = ctrl_wb[`CTRL_REG_WR];
  assign wb_rd_addr = rd_addr_wb;
  assign wb_data_fwd = wb_rd_data;  // Forwarding tap

  // ════════════════════════════════════════════════════════════════════
  // CSR Unit
  // ════════════════════════════════════════════════════════════════════
  wire [31:0] csr_wdata;
  // ctrl_mem[CTRL_CSR_OP] is bits [19:17]; bit 2 of that = bit 19
  // RISCV_1 修复:
  //   CSR 的写数据来源是 rs1:  寄存器形式 -> rs1 寄存器的值;
  //                            立即数形式(csrrwi/csrsi/csrrci) -> rs1 字段当作 5 位无符号立即数。
  //   原版用 ex_result_mem(ALU 结果) 是错的 —— 例如 csrw mtvec,t0 的 rs1/rs2 字段
  //   都指向 t0, ALU 把它算成 t0+t0, 于是 mtvec 被写成了 2 倍的值。
  assign csr_wdata = ctrl_mem[19] ?
    {27'b0, rs1_addr_mem} : rs1_data_mem;

  // RISCV_1 修复(关键): mret 的返回地址必须考虑"上一拍刚写的 mepc"。
  //   处理异常的标准写法就是:
  //       addi t0, t0, 8
  //       csrw mepc, t0      <- MEM 级, 要到时钟沿才写进 mepc
  //       mret               <- EX 级, 同一拍组合地取 pc_next = mepc
  //   两者紧挨着, mret 取到的是【旧的 mepc】, 于是异常处理完又跳回出错指令,
  //   反复触发同一个异常 —— riscv-tests 的 illegal.S 就死在这个死循环里。
  //   这里把 MEM 级正在写的 CSR 新值前递给 mret。
  wire        mem_csr_wr  = ctrl_mem[`CTRL_CSR_EN] && !trap_enter;
  wire        mem_wr_mepc = mem_csr_wr && csr_wr_ok && (ctrl_mem[`CTRL_CSR_ADDR] == CSR_MEPC);

  assign mret_target = mem_wr_mepc ? csr_wr_data : mepc_out;

  // RISCV_1 修复: "写 CSR 紧接读同一个 CSR" 的前递
  //   csrw  mscratch, t0      <- MEM 级(时钟沿才写进去)
  //   csrr  a0, mscratch      <- EX 级, 同一拍做提前读 —— 不前递就会读到旧值
  //   这里把 MEM 级那个"新值"直接前递给 EX 级的读。
  wire mem_csr_same_ex = mem_csr_wr && csr_wr_ok &&
                         (ctrl_mem[`CTRL_CSR_ADDR] == ctrl_ex[`CTRL_CSR_ADDR]);
  assign csr_rdata_ex_f = mem_csr_same_ex ? csr_wr_data : csr_rdata_ex;

  csr u_csr (
    .clk           (clk),
    .rst_n         (rst_n),
    .csr_en        (ctrl_mem[`CTRL_CSR_EN] && !trap_enter),
    .csr_addr      (ctrl_mem[`CTRL_CSR_ADDR]),
    .csr_op        (ctrl_mem[`CTRL_CSR_OP]),
    .csr_wdata     (csr_wdata),
    .csr_rdata     (csr_rdata),
    .csr_addr_ex   (ctrl_ex[`CTRL_CSR_ADDR]),
    .csr_rdata_ex  (csr_rdata_ex),
    .csr_wr_data   (csr_wr_data),
    .csr_wr_ok     (csr_wr_ok),
    .trap_enter    (trap_enter),
    .trap_cause    (trap_cause),
    .trap_val      (trap_val),
    .trap_pc       (trap_pc_eff),
    .mtvec_out     (mtvec_out),
    .mepc_out      (mepc_out),
    .irq_cause     (irq_cause),
    .mret          (mret_ex),
    .ext_irq       (ext_irq),
    .timer_irq     (timer_irq),
    .instr_retired (ctrl_wb[`CTRL_REG_WR] || ctrl_wb[`CTRL_MEM_WR] || ctrl_wb[`CTRL_ECALL]),
    .irq_pending   (irq_pending)
  );

  // ════════════════════════════════════════════════════════════════════
  // Hazard Unit
  // ════════════════════════════════════════════════════════════════════
  hazard_unit u_hazard (
    .id_rs1           (rs1_addr_id),
    .id_rs2           (rs2_addr_id),
    .ex_rs1           (rs1_addr_ex),
    .ex_rs2           (rs2_addr_ex),
    .ex_rd            (rd_addr_ex),
    .ex_reg_wr        (ctrl_ex[`CTRL_REG_WR]),
    .ex_mem_rd        (ctrl_ex[`CTRL_MEM_RD]),
    .ex_wb_sel        (ctrl_ex[`CTRL_WB_SEL]),
    .mem_rd           (rd_addr_mem),
    .mem_reg_wr       (ctrl_mem[`CTRL_REG_WR]),
    .wb_rd            (rd_addr_wb),
    .wb_reg_wr        (ctrl_wb[`CTRL_REG_WR]),
    .branch_mispredict(branch_mispredict_ex || trap_enter || mret_ex),
    .jump_ex          (ctrl_ex[`CTRL_JAL] || ctrl_ex[`CTRL_JALR]),
    .trap_enter       (trap_enter),
    .trap_kill_mem    (trap_kill_mem),
    .muldiv_busy      (muldiv_busy),
    .mem_stall        (mem_stall),
    .fwd_rs1          (fwd_rs1),
    .fwd_rs2          (fwd_rs2),
    .stall_if         (stall_if),
    .stall_id         (stall_id),
    .stall_ex         (stall_ex),
    .flush_id         (flush_id),
    .flush_ex         (flush_ex),
    .flush_mem        (flush_mem)
  );

endmodule
