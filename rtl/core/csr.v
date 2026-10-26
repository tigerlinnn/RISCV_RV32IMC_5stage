// ============================================================================
// csr.v — Control and Status Register Unit for BRV32P
// ============================================================================

module csr (
  input  wire        clk,
  input  wire        rst_n,

  // CSR access
  input  wire        csr_en,
  input  wire [11:0] csr_addr,
  input  wire [2:0]  csr_op,
  input  wire [31:0] csr_wdata,
  output reg  [31:0] csr_rdata,

  // RISCV_1 新增: EX 级的"提前读"端口。
  //   原因: csrrw/csrrs/csrrc 要求 rd 读回的是【写之前】的旧值。
  //   而 MEM 级可能因为 I-Cache 填充/D-Cache 停顿被冻结好几拍,
  //   这期间 CSR 已经被自己写过了 —— 如果等到真正锁进 MEM/WB 时才去取
  //   csr_rdata, 取到的就是【写之后】的新值(riscv-tests 的 csr.S 第 3 条
  //   就是这么挂的: csrrsi a3, mscratch, 4 读回来 6 而不是 2)。
  //   所以在 EX 级就用 EX 的 CSR 地址读一次, 在进 MEM 的那一拍锁住,
  //   保证拿到的永远是旧值。
  input  wire [11:0] csr_addr_ex,
  output reg  [31:0] csr_rdata_ex,

  // RISCV_1 新增: "这一拍真正要写进 CSR 的值"(已经过各寄存器的 WARL 掩码,
  //   比如 mtvec/mepc 的低 2 位强制为 0、mstatus 只保留 MPIE/MIE)。
  //   用途: 把 MEM 级正在写的值前递给 EX 级的读(csr_rdata_ex_f),
  //         前递【必须是掩码之后的值】, 否则会读出根本写不进去的位。
  output wire [31:0] csr_wr_data,

  // RISCV_1 新增: 该地址是否真的"会被写进去"。
  //   未实现的 CSR(比如调试用的 tselect/tdata1/tcontrol)写操作是【被忽略】的,
  //   所以不能把它们前递给后面的读, 否则会谎报"写成功了"。
  output wire        csr_wr_ok,
  // Trap interface
  input  wire        trap_enter,
  input  wire [31:0] trap_cause,
  input  wire [31:0] trap_val,
  input  wire [31:0] trap_pc,
  output wire [31:0] mtvec_out,
  output wire [31:0] mepc_out,

  // RISCV_1 新增: 中断原因编码 (给 core 的 trap_cause 用)
  //   规范里中断原因按"哪一路中断"编码, 不能一律写 11:
  //     机器外部中断 = 11 (bit11, 来自 GPIO/UART)
  //     机器定时器中断 = 7 (bit7, 来自 timer)
  //   中断优先级: 外部 > 定时器 (规范里 MEI 高于 MTI)。
  output wire [31:0] irq_cause,

  // MRET
  input  wire        mret,

  // Interrupts
  input  wire        ext_irq,
  input  wire        timer_irq,
  input  wire        instr_retired,
  output wire        irq_pending
);

  `include "brv32p_defs.vh"

  reg [31:0] mstatus, mie, mtvec, mscratch, mepc, mcause, mtval;
  reg [31:0] mip;
  reg [63:0] mcycle, minstret;

  assign mtvec_out = mtvec;
  assign mepc_out  = mepc;

  // Interrupt pending
  always @(*) begin
    mip = 32'b0;
    mip[11] = ext_irq;
    mip[7]  = timer_irq;
  end
  assign irq_pending = mstatus[3] & |(mip & mie);

  // RISCV_1: 中断原因(带最高位 1 表示中断)
  wire irq_ext_on = mip[11] & mie[11];
  wire irq_tmr_on = mip[7]  & mie[7];
  assign irq_cause = irq_ext_on ? {1'b1, 31'd11} :
                     irq_tmr_on ? {1'b1, 31'd7}  : {1'b1, 31'd11};

// CSR Read
  // RISCV_1 修改: 两路组合读 ——
  //   csr_rdata    : 用 MEM 级地址(csr_addr), 给写通路的 读-改-写 用
  //   csr_rdata_ex : 用 EX 级的提前地址(csr_addr_ex), 在进 MEM 那一拍锁住,
  //                  保证 rd 读回的永远是【写之前】的旧值
  always @(*) begin
    csr_rdata = 32'b0;
    case (csr_addr)
      CSR_MSTATUS:  csr_rdata = mstatus;
      CSR_MISA:     csr_rdata = 32'h4000_1104;
      CSR_MIE:      csr_rdata = mie;
      CSR_MTVEC:    csr_rdata = mtvec;
      CSR_MSCRATCH: csr_rdata = mscratch;
      CSR_MEPC:     csr_rdata = mepc;
      CSR_MCAUSE:   csr_rdata = mcause;
      CSR_MTVAL:    csr_rdata = mtval;
      CSR_MIP:      csr_rdata = mip;
      CSR_MCYCLE:   csr_rdata = mcycle[31:0];
      CSR_MINSTRET: csr_rdata = minstret[31:0];
      CSR_MHARTID:  csr_rdata = 32'd0;
      default:      csr_rdata = 32'b0;
    endcase
  end

  always @(*) begin
    csr_rdata_ex = 32'b0;
    case (csr_addr_ex)
      CSR_MSTATUS:  csr_rdata_ex = mstatus;
      CSR_MISA:     csr_rdata_ex = 32'h4000_1104;
      CSR_MIE:      csr_rdata_ex = mie;
      CSR_MTVEC:    csr_rdata_ex = mtvec;
      CSR_MSCRATCH: csr_rdata_ex = mscratch;
      CSR_MEPC:     csr_rdata_ex = mepc;
      CSR_MCAUSE:   csr_rdata_ex = mcause;
      CSR_MTVAL:    csr_rdata_ex = mtval;
      CSR_MIP:      csr_rdata_ex = mip;
      CSR_MCYCLE:   csr_rdata_ex = mcycle[31:0];
      CSR_MINSTRET: csr_rdata_ex = minstret[31:0];
      CSR_MHARTID:  csr_rdata_ex = 32'd0;
      default:      csr_rdata_ex = 32'b0;
    endcase
  end

  // RISCV_1: 组合地算出"写到 CSR 里的值"(含 WARL 掩码), 供写通路和读前递共用
  reg [31:0] nv;
  always @(*) begin
    case (csr_op[1:0])
      2'b01: nv = csr_wdata;
      2'b10: nv = csr_rdata | csr_wdata;
      2'b11: nv = csr_rdata & ~csr_wdata;
      default: nv = csr_rdata;
    endcase
  end

  reg [31:0] wr_data;
  always @(*) begin
    case (csr_addr)
      CSR_MSTATUS:  wr_data = nv & 32'h88;      // 只实现 MPIE(7) / MIE(3)
      CSR_MTVEC:    wr_data = {nv[31:2], 2'b00}; // 本核不支持向量模式, 低 2 位恒 0
      CSR_MEPC:     wr_data = {nv[31:2], 2'b00}; // 指令地址 4/2 字节对齐
      CSR_MIE:      wr_data = nv;
      CSR_MSCRATCH: wr_data = nv;
      CSR_MCAUSE:   wr_data = nv;
      CSR_MTVAL:    wr_data = nv;
      default:      wr_data = nv;
    endcase
  end

  assign csr_wr_data = wr_data;

  reg wr_ok;
  always @(*) begin
    case (csr_addr)
      CSR_MSTATUS, CSR_MIE, CSR_MTVEC, CSR_MSCRATCH,
      CSR_MEPC, CSR_MCAUSE, CSR_MTVAL: wr_ok = 1'b1;
      default:                         wr_ok = 1'b0;
    endcase
  end
  assign csr_wr_ok = wr_ok;

  // CSR Write / Trap
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mstatus  <= 32'b0;
      mie      <= 32'b0;
      mtvec    <= 32'b0;
      mscratch <= 32'b0;
      mepc     <= 32'b0;
      mcause   <= 32'b0;
      mtval    <= 32'b0;
      mcycle   <= 64'b0;
      minstret <= 64'b0;
    end else begin
      mcycle <= mcycle + 1'b1;
      if (instr_retired)
        minstret <= minstret + 1'b1;

      if (trap_enter) begin
        mepc       <= trap_pc;
        mcause     <= trap_cause;
        mtval      <= trap_val;
        mstatus[7] <= mstatus[3];
        mstatus[3] <= 1'b0;
      end else if (mret) begin
        mstatus[3] <= mstatus[7];
        mstatus[7] <= 1'b1;
      end else if (csr_en) begin
        case (csr_addr)
          CSR_MSTATUS:  mstatus  <= wr_data;
          CSR_MIE:      mie      <= wr_data;
          CSR_MTVEC:    mtvec    <= wr_data;
          CSR_MSCRATCH: mscratch <= wr_data;
          CSR_MEPC:     mepc     <= wr_data;
          CSR_MCAUSE:   mcause   <= wr_data;
          CSR_MTVAL:    mtval    <= wr_data;
          default: ;
        endcase
      end
    end
  end

endmodule
