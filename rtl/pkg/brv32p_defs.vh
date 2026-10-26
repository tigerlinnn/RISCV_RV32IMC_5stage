// ============================================================================
// brv32p_defs.vh — Shared definitions for BRV32P pipelined RISC-V MCU
// ============================================================================
// Converted from SystemVerilog package for iverilog 10.1 compatibility.
// Include this file inside module bodies after port declarations.
// ============================================================================

// ── ctrl_t field position defines (global, guarded) ──────────────────
`ifndef BRV32P_CTRL_DEFS
`define BRV32P_CTRL_DEFS
`define CTRL_W          44
`define CTRL_ALU_OP     43:40
`define CTRL_ALU_SRC    39
`define CTRL_MULDIV_EN  38
`define CTRL_MULDIV_OP  37:35
`define CTRL_MEM_RD     34
`define CTRL_MEM_WR     33
`define CTRL_MEM_WIDTH  32:31
`define CTRL_MEM_SIGN   30
`define CTRL_REG_WR     29
`define CTRL_WB_SEL     28:26
`define CTRL_BR_TYPE    25:23
`define CTRL_JAL        22
`define CTRL_JALR       21
`define CTRL_CSR_EN     20
`define CTRL_CSR_OP     19:17
`define CTRL_CSR_ADDR   16:5
`define CTRL_ECALL      4
`define CTRL_EBREAK     3
`define CTRL_MRET       2
`define CTRL_FENCE_I    1   // 原 CTRL_FENCE: 本核只关心 fence.i（普通 fence 对单核无副作用）
`define CTRL_ILLEGAL    0
`endif

// ── Instruction Opcodes ──────────────────────────────────────────────
localparam [6:0] OP_LUI    = 7'b0110111;
localparam [6:0] OP_AUIPC  = 7'b0010111;
localparam [6:0] OP_JAL    = 7'b1101111;
localparam [6:0] OP_JALR   = 7'b1100111;
localparam [6:0] OP_BRANCH = 7'b1100011;
localparam [6:0] OP_LOAD   = 7'b0000011;
localparam [6:0] OP_STORE  = 7'b0100011;
localparam [6:0] OP_IMM    = 7'b0010011;
localparam [6:0] OP_REG    = 7'b0110011;
localparam [6:0] OP_FENCE  = 7'b0001111;
localparam [6:0] OP_SYSTEM = 7'b1110011;

// ── ALU Operations ───────────────────────────────────────────────────
localparam [3:0] ALU_ADD    = 4'b0000;
localparam [3:0] ALU_SUB    = 4'b1000;
localparam [3:0] ALU_SLL    = 4'b0001;
localparam [3:0] ALU_SLT    = 4'b0010;
localparam [3:0] ALU_SLTU   = 4'b0011;
localparam [3:0] ALU_XOR    = 4'b0100;
localparam [3:0] ALU_SRL    = 4'b0101;
localparam [3:0] ALU_SRA    = 4'b1101;
localparam [3:0] ALU_OR     = 4'b0110;
localparam [3:0] ALU_AND    = 4'b0111;
localparam [3:0] ALU_PASS_B = 4'b1111;

// ── MUL/DIV Operations (M extension) ─────────────────────────────────
localparam [2:0] MD_MUL    = 3'b000;
localparam [2:0] MD_MULH   = 3'b001;
localparam [2:0] MD_MULHSU = 3'b010;
localparam [2:0] MD_MULHU  = 3'b011;
localparam [2:0] MD_DIV    = 3'b100;
localparam [2:0] MD_DIVU   = 3'b101;
localparam [2:0] MD_REM    = 3'b110;
localparam [2:0] MD_REMU   = 3'b111;

// ── Writeback source select ──────────────────────────────────────────
localparam [2:0] WB_ALU    = 3'd0;
localparam [2:0] WB_MEM    = 3'd1;
localparam [2:0] WB_PC4    = 3'd2;
localparam [2:0] WB_CSR    = 3'd3;
localparam [2:0] WB_MULDIV = 3'd4;
// RISCV_1 修复: AUIPC 必须写回 "pc + imm", 原来译码器把它当普通 ADD,
// 结果算成 rs1 + imm (AUIPC 的 rs1 字段其实是立即数的一部分), 完全错误。
// 这里给 AUIPC 单独一个写回来源编码 (3'd5 原本未使用)。
localparam [2:0] WB_AUIPC  = 3'd5;

// ── Memory access width ──────────────────────────────────────────────
localparam [1:0] MEM_BYTE = 2'b00;
localparam [1:0] MEM_HALF = 2'b01;
localparam [1:0] MEM_WORD = 2'b10;

// ── Branch type ──────────────────────────────────────────────────────
localparam [2:0] BR_NONE = 3'b000;
localparam [2:0] BR_EQ   = 3'b001;
localparam [2:0] BR_NE   = 3'b010;
localparam [2:0] BR_LT   = 3'b011;
localparam [2:0] BR_GE   = 3'b100;
localparam [2:0] BR_LTU  = 3'b101;
localparam [2:0] BR_GEU  = 3'b110;

// ── Forward select ───────────────────────────────────────────────────
localparam [1:0] FWD_NONE   = 2'b00;
localparam [1:0] FWD_EX_MEM = 2'b01;
localparam [1:0] FWD_MEM_WB = 2'b10;

// ── CSR Addresses ────────────────────────────────────────────────────
localparam [11:0] CSR_MSTATUS  = 12'h300;
localparam [11:0] CSR_MISA     = 12'h301;
localparam [11:0] CSR_MIE      = 12'h304;
localparam [11:0] CSR_MTVEC    = 12'h305;
localparam [11:0] CSR_MSCRATCH = 12'h340;
localparam [11:0] CSR_MEPC     = 12'h341;
localparam [11:0] CSR_MCAUSE   = 12'h342;
localparam [11:0] CSR_MTVAL    = 12'h343;
localparam [11:0] CSR_MIP      = 12'h344;
localparam [11:0] CSR_MCYCLE   = 12'hB00;
localparam [11:0] CSR_MINSTRET = 12'hB02;
localparam [11:0] CSR_MHARTID  = 12'hF14;

// ── Memory Map ───────────────────────────────────────────────────────
localparam [31:0] RESET_VECTOR = 32'h0000_0000;
localparam [31:0] IMEM_BASE    = 32'h0000_0000;
// RISCV_1 存储器映射:
//   0x0000_0000 - 0x0000_7FFF  主存 32KB (代码+数据, 统一编址, BRAM 实现)
//                               代码从 0x0000_0000 开始, 数据区请用 0x0000_1000 起
//   0x2000_0000 - 0x2000_00FF  GPIO
//   0x2000_0100 - 0x2000_01FF  UART
//   0x2000_0200 - 0x2000_02FF  TIMER
//   0x2000_0300 - 0x2000_03FF  CNN accelerator (accel_mmio)
// 旧版把 0x1000_0000 当数据区, 但存储器忽略地址高位, 数据实际写到了第 0 个字上
// (覆盖指令)。RISCV_1 的 axi_sram 增加了地址窗口译码, 窗口外不再折回。
localparam [31:0] MEM_BASE     = 32'h0000_0000;
localparam [31:0] MEM_SIZE     = 32'h0000_8000;
localparam [31:0] DMEM_BASE    = 32'h0000_1000;
localparam [31:0] PERIPH_BASE  = 32'h2000_0000;
localparam [31:0] GPIO_BASE    = 32'h2000_0000;
localparam [31:0] UART_BASE    = 32'h2000_0100;
localparam [31:0] TIMER_BASE   = 32'h2000_0200;
localparam [31:0] ACCEL_BASE   = 32'h2000_0300;

// ── Cache parameters ─────────────────────────────────────────────────
// RISCV_1: Cache 容量(性能优化)。下面这些值由 brv32p_soc.v 显式传给 icache/dcache。
//   容量 = SETS × WAYS × (LINE_W/8) 字节
//     I-Cache: 256 × 2 × 16B = 8KB   (原来 64×2×16B = 2KB, 程序有 12.9KB, 太小)
//     D-Cache: 128 × 2 × 16B = 4KB   (原来 2KB, CoreMark 工作集约 2~3KB)
localparam ICACHE_SETS   = 256;
localparam ICACHE_WAYS   = 2;
localparam ICACHE_LINE_W = 128;
localparam DCACHE_SETS   = 128;
localparam DCACHE_WAYS   = 2;
localparam DCACHE_LINE_W = 128;

// ── AXI4-Lite parameters ─────────────────────────────────────────────
localparam AXI_ADDR_W = 32;
localparam AXI_DATA_W = 32;
localparam AXI_STRB_W = AXI_DATA_W / 8;
