// ============================================================================
// axi_periph_bridge.v — AXI4-Lite to Simple Bus Peripheral Bridge
// ============================================================================
module axi_periph_bridge (
  input  wire        clk,
  input  wire        rst_n,
  // AXI read
  input  wire [31:0] araddr,  input  wire arvalid, output wire arready,
  output wire [31:0] rdata,   output wire rvalid,  input  wire rready,
  // AXI write
  input  wire [31:0] awaddr,  input  wire awvalid, output wire awready,
  input  wire [31:0] wdata,   input  wire [3:0] wstrb,
  input  wire        wvalid,  output wire wready,
  output wire        bvalid,  input  wire bready,
  // GPIO
  output reg  [7:0]  gpio_addr, output reg gpio_rd, output reg gpio_wr,
  output reg  [31:0] gpio_wdata, input wire [31:0] gpio_rdata,
  // UART
  output reg  [7:0]  uart_addr, output reg uart_rd, output reg uart_wr,
  output reg  [31:0] uart_wdata, input wire [31:0] uart_rdata,
  // Timer
  output reg  [7:0]  timer_addr, output reg timer_rd, output reg timer_wr,
  output reg  [31:0] timer_wdata, input wire [31:0] timer_rdata,
  // CNN accelerator
  output reg  [7:0]  accel_addr, output reg accel_rd, output reg accel_wr,
  output reg  [31:0] accel_wdata, input wire [31:0] accel_rdata
);

  function [1:0] psel;
    input [31:0] a;
    begin
      case (a[11:8])
        4'h0: psel = 2'd0;
        4'h1: psel = 2'd1;
        4'h2: psel = 2'd2;
        4'h3: psel = 2'd3;
        default: psel = 2'd0;
      endcase
    end
  endfunction

  // 只有落在 GPIO(0x..00) / UART(0x..01) / TIMER(0x..02) / ACCEL(0x..03)
  // 四个 256 字节窗口内的地址才算"已知外设"。旧版把其它地址默认当作 GPIO
  // 处理，会误写 GPIO 寄存器。
  function known_win;
    input [31:0] a;
    begin
      known_win = (a[11:8] == 4'h0) || (a[11:8] == 4'h1) ||
                  (a[11:8] == 4'h2) || (a[11:8] == 4'h3);
    end
  endfunction

  // Read FSM
  localparam [1:0] RI = 2'd0, RA = 2'd1, RR = 2'd2;
  reg [1:0] rfsm;
  reg [31:0] rdr, rar;
  assign arready = (rfsm == RI);
  assign rdata = rdr;
  assign rvalid = (rfsm == RR);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rfsm <= RI; rdr <= 0; rar <= 0; end
    else case (rfsm)
      RI: if (arvalid) begin rar <= araddr; rfsm <= RA; end
      RA: begin
        if (known_win(rar))
          case (psel(rar)) 2'd0: rdr<=gpio_rdata; 2'd1: rdr<=uart_rdata; 2'd2: rdr<=timer_rdata; 2'd3: rdr<=accel_rdata; default: rdr<=0; endcase
        else
          rdr <= 32'b0;
        rfsm <= RR;
      end
      RR: if (rready) rfsm <= RI;
      default: rfsm <= RI;
    endcase
  end

  // Write FSM
  localparam [1:0] WI = 2'd0, WA = 2'd1, WRR = 2'd2;
  reg [1:0] wfsm;
  reg [31:0] war, wdr;
  assign awready = (wfsm==WI) && awvalid && wvalid;
  assign wready  = (wfsm==WI) && awvalid && wvalid;
  assign bvalid  = (wfsm==WRR);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin wfsm<=WI; war<=0; wdr<=0; end
    else case (wfsm)
      WI: if (awvalid&&wvalid) begin war<=awaddr; wdr<=wdata; wfsm<=WA; end
      WA: wfsm <= WRR;
      WRR: if (bready) wfsm <= WI;
      default: wfsm <= WI;
    endcase
  end

  // Unified read/write address and control
  always @(*) begin
    gpio_rd=0; uart_rd=0; timer_rd=0; accel_rd=0;
    gpio_wr=0; uart_wr=0; timer_wr=0; accel_wr=0;
    gpio_wdata=wdr; uart_wdata=wdr; timer_wdata=wdr; accel_wdata=wdr;

    gpio_addr=rar[7:0]; uart_addr=rar[7:0]; timer_addr=rar[7:0]; accel_addr=rar[7:0];

    if (rfsm==RA && known_win(rar))
      case(psel(rar)) 2'd0:gpio_rd=1; 2'd1:uart_rd=1; 2'd2:timer_rd=1; 2'd3:accel_rd=1; default:; endcase

    if (wfsm==WA) begin
      gpio_addr=war[7:0]; uart_addr=war[7:0]; timer_addr=war[7:0]; accel_addr=war[7:0];
      if (known_win(war))
        case(psel(war)) 2'd0:gpio_wr=1; 2'd1:uart_wr=1; 2'd2:timer_wr=1; 2'd3:accel_wr=1; default:; endcase
    end
  end

endmodule
