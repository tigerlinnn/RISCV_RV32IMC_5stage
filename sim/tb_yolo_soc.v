`timescale 1ns / 1ps
// ============================================================================
// tb_yolo_soc.v — Phase A at SoC level: CPU firmware drives the accelerator
// ----------------------------------------------------------------------------
// Loads firmware.hex into the BRV32P SoC (same image that goes on the board),
// reconstructs UART output by snooping UART TX register writes, and finishes
// when the firmware sets a terminal LED pattern:
//     0x5A      -> PASS
//     bit7 = 1  -> FAIL
// Run from a directory containing firmware.hex.
// ============================================================================
module tb_yolo_soc;

  reg         clk = 0;
  always #18.518 clk = ~clk;     // 27 MHz

  reg         rst_n = 0;
  reg  [31:0] gpio_in = 32'h0;
  wire [31:0] gpio_out;
  reg         uart_rx = 1'b1;
  wire        uart_tx;

  localparam integer MAX_CYCLES = 4000000;
  localparam integer BACKSTOP   = 200000;

  integer cycles = 0;
  integer holdoff = 0;
  integer found = 0;             // 0 none, 1 pass string, 2 fail string
  reg [31:0] tail = 32'd0;
  reg [7:0]  last_led = 8'h00;
  reg        reported = 0;

  brv32p_soc #(
    .MEM_DEPTH (8192),
    .INIT_FILE ("firmware.hex")
  ) dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .gpio_in  (gpio_in),
    .gpio_out (gpio_out),
    .uart_rx  (uart_rx),
    .uart_tx  (uart_tx)
  );

  initial begin
    #200;
    rst_n = 1'b1;
    $display("[SOC] reset released");
  end

  task report;
    input pass;
    begin
      if (!reported) begin
        reported = 1;
        $display("\n[SOC] LED=%02h", gpio_out[7:0]);
        if (pass)
          $display("[SOC] === PASS === cycles=%0d", cycles);
        else
          $display("[SOC] === FAIL === cycles=%0d", cycles);
        $finish;
      end
    end
  endtask

  always @(posedge clk) begin
    cycles <= cycles + 1;

    if (rst_n && dut.u_uart.wr_en && (dut.u_uart.addr[3:2] == 2'd0)) begin
      $write("%c", dut.u_uart.wdata[7:0]);
      $fflush;
      tail <= {tail[23:0], dut.u_uart.wdata[7:0]};
      if ({tail[23:0], dut.u_uart.wdata[7:0]} == "PASS")
        found <= 1;
      else if ({tail[23:0], dut.u_uart.wdata[7:0]} == "FAIL")
        found <= 2;
    end

    if (gpio_out[7:0] != last_led) begin
      $display("\n[SOC] led -> %02h", gpio_out[7:0]);
      last_led <= gpio_out[7:0];
    end

    if (found != 0) begin
      holdoff <= holdoff + 1;
      if (gpio_out[7:0] == 8'h5A)
        report(1'b1);
      else if (gpio_out[7])
        report(1'b0);
      else if (holdoff > BACKSTOP)
        report(found == 1);
    end

    if (cycles > MAX_CYCLES)
      report(1'b0);
  end

endmodule