`timescale 1ns/1ps
// Unit testbench for real-layer vectors (upstream YOLOv3-tiny layer 0).
// Drives accel_mmio over the simple peripheral bus, streams LUT/bias/weights/
// IFM, runs one conv+send per output-channel group, and compares the captured
// RX beats directly against the Python golden model.
`include "real_defs.vh"
module tb_real_layer;

  reg clk = 0;
  always #18.518 clk = ~clk;        // 27 MHz

  reg rst_n = 0;
  reg [7:0]  addr  = 0;
  reg        wr_en = 0, rd_en = 0;
  reg [31:0] wdata = 0;
  wire [31:0] rdata;

  accel_mmio dut (
    .clk(clk), .rst_n(rst_n), .addr(addr),
    .wr_en(wr_en), .rd_en(rd_en), .wdata(wdata), .rdata(rdata)
  );

  reg [63:0] ifm  [0:4095];
  reg [63:0] wgt  [0:511];
  reg [63:0] bia  [0:63];
  reg [63:0] lky  [0:255];
  reg [63:0] gold [0:4095];

  integer i, g, fd;
  integer errors = 0;
  reg [31:0] id, st, rxstat;
  reg [63:0] beat;

  localparam [31:0] R_STATIC =
      ((`R_ROW) << 24) | ((`R_SEL) << 10) | ((`R_POOL) << 5) |
      ((`R_RELU_TYPE) << 16) | (1 << 6) | (1 << 7) | (1 << 8);

  task bus_write;
    input [7:0]  a;
    input [31:0] d;
    begin
      @(posedge clk);
      addr <= a; wdata <= d; wr_en <= 1'b1;
      @(posedge clk);
      wr_en <= 1'b0;
      repeat (3) @(posedge clk);
    end
  endtask

  task bus_read;
    input  [7:0]  a;
    output [31:0] d;
    begin
      @(posedge clk);
      addr <= a; rd_en <= 1'b1;
      @(posedge clk);
      #1 d = rdata;
      rd_en <= 1'b0;
      repeat (3) @(posedge clk);
    end
  endtask

  task push64;
    input [63:0] d;
    begin
      bus_write(8'h14, d[31:0]);
      bus_write(8'h18, d[63:32]);
      bus_write(8'h1C, 32'h0);
    end
  endtask

  task wait_status;
    input [31:0] mask;
    integer t;
    reg [31:0] s;
    begin
      t = 0; s = 0;
      while (((s & mask) != mask) && (t < 400000)) begin
        bus_read(8'h34, s);
        t = t + 1;
      end
      if ((s & mask) != mask) begin
        $display("TIMEOUT waiting status mask=%h got=%h", mask, s);
        errors = errors + 1;
      end
    end
  endtask

  task do_stream;
    input [31:0]  len;
    input integer which;          // 0=IFM 1=wgt 2=bia 3=lky
    input [31:0]  bufsel;
    integer k;
    begin
      bus_write(8'h10, len);
      $display("STREAM which=%0d len=%0d bufsel=%0d", which, len, bufsel);
      bus_write(8'h38, 32'h4);    // clear recv_done
      for (k = 0; k < len; k = k + 1) begin
        case (which)
          0: beat = ifm[k];
          1: beat = wgt[k];
          2: beat = bia[k];
          3: beat = lky[k];
          default: beat = 64'h0;
        endcase
        push64(beat);
      end
      bus_write(8'h00, 32'h201 | (bufsel << 2));
      bus_write(8'h00, (bufsel << 2));
      wait_status(32'h4);         // recv_done
      bus_read(8'h34, st);
      $display("STREAM done which=%0d push=%0d status=%h", which,
               dut.tx_push_count, st);
    end
  endtask

  task run_group;
    input integer og;
    integer k;
    begin
      bus_write(8'h0C, og | (og << 16));    // weight tile / bias tile
      bus_write(8'h38, 32'h2);              // clear conv_done
      bus_write(8'h00, R_STATIC | (1 << 9) | (1 << 4));
      bus_write(8'h00, R_STATIC);
      wait_status(32'h2);                   // conv_done
      bus_write(8'h38, 32'h8 | 32'h10);     // clear send/rx done
      bus_write(8'h00, R_STATIC | (1 << 9) | (1 << 1));
      bus_write(8'h00, R_STATIC);
      wait_status(32'h10);                  // rx_done
      bus_read(8'h30, rxstat);
      $display("OG %0d rx count=%0d (expect %0d)", og, rxstat & 32'h1FFF,
               `R_GOLD_PER_OG);
      if ((rxstat & 32'h1FFF) != `R_GOLD_PER_OG)
        errors = errors + 1;
      for (k = 0; k < `R_GOLD_PER_OG; k = k + 1) begin
        if (dut.rx_mem[k] !== gold[og * `R_GOLD_PER_OG + k]) begin
          if (errors < 10)
            $display("MISMATCH og=%0d beat=%0d got=%016h gold=%016h",
                     og, k, dut.rx_mem[k], gold[og * `R_GOLD_PER_OG + k]);
          errors = errors + 1;
        end
      end
    end
  endtask

  initial begin
    $readmemh("ifm.hex", ifm);
    $readmemh("wgt.hex", wgt);
    $readmemh("bia.hex", bia);
    $readmemh("lky.hex", lky);
    $readmemh("ofm_gold.hex", gold);

    $display("TB real layer: row=%0d sel=%0d groups=%0d ifm_beats=%0d",
             `R_ROW, `R_SEL, `R_OG_N, `R_IFM_BEATS);
    repeat (20) @(posedge clk);
    rst_n = 1;
    repeat (20) @(posedge clk);

    bus_read(8'h3C, id);
    $display("ID = %h (expect ACCE0001)", id);
    if (id !== 32'hACCE0001) errors = errors + 1;

    bus_write(8'h40, 32'h1);        // soft reset
    repeat (50) @(posedge clk);

    do_stream(`R_LUT_BEATS, 3, 2);  // LUT
    do_stream(`R_BIA_BEATS, 2, 3);  // bias
    do_stream(`R_WGT_BEATS, 1, 1);  // weights
    do_stream(`R_IFM_BEATS, 0, 0);  // IFM (writes bank 0)

    bus_write(8'h04, `R_SCALE | (`R_SHIFT << 16));
    bus_write(8'h08, `R_ZP_IN | (`R_ZP_OUT << 8) | (`R_ZP_ACT << 16));

    for (g = 0; g < `R_OG_N; g = g + 1)
      run_group(g);

    $display("COMPARE %0s (%0d errors, gold beats=%0d)",
             (errors == 0) ? "PASS" : "FAIL", errors, `R_GOLD_BEATS);
    $display("TB done");
    $finish;
  end

endmodule