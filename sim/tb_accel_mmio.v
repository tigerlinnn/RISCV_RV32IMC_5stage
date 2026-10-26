`timescale 1ns/1ps
// Unit testbench: drive accel_mmio over the simple peripheral bus, run one
// streamed conv layer, dump the captured OFM beats to rxdump.hex.
module tb_accel_mmio;

  reg clk = 0;
  always #18.518 clk = ~clk;       // 27 MHz

  reg rst_n = 0;
  reg [7:0]  addr  = 0;
  reg        wr_en = 0, rd_en = 0;
  reg [31:0] wdata = 0;
  wire [31:0] rdata;

  accel_mmio dut (
    .clk(clk), .rst_n(rst_n), .addr(addr),
    .wr_en(wr_en), .rd_en(rd_en), .wdata(wdata), .rdata(rdata)
  );

  reg [63:0] ifm [0:4095];
  reg [63:0] wgt [0:511];
  reg [63:0] bia [0:63];
  reg [63:0] lky [0:255];
  reg [31:0] ctrl [0:31];
  reg [63:0] gold [0:4095];
  reg [63:0] rxdump [0:4095];

  integer i, n, fd;
  integer beatcnt = 0;
  integer ofmcnt = 0;
  reg [31:0] id, st, rxstat, lo, hi;
  reg [63:0] beat;

  always @(posedge clk) begin
    if (dut.u_accel.u_accel_top.ofmbuf_bram_en_write && ofmcnt < 30) begin
      $display("OFMW addr=%0d bundle=%h q0=%h acc0=%h scale=%h shift=%0d zp=%h",
               dut.u_accel.u_accel_top.ofmbuf_bram_write_addr,
               dut.u_accel.u_accel_top.ofm_store_bundle,
               dut.u_accel.u_accel_top.quant_result_0,
               dut.u_accel.u_accel_top.acc_result_0,
               dut.u_accel.u_accel_top.scale,
               dut.u_accel.u_accel_top.shift,
               dut.u_accel.u_accel_top.zero_point_out);
      ofmcnt = ofmcnt + 1;
    end
  end

  always @(posedge clk) begin
    if (dut.u_accel.S_AXIS_tvalid && dut.u_accel.S_AXIS_tready) begin
      if (beatcnt < 6 || (beatcnt >= 252 && beatcnt < 276) ||
          (beatcnt >= 330 && beatcnt < 346))
        $display("AXIS[%0d] data=%h last=%b buf=%0d ifmsel=%b wgt_en=%b ifm_en=%b",
                 beatcnt, dut.u_accel.S_AXIS_tdata, dut.u_accel.S_AXIS_tlast,
                 dut.u_accel.axis_buf_sel, dut.u_accel.ifmbuf_sel,
                 dut.u_accel.write_enable_weight, dut.u_accel.write_enable_ifm);
      beatcnt = beatcnt + 1;
    end
    if (dut.u_accel.S_AXIS_tvalid && dut.u_accel.S_AXIS_tready && dut.u_accel.S_AXIS_tlast)
      $display("TLAST beat=%0d buf=%0d data=%h", beatcnt-1,
               dut.u_accel.axis_buf_sel, dut.u_accel.S_AXIS_tdata);
    if (dut.tx_push)
      $display("PUSH beat=%0d len=%0d pushcnt=%0d ok=%b bad=%b full=%b count=%0d lo=%h hi=%h",
               beatcnt, dut.tx_len, dut.tx_push_count, dut.tx_push_ok,
               dut.tx_push_bad, dut.tx_full, dut.tx_raddr, dut.tx_lo, dut.tx_hi);
    if (beatcnt >= 258 && beatcnt < 280 && dut.u_accel.S_AXIS_tvalid)
      $display("STATE beat=%0d tready=%b state=%b recv_en=%b rst=%b soft=%b tlast=%b count=%0d",
               beatcnt, dut.u_accel.S_AXIS_tready,
               dut.u_accel.inst_interface_axis_slave.curr_state,
               dut.u_accel.recv_enable, dut.accel_rst, dut.soft_rst,
               dut.u_accel.S_AXIS_tlast, dut.tx_raddr);
  end

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
      while (((s & mask) != mask) && (t < 200000)) begin
        bus_read(8'h34, s);
        t = t + 1;
      end
      if ((s & mask) != mask) begin
        $display("TIMEOUT waiting status mask=%h got=%h", mask, s);
        $finish;
      end
    end
  endtask

  task do_stream;
    input [31:0]  reg_recv;
    input [31:0]  reg_idle;
    input integer len;
    input integer which;
    integer k;
    begin
      bus_write(8'h10, len);
      $display("STREAM start which=%0d len=%0d tx_len=%0d", which, len, dut.tx_len);
      bus_write(8'h38, 32'h4);        // clear recv_done
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
      bus_write(8'h00, reg_recv);
      bus_write(8'h00, reg_idle);
      wait_status(32'h4);             // recv_done
      bus_read(8'h34, st);
      $display("STREAM done which=%0d push=%0d status=%h", which,
               dut.tx_push_count, st);
    end
  endtask

  initial begin
    $readmemh("ifm.hex", ifm);
    $readmemh("wgt.hex", wgt);
    $readmemh("bia.hex", bia);
    $readmemh("lky.hex", lky);
    $readmemh("ctrl.hex", ctrl);
    $readmemh("ofm_gold.hex", gold);

    $display("TB accel_mmio start");
    repeat (20) @(posedge clk);
    rst_n = 1;
    repeat (20) @(posedge clk);

    bus_read(8'h3C, id);
    $display("ID = %h (expect ACCE0001)", id);

    bus_write(8'h40, 32'h1);          // soft reset
    repeat (50) @(posedge clk);

    do_stream(ctrl[0], ctrl[1], ctrl[15], 3);   // LUT
    do_stream(ctrl[2], ctrl[3], ctrl[16], 2);   // bias
    do_stream(ctrl[4], ctrl[5], ctrl[17], 1);   // weights
    do_stream(ctrl[6], ctrl[7], ctrl[18], 0);   // IFM

    $display("DBG wgt00=%h wgt11=%h wgt22=%h",
             dut.u_accel.u_accel_top.weight_win3x3_00,
             dut.u_accel.u_accel_top.weight_win3x3_11,
             dut.u_accel.u_accel_top.weight_win3x3_22);
    $display("DBG ifm a0=%h a1=%h a15=%h a16=%h a89=%h",
             dut.u_accel.u_accel_top.u_buffer_ifm_1x8.u_com_dual_port_ram_0.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_ifm_1x8.u_com_dual_port_ram_0.RAM[1],
             dut.u_accel.u_accel_top.u_buffer_ifm_1x8.u_com_dual_port_ram_0.RAM[15],
             dut.u_accel.u_accel_top.u_buffer_ifm_1x8.u_com_dual_port_ram_0.RAM[16],
             dut.u_accel.u_accel_top.u_buffer_ifm_1x8.u_com_dual_port_ram_0.RAM[89]);
    $display("DBG txlen=%0d push=%0d count=%0d err=%b",
             dut.tx_len, dut.tx_push_count, dut.tx_raddr, dut.tx_error);

    $display("DBG wram OC0: %018h %018h %018h %018h",
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_0.u_com_simple_dual_port_ram_0.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_0.u_com_simple_dual_port_ram_1.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_0.u_com_simple_dual_port_ram_2.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_0.u_com_simple_dual_port_ram_3.RAM[0]);
    $display("DBG wram OC0: %018h %018h %018h %018h",
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_0.u_com_simple_dual_port_ram_4.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_0.u_com_simple_dual_port_ram_5.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_0.u_com_simple_dual_port_ram_6.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_0.u_com_simple_dual_port_ram_7.RAM[0]);
    $display("DBG wram OC1..OC7 IC0: %018h %018h %018h %018h %018h %018h %018h",
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_1.u_com_simple_dual_port_ram_0.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_2.u_com_simple_dual_port_ram_0.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_3.u_com_simple_dual_port_ram_0.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_4.u_com_simple_dual_port_ram_0.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_5.u_com_simple_dual_port_ram_0.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_6.u_com_simple_dual_port_ram_0.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_weight_1x8x8.u_buffer_weight_1x8_7.u_com_simple_dual_port_ram_0.RAM[0]);

    bus_write(8'h04, ctrl[12]);       // reg1 scale/shift
    bus_write(8'h08, ctrl[13]);       // reg2 zero points
    bus_write(8'h0C, ctrl[14]);       // reg3 weight/bias tile
    bus_write(8'h38, 32'h2);          // clear conv_done

    bus_write(8'h00, ctrl[8]);        // CONV task
    bus_write(8'h00, ctrl[9]);
    wait_status(32'h2);               // conv_done

    $display("DBG conv: shutdown=%b reg0=%h sel=%0d row=%0d ifmbuf=%b",
             dut.u_accel.conv_shutdown, dut.reg0, dut.u_accel.sel,
             dut.u_accel.row, dut.u_accel.ifmbuf_sel);
    $display("DBG ofm 0=%h 15=%h 16=%h 17=%h 31=%h",
             dut.u_accel.u_accel_top.u_buffer_ofm.u_com_simple_dual_port_ram.RAM[0],
             dut.u_accel.u_accel_top.u_buffer_ofm.u_com_simple_dual_port_ram.RAM[15],
             dut.u_accel.u_accel_top.u_buffer_ofm.u_com_simple_dual_port_ram.RAM[16],
             dut.u_accel.u_accel_top.u_buffer_ofm.u_com_simple_dual_port_ram.RAM[17],
             dut.u_accel.u_accel_top.u_buffer_ofm.u_com_simple_dual_port_ram.RAM[31]);

    bus_write(8'h00, ctrl[10]);       // SEND task
    bus_write(8'h00, ctrl[11]);
    wait_status(32'h10);              // rx_done

    bus_read(8'h30, rxstat);
    n = rxstat & 32'h1FFF;
    $display("RX count = %0d (gold = %0d)", n, ctrl[19]);

    for (i = 0; i < n; i = i + 1) begin
      bus_write(8'h24, i[31:0]);
      repeat (4) @(posedge clk);
      bus_read(8'h28, lo);
      bus_read(8'h2C, hi);
      rxdump[i] = {hi, lo};
    end

    fd = $fopen("rxdump.hex", "w");
    for (i = 0; i < n; i = i + 1)
      $fdisplay(fd, "%016h", rxdump[i]);
    $fclose(fd);

    for (i = 0; i < 12 && i < n; i = i + 1)
      $display("rx[%0d] = %016h", i, rxdump[i]);

    n = 0;
    for (i = 0; i < (rxstat & 32'h1FFF); i = i + 1) begin
      if (rxdump[i] !== gold[i]) begin
        if (n < 10)
          $display("MISMATCH beat=%0d got=%016h gold=%016h",
                   i, rxdump[i], gold[i]);
        n = n + 1;
      end
    end
    $display("COMPARE %0s (%0d beat errors, gold beats=%0d)",
             (n == 0) ? "PASS" : "FAIL", n, ctrl[19]);
    $display("TB done");
    $finish;
  end

endmodule