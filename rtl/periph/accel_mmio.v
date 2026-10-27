// ============================================================================
// accel_mmio.v — simple-bus wrapper for the YOLOv3-tiny conv accelerator
// ----------------------------------------------------------------------------
// Register map (word offsets from ACCEL_BASE = 0x2000_0300):
//   0x00 reg0   control (same bits as Accel_Conv.v)
//   0x04 reg1   scale[15:0] | shift[19:16]
//   0x08 reg2   zp_in | zp_out<<8 | zp_act<<16
//   0x0C reg3   weight tile addr[6:0] | bias tile addr[22:16]
//   0x10 TXLEN  beats in the next AXI-Stream receive task
//   0x14 TX_LO / 0x18 TX_HI / 0x1C TXPUSH (write) -> push one 64-bit beat
//   0x20 TXSTAT {8'b0,err,empty,full,count[4:0],push_count[15:0]}
//   0x24 RXADDR (write triggers a BRAM read)
//   0x28 RX_LO / 0x2C RX_HI
//   0x30 RXSTAT {17'b0,overflow,done,count[12:0]}
//   0x34 STATUS {23'b0,busy,overflow,err,full,rx_done,send,recv,conv,ap}
//   0x38 STATUS_CLR (write-1-clear bits 0..6)
//   0x3C ID = 0xACCE0001
//   0x40 RESET (write 1 -> 15-cycle soft reset)
//   0x44 RXCLR (write 1 -> clear RX pointer/flags)
// ============================================================================
module accel_mmio (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  addr,
    input  wire        wr_en,
    input  wire        rd_en,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata
);

    localparam RX_DEPTH = 4096;

    // ── accelerator control registers ──────────────────────────────────────
    reg [31:0] reg0, reg1, reg2, reg3;

    // ── TX stream (CPU -> accelerator S_AXIS) ──────────────────────────────
    // The CPU writes the whole stream into tx_mem first; a recv task then
    // releases a sequential read pointer, so tlast is simply raddr == len-1.
    // 4096 beats covers one full-width (416 px + 2) row chunk of 8 rows.
    reg  [15:0] tx_len;
    reg  [31:0] tx_lo, tx_hi;
    reg  [15:0] tx_wptr, tx_raddr, tx_push_count;
    reg         tx_started, tx_error;
    reg  [63:0] tx_mem [0:4095];

    wire        tx_full  = (tx_wptr == 16'd4096);
    // Start on the task_valid falling edge (the second reg0 write).  The core
    // clears its weight/bias write counters while task_valid is high, so beats
    // must not be accepted until it drops or the first few are lost.
    wire        tx_start = wr_en && (addr[6:2] == 5'd0) && reg0[9] && !wdata[9] && reg0[0];
    wire        acc_s_tvalid = tx_started && (tx_raddr < tx_len);
    wire [63:0] acc_s_tdata  = tx_mem[tx_raddr[11:0]];
    wire        acc_s_tlast  = (tx_raddr == (tx_len - 16'd1));
    wire        acc_s_tready;
    wire        acc_s_pop    = acc_s_tvalid && acc_s_tready;

    wire        tx_push    = wr_en && (addr[6:2] == 5'd7);
    wire        tx_push_ok = tx_push && !tx_full && (tx_wptr < tx_len);
    wire        tx_push_bad = tx_push && !tx_push_ok;

    // ── RX stream (accelerator M_AXIS -> BRAM) ─────────────────────────────
    reg  [63:0] rx_mem [0:RX_DEPTH-1];
    reg  [12:0] rx_wptr;
    reg         rx_done, rx_overflow;
    reg  [11:0] rx_addr;
    reg  [63:0] rx_rdata;
    reg         rx_rd_req;

    wire        acc_m_tvalid;
    wire [63:0] acc_m_tdata;
    wire        acc_m_tlast;
    wire        acc_m_tready = (rx_wptr < RX_DEPTH);
    wire        acc_m_fire   = acc_m_tvalid && acc_m_tready;

    // ── accelerator done/status ────────────────────────────────────────────
    wire        acc_ap_done, acc_conv_done, acc_recv_done, acc_send_done;
    reg         ap_done_s, conv_done_s, recv_done_s, send_done_s;

    // ── soft reset ─────────────────────────────────────────────────────────
    reg  [4:0]  soft_cnt;
    wire        soft_rst  = (soft_cnt != 5'd0);
    wire        accel_rst = ~rst_n | soft_rst;

    // ── bus decode helpers ─────────────────────────────────────────────────
    wire [4:0]  reg_sel     = addr[6:2];
    wire        status_clr  = wr_en && (reg_sel == 5'd14);
    wire        send_start  = wr_en && (reg_sel == 5'd0) && wdata[9] && wdata[1];

    // ── control registers ──────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg0 <= 0; reg1 <= 0; reg2 <= 0; reg3 <= 0;
        end else if (soft_rst) begin
            reg0 <= 0; reg1 <= 0; reg2 <= 0; reg3 <= 0;
        end else if (wr_en) begin
            case (reg_sel)
                5'd0: reg0 <= wdata;
                5'd1: reg1 <= wdata;
                5'd2: reg2 <= wdata;
                5'd3: reg3 <= wdata;
                default: ;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            soft_cnt <= 5'd0;
        else if (wr_en && (reg_sel == 5'd16))
            soft_cnt <= 5'd15;
        else if (soft_cnt != 5'd0)
            soft_cnt <= soft_cnt - 5'd1;
    end

    // ── TX buffer ──────────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_len <= 0; tx_lo <= 0; tx_hi <= 0;
            tx_wptr <= 0; tx_raddr <= 0; tx_push_count <= 0;
            tx_started <= 0; tx_error <= 0;
        end else if (soft_rst) begin
            tx_len <= 0; tx_lo <= 0; tx_hi <= 0;
            tx_wptr <= 0; tx_raddr <= 0; tx_push_count <= 0;
            tx_started <= 0; tx_error <= 0;
        end else begin
            if (wr_en) begin
                case (reg_sel)
                    5'd4: begin
                        tx_len <= wdata[15:0];
                        tx_wptr <= 0; tx_raddr <= 0; tx_push_count <= 0;
                        tx_started <= 0; tx_error <= 0;
                    end
                    5'd5: tx_lo <= wdata;
                    5'd6: tx_hi <= wdata;
                    default: ;
                endcase
            end
            if (tx_push_ok) begin
                tx_mem[tx_wptr[11:0]] <= {tx_hi, tx_lo};
                tx_wptr <= tx_wptr + 16'd1;
                tx_push_count <= tx_push_count + 16'd1;
            end
            if (tx_push_bad)
                tx_error <= 1'b1;
            if (tx_start) begin
                tx_raddr <= 16'd0;
                tx_started <= 1'b1;
            end
            if (acc_s_pop) begin
                tx_raddr <= tx_raddr + 16'd1;
                if (tx_raddr == (tx_len - 16'd1))
                    tx_started <= 1'b0;
            end
            if (status_clr && wdata[5])
                tx_error <= 1'b0;
        end
    end

    // ── RX BRAM + pointer ──────────────────────────────────────────────────
    always @(posedge clk) begin
        if (acc_m_fire)
            rx_mem[rx_wptr[11:0]] <= acc_m_tdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_wptr <= 0; rx_done <= 0; rx_overflow <= 0;
            rx_addr <= 0; rx_rdata <= 0; rx_rd_req <= 0;
        end else if (soft_rst) begin
            rx_wptr <= 0; rx_done <= 0; rx_overflow <= 0;
            rx_addr <= 0; rx_rdata <= 0; rx_rd_req <= 0;
        end else begin
            rx_rd_req <= 1'b0;

            if (send_start || (wr_en && (reg_sel == 5'd17))) begin
                rx_wptr <= 0; rx_done <= 0; rx_overflow <= 0;
            end
            if (wr_en && (reg_sel == 5'd9)) begin
                rx_addr  <= wdata[11:0];
                rx_rd_req <= 1'b1;
            end
            if (status_clr && wdata[4])
                rx_done <= 1'b0;
            if (status_clr && wdata[6])
                rx_overflow <= 1'b0;

            if (acc_m_fire) begin
                rx_wptr <= rx_wptr + 13'd1;
                if (acc_m_tlast)
                    rx_done <= 1'b1;
            end
            if (acc_m_tvalid && !acc_m_tready)
                rx_overflow <= 1'b1;

            if (rx_rd_req)
                rx_rdata <= rx_mem[rx_addr];
        end
    end

    // ── sticky done flags ──────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ap_done_s <= 0; conv_done_s <= 0; recv_done_s <= 0; send_done_s <= 0;
        end else if (soft_rst) begin
            ap_done_s <= 0; conv_done_s <= 0; recv_done_s <= 0; send_done_s <= 0;
        end else begin
            ap_done_s   <= (ap_done_s   | acc_ap_done)   & ~(status_clr && wdata[0]);
            conv_done_s <= (conv_done_s | acc_conv_done) & ~(status_clr && wdata[1]);
            recv_done_s <= (recv_done_s | acc_recv_done) & ~(status_clr && wdata[2]);
            send_done_s <= (send_done_s | acc_send_done) & ~(status_clr && wdata[3]);
        end
    end

    // ── read mux (combinational, sampled by axi_periph_bridge) ─────────────
    wire [31:0] tx_stat = {1'b0, tx_error, tx_full, tx_started,
                           tx_raddr[11:0], tx_push_count};
    wire [31:0] rx_stat = {17'b0, rx_overflow, rx_done, rx_wptr[12:0]};
    wire [31:0] status  = {23'b0, tx_started, rx_overflow, tx_error,
                           tx_full, rx_done, send_done_s, recv_done_s,
                           conv_done_s, ap_done_s};

    always @(*) begin
        rdata = 32'b0;
        if (rd_en) begin
            case (reg_sel)
                5'd0:  rdata = reg0;
                5'd1:  rdata = reg1;
                5'd2:  rdata = reg2;
                5'd3:  rdata = reg3;
                5'd4:  rdata = {16'b0, tx_len};
                5'd5:  rdata = tx_lo;
                5'd6:  rdata = tx_hi;
                5'd8:  rdata = tx_stat;
                5'd9:  rdata = {20'b0, rx_addr};
                5'd10: rdata = rx_rdata[31:0];
                5'd11: rdata = rx_rdata[63:32];
                5'd12: rdata = rx_stat;
                5'd13: rdata = status;
                5'd15: rdata = 32'hACCE0001;
                default: rdata = 32'b0;
            endcase
        end
    end

    // ── accelerator instance (AXI-Lite removed in the copied Accel_Conv.v) ─
    Accel_Conv u_accel (
        .clk           (clk),
        .rst           (accel_rst),
        .ap_done       (acc_ap_done),
        .reg_0         (reg0),
        .reg_1         (reg1),
        .reg_2         (reg2),
        .reg_3         (reg3),
        .conv_done_o   (acc_conv_done),
        .recv_done_o   (acc_recv_done),
        .send_done_o   (acc_send_done),
        .S_AXIS_tready (acc_s_tready),
        .S_AXIS_tdata  (acc_s_tdata),
        .S_AXIS_tlast  (acc_s_tlast),
        .S_AXIS_tvalid (acc_s_tvalid),
        .M_AXIS_tvalid (acc_m_tvalid),
        .M_AXIS_tdata  (acc_m_tdata),
        .M_AXIS_tlast  (acc_m_tlast),
        .M_AXIS_tready (acc_m_tready)
    );

endmodule