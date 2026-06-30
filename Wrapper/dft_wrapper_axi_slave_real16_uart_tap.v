`timescale 1ns / 1ps

// ============================================================================
// dft_wrapper_axi_slave_real16
//
// AXI-Lite controlled AXI4 master wrapper for the actual 16-sample DFT core.
//
// Actual DFT core:
//   dft_core_top
//     input  : 8 x 32-bit words = 16 samples
//     output : 1 x 32-bit packed word
//              [3:0] bin0, [7:4] bin1, ... [31:28] bin7
//
// This wrapper adapts that output to the current LED Matrix Writer format:
//   BRAM[OUT_ADDR + 0x00] = {28'd0, bin0}
//   BRAM[OUT_ADDR + 0x04] = {28'd0, bin1}
//   ...
//   BRAM[OUT_ADDR + 0x1C] = {28'd0, bin7}
//
// Register map:
//   0x00 CTRL        bit0=start, bit1=clear
//   0x04 STATUS      bit0=done, bit1=busy, bit3=error
//   0x08 IN_ADDR     packed input buffer base address
//   0x0C OUT_ADDR    result buffer base address
//   0x10 FRAME_SIZE  must be 16 for this core
//
// Done timing:
//   STATUS.done is asserted only after all 8 unpacked result words are written
//   to BRAM and the final AXI B response is received.
// ============================================================================

module dft_wrapper_axi_slave_real16 #(
    parameter AXIL_ADDR_W = 32,
    parameter AXIL_DATA_W = 32,
    parameter AXI_ADDR_W  = 32,
    parameter AXI_DATA_W  = 32,
    parameter LEN_W       = 16,

    // 0: pass packed13 data as positive signed 16-bit values.
    // 1: convert unsigned 13-bit XADC sample to signed by subtracting 4096.
    parameter CENTER_UNSIGNED_XADC = 0
)(
    input  wire                     clk,
    input  wire                     rst,

    // AXI-Lite slave
    input  wire [AXIL_ADDR_W-1:0]   s_axil_awaddr,
    input  wire                     s_axil_awvalid,
    output reg                      s_axil_awready,
    input  wire [AXIL_DATA_W-1:0]   s_axil_wdata,
    input  wire [AXIL_DATA_W/8-1:0] s_axil_wstrb,
    input  wire                     s_axil_wvalid,
    output reg                      s_axil_wready,
    output reg  [1:0]               s_axil_bresp,
    output reg                      s_axil_bvalid,
    input  wire                     s_axil_bready,
    input  wire [AXIL_ADDR_W-1:0]   s_axil_araddr,
    input  wire                     s_axil_arvalid,
    output reg                      s_axil_arready,
    output reg  [AXIL_DATA_W-1:0]   s_axil_rdata,
    output reg  [1:0]               s_axil_rresp,
    output reg                      s_axil_rvalid,
    input  wire                     s_axil_rready,

    // AXI4 master read channel
    output reg  [AXI_ADDR_W-1:0]    m_axi_araddr,
    output reg                      m_axi_arvalid,
    input  wire                     m_axi_arready,
    output wire [7:0]               m_axi_arlen,
    output wire [2:0]               m_axi_arsize,
    output wire [1:0]               m_axi_arburst,
    output wire                     m_axi_arlock,
    output wire [3:0]               m_axi_arcache,
    output wire [2:0]               m_axi_arprot,
    output wire [3:0]               m_axi_arqos,
    input  wire [AXI_DATA_W-1:0]    m_axi_rdata,
    input  wire [1:0]               m_axi_rresp,
    input  wire                     m_axi_rvalid,
    output reg                      m_axi_rready,
    input  wire                     m_axi_rlast,

    // AXI4 master write channel
    output reg  [AXI_ADDR_W-1:0]    m_axi_awaddr,
    output reg                      m_axi_awvalid,
    input  wire                     m_axi_awready,
    output wire [7:0]               m_axi_awlen,
    output wire [2:0]               m_axi_awsize,
    output wire [1:0]               m_axi_awburst,
    output wire                     m_axi_awlock,
    output wire [3:0]               m_axi_awcache,
    output wire [2:0]               m_axi_awprot,
    output wire [3:0]               m_axi_awqos,
    output reg  [AXI_DATA_W-1:0]    m_axi_wdata,
    output wire [AXI_DATA_W/8-1:0]  m_axi_wstrb,
    output reg                      m_axi_wvalid,
    input  wire                     m_axi_wready,
    output wire                     m_axi_wlast,
    input  wire [1:0]               m_axi_bresp,
    input  wire                     m_axi_bvalid,
    output reg                      m_axi_bready,

    // Real-board UART/ILA tap
    // 1-cycle pulse when the packed DFT result is generated.
    output reg                      dft_result_valid,
    output reg  [31:0]              dft_result_packed
);

    localparam REG_CTRL       = 8'h00;
    localparam REG_STATUS     = 8'h04;
    localparam REG_IN_ADDR    = 8'h08;
    localparam REG_OUT_ADDR   = 8'h0C;
    localparam REG_FRAME_SIZE = 8'h10;

    localparam STATUS_DONE  = 0;
    localparam STATUS_BUSY  = 1;
    localparam STATUS_ERROR = 3;

    localparam ST_IDLE             = 5'd0;
    localparam ST_READ_AR          = 5'd1;
    localparam ST_READ_R           = 5'd2;
    localparam ST_FEED_HOLD        = 5'd3;
    localparam ST_FRAME_START_SET  = 5'd4;
    localparam ST_FRAME_START_HOLD = 5'd5;
    localparam ST_WAIT_DFT_WE      = 5'd6;
    localparam ST_WRITE_SETUP      = 5'd7;
    localparam ST_WRITE_WAIT       = 5'd8;
    localparam ST_WRITE_B          = 5'd9;
    localparam ST_WRITE_NEXT       = 5'd10;
    localparam ST_DONE             = 5'd11;
    localparam ST_ERROR            = 5'd12;

    reg [31:0] ctrl_reg;
    reg [31:0] status_reg;
    reg [31:0] in_addr_reg;
    reg [31:0] out_addr_reg;
    reg [31:0] frame_size_reg;

    reg [4:0]  state;
    reg [2:0]  read_idx;
    reg [2:0]  write_idx;
    reg [31:0] read_word_reg;
    reg [31:0] result_packed_reg;
    reg        aw_done;
    reg        w_done;

    reg        core_frame_start;
    wire       core_frame_valid;
    reg        core_input_valid;
    reg [31:0] core_input_data;
    wire [3:0] core_system_dft_addr;
    wire [31:0] core_system_dft_wdata;
    wire       core_system_dft_we;

    assign m_axi_arlen   = 8'd0;
    assign m_axi_arsize  = 3'b010;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arqos   = 4'b0000;

    assign m_axi_awlen   = 8'd0;
    assign m_axi_awsize  = 3'b010;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awqos   = 4'b0000;
    assign m_axi_wstrb   = {AXI_DATA_W/8{1'b1}};
    assign m_axi_wlast   = 1'b1;

    dft_core_top u_dft_core_top (
        .clk              (clk),
        .rst              (rst),
        .frame_start      (core_frame_start),
        .frame_valid      (core_frame_valid),
        .input_buf_valid  (core_input_valid),
        .input_buf_data   (core_input_data),
        .system_dft_addr  (core_system_dft_addr),
        .system_dft_wdata (core_system_dft_wdata),
        .system_dft_we    (core_system_dft_we)
    );

    // ------------------------------------------------------------------------
    // DFT result tap for real-board UART CSV streaming.
    // The core output is captured as a clean 1-clock valid/data pair.
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            dft_result_valid  <= 1'b0;
            dft_result_packed <= 32'd0;
        end else begin
            dft_result_valid <= 1'b0;

            if (core_system_dft_we) begin
                dft_result_valid  <= 1'b1;
                dft_result_packed <= core_system_dft_wdata;
            end
        end
    end

    function [15:0] maybe_center_sample;
        input [12:0] sample13;
        reg signed [15:0] centered;
        begin
            if (CENTER_UNSIGNED_XADC) begin
                centered = $signed({3'b000, sample13}) - 16'sd4096;
                maybe_center_sample = centered[15:0];
            end else begin
                maybe_center_sample = {3'b000, sample13};
            end
        end
    endfunction

    function [31:0] adapt_input_word;
        input [31:0] packed13_word;
        reg [15:0] even16;
        reg [15:0] odd16;
        begin
            even16 = maybe_center_sample(packed13_word[12:0]);
            odd16  = maybe_center_sample(packed13_word[28:16]);
            adapt_input_word = {odd16, even16};
        end
    endfunction

    function [31:0] unpack_bin_word;
        input [31:0] packed_bins;
        input [2:0]  idx;
        begin
            case (idx)
                3'd0: unpack_bin_word = {28'd0, packed_bins[3:0]};
                3'd1: unpack_bin_word = {28'd0, packed_bins[7:4]};
                3'd2: unpack_bin_word = {28'd0, packed_bins[11:8]};
                3'd3: unpack_bin_word = {28'd0, packed_bins[15:12]};
                3'd4: unpack_bin_word = {28'd0, packed_bins[19:16]};
                3'd5: unpack_bin_word = {28'd0, packed_bins[23:20]};
                3'd6: unpack_bin_word = {28'd0, packed_bins[27:24]};
                3'd7: unpack_bin_word = {28'd0, packed_bins[31:28]};
                default: unpack_bin_word = 32'd0;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------------
    // AXI-Lite slave
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            s_axil_awready <= 1'b0;
            s_axil_wready  <= 1'b0;
            s_axil_bresp   <= 2'b00;
            s_axil_bvalid  <= 1'b0;
            s_axil_arready <= 1'b0;
            s_axil_rdata   <= 32'd0;
            s_axil_rresp   <= 2'b00;
            s_axil_rvalid  <= 1'b0;

            ctrl_reg       <= 32'd0;
            in_addr_reg    <= 32'd0;
            out_addr_reg   <= 32'd0;
            frame_size_reg <= 32'd16;
        end else begin
            s_axil_awready <= 1'b0;
            s_axil_wready  <= 1'b0;
            s_axil_arready <= 1'b0;

            if (!s_axil_bvalid && s_axil_awvalid && s_axil_wvalid) begin
                s_axil_awready <= 1'b1;
                s_axil_wready  <= 1'b1;
                s_axil_bvalid  <= 1'b1;
                s_axil_bresp   <= 2'b00;

                case (s_axil_awaddr[7:0])
                    REG_CTRL:       ctrl_reg <= s_axil_wdata;
                    REG_IN_ADDR:    in_addr_reg <= s_axil_wdata;
                    REG_OUT_ADDR:   out_addr_reg <= s_axil_wdata;
                    REG_FRAME_SIZE: frame_size_reg <= s_axil_wdata;
                    default: begin end
                endcase
            end else if (s_axil_bvalid && s_axil_bready) begin
                s_axil_bvalid <= 1'b0;
            end

            if (!s_axil_rvalid && s_axil_arvalid) begin
                s_axil_arready <= 1'b1;
                s_axil_rvalid  <= 1'b1;
                s_axil_rresp   <= 2'b00;

                case (s_axil_araddr[7:0])
                    REG_CTRL:       s_axil_rdata <= ctrl_reg;
                    REG_STATUS:     s_axil_rdata <= status_reg;
                    REG_IN_ADDR:    s_axil_rdata <= in_addr_reg;
                    REG_OUT_ADDR:   s_axil_rdata <= out_addr_reg;
                    REG_FRAME_SIZE: s_axil_rdata <= frame_size_reg;
                    default:        s_axil_rdata <= 32'd0;
                endcase
            end else if (s_axil_rvalid && s_axil_rready) begin
                s_axil_rvalid <= 1'b0;
            end
        end
    end

    wire clear_cmd_fire = (!s_axil_bvalid && s_axil_awvalid && s_axil_wvalid &&
                           s_axil_awaddr[7:0] == REG_CTRL && s_axil_wdata[1]);

    wire start_cmd_fire = (!s_axil_bvalid && s_axil_awvalid && s_axil_wvalid &&
                           s_axil_awaddr[7:0] == REG_CTRL && s_axil_wdata[0]);

    // ------------------------------------------------------------------------
    // Read input buffer -> feed DFT core -> write unpacked result words
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            status_reg        <= 32'd0;
            state             <= ST_IDLE;
            read_idx          <= 3'd0;
            write_idx         <= 3'd0;
            read_word_reg     <= 32'd0;
            result_packed_reg <= 32'd0;
            aw_done           <= 1'b0;
            w_done            <= 1'b0;

            core_frame_start  <= 1'b0;
            core_input_valid  <= 1'b0;
            core_input_data   <= 32'd0;

            m_axi_araddr      <= {AXI_ADDR_W{1'b0}};
            m_axi_arvalid     <= 1'b0;
            m_axi_rready      <= 1'b0;
            m_axi_awaddr      <= {AXI_ADDR_W{1'b0}};
            m_axi_awvalid     <= 1'b0;
            m_axi_wdata       <= {AXI_DATA_W{1'b0}};
            m_axi_wvalid      <= 1'b0;
            m_axi_bready      <= 1'b0;
        end else begin
            // default one-cycle pulses
            core_frame_start <= 1'b0;

            if (clear_cmd_fire) begin
                status_reg        <= 32'd0;
                state             <= ST_IDLE;
                read_idx          <= 3'd0;
                write_idx         <= 3'd0;
                result_packed_reg <= 32'd0;
                aw_done           <= 1'b0;
                w_done            <= 1'b0;

                core_frame_start  <= 1'b0;
                core_input_valid  <= 1'b0;
                core_input_data   <= 32'd0;

                m_axi_arvalid     <= 1'b0;
                m_axi_rready      <= 1'b0;
                m_axi_awvalid     <= 1'b0;
                m_axi_wvalid      <= 1'b0;
                m_axi_bready      <= 1'b0;
            end else begin
                case (state)
                    ST_IDLE: begin
                        core_input_valid <= 1'b0;
                        m_axi_arvalid    <= 1'b0;
                        m_axi_rready     <= 1'b0;
                        m_axi_awvalid    <= 1'b0;
                        m_axi_wvalid     <= 1'b0;
                        m_axi_bready     <= 1'b0;

                        if (start_cmd_fire) begin
                            status_reg        <= 32'd0;
                            status_reg[STATUS_BUSY] <= 1'b1;
                            read_idx          <= 3'd0;
                            write_idx         <= 3'd0;
                            aw_done           <= 1'b0;
                            w_done            <= 1'b0;
                            result_packed_reg <= 32'd0;

                            if (frame_size_reg != 32'd16) begin
                                status_reg[STATUS_BUSY]  <= 1'b0;
                                status_reg[STATUS_ERROR] <= 1'b1;
                                state <= ST_ERROR;
                            end else begin
                                state <= ST_READ_AR;
                            end
                        end
                    end

                    ST_READ_AR: begin
                        m_axi_araddr  <= in_addr_reg[AXI_ADDR_W-1:0] + ({29'd0, read_idx} << 2);
                        m_axi_arvalid <= 1'b1;

                        if (m_axi_arvalid && m_axi_arready) begin
                            m_axi_arvalid <= 1'b0;
                            m_axi_rready  <= 1'b1;
                            state <= ST_READ_R;
                        end
                    end

                    ST_READ_R: begin
                        if (m_axi_rvalid && m_axi_rready) begin
                            m_axi_rready <= 1'b0;

                            if (m_axi_rresp != 2'b00) begin
                                status_reg[STATUS_BUSY]  <= 1'b0;
                                status_reg[STATUS_ERROR] <= 1'b1;
                                state <= ST_ERROR;
                            end else begin
                                read_word_reg    <= m_axi_rdata;
                                core_input_data  <= adapt_input_word(m_axi_rdata);
                                core_input_valid <= 1'b1;
                                state <= ST_FEED_HOLD;
                            end
                        end
                    end

                    ST_FEED_HOLD: begin
                        // DFT core samples core_input_valid/core_input_data on this edge.
                        core_input_valid <= 1'b0;

                        if (read_idx == 3'd7) begin
                            state <= ST_FRAME_START_SET;
                        end else begin
                            read_idx <= read_idx + 3'd1;
                            state <= ST_READ_AR;
                        end
                    end

                    ST_FRAME_START_SET: begin
                        // One clean cycle after the 8th feed, fifo_prog_full is already stable.
                        core_frame_start <= 1'b1;
                        state <= ST_FRAME_START_HOLD;
                    end

                    ST_FRAME_START_HOLD: begin
                        // DFT core samples frame_start on this edge.
                        core_frame_start <= 1'b0;
                        state <= ST_WAIT_DFT_WE;
                    end

                    ST_WAIT_DFT_WE: begin
                        if (core_system_dft_we) begin
                            result_packed_reg <= core_system_dft_wdata;
                            write_idx <= 3'd0;
                            state <= ST_WRITE_SETUP;
                        end
                    end

                    ST_WRITE_SETUP: begin
                        m_axi_awaddr  <= out_addr_reg[AXI_ADDR_W-1:0] + ({29'd0, write_idx} << 2);
                        m_axi_wdata   <= unpack_bin_word(result_packed_reg, write_idx);
                        m_axi_awvalid <= 1'b1;
                        m_axi_wvalid  <= 1'b1;
                        aw_done       <= 1'b0;
                        w_done        <= 1'b0;
                        state <= ST_WRITE_WAIT;
                    end

                    ST_WRITE_WAIT: begin
                        if (m_axi_awvalid && m_axi_awready) begin
                            m_axi_awvalid <= 1'b0;
                            aw_done <= 1'b1;
                        end

                        if (m_axi_wvalid && m_axi_wready) begin
                            m_axi_wvalid <= 1'b0;
                            w_done <= 1'b1;
                        end

                        if ((aw_done || (m_axi_awvalid && m_axi_awready)) &&
                            (w_done  || (m_axi_wvalid  && m_axi_wready))) begin
                            m_axi_awvalid <= 1'b0;
                            m_axi_wvalid  <= 1'b0;
                            m_axi_bready  <= 1'b1;
                            state <= ST_WRITE_B;
                        end
                    end

                    ST_WRITE_B: begin
                        if (m_axi_bvalid && m_axi_bready) begin
                            m_axi_bready <= 1'b0;
                            aw_done <= 1'b0;
                            w_done  <= 1'b0;

                            if (m_axi_bresp != 2'b00) begin
                                status_reg[STATUS_BUSY]  <= 1'b0;
                                status_reg[STATUS_ERROR] <= 1'b1;
                                state <= ST_ERROR;
                            end else begin
                                state <= ST_WRITE_NEXT;
                            end
                        end
                    end

                    ST_WRITE_NEXT: begin
                        if (write_idx == 3'd7) begin
                            state <= ST_DONE;
                        end else begin
                            write_idx <= write_idx + 3'd1;
                            state <= ST_WRITE_SETUP;
                        end
                    end

                    ST_DONE: begin
                        status_reg[STATUS_BUSY] <= 1'b0;
                        status_reg[STATUS_DONE] <= 1'b1;
                        state <= ST_IDLE;
                    end

                    ST_ERROR: begin
                        status_reg[STATUS_BUSY] <= 1'b0;
                        state <= ST_ERROR;
                    end

                    default: begin
                        state <= ST_ERROR;
                        status_reg[STATUS_ERROR] <= 1'b1;
                    end
                endcase
            end
        end
    end

endmodule
