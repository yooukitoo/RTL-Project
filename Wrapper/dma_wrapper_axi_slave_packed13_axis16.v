`timescale 1ns / 1ps

// ============================================================================
// dma_wrapper_axi_slave_packed13_axis16
//
// AXI-Lite controlled AXI-Stream to AXI4 memory writer.
// This version packs two 13-bit samples into one 32-bit BRAM word.
//
// Input stream format:
//   s_axis_tdata[12:0] = one 13-bit XADC sample
//   s_axis_tdata[15:13] must be 3'b000
//
// This version is for a 16-bit AXI-Stream FIFO placed between XADC and DMA.
//
// BRAM packed word format:
//   [12:0]   = even sample
//   [15:13]  = 3'b000
//   [28:16]  = odd sample
//   [31:29]  = 3'b000
//
// Register map:
//   0x00 CTRL          bit0=start, bit1=clear
//   0x04 STATUS        bit0=done, bit1=busy, bit3=error
//   0x08 DST_ADDR      destination byte address
//   0x0C LENGTH        number of 13-bit samples, not number of 32-bit words
//   0x10 WRITTEN_COUNT number of samples accepted
//   0x14 LAST_ADDR     last AXI word address written
// ============================================================================

module dma_wrapper_axi_slave_packed13_axis16 #(
    parameter AXIL_ADDR_W = 32,
    parameter AXIL_DATA_W = 32,
    parameter AXI_ADDR_W  = 32,
    parameter AXI_DATA_W  = 32,
    parameter LEN_W       = 16
)(
    input  wire                     clk,
    input  wire                     rst, // active-high

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

    // AXI-Stream input, 16-bit lane from XADC/FIFO
    input  wire [15:0]              s_axis_tdata,
    input  wire                     s_axis_tvalid,
    output wire                     s_axis_tready,
    input  wire                     s_axis_tlast,

    // AXI4 master read channel, unused
    output wire [AXI_ADDR_W-1:0]    m_axi_araddr,
    output wire                     m_axi_arvalid,
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
    output wire                     m_axi_rready,
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
    output reg                      m_axi_bready
);

    localparam REG_CTRL          = 8'h00;
    localparam REG_STATUS        = 8'h04;
    localparam REG_DST_ADDR      = 8'h08;
    localparam REG_LENGTH        = 8'h0C;
    localparam REG_WRITTEN_COUNT = 8'h10;
    localparam REG_LAST_ADDR     = 8'h14;

    reg [31:0] ctrl_reg;
    reg [31:0] status_reg;
    reg [31:0] dst_addr_reg;
    reg [31:0] length_reg;
    reg [31:0] written_count_reg;
    reg [31:0] last_addr_reg;

    reg        running;
    reg [31:0] cur_addr;
    reg [12:0] even_sample;
    reg        have_even;
    reg        aw_done;
    reg        w_done;

    wire       write_active;
    wire       accept_sample;
    wire [12:0] in_sample;
    wire [31:0] next_count;
    wire       is_last_sample;
    wire       need_write_this_sample;
    wire [31:0] packed_pair;
    wire [31:0] packed_odd_last;

    assign write_active = m_axi_awvalid | m_axi_wvalid | m_axi_bready | aw_done | w_done;
    assign s_axis_tready = running && !write_active && (written_count_reg < length_reg);
    assign accept_sample = s_axis_tvalid && s_axis_tready;
    assign in_sample = s_axis_tdata[12:0];
    assign next_count = written_count_reg + 32'd1;
    assign is_last_sample = (next_count >= length_reg);
    assign need_write_this_sample = accept_sample && (have_even || is_last_sample);
    assign packed_pair     = {3'b000, in_sample,   3'b000, even_sample};
    assign packed_odd_last = {3'b000, 13'd0,       3'b000, in_sample};

    assign m_axi_araddr  = {AXI_ADDR_W{1'b0}};
    assign m_axi_arvalid = 1'b0;
    assign m_axi_arlen   = 8'd0;
    assign m_axi_arsize  = 3'b010;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arqos   = 4'b0000;
    assign m_axi_rready  = 1'b0;

    assign m_axi_awlen   = 8'd0;
    assign m_axi_awsize  = 3'b010;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awqos   = 4'b0000;
    assign m_axi_wstrb   = 4'hF;
    assign m_axi_wlast   = 1'b1;

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
            dst_addr_reg   <= 32'd0;
            length_reg     <= 32'd0;
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
                    REG_CTRL: begin
                        ctrl_reg <= s_axil_wdata;
                    end
                    REG_DST_ADDR: begin
                        dst_addr_reg <= s_axil_wdata;
                    end
                    REG_LENGTH: begin
                        length_reg <= s_axil_wdata;
                    end
                    default: begin
                    end
                endcase
            end else if (s_axil_bvalid && s_axil_bready) begin
                s_axil_bvalid <= 1'b0;
            end

            if (!s_axil_rvalid && s_axil_arvalid) begin
                s_axil_arready <= 1'b1;
                s_axil_rvalid  <= 1'b1;
                s_axil_rresp   <= 2'b00;
                case (s_axil_araddr[7:0])
                    REG_CTRL:          s_axil_rdata <= ctrl_reg;
                    REG_STATUS:        s_axil_rdata <= status_reg;
                    REG_DST_ADDR:      s_axil_rdata <= dst_addr_reg;
                    REG_LENGTH:        s_axil_rdata <= length_reg;
                    REG_WRITTEN_COUNT: s_axil_rdata <= written_count_reg;
                    REG_LAST_ADDR:     s_axil_rdata <= last_addr_reg;
                    default:           s_axil_rdata <= 32'd0;
                endcase
            end else if (s_axil_rvalid && s_axil_rready) begin
                s_axil_rvalid <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // DMA pack/write engine
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            status_reg        <= 32'd0;
            written_count_reg <= 32'd0;
            last_addr_reg     <= 32'd0;
            running           <= 1'b0;
            cur_addr          <= 32'd0;
            even_sample       <= 13'd0;
            have_even         <= 1'b0;
            m_axi_awaddr      <= {AXI_ADDR_W{1'b0}};
            m_axi_awvalid     <= 1'b0;
            m_axi_wdata       <= {AXI_DATA_W{1'b0}};
            m_axi_wvalid      <= 1'b0;
            m_axi_bready      <= 1'b0;
            aw_done           <= 1'b0;
            w_done            <= 1'b0;
        end else begin
            // clear command
            if (s_axil_awvalid && s_axil_wvalid && !s_axil_bvalid && s_axil_awaddr[7:0] == REG_CTRL && s_axil_wdata[1]) begin
                status_reg        <= 32'd0;
                written_count_reg <= 32'd0;
                last_addr_reg     <= 32'd0;
                running           <= 1'b0;
                have_even         <= 1'b0;
                m_axi_awvalid     <= 1'b0;
                m_axi_wvalid      <= 1'b0;
                m_axi_bready      <= 1'b0;
                aw_done           <= 1'b0;
                w_done            <= 1'b0;
            end

            // start command
            if (s_axil_awvalid && s_axil_wvalid && !s_axil_bvalid && s_axil_awaddr[7:0] == REG_CTRL && s_axil_wdata[0]) begin
                status_reg        <= 32'h0000_0002; // busy
                written_count_reg <= 32'd0;
                last_addr_reg     <= dst_addr_reg;
                running           <= 1'b1;
                cur_addr          <= dst_addr_reg;
                have_even         <= 1'b0;
                m_axi_awvalid     <= 1'b0;
                m_axi_wvalid      <= 1'b0;
                m_axi_bready      <= 1'b0;
                aw_done           <= 1'b0;
                w_done            <= 1'b0;
            end

            if (accept_sample) begin
                written_count_reg <= next_count;

                if (!have_even && !is_last_sample) begin
                    even_sample <= in_sample;
                    have_even   <= 1'b1;
                end else begin
                    m_axi_awaddr  <= cur_addr;
                    m_axi_awvalid <= 1'b1;
                    m_axi_wdata   <= have_even ? packed_pair : packed_odd_last;
                    m_axi_wvalid  <= 1'b1;
                    last_addr_reg <= cur_addr;
                    cur_addr      <= cur_addr + 32'd4;
                    have_even     <= 1'b0;
                    aw_done       <= 1'b0;
                    w_done        <= 1'b0;
                end
            end

            if (m_axi_awvalid && m_axi_awready) begin
                m_axi_awvalid <= 1'b0;
                aw_done <= 1'b1;
            end

            if (m_axi_wvalid && m_axi_wready) begin
                m_axi_wvalid <= 1'b0;
                w_done <= 1'b1;
            end

            if ((aw_done || (m_axi_awvalid && m_axi_awready)) &&
                (w_done  || (m_axi_wvalid  && m_axi_wready)) &&
                !m_axi_bready) begin
                m_axi_bready <= 1'b1;
            end

            if (m_axi_bready && m_axi_bvalid) begin
                m_axi_bready <= 1'b0;
                aw_done <= 1'b0;
                w_done <= 1'b0;
                if (m_axi_bresp != 2'b00) begin
                    status_reg[3] <= 1'b1;
                    running <= 1'b0;
                    status_reg[1] <= 1'b0;
                end
            end

            if (running && (length_reg == 32'd0)) begin
                running <= 1'b0;
                status_reg <= 32'h0000_0001; // done
            end else if (running && (written_count_reg >= length_reg) && !have_even && !write_active) begin
                running <= 1'b0;
                status_reg[0] <= 1'b1; // done
                status_reg[1] <= 1'b0; // busy clear
            end
        end
    end

endmodule
