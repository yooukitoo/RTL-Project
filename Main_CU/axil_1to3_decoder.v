`timescale 1ns / 1ps

// ============================================================================
// axil_1to3_decoder
//
// Simple AXI-Lite 1-master to 3-slave address decoder.
// Intended for main_CU control path.
//
// Address map:
//   0x4000_0000 ~ 0x4000_FFFF : M0, DMA
//   0x4001_0000 ~ 0x4001_FFFF : M1, DFT
//   0x4002_0000 ~ 0x4002_FFFF : M2, LED Writer
//
// Notes:
//   - This decoder is intentionally simple for this project.
//   - Write AW/W are accepted together. This matches main_cu_axil_master_frame,
//     which asserts AWVALID and WVALID together.
//   - Unsupported addresses return DECERR.
// ============================================================================

module axil_1to3_decoder #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter [31:0] M0_BASE = 32'h4000_0000,
    parameter [31:0] M1_BASE = 32'h4001_0000,
    parameter [31:0] M2_BASE = 32'h4002_0000
)(
    input  wire                    clk,
    input  wire                    rst,

    // Slave side, connected to one AXI-Lite master
    input  wire [ADDR_W-1:0]       s_awaddr,
    input  wire                    s_awvalid,
    output reg                     s_awready,

    input  wire [DATA_W-1:0]       s_wdata,
    input  wire [DATA_W/8-1:0]     s_wstrb,
    input  wire                    s_wvalid,
    output reg                     s_wready,

    output reg  [1:0]              s_bresp,
    output reg                     s_bvalid,
    input  wire                    s_bready,

    input  wire [ADDR_W-1:0]       s_araddr,
    input  wire                    s_arvalid,
    output reg                     s_arready,

    output reg  [DATA_W-1:0]       s_rdata,
    output reg  [1:0]              s_rresp,
    output reg                     s_rvalid,
    input  wire                    s_rready,

    // M0 side
    output reg  [ADDR_W-1:0]       m0_awaddr,
    output reg                     m0_awvalid,
    input  wire                    m0_awready,
    output reg  [DATA_W-1:0]       m0_wdata,
    output reg  [DATA_W/8-1:0]     m0_wstrb,
    output reg                     m0_wvalid,
    input  wire                    m0_wready,
    input  wire [1:0]              m0_bresp,
    input  wire                    m0_bvalid,
    output reg                     m0_bready,
    output reg  [ADDR_W-1:0]       m0_araddr,
    output reg                     m0_arvalid,
    input  wire                    m0_arready,
    input  wire [DATA_W-1:0]       m0_rdata,
    input  wire [1:0]              m0_rresp,
    input  wire                    m0_rvalid,
    output reg                     m0_rready,

    // M1 side
    output reg  [ADDR_W-1:0]       m1_awaddr,
    output reg                     m1_awvalid,
    input  wire                    m1_awready,
    output reg  [DATA_W-1:0]       m1_wdata,
    output reg  [DATA_W/8-1:0]     m1_wstrb,
    output reg                     m1_wvalid,
    input  wire                    m1_wready,
    input  wire [1:0]              m1_bresp,
    input  wire                    m1_bvalid,
    output reg                     m1_bready,
    output reg  [ADDR_W-1:0]       m1_araddr,
    output reg                     m1_arvalid,
    input  wire                    m1_arready,
    input  wire [DATA_W-1:0]       m1_rdata,
    input  wire [1:0]              m1_rresp,
    input  wire                    m1_rvalid,
    output reg                     m1_rready,

    // M2 side
    output reg  [ADDR_W-1:0]       m2_awaddr,
    output reg                     m2_awvalid,
    input  wire                    m2_awready,
    output reg  [DATA_W-1:0]       m2_wdata,
    output reg  [DATA_W/8-1:0]     m2_wstrb,
    output reg                     m2_wvalid,
    input  wire                    m2_wready,
    input  wire [1:0]              m2_bresp,
    input  wire                    m2_bvalid,
    output reg                     m2_bready,
    output reg  [ADDR_W-1:0]       m2_araddr,
    output reg                     m2_arvalid,
    input  wire                    m2_arready,
    input  wire [DATA_W-1:0]       m2_rdata,
    input  wire [1:0]              m2_rresp,
    input  wire                    m2_rvalid,
    output reg                     m2_rready
);

    localparam [1:0] SEL_NONE = 2'd0;
    localparam [1:0] SEL_M0   = 2'd1;
    localparam [1:0] SEL_M1   = 2'd2;
    localparam [1:0] SEL_M2   = 2'd3;

    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_DECERR = 2'b11;

    reg [1:0] wr_sel;
    reg [1:0] rd_sel;
    reg       wr_wait_resp;
    reg       rd_wait_resp;

    function [1:0] decode_addr;
        input [31:0] addr;
        begin
            if (addr[31:16] == M0_BASE[31:16]) begin
                decode_addr = SEL_M0;
            end else if (addr[31:16] == M1_BASE[31:16]) begin
                decode_addr = SEL_M1;
            end else if (addr[31:16] == M2_BASE[31:16]) begin
                decode_addr = SEL_M2;
            end else begin
                decode_addr = SEL_NONE;
            end
        end
    endfunction

    always @(*) begin
        // defaults
        s_awready = 1'b0;
        s_wready  = 1'b0;
        s_arready = 1'b0;

        m0_awaddr  = s_awaddr;
        m0_awvalid = 1'b0;
        m0_wdata   = s_wdata;
        m0_wstrb   = s_wstrb;
        m0_wvalid  = 1'b0;
        m0_bready  = 1'b0;
        m0_araddr  = s_araddr;
        m0_arvalid = 1'b0;
        m0_rready  = 1'b0;

        m1_awaddr  = s_awaddr;
        m1_awvalid = 1'b0;
        m1_wdata   = s_wdata;
        m1_wstrb   = s_wstrb;
        m1_wvalid  = 1'b0;
        m1_bready  = 1'b0;
        m1_araddr  = s_araddr;
        m1_arvalid = 1'b0;
        m1_rready  = 1'b0;

        m2_awaddr  = s_awaddr;
        m2_awvalid = 1'b0;
        m2_wdata   = s_wdata;
        m2_wstrb   = s_wstrb;
        m2_wvalid  = 1'b0;
        m2_bready  = 1'b0;
        m2_araddr  = s_araddr;
        m2_arvalid = 1'b0;
        m2_rready  = 1'b0;

        s_bresp  = RESP_OKAY;
        s_bvalid = 1'b0;
        s_rdata  = {DATA_W{1'b0}};
        s_rresp  = RESP_OKAY;
        s_rvalid = 1'b0;

        // Write address/data forwarding.
        // This decoder accepts AW/W together only.
        if (!wr_wait_resp && s_awvalid && s_wvalid) begin
            case (decode_addr(s_awaddr))
                SEL_M0: begin
                    m0_awvalid = 1'b1;
                    m0_wvalid  = 1'b1;
                    s_awready  = m0_awready && m0_wready;
                    s_wready   = m0_awready && m0_wready;
                end
                SEL_M1: begin
                    m1_awvalid = 1'b1;
                    m1_wvalid  = 1'b1;
                    s_awready  = m1_awready && m1_wready;
                    s_wready   = m1_awready && m1_wready;
                end
                SEL_M2: begin
                    m2_awvalid = 1'b1;
                    m2_wvalid  = 1'b1;
                    s_awready  = m2_awready && m2_wready;
                    s_wready   = m2_awready && m2_wready;
                end
                default: begin
                    s_awready = 1'b1;
                    s_wready  = 1'b1;
                end
            endcase
        end

        // Write response routing.
        if (wr_wait_resp) begin
            case (wr_sel)
                SEL_M0: begin
                    s_bvalid   = m0_bvalid;
                    s_bresp    = m0_bresp;
                    m0_bready  = s_bready;
                end
                SEL_M1: begin
                    s_bvalid   = m1_bvalid;
                    s_bresp    = m1_bresp;
                    m1_bready  = s_bready;
                end
                SEL_M2: begin
                    s_bvalid   = m2_bvalid;
                    s_bresp    = m2_bresp;
                    m2_bready  = s_bready;
                end
                default: begin
                    s_bvalid = 1'b1;
                    s_bresp  = RESP_DECERR;
                end
            endcase
        end

        // Read address forwarding.
        if (!rd_wait_resp && s_arvalid) begin
            case (decode_addr(s_araddr))
                SEL_M0: begin
                    m0_arvalid = 1'b1;
                    s_arready  = m0_arready;
                end
                SEL_M1: begin
                    m1_arvalid = 1'b1;
                    s_arready  = m1_arready;
                end
                SEL_M2: begin
                    m2_arvalid = 1'b1;
                    s_arready  = m2_arready;
                end
                default: begin
                    s_arready = 1'b1;
                end
            endcase
        end

        // Read data routing.
        if (rd_wait_resp) begin
            case (rd_sel)
                SEL_M0: begin
                    s_rvalid  = m0_rvalid;
                    s_rdata   = m0_rdata;
                    s_rresp   = m0_rresp;
                    m0_rready = s_rready;
                end
                SEL_M1: begin
                    s_rvalid  = m1_rvalid;
                    s_rdata   = m1_rdata;
                    s_rresp   = m1_rresp;
                    m1_rready = s_rready;
                end
                SEL_M2: begin
                    s_rvalid  = m2_rvalid;
                    s_rdata   = m2_rdata;
                    s_rresp   = m2_rresp;
                    m2_rready = s_rready;
                end
                default: begin
                    s_rvalid = 1'b1;
                    s_rdata  = {DATA_W{1'b0}};
                    s_rresp  = RESP_DECERR;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            wr_wait_resp <= 1'b0;
            rd_wait_resp <= 1'b0;
            wr_sel       <= SEL_NONE;
            rd_sel       <= SEL_NONE;
        end else begin
            if (!wr_wait_resp && s_awvalid && s_wvalid && s_awready && s_wready) begin
                wr_wait_resp <= 1'b1;
                wr_sel       <= decode_addr(s_awaddr);
            end else if (wr_wait_resp && s_bvalid && s_bready) begin
                wr_wait_resp <= 1'b0;
                wr_sel       <= SEL_NONE;
            end

            if (!rd_wait_resp && s_arvalid && s_arready) begin
                rd_wait_resp <= 1'b1;
                rd_sel       <= decode_addr(s_araddr);
            end else if (rd_wait_resp && s_rvalid && s_rready) begin
                rd_wait_resp <= 1'b0;
                rd_sel       <= SEL_NONE;
            end
        end
    end

endmodule
