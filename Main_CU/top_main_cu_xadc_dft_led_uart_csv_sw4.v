`timescale 1ns / 1ps

// ============================================================================
// top_main_cu_xadc_dft_led_autorun
//
// Board top for:
//   XADC Wizard AXI-Stream 16-bit
//     -> xadc_capture_to_axis_fifo16
//     -> 16-bit AXI-Stream FIFO
//     -> dma_wrapper_axi_slave_packed13_axis16
//     -> AXI BRAM BD
//     -> dft_wrapper_axi_slave_real16 + actual DFT core
//     -> led_matrix_writer_wrapper_axi_slave
//
// Notes:
// - This top uses the real xadc_wiz_0 IP.
// - start_btn is used as continuous RUN enable.
// - This top does NOT use the TB XADC stream model.
// - XADC raw format assumed:
//      xadc_tdata[15:4] = raw12
//      xadc_tdata[3:0]  = unused/padding
// - xadc_capture_to_axis_fifo16 converts:
//      sample13 = {raw12, 1'b0}
//      fifo16   = {3'b000, sample13}
// - DFT wrapper must use CENTER_UNSIGNED_XADC=1.
// ============================================================================

module top_main_cu_xadc_dft_led_uart_csv_sw4 #(
    parameter AXIL_ADDR_W = 32,
    parameter AXIL_DATA_W = 32,
    parameter AXI_ADDR_W  = 32,
    parameter AXI_DATA_W  = 32,

    parameter [31:0] FRAME_SIZE = 32'd16,
    parameter [31:0] LED_BINS   = 32'd8,
    parameter [31:0] LED_SCALE  = 32'd0,

    // 100 MHz 기준: 20_000_000 cycles = 200 ms.
    // LED Matrix refresh 속도가 아니라 DFT frame 갱신 속도를 조절하는 값.
    // 너무 빠르면 50_000_000(0.5초)까지 늘려도 됨.
    parameter [31:0] INTER_FRAME_DELAY_CYCLES = 32'd20_000_000,

    // Basys3 USB-UART output baudrate
    parameter UART_BAUD = 115200
)(
    input  wire clk,          // Basys3 100 MHz clock
    input  wire rstn,         // active-low reset, recommended: connect to SW or inverted button
    input  wire start_btn,    // auto-run enable switch: 1=continuous frames, 0=stop after current frame
    input  wire clear_btn,    // manual clear pulse input

    // XADC VAUX6 pins
    input  wire vauxp6,
    input  wire vauxn6,

    // MAX7219 LED Matrix
    output wire max7219_din,
    output wire max7219_cs,
    output wire max7219_clk,

    // Optional board debug LEDs
    output wire [15:0] debug_led,

    // SW4 = 1이면 DFT 결과 UART 기록 enable
    input  wire        uart_log_enable,

    // Basys3 USB-UART TX to PC
    output wire uart_tx
);

    wire rst = ~rstn;
    wire bram_aresetn = ~rst;

    // ------------------------------------------------------------------------
    // Auto-run control
    // ------------------------------------------------------------------------
    // start_btn is used as RUN enable in this auto-run top.
    //   start_btn = 1 : continuously process frames
    //   start_btn = 0 : stop after current frame and return to idle
    // clear_btn can still be used as a manual clear pulse.
    //
    // main_CU itself is still a one-frame controller.
    // This top-level auto controller repeatedly gives:
    //   start pulse -> wait done -> clear pulse -> delay -> next start pulse
    // ------------------------------------------------------------------------
    wire main_start_i;
    wire main_clear_i;

    // ------------------------------------------------------------------------
    // AXI-Lite / AXI-Full macros
    // ------------------------------------------------------------------------
`define AXIL_DECL(P) \
    wire [AXIL_ADDR_W-1:0]   P``_awaddr; \
    wire                     P``_awvalid; \
    wire                     P``_awready; \
    wire [AXIL_DATA_W-1:0]   P``_wdata; \
    wire [AXIL_DATA_W/8-1:0] P``_wstrb; \
    wire                     P``_wvalid; \
    wire                     P``_wready; \
    wire [1:0]               P``_bresp; \
    wire                     P``_bvalid; \
    wire                     P``_bready; \
    wire [AXIL_ADDR_W-1:0]   P``_araddr; \
    wire                     P``_arvalid; \
    wire                     P``_arready; \
    wire [AXIL_DATA_W-1:0]   P``_rdata; \
    wire [1:0]               P``_rresp; \
    wire                     P``_rvalid; \
    wire                     P``_rready;

`define AXIL_SLAVE_CONNECT(P) \
    .s_axil_awaddr(P``_awaddr), \
    .s_axil_awvalid(P``_awvalid), \
    .s_axil_awready(P``_awready), \
    .s_axil_wdata(P``_wdata), \
    .s_axil_wstrb(P``_wstrb), \
    .s_axil_wvalid(P``_wvalid), \
    .s_axil_wready(P``_wready), \
    .s_axil_bresp(P``_bresp), \
    .s_axil_bvalid(P``_bvalid), \
    .s_axil_bready(P``_bready), \
    .s_axil_araddr(P``_araddr), \
    .s_axil_arvalid(P``_arvalid), \
    .s_axil_arready(P``_arready), \
    .s_axil_rdata(P``_rdata), \
    .s_axil_rresp(P``_rresp), \
    .s_axil_rvalid(P``_rvalid), \
    .s_axil_rready(P``_rready)

`define AXI_FULL_DECL(P) \
    wire [AXI_ADDR_W-1:0]   P``_araddr; \
    wire                    P``_arvalid; \
    wire                    P``_arready; \
    wire [7:0]              P``_arlen; \
    wire [2:0]              P``_arsize; \
    wire [1:0]              P``_arburst; \
    wire                    P``_arlock; \
    wire [3:0]              P``_arcache; \
    wire [2:0]              P``_arprot; \
    wire [3:0]              P``_arqos; \
    wire [AXI_DATA_W-1:0]   P``_rdata; \
    wire [1:0]              P``_rresp; \
    wire                    P``_rvalid; \
    wire                    P``_rready; \
    wire                    P``_rlast; \
    wire [AXI_ADDR_W-1:0]   P``_awaddr; \
    wire                    P``_awvalid; \
    wire                    P``_awready; \
    wire [7:0]              P``_awlen; \
    wire [2:0]              P``_awsize; \
    wire [1:0]              P``_awburst; \
    wire                    P``_awlock; \
    wire [3:0]              P``_awcache; \
    wire [2:0]              P``_awprot; \
    wire [3:0]              P``_awqos; \
    wire [AXI_DATA_W-1:0]   P``_wdata; \
    wire [AXI_DATA_W/8-1:0] P``_wstrb; \
    wire                    P``_wvalid; \
    wire                    P``_wready; \
    wire                    P``_wlast; \
    wire [1:0]              P``_bresp; \
    wire                    P``_bvalid; \
    wire                    P``_bready;

`define AXI_MASTER_CONNECT(P) \
    .m_axi_araddr(P``_araddr), \
    .m_axi_arvalid(P``_arvalid), \
    .m_axi_arready(P``_arready), \
    .m_axi_arlen(P``_arlen), \
    .m_axi_arsize(P``_arsize), \
    .m_axi_arburst(P``_arburst), \
    .m_axi_arlock(P``_arlock), \
    .m_axi_arcache(P``_arcache), \
    .m_axi_arprot(P``_arprot), \
    .m_axi_arqos(P``_arqos), \
    .m_axi_rdata(P``_rdata), \
    .m_axi_rresp(P``_rresp), \
    .m_axi_rvalid(P``_rvalid), \
    .m_axi_rready(P``_rready), \
    .m_axi_rlast(P``_rlast), \
    .m_axi_awaddr(P``_awaddr), \
    .m_axi_awvalid(P``_awvalid), \
    .m_axi_awready(P``_awready), \
    .m_axi_awlen(P``_awlen), \
    .m_axi_awsize(P``_awsize), \
    .m_axi_awburst(P``_awburst), \
    .m_axi_awlock(P``_awlock), \
    .m_axi_awcache(P``_awcache), \
    .m_axi_awprot(P``_awprot), \
    .m_axi_awqos(P``_awqos), \
    .m_axi_wdata(P``_wdata), \
    .m_axi_wstrb(P``_wstrb), \
    .m_axi_wvalid(P``_wvalid), \
    .m_axi_wready(P``_wready), \
    .m_axi_wlast(P``_wlast), \
    .m_axi_bresp(P``_bresp), \
    .m_axi_bvalid(P``_bvalid), \
    .m_axi_bready(P``_bready)

`define BRAM_PORT_CONNECT(PORT, P) \
    .PORT``_araddr(P``_araddr), \
    .PORT``_arvalid(P``_arvalid), \
    .PORT``_arready(P``_arready), \
    .PORT``_arlen(P``_arlen), \
    .PORT``_arsize(P``_arsize), \
    .PORT``_arburst(P``_arburst), \
    .PORT``_arlock(P``_arlock), \
    .PORT``_arcache(P``_arcache), \
    .PORT``_arprot(P``_arprot), \
    .PORT``_arqos(P``_arqos), \
    .PORT``_rdata(P``_rdata), \
    .PORT``_rresp(P``_rresp), \
    .PORT``_rvalid(P``_rvalid), \
    .PORT``_rready(P``_rready), \
    .PORT``_rlast(P``_rlast), \
    .PORT``_awaddr(P``_awaddr), \
    .PORT``_awvalid(P``_awvalid), \
    .PORT``_awready(P``_awready), \
    .PORT``_awlen(P``_awlen), \
    .PORT``_awsize(P``_awsize), \
    .PORT``_awburst(P``_awburst), \
    .PORT``_awlock(P``_awlock), \
    .PORT``_awcache(P``_awcache), \
    .PORT``_awprot(P``_awprot), \
    .PORT``_awqos(P``_awqos), \
    .PORT``_wdata(P``_wdata), \
    .PORT``_wstrb(P``_wstrb), \
    .PORT``_wvalid(P``_wvalid), \
    .PORT``_wready(P``_wready), \
    .PORT``_wlast(P``_wlast), \
    .PORT``_bresp(P``_bresp), \
    .PORT``_bvalid(P``_bvalid), \
    .PORT``_bready(P``_bready)

    `AXIL_DECL(main_axil)
    `AXIL_DECL(dma_axil)
    `AXIL_DECL(dft_axil)
    `AXIL_DECL(led_axil)

    `AXI_FULL_DECL(dma_axi)
    `AXI_FULL_DECL(dft_axi)
    `AXI_FULL_DECL(led_axi)

    // ------------------------------------------------------------------------
    // main CU / capture status
    // ------------------------------------------------------------------------
    wire capture_start;
    wire capture_done;
    wire capture_busy;

    wire main_busy;
    wire main_done;
    wire main_error;
    wire [7:0]  main_led_debug;
    wire [7:0]  main_state;
    wire [31:0] main_last_status;

    // ------------------------------------------------------------------------
    // Auto-run FSM
    // ------------------------------------------------------------------------
    localparam [2:0] AUTO_IDLE       = 3'd0;
    localparam [2:0] AUTO_WAIT_DONE  = 3'd1;
    localparam [2:0] AUTO_WAIT_CLEAR = 3'd2;
    localparam [2:0] AUTO_GAP        = 3'd3;
    localparam [2:0] AUTO_ERROR      = 3'd4;

    reg [2:0]  auto_state;
    reg [31:0] inter_frame_cnt;
    reg        main_start_r;
    reg        main_clear_r;
    reg        clear_d;
    reg        frame_seen_latched;

    wire clear_edge = clear_btn & ~clear_d;

    assign main_start_i = main_start_r;
    assign main_clear_i = main_clear_r | clear_edge;

    always @(posedge clk) begin
        if (rst) begin
            clear_d            <= 1'b0;
            auto_state         <= AUTO_IDLE;
            inter_frame_cnt    <= 32'd0;
            main_start_r       <= 1'b0;
            main_clear_r       <= 1'b0;
            frame_seen_latched <= 1'b0;
        end else begin
            clear_d      <= clear_btn;
            main_start_r <= 1'b0;
            main_clear_r <= 1'b0;

            if (clear_edge && !start_btn) begin
                frame_seen_latched <= 1'b0;
            end

            case (auto_state)
                AUTO_IDLE: begin
                    inter_frame_cnt <= 32'd0;

                    if (start_btn) begin
                        if (main_done || main_error) begin
                            main_clear_r <= 1'b1;
                            auto_state   <= AUTO_WAIT_CLEAR;
                        end else begin
                            main_start_r <= 1'b1;
                            auto_state   <= AUTO_WAIT_DONE;
                        end
                    end
                end

                AUTO_WAIT_DONE: begin
                    if (main_error) begin
                        auto_state <= AUTO_ERROR;
                    end else if (main_done) begin
                        frame_seen_latched <= 1'b1;
                        main_clear_r       <= 1'b1;
                        auto_state         <= AUTO_WAIT_CLEAR;
                    end
                end

                AUTO_WAIT_CLEAR: begin
                    if (!main_done && !main_error) begin
                        inter_frame_cnt <= 32'd0;
                        if (start_btn)
                            auto_state <= AUTO_GAP;
                        else
                            auto_state <= AUTO_IDLE;
                    end
                end

                AUTO_GAP: begin
                    if (!start_btn) begin
                        auto_state <= AUTO_IDLE;
                    end else if (inter_frame_cnt >= INTER_FRAME_DELAY_CYCLES) begin
                        inter_frame_cnt <= 32'd0;
                        main_start_r    <= 1'b1;
                        auto_state      <= AUTO_WAIT_DONE;
                    end else begin
                        inter_frame_cnt <= inter_frame_cnt + 32'd1;
                    end
                end

                AUTO_ERROR: begin
                    if (!start_btn || clear_edge) begin
                        main_clear_r <= 1'b1;
                        auto_state   <= AUTO_WAIT_CLEAR;
                    end
                end

                default: begin
                    auto_state <= AUTO_IDLE;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // XADC AXI-Stream
    // ------------------------------------------------------------------------
    (* mark_debug = "true" *) wire [15:0] xadc_tdata;
    (* mark_debug = "true" *) wire        xadc_tvalid;
    (* mark_debug = "true" *) wire        xadc_tready;
    (* mark_debug = "true" *) wire [4:0]  xadc_tid;

    wire [4:0]  xadc_channel_out;
    wire        xadc_busy_out;
    wire        xadc_eoc_out;
    wire        xadc_eos_out;
    wire        xadc_alarm_out;

    // ------------------------------------------------------------------------
    // 16-bit AXI-Stream FIFO between XADC capture and custom DMA
    // ------------------------------------------------------------------------
    (* mark_debug = "true" *) wire [15:0] cap_fifo_tdata;
    (* mark_debug = "true" *) wire        cap_fifo_tvalid;
    (* mark_debug = "true" *) wire        cap_fifo_tready;
    (* mark_debug = "true" *) wire        cap_fifo_tlast;

    (* mark_debug = "true" *) wire [15:0] fifo_dma_tdata;
    (* mark_debug = "true" *) wire        fifo_dma_tvalid;
    (* mark_debug = "true" *) wire        fifo_dma_tready;
    (* mark_debug = "true" *) wire        fifo_dma_tlast;

    // ------------------------------------------------------------------------
    // main CU
    // ------------------------------------------------------------------------
    main_cu_axil_master_frame_real #(
        .FRAME_SIZE(FRAME_SIZE),
        .LED_BIN_COUNT(LED_BINS),
        .LED_SCALE(LED_SCALE)
    ) u_main_cu (
        .clk(clk),
        .rst(rst),
        .start_i(main_start_i),
        .clear_i(main_clear_i),

        .xadc_start_o(capture_start),
        .xadc_done_i(capture_done),
        .xadc_busy_i(capture_busy),

        .busy_o(main_busy),
        .done_o(main_done),
        .error_o(main_error),
        .led_value_o(main_led_debug),
        .state_o(main_state),
        .last_status_o(main_last_status),

        .m_axil_awaddr(main_axil_awaddr),
        .m_axil_awvalid(main_axil_awvalid),
        .m_axil_awready(main_axil_awready),
        .m_axil_wdata(main_axil_wdata),
        .m_axil_wstrb(main_axil_wstrb),
        .m_axil_wvalid(main_axil_wvalid),
        .m_axil_wready(main_axil_wready),
        .m_axil_bresp(main_axil_bresp),
        .m_axil_bvalid(main_axil_bvalid),
        .m_axil_bready(main_axil_bready),
        .m_axil_araddr(main_axil_araddr),
        .m_axil_arvalid(main_axil_arvalid),
        .m_axil_arready(main_axil_arready),
        .m_axil_rdata(main_axil_rdata),
        .m_axil_rresp(main_axil_rresp),
        .m_axil_rvalid(main_axil_rvalid),
        .m_axil_rready(main_axil_rready)
    );

    // ------------------------------------------------------------------------
    // AXI-Lite decoder: main_CU -> DMA / DFT / LED
    // ------------------------------------------------------------------------
    axil_1to3_decoder u_axil_dec (
        .clk(clk), .rst(rst),

        .s_awaddr(main_axil_awaddr), .s_awvalid(main_axil_awvalid), .s_awready(main_axil_awready),
        .s_wdata(main_axil_wdata), .s_wstrb(main_axil_wstrb), .s_wvalid(main_axil_wvalid), .s_wready(main_axil_wready),
        .s_bresp(main_axil_bresp), .s_bvalid(main_axil_bvalid), .s_bready(main_axil_bready),
        .s_araddr(main_axil_araddr), .s_arvalid(main_axil_arvalid), .s_arready(main_axil_arready),
        .s_rdata(main_axil_rdata), .s_rresp(main_axil_rresp), .s_rvalid(main_axil_rvalid), .s_rready(main_axil_rready),

        .m0_awaddr(dma_axil_awaddr), .m0_awvalid(dma_axil_awvalid), .m0_awready(dma_axil_awready),
        .m0_wdata(dma_axil_wdata), .m0_wstrb(dma_axil_wstrb), .m0_wvalid(dma_axil_wvalid), .m0_wready(dma_axil_wready),
        .m0_bresp(dma_axil_bresp), .m0_bvalid(dma_axil_bvalid), .m0_bready(dma_axil_bready),
        .m0_araddr(dma_axil_araddr), .m0_arvalid(dma_axil_arvalid), .m0_arready(dma_axil_arready),
        .m0_rdata(dma_axil_rdata), .m0_rresp(dma_axil_rresp), .m0_rvalid(dma_axil_rvalid), .m0_rready(dma_axil_rready),

        .m1_awaddr(dft_axil_awaddr), .m1_awvalid(dft_axil_awvalid), .m1_awready(dft_axil_awready),
        .m1_wdata(dft_axil_wdata), .m1_wstrb(dft_axil_wstrb), .m1_wvalid(dft_axil_wvalid), .m1_wready(dft_axil_wready),
        .m1_bresp(dft_axil_bresp), .m1_bvalid(dft_axil_bvalid), .m1_bready(dft_axil_bready),
        .m1_araddr(dft_axil_araddr), .m1_arvalid(dft_axil_arvalid), .m1_arready(dft_axil_arready),
        .m1_rdata(dft_axil_rdata), .m1_rresp(dft_axil_rresp), .m1_rvalid(dft_axil_rvalid), .m1_rready(dft_axil_rready),

        .m2_awaddr(led_axil_awaddr), .m2_awvalid(led_axil_awvalid), .m2_awready(led_axil_awready),
        .m2_wdata(led_axil_wdata), .m2_wstrb(led_axil_wstrb), .m2_wvalid(led_axil_wvalid), .m2_wready(led_axil_wready),
        .m2_bresp(led_axil_bresp), .m2_bvalid(led_axil_bvalid), .m2_bready(led_axil_bready),
        .m2_araddr(led_axil_araddr), .m2_arvalid(led_axil_arvalid), .m2_arready(led_axil_arready),
        .m2_rdata(led_axil_rdata), .m2_rresp(led_axil_rresp), .m2_rvalid(led_axil_rvalid), .m2_rready(led_axil_rready)
    );

    // ------------------------------------------------------------------------
    // Real XADC Wizard IP
    //
    // If your generated xadc_wiz_0 port names differ, open xadc_wiz_0.v and
    // adjust this instance to match the exact module declaration.
    // ------------------------------------------------------------------------
    xadc_wiz_0 u_xadc_wiz_0 (
        .s_axi_aclk    (clk),
        .s_axi_aresetn (~rst),

        .s_axi_awaddr  (11'd0),
        .s_axi_awvalid (1'b0),
        .s_axi_awready (),

        .s_axi_wdata   (32'd0),
        .s_axi_wstrb   (4'd0),
        .s_axi_wvalid  (1'b0),
        .s_axi_wready  (),

        .s_axi_bresp   (),
        .s_axi_bvalid  (),
        .s_axi_bready  (1'b1),

        .s_axi_araddr  (11'd0),
        .s_axi_arvalid (1'b0),
        .s_axi_arready (),

        .s_axi_rdata   (),
        .s_axi_rresp   (),
        .s_axi_rvalid  (),
        .s_axi_rready  (1'b1),

        .ip2intc_irpt  (),

        .s_axis_aclk   (clk),
        .m_axis_tdata  (xadc_tdata),
        .m_axis_tvalid (xadc_tvalid),
        .m_axis_tid    (xadc_tid),
        .m_axis_tready (xadc_tready),

        .vauxp6        (vauxp6),
        .vauxn6        (vauxn6),

        .channel_out   (xadc_channel_out),
        .busy_out      (xadc_busy_out),
        .eoc_out       (xadc_eoc_out),
        .eos_out       (xadc_eos_out),
        .alarm_out     (xadc_alarm_out),

        .vp_in         (1'b0),
        .vn_in         (1'b0)
    );

    // ------------------------------------------------------------------------
    // XADC capture: take 16 VAUX6 samples and emit 16-bit stream to FIFO
    // ------------------------------------------------------------------------
    xadc_capture_to_axis_fifo16 #(
        .FRAME_SIZE(16),
        .TID_W(5),
        .USE_TID_FILTER(1'b0),     // VAUX6 only enabled. Set to 1 if more channels are enabled.
        .TID_VALUE(5'h16),
        .RAW12_TO_13_SHIFT(1'b1)
    ) u_xadc_capture (
        .clk            (clk),
        .rst            (rst),

        .start_i        (capture_start),
        .busy_o         (capture_busy),
        .done_o         (capture_done),

        .s_xadc_tdata   (xadc_tdata),
        .s_xadc_tvalid  (xadc_tvalid),
        .s_xadc_tready  (xadc_tready),
        .s_xadc_tid     (xadc_tid),

        .m_axis_tdata   (cap_fifo_tdata),
        .m_axis_tvalid  (cap_fifo_tvalid),
        .m_axis_tready  (cap_fifo_tready),
        .m_axis_tlast   (cap_fifo_tlast)
    );

    // ------------------------------------------------------------------------
    // 16-bit AXI-Stream FIFO
    // ------------------------------------------------------------------------
    axis_fifo_32_simple #(
        .DATA_W(16),
        .DEPTH(32),
        .PTR_W(5)
    ) u_axis_fifo (
        .clk(clk),
        .rst(rst),

        .s_axis_tdata(cap_fifo_tdata),
        .s_axis_tvalid(cap_fifo_tvalid),
        .s_axis_tready(cap_fifo_tready),
        .s_axis_tlast(cap_fifo_tlast),

        .m_axis_tdata(fifo_dma_tdata),
        .m_axis_tvalid(fifo_dma_tvalid),
        .m_axis_tready(fifo_dma_tready),
        .m_axis_tlast(fifo_dma_tlast)
    );

    // ------------------------------------------------------------------------
    // Custom DMA: 16-bit stream samples -> 32-bit AXI-MM BRAM writes
    // ------------------------------------------------------------------------
    dma_wrapper_axi_slave_packed13_axis16 u_dma (
        .clk(clk),
        .rst(rst),
        `AXIL_SLAVE_CONNECT(dma_axil),

        .s_axis_tdata(fifo_dma_tdata),
        .s_axis_tvalid(fifo_dma_tvalid),
        .s_axis_tready(fifo_dma_tready),
        .s_axis_tlast(fifo_dma_tlast),

        `AXI_MASTER_CONNECT(dma_axi)
    );

    // ------------------------------------------------------------------------
    // DFT result tap for UART CSV output
    // ------------------------------------------------------------------------
    wire        dft_result_valid;
    wire [31:0] dft_result_packed;

    // ------------------------------------------------------------------------
    // DFT wrapper/core
    // CENTER_UNSIGNED_XADC=1 because BRAM raw samples are unsigned 13-bit:
    //   sample13 = {xadc_raw12, 1'b0}; center ~= 4096
    // ------------------------------------------------------------------------
    dft_wrapper_axi_slave_real16 #(
        .CENTER_UNSIGNED_XADC(1)
    ) u_dft (
        .clk(clk),
        .rst(rst),
        `AXIL_SLAVE_CONNECT(dft_axil),
        `AXI_MASTER_CONNECT(dft_axi),

        .dft_result_valid (dft_result_valid),
        .dft_result_packed(dft_result_packed)
    );

    // ------------------------------------------------------------------------
    // UART CSV output for real board
    //
    // UART line format:
    //   0xFRAME,0xPACKED\r\n
    //
    // Python expands PACKED to bin0~bin7 and saves CSV.
    // ------------------------------------------------------------------------
    dft_uart_csv_streamer #(
        .CLK_FREQ_HZ(100_000_000),
        .BAUD       (UART_BAUD)
    ) u_dft_uart_csv_streamer (
        .clk          (clk),
        .rst          (rst),
        .result_valid (dft_result_valid & uart_log_enable),
        .result_packed(dft_result_packed),
        .uart_tx      (uart_tx),
        .busy         ()
    );

    // ------------------------------------------------------------------------
    // LED Matrix writer
    // ------------------------------------------------------------------------
    led_matrix_writer_wrapper_axi_slave u_led (
        .clk(clk),
        .rst(rst),
        `AXIL_SLAVE_CONNECT(led_axil),
        `AXI_MASTER_CONNECT(led_axi),

        .max7219_din(max7219_din),
        .max7219_cs(max7219_cs),
        .max7219_clk(max7219_clk)
    );

    // ------------------------------------------------------------------------
    // AXI BRAM Block Design
    //
    // This instance assumes your new BRAM BD wrapper is named design_2_wrapper.
    // If your BD wrapper has a different name, change only this module name.
    // ------------------------------------------------------------------------
    design_2_wrapper u_bram_bd (
        .ACLK_0(clk),
        .ARESETN_0(bram_aresetn),

        .S00_AXI_0_arid(4'd0),
        .S00_AXI_0_awid(4'd0),

        .S01_AXI_0_arid(4'd0),
        .S01_AXI_0_awid(4'd0),

        .S02_AXI_0_arid(4'd0),
        .S02_AXI_0_awid(4'd0),

        `BRAM_PORT_CONNECT(S00_AXI_0, dma_axi),
        `BRAM_PORT_CONNECT(S01_AXI_0, dft_axi),
        `BRAM_PORT_CONNECT(S02_AXI_0, led_axi)
    );

    // ------------------------------------------------------------------------
    // Debug LED assignment
    // ------------------------------------------------------------------------
    assign debug_led[0]  = main_busy;
    // In auto-run mode main_done is cleared immediately for the next frame.
    // Therefore LED1 is a latched "at least one frame completed" indicator.
    assign debug_led[1]  = frame_seen_latched;
    assign debug_led[2]  = main_error;
    assign debug_led[3]  = capture_busy;
    assign debug_led[4]  = capture_done;
    assign debug_led[5]  = xadc_tvalid;
    assign debug_led[6]  = xadc_tready;
    assign debug_led[7]  = (auto_state != AUTO_IDLE);

    assign debug_led[15:8] = main_state;

`undef AXIL_DECL
`undef AXIL_SLAVE_CONNECT
`undef AXI_FULL_DECL
`undef AXI_MASTER_CONNECT
`undef BRAM_PORT_CONNECT

endmodule
