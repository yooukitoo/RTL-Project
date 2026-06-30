`timescale 1ns / 1ps

// ============================================================================
// tb_main_cu_dma_dft_led_real_no_xadc
//
// XADC core is not included yet.
// This TB verifies:
//
//   main_CU
//     -> AXI-Lite decoder
//       -> DMA packed13
//       -> actual DFT wrapper + actual dft_core_top
//       -> actual LED Matrix Writer
//
// Data source is a TB-only XADC AXI-Stream model.
// Later, the board top will replace this model with the real xadc_wiz_0 IP.
// ============================================================================

module tb_main_cu_dma_dft_led_real_no_xadc;

    localparam AXIL_ADDR_W = 32;
    localparam AXIL_DATA_W = 32;
    localparam AXI_ADDR_W  = 32;
    localparam AXI_DATA_W  = 32;
    localparam LEN_W       = 16;

    localparam FRAME_SIZE  = 32'd16;
    localparam LED_BINS    = 32'd8;
    localparam LED_SCALE   = 32'd0;

    reg clk;
    reg rst;
    wire bram_aresetn = ~rst;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    initial begin
        $dumpfile("tb_main_cu_dma_dft_led_real_no_xadc.vcd");
        $dumpvars(0, tb_main_cu_dma_dft_led_real_no_xadc);
    end

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

    reg main_start_i;
    reg main_clear_i;

    wire capture_start;
    wire capture_done;
    wire capture_busy;


    wire [15:0] xadc_tdata;
    wire        xadc_tvalid;
    wire        xadc_tready;
    wire [4:0]  xadc_tid;

    wire main_busy;
    wire main_done;
    wire main_error;
    wire [7:0] main_led_debug;
    wire [7:0] main_state;
    wire [31:0] main_last_status;

    wire [15:0] cap_fifo_tdata;
    wire        cap_fifo_tvalid;
    wire        cap_fifo_tready;
    wire        cap_fifo_tlast;

    wire [15:0] fifo_dma_tdata;
    wire        fifo_dma_tvalid;
    wire        fifo_dma_tready;
    wire        fifo_dma_tlast;

    wire max7219_din;
    wire max7219_cs;
    wire max7219_clk;

    integer timeout;
    integer fail_count;

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

    reg [15:0] xadc_tdata_r;
    reg        xadc_tvalid_r;
    reg [4:0]  xadc_tid_r;
    reg [11:0] xadc_raw12_r;

    assign xadc_tdata  = xadc_tdata_r;
    assign xadc_tvalid = xadc_tvalid_r;
    assign xadc_tid    = xadc_tid_r;

    always @(posedge clk) begin
        if (rst) begin
            xadc_tdata_r  <= 16'd0;
            xadc_tvalid_r <= 1'b0;
            xadc_tid_r    <= 5'h16;
            xadc_raw12_r  <= 12'h700;
        end else begin
            xadc_tvalid_r <= 1'b1;
            xadc_tid_r    <= 5'h16;

            if (xadc_tready) begin
                xadc_tdata_r <= {xadc_raw12_r, 4'b0000};

                // Repeat a simple waveform-like ramp around the XADC mid-scale.
                if (xadc_raw12_r >= 12'hB00)
                    xadc_raw12_r <= 12'h700;
                else
                    xadc_raw12_r <= xadc_raw12_r + 12'h080;
            end
        end
    end
    xadc_capture_to_axis_fifo16 #(
    .FRAME_SIZE(16),
    .TID_W(5),
    .USE_TID_FILTER(1'b0),
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

    dft_wrapper_axi_slave_real16 #(
        .CENTER_UNSIGNED_XADC(1)
    ) u_dft (
        .clk(clk),
        .rst(rst),
        `AXIL_SLAVE_CONNECT(dft_axil),
        `AXI_MASTER_CONNECT(dft_axi)
    );

    led_matrix_writer_wrapper_axi_slave u_led (
        .clk(clk),
        .rst(rst),
        `AXIL_SLAVE_CONNECT(led_axil),
        `AXI_MASTER_CONNECT(led_axi),

        .max7219_din(max7219_din),
        .max7219_cs(max7219_cs),
        .max7219_clk(max7219_clk)
    );

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

    initial begin
        fail_count = 0;
        timeout = 0;
        main_start_i = 1'b0;
        main_clear_i = 1'b0;

        rst = 1'b1;
        repeat (30) @(posedge clk);
        rst = 1'b0;
        repeat (20) @(posedge clk);

        $display("============================================================");
        $display("TB: main_CU + DMA + actual DFT core + actual LED Matrix Writer");
        $display("XADC is replaced by TB AXI-Stream model");
        $display("============================================================");

        @(negedge clk);
        main_start_i = 1'b1;
        @(negedge clk);
        main_start_i = 1'b0;

        while (!main_done && !main_error && timeout < 30000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        if (timeout >= 30000) begin
            $display("[FAIL] Timeout. main_state=%0d last_status=0x%08h", main_state, main_last_status);
            fail_count = fail_count + 1;
        end

        if (main_error) begin
            $display("[FAIL] main_CU error. main_state=%0d last_status=0x%08h", main_state, main_last_status);
            fail_count = fail_count + 1;
        end else begin
            $display("[PASS] main_CU error is 0");
        end

        if (!main_done) begin
            $display("[FAIL] main_CU done not asserted");
            fail_count = fail_count + 1;
        end else begin
            $display("[PASS] main_CU done asserted");
        end

        if (u_dft.status_reg[0] !== 1'b1) begin
            $display("[FAIL] DFT done status not set. status=0x%08h", u_dft.status_reg);
            fail_count = fail_count + 1;
        end else begin
            $display("[PASS] DFT done status set. packed_result=0x%08h", u_dft.result_packed_reg);
        end

        if (u_led.led_done_latched !== 1'b1) begin
            $display("[FAIL] LED done latch not set. status=0x%08h", u_led.led_status_value);
            fail_count = fail_count + 1;
        end else begin
            $display("[PASS] LED done latch set. led_status=0x%08h", u_led.led_status_value);
        end

        $display("[INFO] DFT unpacked result read by LED = 0x%064h", u_led.read_data_flat);
        $display("[INFO] LED row0 debug from main_CU = 0x%02h", main_led_debug);

        $display("============================================================");
        if (fail_count == 0) begin
            $display("[FINAL PASS] tb_main_cu_dma_dft_led_real_no_xadc");
        end else begin
            $display("[FINAL FAIL] fail_count=%0d", fail_count);
        end
        $display("============================================================");

        repeat (50) @(posedge clk);
        $finish;
    end

endmodule