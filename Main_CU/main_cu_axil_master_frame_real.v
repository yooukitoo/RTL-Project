`timescale 1ns / 1ps

// ============================================================================
// main_cu_axil_master_frame.v
//
// Main Control Unit for the current fixed structure:
//
// [XADC 16-bit AXI-Stream]
//      ↓
// [XADC Capture Wrapper / 32-bit FIFO]
//      ↓
// [DMA packed13]
//      ↓ M_AXI
// [64KB BRAM BD]
//      ↓
// [DFT unpack13]
//      ↓ M_AXI
// [BRAM result buffer]
//      ↓
// [LED Writer]
//
// This main_CU DOES NOT change the big data structure.
// It only controls the frame sequence.
//
// Control method:
//   - XADC Capture Wrapper is controlled by direct start/done signals.
//   - DMA / DFT / LED Writer are controlled through AXI-Lite register writes.
//   - DMA/DFT/LED status are checked through AXI-Lite polling.
//
// Frame sequence:
//   1. DMA clear
//   2. DMA config: DST_ADDR, LENGTH
//   3. DMA start
//   4. XADC capture start
//   5. wait XADC capture done
//   6. wait DMA done
//   7. DFT clear/config/start
//   8. wait DFT done
//   9. LED clear/config/start
//  10. wait LED done
//  11. read LED_VALUE
//  12. main done
//
// AXI-Lite address map:
//   DMA CSR : 0x4000_0000
//   DFT CSR : 0x4001_0000
//   LED CSR : 0x4002_0000
//
// Shared BRAM address map:
//   RAW_SAMPLE_BASE = 0xC000_0000
//   RESULT_BASE     = 0xC000_F000
// ============================================================================

module main_cu_axil_master_frame_real #(
    parameter AXIL_ADDR_W = 32,
    parameter AXIL_DATA_W = 32,

    parameter [31:0] DMA_BASE_ADDR    = 32'h4000_0000,
    parameter [31:0] DFT_BASE_ADDR    = 32'h4001_0000,
    parameter [31:0] LED_BASE_ADDR    = 32'h4002_0000,

    parameter [31:0] RAW_SAMPLE_BASE  = 32'hC000_0000,
    parameter [31:0] RESULT_BASE      = 32'hC000_F000,

    // Actual DFT core uses 16 samples per frame.
    // Later, for long capture, use frame segmentation rather than sending 30000 samples into this 16-point DFT core at once.
    parameter [31:0] FRAME_SIZE       = 32'd16,

    // LED Writer reads DFT result bins.
    parameter [31:0] LED_BIN_COUNT    = 32'd8,
    parameter [31:0] LED_SCALE        = 32'd0,

    parameter [15:0] POLL_WAIT_CYCLES = 16'd16
)(
    input  wire                     clk,
    input  wire                     rst,        // active-high reset

    input  wire                     start_i,    // one-cycle pulse recommended
    input  wire                     clear_i,    // clear main CU done/error state

    // XADC Capture Wrapper direct control
    output reg                      xadc_start_o, // one-cycle pulse
    input  wire                     xadc_done_i,
    input  wire                     xadc_busy_i,

    // Main status
    output reg                      busy_o,
    output reg                      done_o,
    output reg                      error_o,
    output reg  [7:0]               led_value_o,
    output reg  [7:0]               state_o,
    output reg  [31:0]              last_status_o,

    // AXI-Lite master
    output reg  [AXIL_ADDR_W-1:0]   m_axil_awaddr,
    output reg                      m_axil_awvalid,
    input  wire                     m_axil_awready,

    output reg  [AXIL_DATA_W-1:0]   m_axil_wdata,
    output reg  [AXIL_DATA_W/8-1:0] m_axil_wstrb,
    output reg                      m_axil_wvalid,
    input  wire                     m_axil_wready,

    input  wire [1:0]               m_axil_bresp,
    input  wire                     m_axil_bvalid,
    output reg                      m_axil_bready,

    output reg  [AXIL_ADDR_W-1:0]   m_axil_araddr,
    output reg                      m_axil_arvalid,
    input  wire                     m_axil_arready,

    input  wire [AXIL_DATA_W-1:0]   m_axil_rdata,
    input  wire [1:0]               m_axil_rresp,
    input  wire                     m_axil_rvalid,
    output reg                      m_axil_rready
);

    // ------------------------------------------------------------------------
    // Register offsets
    // ------------------------------------------------------------------------
    localparam [31:0] REG_CTRL        = 32'h0000_0000;
    localparam [31:0] REG_STATUS      = 32'h0000_0004;

    localparam [31:0] DMA_DST_ADDR    = 32'h0000_0008;
    localparam [31:0] DMA_LENGTH      = 32'h0000_000C;

    localparam [31:0] DFT_IN_ADDR     = 32'h0000_0008;
    localparam [31:0] DFT_OUT_ADDR    = 32'h0000_000C;
    localparam [31:0] DFT_FRAME_SIZE  = 32'h0000_0010;

    localparam [31:0] LED_RESULT_ADDR = 32'h0000_0008;
    localparam [31:0] LED_BIN_COUNT_R = 32'h0000_000C;
    localparam [31:0] LED_SCALE_R     = 32'h0000_0010;
    localparam [31:0] LED_VALUE_R     = 32'h0000_0014;

    localparam [31:0] CTRL_START      = 32'h0000_0001;
    localparam [31:0] CTRL_CLEAR      = 32'h0000_0002;

    // STATUS bit assumption
    // bit0 = done, bit1 = busy, bit3 = error
    localparam STATUS_DONE_BIT  = 0;
    localparam STATUS_ERROR_BIT = 3;

    // ------------------------------------------------------------------------
    // FSM states
    // ------------------------------------------------------------------------
    localparam [7:0] ST_IDLE              = 8'd0;

    localparam [7:0] ST_DMA_CLEAR         = 8'd10;
    localparam [7:0] ST_DMA_CFG_DST       = 8'd11;
    localparam [7:0] ST_DMA_CFG_LEN       = 8'd12;
    localparam [7:0] ST_DMA_START         = 8'd13;
    localparam [7:0] ST_XADC_START        = 8'd14;
    localparam [7:0] ST_XADC_WAIT_DONE    = 8'd15;
    localparam [7:0] ST_DMA_POLL          = 8'd16;
    localparam [7:0] ST_DMA_CHECK         = 8'd17;
    localparam [7:0] ST_DMA_WAIT          = 8'd18;

    localparam [7:0] ST_DFT_CLEAR         = 8'd30;
    localparam [7:0] ST_DFT_CFG_IN        = 8'd31;
    localparam [7:0] ST_DFT_CFG_OUT       = 8'd32;
    localparam [7:0] ST_DFT_CFG_SIZE      = 8'd33;
    localparam [7:0] ST_DFT_START         = 8'd34;
    localparam [7:0] ST_DFT_POLL          = 8'd35;
    localparam [7:0] ST_DFT_CHECK         = 8'd36;
    localparam [7:0] ST_DFT_WAIT          = 8'd37;

    localparam [7:0] ST_LED_CLEAR         = 8'd50;
    localparam [7:0] ST_LED_CFG_ADDR      = 8'd51;
    localparam [7:0] ST_LED_CFG_COUNT     = 8'd52;
    localparam [7:0] ST_LED_CFG_THRESH    = 8'd53;
    localparam [7:0] ST_LED_START         = 8'd54;
    localparam [7:0] ST_LED_POLL          = 8'd55;
    localparam [7:0] ST_LED_CHECK         = 8'd56;
    localparam [7:0] ST_LED_WAIT          = 8'd57;
    localparam [7:0] ST_LED_READ_VALUE    = 8'd58;
    localparam [7:0] ST_LED_SAVE_VALUE    = 8'd59;

    localparam [7:0] ST_DONE              = 8'd80;
    localparam [7:0] ST_ERROR             = 8'd81;

    // AXI-Lite helper states
    localparam [7:0] ST_AXIL_WRITE        = 8'd100;
    localparam [7:0] ST_AXIL_WRITE_RESP   = 8'd101;
    localparam [7:0] ST_AXIL_READ         = 8'd102;
    localparam [7:0] ST_AXIL_READ_DATA    = 8'd103;

    reg [7:0]  state;
    reg [7:0]  return_state;

    reg [31:0] wr_addr;
    reg [31:0] wr_data;
    reg [31:0] rd_addr;
    reg [31:0] rd_data;

    reg        aw_done;
    reg        w_done;
    reg [15:0] poll_wait_cnt;

    reg        start_d;
    wire       start_pulse;

    assign start_pulse = start_i & ~start_d;

    // ------------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state             <= ST_IDLE;
            return_state      <= ST_IDLE;

            xadc_start_o      <= 1'b0;
            busy_o            <= 1'b0;
            done_o            <= 1'b0;
            error_o           <= 1'b0;
            led_value_o       <= 8'd0;
            state_o           <= ST_IDLE;
            last_status_o     <= 32'd0;

            wr_addr           <= 32'd0;
            wr_data           <= 32'd0;
            rd_addr           <= 32'd0;
            rd_data           <= 32'd0;

            aw_done           <= 1'b0;
            w_done            <= 1'b0;
            poll_wait_cnt     <= 16'd0;
            start_d           <= 1'b0;

            m_axil_awaddr     <= {AXIL_ADDR_W{1'b0}};
            m_axil_awvalid    <= 1'b0;
            m_axil_wdata      <= {AXIL_DATA_W{1'b0}};
            m_axil_wstrb      <= {AXIL_DATA_W/8{1'b1}};
            m_axil_wvalid     <= 1'b0;
            m_axil_bready     <= 1'b0;

            m_axil_araddr     <= {AXIL_ADDR_W{1'b0}};
            m_axil_arvalid    <= 1'b0;
            m_axil_rready     <= 1'b0;
        end else begin
            start_d <= start_i;
            state_o <= state;

            // Default pulse value
            xadc_start_o <= 1'b0;

            if (clear_i) begin
                state          <= ST_IDLE;
                xadc_start_o   <= 1'b0;
                busy_o         <= 1'b0;
                done_o         <= 1'b0;
                error_o        <= 1'b0;
                led_value_o    <= 8'd0;
                last_status_o  <= 32'd0;

                m_axil_awvalid <= 1'b0;
                m_axil_wvalid  <= 1'b0;
                m_axil_bready  <= 1'b0;
                m_axil_arvalid <= 1'b0;
                m_axil_rready  <= 1'b0;
                aw_done        <= 1'b0;
                w_done         <= 1'b0;
            end else begin
                case (state)

                    // --------------------------------------------------------
                    // IDLE
                    // --------------------------------------------------------
                    ST_IDLE: begin
                        busy_o <= 1'b0;
                        done_o <= 1'b0;
                        error_o <= 1'b0;

                        if (start_pulse) begin
                            busy_o        <= 1'b1;
                            done_o        <= 1'b0;
                            error_o       <= 1'b0;
                            led_value_o   <= 8'd0;
                            last_status_o <= 32'd0;
                            state         <= ST_DMA_CLEAR;
                        end
                    end

                    // --------------------------------------------------------
                    // DMA setup/start
                    // --------------------------------------------------------
                    ST_DMA_CLEAR: begin
                        wr_addr      <= DMA_BASE_ADDR + REG_CTRL;
                        wr_data      <= CTRL_CLEAR;
                        return_state <= ST_DMA_CFG_DST;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_DMA_CFG_DST: begin
                        wr_addr      <= DMA_BASE_ADDR + DMA_DST_ADDR;
                        wr_data      <= RAW_SAMPLE_BASE;
                        return_state <= ST_DMA_CFG_LEN;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_DMA_CFG_LEN: begin
                        wr_addr      <= DMA_BASE_ADDR + DMA_LENGTH;
                        wr_data      <= FRAME_SIZE;
                        return_state <= ST_DMA_START;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_DMA_START: begin
                        wr_addr      <= DMA_BASE_ADDR + REG_CTRL;
                        wr_data      <= CTRL_START;
                        return_state <= ST_XADC_START;
                        state        <= ST_AXIL_WRITE;
                    end

                    // --------------------------------------------------------
                    // XADC capture frame
                    // --------------------------------------------------------
                    ST_XADC_START: begin
                        // One-clock start pulse to XADC Capture Wrapper.
                        // The capture wrapper will push FRAME_SIZE samples
                        // into the 32-bit FIFO feeding the packed13 DMA.
                        if (!xadc_busy_i) begin
                            xadc_start_o <= 1'b1;
                            state <= ST_XADC_WAIT_DONE;
                        end else begin
                            state <= ST_XADC_START;
                        end
                    end

                    ST_XADC_WAIT_DONE: begin
                        if (xadc_done_i) begin
                            state <= ST_DMA_POLL;
                        end
                    end

                    // --------------------------------------------------------
                    // DMA polling
                    // --------------------------------------------------------
                    ST_DMA_POLL: begin
                        rd_addr      <= DMA_BASE_ADDR + REG_STATUS;
                        return_state <= ST_DMA_CHECK;
                        state        <= ST_AXIL_READ;
                    end

                    ST_DMA_CHECK: begin
                        last_status_o <= rd_data;
                        if (rd_data[STATUS_ERROR_BIT]) begin
                            state <= ST_ERROR;
                        end else if (rd_data[STATUS_DONE_BIT]) begin
                            state <= ST_DFT_CLEAR;
                        end else begin
                            poll_wait_cnt <= POLL_WAIT_CYCLES;
                            state <= ST_DMA_WAIT;
                        end
                    end

                    ST_DMA_WAIT: begin
                        if (poll_wait_cnt == 16'd0) begin
                            state <= ST_DMA_POLL;
                        end else begin
                            poll_wait_cnt <= poll_wait_cnt - 16'd1;
                        end
                    end

                    // --------------------------------------------------------
                    // DFT setup/start
                    // --------------------------------------------------------
                    ST_DFT_CLEAR: begin
                        wr_addr      <= DFT_BASE_ADDR + REG_CTRL;
                        wr_data      <= CTRL_CLEAR;
                        return_state <= ST_DFT_CFG_IN;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_DFT_CFG_IN: begin
                        wr_addr      <= DFT_BASE_ADDR + DFT_IN_ADDR;
                        wr_data      <= RAW_SAMPLE_BASE;
                        return_state <= ST_DFT_CFG_OUT;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_DFT_CFG_OUT: begin
                        wr_addr      <= DFT_BASE_ADDR + DFT_OUT_ADDR;
                        wr_data      <= RESULT_BASE;
                        return_state <= ST_DFT_CFG_SIZE;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_DFT_CFG_SIZE: begin
                        wr_addr      <= DFT_BASE_ADDR + DFT_FRAME_SIZE;
                        wr_data      <= FRAME_SIZE;
                        return_state <= ST_DFT_START;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_DFT_START: begin
                        wr_addr      <= DFT_BASE_ADDR + REG_CTRL;
                        wr_data      <= CTRL_START;
                        return_state <= ST_DFT_POLL;
                        state        <= ST_AXIL_WRITE;
                    end

                    // --------------------------------------------------------
                    // DFT polling
                    // --------------------------------------------------------
                    ST_DFT_POLL: begin
                        rd_addr      <= DFT_BASE_ADDR + REG_STATUS;
                        return_state <= ST_DFT_CHECK;
                        state        <= ST_AXIL_READ;
                    end

                    ST_DFT_CHECK: begin
                        last_status_o <= rd_data;
                        if (rd_data[STATUS_ERROR_BIT]) begin
                            state <= ST_ERROR;
                        end else if (rd_data[STATUS_DONE_BIT]) begin
                            state <= ST_LED_CLEAR;
                        end else begin
                            poll_wait_cnt <= POLL_WAIT_CYCLES;
                            state <= ST_DFT_WAIT;
                        end
                    end

                    ST_DFT_WAIT: begin
                        if (poll_wait_cnt == 16'd0) begin
                            state <= ST_DFT_POLL;
                        end else begin
                            poll_wait_cnt <= poll_wait_cnt - 16'd1;
                        end
                    end

                    // --------------------------------------------------------
                    // LED setup/start
                    // --------------------------------------------------------
                    ST_LED_CLEAR: begin
                        wr_addr      <= LED_BASE_ADDR + REG_CTRL;
                        wr_data      <= CTRL_CLEAR;
                        return_state <= ST_LED_CFG_ADDR;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_LED_CFG_ADDR: begin
                        wr_addr      <= LED_BASE_ADDR + LED_RESULT_ADDR;
                        wr_data      <= RESULT_BASE;
                        return_state <= ST_LED_CFG_COUNT;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_LED_CFG_COUNT: begin
                        wr_addr      <= LED_BASE_ADDR + LED_BIN_COUNT_R;
                        wr_data      <= LED_BIN_COUNT;
                        return_state <= ST_LED_CFG_THRESH;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_LED_CFG_THRESH: begin
                        wr_addr      <= LED_BASE_ADDR + LED_SCALE_R;
                        wr_data      <= LED_SCALE;
                        return_state <= ST_LED_START;
                        state        <= ST_AXIL_WRITE;
                    end

                    ST_LED_START: begin
                        wr_addr      <= LED_BASE_ADDR + REG_CTRL;
                        wr_data      <= CTRL_START;
                        return_state <= ST_LED_POLL;
                        state        <= ST_AXIL_WRITE;
                    end

                    // --------------------------------------------------------
                    // LED polling
                    // --------------------------------------------------------
                    ST_LED_POLL: begin
                        rd_addr      <= LED_BASE_ADDR + REG_STATUS;
                        return_state <= ST_LED_CHECK;
                        state        <= ST_AXIL_READ;
                    end

                    ST_LED_CHECK: begin
                        last_status_o <= rd_data;
                        if (rd_data[STATUS_ERROR_BIT]) begin
                            state <= ST_ERROR;
                        end else if (rd_data[STATUS_DONE_BIT]) begin
                            state <= ST_LED_READ_VALUE;
                        end else begin
                            poll_wait_cnt <= POLL_WAIT_CYCLES;
                            state <= ST_LED_WAIT;
                        end
                    end

                    ST_LED_WAIT: begin
                        if (poll_wait_cnt == 16'd0) begin
                            state <= ST_LED_POLL;
                        end else begin
                            poll_wait_cnt <= poll_wait_cnt - 16'd1;
                        end
                    end

                    ST_LED_READ_VALUE: begin
                        rd_addr      <= LED_BASE_ADDR + LED_VALUE_R;
                        return_state <= ST_LED_SAVE_VALUE;
                        state        <= ST_AXIL_READ;
                    end

                    ST_LED_SAVE_VALUE: begin
                        led_value_o <= rd_data[7:0];
                        state <= ST_DONE;
                    end

                    // --------------------------------------------------------
                    // DONE / ERROR
                    // --------------------------------------------------------
                    ST_DONE: begin
                        busy_o  <= 1'b0;
                        done_o  <= 1'b1;
                        error_o <= 1'b0;

                        // Allow re-start without requiring external clear.
                        if (start_pulse) begin
                            busy_o        <= 1'b1;
                            done_o        <= 1'b0;
                            error_o       <= 1'b0;
                            led_value_o   <= 8'd0;
                            last_status_o <= 32'd0;
                            state         <= ST_DMA_CLEAR;
                        end
                    end

                    ST_ERROR: begin
                        busy_o  <= 1'b0;
                        done_o  <= 1'b0;
                        error_o <= 1'b1;
                        // Hold until clear_i.
                    end

                    // --------------------------------------------------------
                    // AXI-Lite write helper
                    // --------------------------------------------------------
                    ST_AXIL_WRITE: begin
                        if (!aw_done && !m_axil_awvalid) begin
                            m_axil_awaddr  <= wr_addr[AXIL_ADDR_W-1:0];
                            m_axil_awvalid <= 1'b1;
                        end

                        if (!w_done && !m_axil_wvalid) begin
                            m_axil_wdata  <= wr_data;
                            m_axil_wstrb  <= {AXIL_DATA_W/8{1'b1}};
                            m_axil_wvalid <= 1'b1;
                        end

                        if (m_axil_awvalid && m_axil_awready) begin
                            m_axil_awvalid <= 1'b0;
                            aw_done <= 1'b1;
                        end

                        if (m_axil_wvalid && m_axil_wready) begin
                            m_axil_wvalid <= 1'b0;
                            w_done <= 1'b1;
                        end

                        if ((aw_done || (m_axil_awvalid && m_axil_awready)) &&
                            (w_done  || (m_axil_wvalid  && m_axil_wready))) begin
                            m_axil_awvalid <= 1'b0;
                            m_axil_wvalid  <= 1'b0;
                            m_axil_bready  <= 1'b1;
                            state <= ST_AXIL_WRITE_RESP;
                        end
                    end

                    ST_AXIL_WRITE_RESP: begin
                        if (m_axil_bvalid && m_axil_bready) begin
                            m_axil_bready <= 1'b0;
                            aw_done <= 1'b0;
                            w_done  <= 1'b0;

                            if (m_axil_bresp != 2'b00) begin
                                last_status_o <= {30'd0, m_axil_bresp};
                                state <= ST_ERROR;
                            end else begin
                                state <= return_state;
                            end
                        end
                    end

                    // --------------------------------------------------------
                    // AXI-Lite read helper
                    // --------------------------------------------------------
                    ST_AXIL_READ: begin
                        if (!m_axil_arvalid) begin
                            m_axil_araddr  <= rd_addr[AXIL_ADDR_W-1:0];
                            m_axil_arvalid <= 1'b1;
                        end

                        if (m_axil_arvalid && m_axil_arready) begin
                            m_axil_arvalid <= 1'b0;
                            m_axil_rready  <= 1'b1;
                            state <= ST_AXIL_READ_DATA;
                        end
                    end

                    ST_AXIL_READ_DATA: begin
                        if (m_axil_rvalid && m_axil_rready) begin
                            m_axil_rready <= 1'b0;
                            rd_data <= m_axil_rdata;

                            if (m_axil_rresp != 2'b00) begin
                                last_status_o <= {30'd0, m_axil_rresp};
                                state <= ST_ERROR;
                            end else begin
                                state <= return_state;
                            end
                        end
                    end

                    default: begin
                        state <= ST_ERROR;
                    end
                endcase
            end
        end
    end

endmodule
