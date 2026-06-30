`timescale 1ns / 1ps

// ============================================================
// LED Matrix Writer AXI 통합 모듈
// ------------------------------------------------------------
// AXI-Lite로 시작, 초기화, 데이터 주소, 표시 크기 설정값을 받는다.
// AXI4 읽기 채널로 BRAM의 DFT 크기값 8개를 읽어 내부에 저장한다.
// 읽은 크기값은 led_data_mapper를 거쳐 32x8 LED bitmap으로 변환된다.
// 변환된 bitmap을 max7219_controller에 넘겨 DIN, CS, CLK로 출력한다.
// 즉, 제어 레지스터부터 BRAM 읽기와 LED 출력까지 전체 흐름을 연결한다.
//
// 레지스터 주소표:
//   0x00 LED_CTRL
//        bit[0] 시작/화면 갱신
//        bit[1] 화면 지우기
//
//   0x04 LED_STATUS
//        bit[0] 완료
//        bit[1] 동작 중
//        bit[2] 준비 완료
//
//   0x08 LED_RESULT_ADDR
//        DFT 결과가 저장된 BRAM의 byte 주소
//
//   0x0C LED_BIN_COUNT
//        읽어올 결과 word 개수
//        기본값은 8이며 내부에서 BIN_COUNT(8)를 넘지 않도록 제한
//
//   0x10 LED_SCALE
//        led_data_mapper에서 사용할 scale_shift 값
//        기존 LED_THRESHOLD 주소를 현재 설계에서는 scale_shift용으로 재사용
//
//   0x14 LED_VALUE
//        디버깅용 읽기 값
//        row_bitmap_flat[31:0], 즉 첫 번째 행의 bitmap을 반환
// ============================================================

module led_matrix_writer_wrapper_axi_slave #(
    parameter AXIL_ADDR_W = 32,
    parameter AXIL_DATA_W = 32,
    parameter AXI_ADDR_W  = 32,
    parameter AXI_DATA_W  = 32,
    parameter LEN_W       = 16,
    parameter BIN_COUNT   = 8
)(
    input  wire                     clk,
    input  wire                     rst,      // active-high reset

    // ========================================================
    // AXI-Lite Slave Write Address Channel
    // ========================================================
    input  wire [AXIL_ADDR_W-1:0]   s_axil_awaddr,
    input  wire                     s_axil_awvalid,
    output wire                     s_axil_awready,

    // ========================================================
    // AXI-Lite Slave Write Data Channel
    // ========================================================
    input  wire [AXIL_DATA_W-1:0]   s_axil_wdata,
    input  wire [AXIL_DATA_W/8-1:0] s_axil_wstrb,
    input  wire                     s_axil_wvalid,
    output wire                     s_axil_wready,

    // ========================================================
    // AXI-Lite Slave Write Response Channel
    // ========================================================
    output wire [1:0]               s_axil_bresp,
    output wire                     s_axil_bvalid,
    input  wire                     s_axil_bready,

    // ========================================================
    // AXI-Lite Slave Read Address Channel
    // ========================================================
    input  wire [AXIL_ADDR_W-1:0]   s_axil_araddr,
    input  wire                     s_axil_arvalid,
    output wire                     s_axil_arready,

    // ========================================================
    // AXI-Lite Slave Read Data Channel
    // ========================================================
    output wire [AXIL_DATA_W-1:0]   s_axil_rdata,
    output wire [1:0]               s_axil_rresp,
    output wire                     s_axil_rvalid,
    input  wire                     s_axil_rready,

    // ========================================================
    // AXI4-Full Master Read Address Channel
    // ========================================================
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

    // ========================================================
    // AXI4-Full Master Read Data Channel
    // ========================================================
    input  wire [AXI_DATA_W-1:0]    m_axi_rdata,
    input  wire [1:0]               m_axi_rresp,
    input  wire                     m_axi_rvalid,
    output reg                      m_axi_rready,
    input  wire                     m_axi_rlast,

    // ========================================================
    // AXI4-Full Master Write Address Channel
    // Not used. Tied off.
    // ========================================================
    output wire [AXI_ADDR_W-1:0]    m_axi_awaddr,
    output wire                     m_axi_awvalid,
    input  wire                     m_axi_awready,
    output wire [7:0]               m_axi_awlen,
    output wire [2:0]               m_axi_awsize,
    output wire [1:0]               m_axi_awburst,
    output wire                     m_axi_awlock,
    output wire [3:0]               m_axi_awcache,
    output wire [2:0]               m_axi_awprot,
    output wire [3:0]               m_axi_awqos,

    // ========================================================
    // AXI4-Full Master Write Data Channel
    // Not used. Tied off.
    // ========================================================
    output wire [AXI_DATA_W-1:0]    m_axi_wdata,
    output wire [AXI_DATA_W/8-1:0]  m_axi_wstrb,
    output wire                     m_axi_wvalid,
    input  wire                     m_axi_wready,
    output wire                     m_axi_wlast,

    // ========================================================
    // AXI4-Full Master Write Response Channel
    // Not used. Always ready.
    // ========================================================
    input  wire [1:0]               m_axi_bresp,
    input  wire                     m_axi_bvalid,
    output wire                     m_axi_bready,

    // ========================================================
    // MAX7219 LED Matrix Output
    // ========================================================
    output wire                     max7219_din,
    output wire                     max7219_cs,
    output wire                     max7219_clk
);

    // ========================================================
    // Register offsets
    // Team register map 유지
    // ========================================================
    localparam [7:0] LED_CTRL_OFFSET        = 8'h00;
    localparam [7:0] LED_STATUS_OFFSET      = 8'h04;
    localparam [7:0] LED_RESULT_ADDR_OFFSET = 8'h08;
    localparam [7:0] LED_BIN_COUNT_OFFSET   = 8'h0C;
    localparam [7:0] LED_SCALE_OFFSET       = 8'h10;
    localparam [7:0] LED_VALUE_OFFSET       = 8'h14;

    // ========================================================
    // AXI-Lite internal register interface
    //
    // axil_slave_reg_if가 AXI-Lite handshake를 처리하고,
    // wrapper 내부에서는 wr_en / wr_addr / wr_data처럼 단순 신호로 사용한다.
    // ========================================================
    wire                     wr_en;
    wire [AXIL_ADDR_W-1:0]   wr_addr;
    wire [AXIL_DATA_W-1:0]   wr_data;
    wire [AXIL_DATA_W/8-1:0] wr_strb;

    wire                     rd_en;
    wire [AXIL_ADDR_W-1:0]   rd_addr;
    reg  [AXIL_DATA_W-1:0]   rd_data;

    // ========================================================
    // CSR registers
    // ========================================================
    reg [31:0] led_result_addr_reg;  // BRAM에서 DFT 결과가 시작되는 byte address
    reg [31:0] led_bin_count_reg;    // 읽을 bin 개수. 기본 8
    reg [31:0] led_scale_reg;        // led_data_mapper의 scale_shift로 사용

    reg        led_start_pulse;      // CTRL[0] write 시 1클럭 pulse
    reg        led_clear_pulse;      // CTRL[1] write 시 1클럭 pulse
    reg        led_done_latched;     // done 상태 latch

    // ========================================================
    // AXI read FSM status
    // ========================================================
    reg        led_busy;
    reg        led_done;
    wire       led_ready;

    assign led_ready = !led_busy;

    wire [31:0] led_status_value;
    assign led_status_value = {
        29'd0,
        led_ready,          // bit[2]
        led_busy,           // bit[1]
        led_done_latched    // bit[0]
    };

    // ========================================================
    // DFT magnitude buffer
    //
    // BRAM에서 읽은 32bit magnitude 8개를 여기에 모은다.
    //
    // read_data_flat[31:0]      = bin0
    // read_data_flat[63:32]     = bin1
    // ...
    // read_data_flat[255:224]   = bin7
    // ========================================================
    reg  [AXI_DATA_W*BIN_COUNT-1:0] read_data_flat;
    wire [255:0]                    row_bitmap_flat;

    // MAX7219 controller status
    wire display_init_done;
    wire display_busy;

    // AXI read가 끝나고 MAX7219 controller를 시작시키는 pulse
    reg  display_start_pulse;

    // clear 시 화면을 0으로 만든 뒤 controller도 한번 시작시키기 위해 사용
    wire controller_start;
    assign controller_start = display_start_pulse | led_clear_pulse;

    // 0x14 read debug value
    // 여기서는 row0 bitmap만 readback한다.
    wire [31:0] led_value_debug;
    assign led_value_debug = row_bitmap_flat[31:0];

    // ========================================================
    // AXI4-Full single-beat read setting
    // ========================================================
    assign m_axi_arlen   = 8'd0;      // single beat
    assign m_axi_arsize  = 3'd2;      // 4 byte = 32-bit word
    assign m_axi_arburst = 2'b01;     // INCR
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arqos   = 4'b0000;

    // ========================================================
    // AXI4-Full write channel tie-off
    // LED Matrix Writer only reads memory.
    // ========================================================
    assign m_axi_awaddr  = {AXI_ADDR_W{1'b0}};
    assign m_axi_awvalid = 1'b0;
    assign m_axi_awlen   = 8'd0;
    assign m_axi_awsize  = 3'd2;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awqos   = 4'b0000;

    assign m_axi_wdata   = {AXI_DATA_W{1'b0}};
    assign m_axi_wstrb   = {(AXI_DATA_W/8){1'b0}};
    assign m_axi_wvalid  = 1'b0;
    assign m_axi_wlast   = 1'b0;

    assign m_axi_bready  = 1'b1;

    // ========================================================
    // AXI-Lite adapter
    //
    // 이 모듈은 팀장 코드 그대로 사용한다.
    // AXI-Lite 신호를 내부 wr_en/rd_en 방식으로 바꿔준다.
    // ========================================================
    axil_slave_reg_if #(
        .AXIL_ADDR_W(AXIL_ADDR_W),
        .AXIL_DATA_W(AXIL_DATA_W)
    ) u_axil_slave_reg_if (
        .clk            (clk),
        .rst            (rst),

        .s_axil_awaddr  (s_axil_awaddr),
        .s_axil_awvalid (s_axil_awvalid),
        .s_axil_awready (s_axil_awready),

        .s_axil_wdata   (s_axil_wdata),
        .s_axil_wstrb   (s_axil_wstrb),
        .s_axil_wvalid  (s_axil_wvalid),
        .s_axil_wready  (s_axil_wready),

        .s_axil_bresp   (s_axil_bresp),
        .s_axil_bvalid  (s_axil_bvalid),
        .s_axil_bready  (s_axil_bready),

        .s_axil_araddr  (s_axil_araddr),
        .s_axil_arvalid (s_axil_arvalid),
        .s_axil_arready (s_axil_arready),

        .s_axil_rdata   (s_axil_rdata),
        .s_axil_rresp   (s_axil_rresp),
        .s_axil_rvalid  (s_axil_rvalid),
        .s_axil_rready  (s_axil_rready),

        .wr_en          (wr_en),
        .wr_addr        (wr_addr),
        .wr_data        (wr_data),
        .wr_strb        (wr_strb),

        .rd_en          (rd_en),
        .rd_addr        (rd_addr),
        .rd_data        (rd_data)
    );

    // ========================================================
    // CSR write logic
    //
    // main_CU가 AXI-Lite로 register에 값을 쓰면
    // 여기서 내부 설정값이 바뀐다.
    // ========================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            led_result_addr_reg <= 32'h0000_1000;
            led_bin_count_reg   <= 32'd8;
            led_scale_reg       <= 32'd8;

            led_start_pulse     <= 1'b0;
            led_clear_pulse     <= 1'b0;
            led_done_latched    <= 1'b0;
        end else begin
            // start/clear는 1클럭 pulse
            led_start_pulse <= 1'b0;
            led_clear_pulse <= 1'b0;

            if (wr_en) begin
                case (wr_addr[7:0])
                    LED_CTRL_OFFSET: begin
                        // bit[0] start/update
                        if (wr_data[0]) begin
                            led_start_pulse  <= 1'b1;
                            led_done_latched <= 1'b0;
                        end

                        // bit[1] clear
                        if (wr_data[1]) begin
                            led_clear_pulse  <= 1'b1;
                            led_done_latched <= 1'b0;
                        end
                    end

                    LED_RESULT_ADDR_OFFSET: begin
                        if (wr_strb[0])
                            led_result_addr_reg[7:0] <= wr_data[7:0];
                        if (wr_strb[1])
                            led_result_addr_reg[15:8] <= wr_data[15:8];
                        if (wr_strb[2])
                            led_result_addr_reg[23:16] <= wr_data[23:16];
                        if (wr_strb[3])
                            led_result_addr_reg[31:24] <= wr_data[31:24];
                    end

                    LED_BIN_COUNT_OFFSET: begin
                        if (wr_strb[0])
                            led_bin_count_reg[7:0] <= wr_data[7:0];
                        if (wr_strb[1])
                            led_bin_count_reg[15:8] <= wr_data[15:8];
                        if (wr_strb[2])
                            led_bin_count_reg[23:16] <= wr_data[23:16];
                        if (wr_strb[3])
                            led_bin_count_reg[31:24] <= wr_data[31:24];
                    end

                    LED_SCALE_OFFSET: begin
                        // 기존 팀 코드의 LED_THRESHOLD 주소를
                        // 우리 프로젝트에서는 scale_shift register로 사용한다.
                        if (wr_strb[0])
                            led_scale_reg[7:0] <= wr_data[7:0];
                        if (wr_strb[1])
                            led_scale_reg[15:8] <= wr_data[15:8];
                        if (wr_strb[2])
                            led_scale_reg[23:16] <= wr_data[23:16];
                        if (wr_strb[3])
                            led_scale_reg[31:24] <= wr_data[31:24];
                    end

                    default: begin
                        // 정의되지 않은 주소 write는 무시
                    end
                endcase
            end

            // AXI read FSM이 끝났다는 pulse가 나오면 done latch set
            if (led_done) begin
                led_done_latched <= 1'b1;
            end
        end
    end

    // ========================================================
    // CSR read logic
    //
    // main_CU가 AXI-Lite로 register를 읽으면
    // 여기서 rd_data 값을 만들어준다.
    // ========================================================
    always @(*) begin
        case (rd_addr[7:0])
            LED_CTRL_OFFSET: begin
                rd_data = 32'd0;
            end

            LED_STATUS_OFFSET: begin
                rd_data = led_status_value;
            end

            LED_RESULT_ADDR_OFFSET: begin
                rd_data = led_result_addr_reg;
            end

            LED_BIN_COUNT_OFFSET: begin
                rd_data = led_bin_count_reg;
            end

            LED_SCALE_OFFSET: begin
                rd_data = led_scale_reg;
            end

            LED_VALUE_OFFSET: begin
                rd_data = led_value_debug;
            end

            default: begin
                rd_data = 32'hDEAD_BEEF;
            end
        endcase
    end

    // ========================================================
    // AXI4-Full Read Master FSM
    //
    // start가 들어오면 BRAM에서 DFT 결과값을 1개씩 읽는다.
    // 각 read는 single-beat read이다.
    //
    // 읽은 값은 index에 따라 read_data_flat의 각 32비트 위치에 저장한다.
    // 8개가 다 모이면 led_data_mapper가 32x8 bitmap을 만든다.
    // ========================================================
    localparam S_IDLE      = 3'd0;
    localparam S_READ_ADDR = 3'd1;
    localparam S_READ_DATA = 3'd2;
    localparam S_NEXT      = 3'd3;
    localparam S_DONE      = 3'd4;

    reg [2:0]       state;
    reg [LEN_W-1:0] index;

    localparam [LEN_W-1:0] BIN_COUNT_MAX    = BIN_COUNT;
    localparam [31:0]      BIN_COUNT_MAX_32 = BIN_COUNT;

    wire [LEN_W-1:0] effective_count;
    wire             read_addr_fire;
    wire             read_data_fire;
    wire             last_index;

    assign effective_count =
        (led_bin_count_reg > BIN_COUNT_MAX_32) ?
        BIN_COUNT_MAX :
        led_bin_count_reg[LEN_W-1:0];

    assign read_addr_fire = m_axi_arvalid && m_axi_arready;
    assign read_data_fire = m_axi_rvalid  && m_axi_rready;

    assign last_index = (effective_count != {LEN_W{1'b0}}) &&
                        (index == (effective_count - 1'b1));

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state               <= S_IDLE;
            index               <= {LEN_W{1'b0}};

            led_busy            <= 1'b0;
            led_done            <= 1'b0;

            m_axi_araddr        <= {AXI_ADDR_W{1'b0}};
            m_axi_arvalid       <= 1'b0;
            m_axi_rready        <= 1'b0;

            read_data_flat      <= {(AXI_DATA_W*BIN_COUNT){1'b0}};
            display_start_pulse <= 1'b0;
        end else begin
            led_done            <= 1'b0;
            display_start_pulse <= 1'b0;

            if (led_clear_pulse) begin
                state               <= S_IDLE;
                index               <= {LEN_W{1'b0}};
                led_busy            <= 1'b0;
                led_done            <= 1'b0;
                m_axi_araddr        <= {AXI_ADDR_W{1'b0}};
                m_axi_arvalid       <= 1'b0;
                m_axi_rready        <= 1'b0;
                read_data_flat      <= {(AXI_DATA_W*BIN_COUNT){1'b0}};
                display_start_pulse <= 1'b1;
            end else begin
                case (state)
                    S_IDLE: begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b0;

                        if (led_start_pulse && !led_busy) begin
                            led_busy       <= 1'b1;
                            index          <= {LEN_W{1'b0}};
                            read_data_flat <= {(AXI_DATA_W*BIN_COUNT){1'b0}};

                            if (effective_count == {LEN_W{1'b0}}) begin
                                state <= S_DONE;
                            end else begin
                                state <= S_READ_ADDR;
                            end
                        end
                    end

                    S_READ_ADDR: begin
                        // byte address = base + index * 4
                        // 32bit word 하나가 4byte이므로 index << 2
                        m_axi_araddr  <= led_result_addr_reg[AXI_ADDR_W-1:0] + (index << 2);
                        m_axi_arvalid <= 1'b1;

                        if (read_addr_fire) begin
                            m_axi_arvalid <= 1'b0;
                            state <= S_READ_DATA;
                        end
                    end

                    S_READ_DATA: begin
                        m_axi_rready <= 1'b1;

                        if (read_data_fire) begin
                            m_axi_rready <= 1'b0;

                            // 정상 응답이면 현재 index 위치에 DFT 결과 저장
                            if (m_axi_rresp == 2'b00) begin
                                case (index)
                                    0: read_data_flat[31:0]    <= m_axi_rdata;
                                    1: read_data_flat[63:32]   <= m_axi_rdata;
                                    2: read_data_flat[95:64]   <= m_axi_rdata;
                                    3: read_data_flat[127:96]  <= m_axi_rdata;
                                    4: read_data_flat[159:128] <= m_axi_rdata;
                                    5: read_data_flat[191:160] <= m_axi_rdata;
                                    6: read_data_flat[223:192] <= m_axi_rdata;
                                    7: read_data_flat[255:224] <= m_axi_rdata;
                                    default: begin
                                        // 0~7 이외의 index는 저장하지 않음
                                    end
                                endcase
                            end

                            state <= S_NEXT;
                        end
                    end

                    S_NEXT: begin
                        if (last_index) begin
                            state <= S_DONE;
                        end else begin
                            index <= index + 1'b1;
                            state <= S_READ_ADDR;
                        end
                    end

                    S_DONE: begin
                        led_busy            <= 1'b0;
                        led_done            <= 1'b1;
                        display_start_pulse <= 1'b1;
                        state               <= S_IDLE;
                    end

                    default: begin
                        state <= S_IDLE;
                    end
                endcase
            end
        end
    end

    // ========================================================
    // DFT magnitude -> LED Matrix bitmap mapper
    //
    // read_data_flat에 모인 magnitude 8개를
    // 32열 x 8행 bitmap으로 변환한다.
    // ========================================================
    led_data_mapper #(
        .BIN_COUNT   (BIN_COUNT),
        .DATA_W      (AXI_DATA_W),
        .COL_PER_BIN (4),
        .ROW_COUNT   (8),
        .COL_COUNT   (32)
    ) u_led_data_mapper (
        .magnitude_flat  (read_data_flat),
        .scale_shift     (led_scale_reg[4:0]),
        .row_bitmap_flat (row_bitmap_flat)
    );

    // ========================================================
    // MAX7219 Controller
    //
    // row_bitmap_flat을 받아서 MAX7219 x4 LED Matrix로 출력한다.
    // controller 내부에서 frame_sender를 사용한다고 가정한다.
    // ========================================================
    max7219_controller u_max7219_controller (
        .clk              (clk),
        .reset_p          (rst),

        .start            (controller_start),
        .enable           (1'b1),

        .row_bitmap_flat  (row_bitmap_flat),

        .max7219_din      (max7219_din),
        .max7219_cs       (max7219_cs),
        .max7219_clk      (max7219_clk),

        .init_done        (display_init_done),
        .busy             (display_busy)
    );

endmodule
