// AXI-Lite 슬레이브의 5개 채널 handshake를 처리하는 인터페이스 모듈이다.
// 쓰기 주소와 쓰기 데이터를 각각 받아 한 쌍이 모이면 내부 쓰기 요청을 만든다.
// 읽기 주소를 받으면 wrapper가 제공한 레지스터 값을 AXI 읽기 데이터로 반환한다.
// 복잡한 AXI 신호를 wr_en, rd_en 등의 단순한 레지스터 접근 신호로 변환한다.
// 따라서 상위 wrapper는 AXI 프로토콜보다 레지스터의 실제 동작에 집중할 수 있다.

module axil_slave_reg_if #(
    parameter AXIL_ADDR_W = 32,
    parameter AXIL_DATA_W = 32

    
)(
    input  wire                     clk,
    input  wire                     rst,

    // AXI-Lite Slave Write Address Channel
    input  wire [AXIL_ADDR_W-1:0]   s_axil_awaddr,
    input  wire                     s_axil_awvalid,
    output wire                     s_axil_awready,

    // AXI-Lite Slave Write Data Channel
    input  wire [AXIL_DATA_W-1:0]   s_axil_wdata,
    input  wire [AXIL_DATA_W/8-1:0] s_axil_wstrb,
    input  wire                     s_axil_wvalid,
    output wire                     s_axil_wready,

    // AXI-Lite Slave Write Response Channel
    output wire [1:0]               s_axil_bresp,
    output wire                     s_axil_bvalid,
    input  wire                     s_axil_bready,

    // AXI-Lite Slave Read Address Channel
    input  wire [AXIL_ADDR_W-1:0]   s_axil_araddr,
    input  wire                     s_axil_arvalid,
    output wire                     s_axil_arready,

    // AXI-Lite Slave Read Data Channel
    output wire [AXIL_DATA_W-1:0]   s_axil_rdata,
    output wire [1:0]               s_axil_rresp,
    output wire                     s_axil_rvalid,
    input  wire                     s_axil_rready,

    // Simple Register Interface to Wrapper
    output reg                      wr_en,
    output reg  [AXIL_ADDR_W-1:0]   wr_addr,
    output reg  [AXIL_DATA_W-1:0]   wr_data,
    output reg  [AXIL_DATA_W/8-1:0] wr_strb,

    output reg                      rd_en,
    output wire [AXIL_ADDR_W-1:0]   rd_addr,
    input  wire [AXIL_DATA_W-1:0]   rd_data
);

    reg [AXIL_ADDR_W-1:0]   awaddr_hold;
    reg                     aw_hold;

    reg [AXIL_DATA_W-1:0]   wdata_hold;
    reg [AXIL_DATA_W/8-1:0] wstrb_hold;
    reg                     w_hold;

    reg                     bvalid_reg;

    reg [AXIL_DATA_W-1:0]   rdata_reg;
    reg                     rvalid_reg;

    wire aw_fire;
    wire w_fire;
    wire ar_fire;

    wire have_aw_next;
    wire have_w_next;
    wire write_fire;

    assign s_axil_awready = (!aw_hold) && (!bvalid_reg);
    assign s_axil_wready  = (!w_hold)  && (!bvalid_reg);

    assign aw_fire = s_axil_awvalid && s_axil_awready;
    assign w_fire  = s_axil_wvalid  && s_axil_wready;

    assign have_aw_next = aw_hold || aw_fire;
    assign have_w_next  = w_hold  || w_fire;

    assign write_fire = have_aw_next && have_w_next && (!bvalid_reg);

    assign s_axil_bvalid = bvalid_reg;
    assign s_axil_bresp  = 2'b00;   // OKAY

    assign s_axil_arready = !rvalid_reg;
    assign ar_fire = s_axil_arvalid && s_axil_arready;

    assign rd_addr = s_axil_araddr;

    assign s_axil_rdata  = rdata_reg;
    assign s_axil_rvalid = rvalid_reg;
    assign s_axil_rresp  = 2'b00;   // OKAY

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            awaddr_hold <= {AXIL_ADDR_W{1'b0}};
            aw_hold     <= 1'b0;

            wdata_hold  <= {AXIL_DATA_W{1'b0}};
            wstrb_hold  <= {(AXIL_DATA_W/8){1'b0}};
            w_hold      <= 1'b0;

            bvalid_reg  <= 1'b0;

            wr_en       <= 1'b0;
            wr_addr     <= {AXIL_ADDR_W{1'b0}};
            wr_data     <= {AXIL_DATA_W{1'b0}};
            wr_strb     <= {(AXIL_DATA_W/8){1'b0}};

            rdata_reg   <= {AXIL_DATA_W{1'b0}};
            rvalid_reg  <= 1'b0;
            rd_en       <= 1'b0;
        end else begin
            wr_en <= 1'b0;
            rd_en <= 1'b0;

            // Write address latch
            if (aw_fire) begin
                awaddr_hold <= s_axil_awaddr;
                aw_hold     <= 1'b1;
            end

            // Write data latch
            if (w_fire) begin
                wdata_hold <= s_axil_wdata;
                wstrb_hold <= s_axil_wstrb;
                w_hold     <= 1'b1;
            end

            // When both AW and W are received, issue internal write
            if (write_fire) begin
                wr_en   <= 1'b1;
                wr_addr <= aw_hold ? awaddr_hold : s_axil_awaddr;
                wr_data <= w_hold  ? wdata_hold  : s_axil_wdata;
                wr_strb <= w_hold  ? wstrb_hold  : s_axil_wstrb;

                aw_hold <= 1'b0;
                w_hold  <= 1'b0;

                bvalid_reg <= 1'b1;
            end

            // Write response accepted
            if (bvalid_reg && s_axil_bready) begin
                bvalid_reg <= 1'b0;
            end

            // Read address accepted
            if (ar_fire) begin
                rd_en      <= 1'b1;
                rdata_reg  <= rd_data;
                rvalid_reg <= 1'b1;
            end else if (rvalid_reg && s_axil_rready) begin
                rvalid_reg <= 1'b0;
            end
        end
    end

endmodule
