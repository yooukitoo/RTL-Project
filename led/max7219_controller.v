`timescale 1ns / 1ps

/*
    ============================================================
    max7219_controller.v

    이 모듈은 MAX7219 LED Matrix 출력 흐름을 관리하는 controller이다.

    핵심 역할:
    1. MAX7219 초기화 명령 전송
    2. LED 화면 clear
    3. row_bitmap_flat에서 row 1~8 데이터를 꺼내기
    4. 각 row 데이터를 64비트 frame으로 만들어 frame_sender에 전달
    5. enable이 1이면 row 1~8 갱신을 반복 수행

    주의:
    - max7219_frame_sender는 64비트 frame을 한 번 보내는 역할만 한다.
    - controller는 그 frame_sender를 여러 번 사용해서
      초기화, clear, row 갱신 순서를 제어한다.
    ============================================================
*/

module max7219_controller (
    input  wire        clk,
    input  wire        reset_p,

    input  wire        start,      // controller 전체 동작 시작
    input  wire        enable,     // refresh 동작 허용

    /*
        32x8 LED 화면 전체 bitmap

        row_bitmap_flat[31:0]     = row 1
        row_bitmap_flat[63:32]    = row 2
        ...
        row_bitmap_flat[255:224]  = row 8

        각 row는 32비트이고,
        4개의 8x8 LED Matrix가 가로로 연결된 32칸을 의미한다.
    */
    input  wire [255:0] row_bitmap_flat,

    output wire        max7219_din,
    output wire        max7219_cs,
    output wire        max7219_clk,

    output reg         init_done,  // 초기화 + clear 완료 표시
    output reg         busy        // controller 동작 중 표시
);

    // =====================================================
    // frame_sender 제어 신호
    //
    // controller는 sender_frame에 64비트 데이터를 넣고,
    // sender_start를 1클럭 올려 frame_sender에게 전송을 요청한다.
    // sender_done이 오면 다음 명령 또는 다음 row로 넘어간다.
    // =====================================================

    reg        sender_start;
    reg [63:0] sender_frame;
    wire       sender_busy;
    wire       sender_done;

    max7219_frame_sender #(
        .CLK_DIV(50)
    ) u_frame_sender (
        .clk(clk),
        .reset_p(reset_p),

        .start(sender_start),
        .frame_data(sender_frame),

        .busy(sender_busy),
        .done(sender_done),

        .max7219_din(max7219_din),
        .max7219_cs(max7219_cs),
        .max7219_clk(max7219_clk)
    );

    // =====================================================
    // FSM 상태 정의
    // =====================================================

    localparam ST_IDLE         = 4'd0;  // start 대기
    localparam ST_INIT_SEND    = 4'd1;  // 초기화 명령 전송 요청
    localparam ST_INIT_WAIT    = 4'd2;  // 초기화 명령 전송 완료 대기
    localparam ST_CLEAR_SEND   = 4'd3;  // row clear 명령 전송 요청
    localparam ST_CLEAR_WAIT   = 4'd4;  // row clear 전송 완료 대기
    localparam ST_REFRESH_SEND = 4'd5;  // 현재 row bitmap 전송 요청
    localparam ST_REFRESH_WAIT = 4'd6;  // 현재 row 전송 완료 대기

    reg [3:0] state;
    reg [3:0] init_idx;  // 초기화 명령 순서 index
    reg [3:0] row_idx;   // 현재 처리 중인 row 번호, 1~8
    reg [31:0] row_bits; // 현재 row_idx에 해당하는 32비트 LED 데이터

    // =====================================================
    // row_bitmap_flat에서 현재 row의 32비트 데이터 선택
    //
    // row 1이면 [31:0],
    // row 2이면 [63:32],
    // ...
    // row 8이면 [255:224]를 꺼낸다.
    // =====================================================
    always @(*) begin
        row_bits = 32'd0;

        case (row_idx)
            4'd1: row_bits = row_bitmap_flat[31:0];
            4'd2: row_bits = row_bitmap_flat[63:32];
            4'd3: row_bits = row_bitmap_flat[95:64];
            4'd4: row_bits = row_bitmap_flat[127:96];
            4'd5: row_bits = row_bitmap_flat[159:128];
            4'd6: row_bits = row_bitmap_flat[191:160];
            4'd7: row_bits = row_bitmap_flat[223:192];
            4'd8: row_bits = row_bitmap_flat[255:224];
            default: row_bits = 32'd0;
        endcase
    end

    // =====================================================
    // Main FSM
    //
    // 흐름:
    // IDLE
    // → 초기화 명령 5개 전송
    // → row 1~8 clear
    // → row 1~8 bitmap refresh 반복
    // =====================================================

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            state        <= ST_IDLE;
            init_idx     <= 4'd0;
            row_idx      <= 4'd1;

            sender_start <= 1'b0;
            sender_frame <= 64'd0;

            init_done    <= 1'b0;
            busy         <= 1'b0;
        end else begin
            // sender_start는 기본적으로 0.
            // 필요한 상태에서만 1클럭 pulse로 올린다.
            sender_start <= 1'b0;

            case (state)

                // -------------------------------------------------
                // start 대기 상태
                // -------------------------------------------------
                ST_IDLE: begin
                    busy      <= 1'b0;
                    init_done <= 1'b0;
                    init_idx  <= 4'd0;
                    row_idx   <= 4'd1;

                    if (start) begin
                        busy  <= 1'b1;
                        state <= ST_INIT_SEND;
                    end
                end

                // -------------------------------------------------
                // MAX7219 초기화 명령 선택 후 frame_sender에 전송 요청
                // -------------------------------------------------
                ST_INIT_SEND: begin
                    case (init_idx)
                        4'd0: sender_frame <= {8'h09, 8'h00, 8'h09, 8'h00,
                                              8'h09, 8'h00, 8'h09, 8'h00}; // Decode mode off
                        4'd1: sender_frame <= {8'h0A, 8'h03, 8'h0A, 8'h03,
                                              8'h0A, 8'h03, 8'h0A, 8'h03}; // Intensity
                        4'd2: sender_frame <= {8'h0B, 8'h07, 8'h0B, 8'h07,
                                              8'h0B, 8'h07, 8'h0B, 8'h07}; // Scan limit: 8 rows
                        4'd3: sender_frame <= {8'h0C, 8'h01, 8'h0C, 8'h01,
                                              8'h0C, 8'h01, 8'h0C, 8'h01}; // Shutdown off
                        4'd4: sender_frame <= {8'h0F, 8'h00, 8'h0F, 8'h00,
                                              8'h0F, 8'h00, 8'h0F, 8'h00}; // Display test off
                        default: sender_frame <= 64'd0;
                    endcase

                    sender_start <= 1'b1;
                    state        <= ST_INIT_WAIT;
                end

                // -------------------------------------------------
                // 초기화 명령 하나가 끝날 때까지 대기
                // 끝나면 다음 초기화 명령으로 이동
                // -------------------------------------------------
                ST_INIT_WAIT: begin
                    if (sender_done) begin
                        if (init_idx == 4'd4) begin
                            row_idx <= 4'd1;
                            state   <= ST_CLEAR_SEND;
                        end else begin
                            init_idx <= init_idx + 1'b1;
                            state    <= ST_INIT_SEND;
                        end
                    end
                end

                // -------------------------------------------------
                // row 1~8을 모두 0으로 clear
                // -------------------------------------------------
                ST_CLEAR_SEND: begin
                    sender_frame <= {
                        {4'd0, row_idx}, 8'h00,
                        {4'd0, row_idx}, 8'h00,
                        {4'd0, row_idx}, 8'h00,
                        {4'd0, row_idx}, 8'h00
                    };
                    sender_start <= 1'b1;
                    state        <= ST_CLEAR_WAIT;
                end

                ST_CLEAR_WAIT: begin
                    if (sender_done) begin
                        if (row_idx == 4'd8) begin
                            row_idx   <= 4'd1;
                            init_done <= 1'b1;
                            state     <= ST_REFRESH_SEND;
                        end else begin
                            row_idx <= row_idx + 1'b1;
                            state   <= ST_CLEAR_SEND;
                        end
                    end
                end

                // -------------------------------------------------
                // 현재 row의 bitmap 데이터를 64비트 frame으로 만들어 전송
                // enable이 0이면 refresh를 멈추고 대기
                // -------------------------------------------------
                ST_REFRESH_SEND: begin
                    if (enable) begin
                        busy         <= 1'b1;
                        sender_frame <= {
                            {4'd0, row_idx}, row_bits[31:24],
                            {4'd0, row_idx}, row_bits[23:16],
                            {4'd0, row_idx}, row_bits[15:8],
                            {4'd0, row_idx}, row_bits[7:0]
                        };
                        sender_start <= 1'b1;
                        state        <= ST_REFRESH_WAIT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------
                // 현재 row 전송 완료 대기
                // row 8까지 끝나면 다시 row 1로 돌아가 반복
                // -------------------------------------------------
                ST_REFRESH_WAIT: begin
                    if (sender_done) begin
                        if (row_idx == 4'd8)
                            row_idx <= 4'd1;
                        else
                            row_idx <= row_idx + 1'b1;

                        state <= ST_REFRESH_SEND;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
