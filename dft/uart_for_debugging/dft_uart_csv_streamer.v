`timescale 1ns / 1ps

// ============================================================================
// dft_uart_csv_streamer.v
//
// 실제 보드 동작 중 DFT packed 결과를 UART로 PC에 전송한다.
//
// 입력:
//   result_valid  : DFT 결과가 새로 나온 순간 1클럭 pulse
//   result_packed : [3:0]=bin0, [7:4]=bin1, ... [31:28]=bin7
//
// UART 출력 line:
//   0x00000000,0x53563747\r\n
//
// 의미:
//   frame_hex,packed_hex
//
// PC Python 쪽에서 packed_hex를 bin0~bin7로 풀어서 CSV 저장/그래프 출력한다.
//
// 115200bps 기준 line 23byte 전송 시간은 약 2ms 수준이라,
// 현재 autorun_slow의 frame interval 200ms보다 충분히 빠르다.
// ============================================================================

module dft_uart_csv_streamer #(
    parameter CLK_FREQ_HZ = 100_000_000,
    parameter BAUD        = 115200
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        result_valid,
    input  wire [31:0] result_packed,

    output wire        uart_tx,
    output reg         busy
);

    localparam LINE_LEN = 23;

    localparam ST_IDLE      = 3'd0;
    localparam ST_LOAD_BYTE = 3'd1;
    localparam ST_START_TX  = 3'd2;
    localparam ST_WAIT_TX   = 3'd3;

    reg [2:0]  state;
    reg [4:0]  byte_idx;

    reg [31:0] frame_counter;
    reg [31:0] frame_latched;
    reg [31:0] packed_latched;

    reg        tx_start;
    reg [7:0]  tx_data;
    wire       tx_busy;
    wire       tx_done;

    uart_tx_byte #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD       (BAUD)
    ) u_uart_tx_byte (
        .clk   (clk),
        .rst   (rst),
        .start (tx_start),
        .data  (tx_data),
        .tx    (uart_tx),
        .busy  (tx_busy),
        .done  (tx_done)
    );

    function [7:0] hex_char;
        input [3:0] nibble;
        begin
            if (nibble < 4'd10)
                hex_char = 8'h30 + {4'd0, nibble};       // '0'~'9'
            else
                hex_char = 8'h41 + {4'd0, nibble - 4'd10}; // 'A'~'F'
        end
    endfunction

    function [3:0] get_frame_nibble;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: get_frame_nibble = frame_latched[31:28];
                3'd1: get_frame_nibble = frame_latched[27:24];
                3'd2: get_frame_nibble = frame_latched[23:20];
                3'd3: get_frame_nibble = frame_latched[19:16];
                3'd4: get_frame_nibble = frame_latched[15:12];
                3'd5: get_frame_nibble = frame_latched[11:8];
                3'd6: get_frame_nibble = frame_latched[7:4];
                3'd7: get_frame_nibble = frame_latched[3:0];
            endcase
        end
    endfunction

    function [3:0] get_packed_nibble;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: get_packed_nibble = packed_latched[31:28];
                3'd1: get_packed_nibble = packed_latched[27:24];
                3'd2: get_packed_nibble = packed_latched[23:20];
                3'd3: get_packed_nibble = packed_latched[19:16];
                3'd4: get_packed_nibble = packed_latched[15:12];
                3'd5: get_packed_nibble = packed_latched[11:8];
                3'd6: get_packed_nibble = packed_latched[7:4];
                3'd7: get_packed_nibble = packed_latched[3:0];
            endcase
        end
    endfunction

    function [7:0] line_byte;
        input [4:0] idx;
        begin
            case (idx)
                5'd0 : line_byte = "0";
                5'd1 : line_byte = "x";
                5'd2 : line_byte = hex_char(get_frame_nibble(3'd0));
                5'd3 : line_byte = hex_char(get_frame_nibble(3'd1));
                5'd4 : line_byte = hex_char(get_frame_nibble(3'd2));
                5'd5 : line_byte = hex_char(get_frame_nibble(3'd3));
                5'd6 : line_byte = hex_char(get_frame_nibble(3'd4));
                5'd7 : line_byte = hex_char(get_frame_nibble(3'd5));
                5'd8 : line_byte = hex_char(get_frame_nibble(3'd6));
                5'd9 : line_byte = hex_char(get_frame_nibble(3'd7));
                5'd10: line_byte = ",";
                5'd11: line_byte = "0";
                5'd12: line_byte = "x";
                5'd13: line_byte = hex_char(get_packed_nibble(3'd0));
                5'd14: line_byte = hex_char(get_packed_nibble(3'd1));
                5'd15: line_byte = hex_char(get_packed_nibble(3'd2));
                5'd16: line_byte = hex_char(get_packed_nibble(3'd3));
                5'd17: line_byte = hex_char(get_packed_nibble(3'd4));
                5'd18: line_byte = hex_char(get_packed_nibble(3'd5));
                5'd19: line_byte = hex_char(get_packed_nibble(3'd6));
                5'd20: line_byte = hex_char(get_packed_nibble(3'd7));
                5'd21: line_byte = 8'h0D; // \r
                5'd22: line_byte = 8'h0A; // \n
                default: line_byte = 8'h0A;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            state          <= ST_IDLE;
            byte_idx       <= 5'd0;
            frame_counter  <= 32'd0;
            frame_latched  <= 32'd0;
            packed_latched <= 32'd0;
            tx_start       <= 1'b0;
            tx_data        <= 8'd0;
            busy           <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy     <= 1'b0;
                    byte_idx <= 5'd0;

                    if (result_valid) begin
                        frame_latched  <= frame_counter;
                        frame_counter  <= frame_counter + 1'b1;
                        packed_latched <= result_packed;
                        busy           <= 1'b1;
                        state          <= ST_LOAD_BYTE;
                    end
                end

                ST_LOAD_BYTE: begin
                    busy <= 1'b1;

                    if (!tx_busy) begin
                        tx_data <= line_byte(byte_idx);
                        state   <= ST_START_TX;
                    end
                end

                ST_START_TX: begin
                    busy     <= 1'b1;
                    tx_start <= 1'b1;
                    state    <= ST_WAIT_TX;
                end

                ST_WAIT_TX: begin
                    busy <= 1'b1;

                    if (tx_done) begin
                        if (byte_idx == LINE_LEN - 1) begin
                            state <= ST_IDLE;
                        end else begin
                            byte_idx <= byte_idx + 1'b1;
                            state    <= ST_LOAD_BYTE;
                        end
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
