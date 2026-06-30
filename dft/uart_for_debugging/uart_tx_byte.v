`timescale 1ns / 1ps

// ============================================================================
// uart_tx_byte.v
//
// 8N1 UART transmitter.
// - 1 start bit
// - 8 data bits, LSB first
// - 1 stop bit
//
// Basys3 100MHz, 115200bps 기준:
//   CLKS_PER_BIT = 100_000_000 / 115200 ~= 868
// ============================================================================

module uart_tx_byte #(
    parameter CLK_FREQ_HZ = 100_000_000,
    parameter BAUD        = 115200,
    parameter CLKS_PER_BIT = CLK_FREQ_HZ / BAUD
)(
    input  wire       clk,
    input  wire       rst,

    input  wire       start,
    input  wire [7:0] data,

    output reg        tx,
    output reg        busy,
    output reg        done
);

    localparam ST_IDLE  = 3'd0;
    localparam ST_START = 3'd1;
    localparam ST_DATA  = 3'd2;
    localparam ST_STOP  = 3'd3;
    localparam ST_DONE  = 3'd4;

    reg [2:0]  state;
    reg [31:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  data_reg;

    always @(posedge clk) begin
        if (rst) begin
            state    <= ST_IDLE;
            clk_cnt  <= 32'd0;
            bit_idx  <= 3'd0;
            data_reg <= 8'd0;

            tx       <= 1'b1;  // UART idle high
            busy     <= 1'b0;
            done     <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    tx      <= 1'b1;
                    busy    <= 1'b0;
                    clk_cnt <= 32'd0;
                    bit_idx <= 3'd0;

                    if (start) begin
                        data_reg <= data;
                        busy     <= 1'b1;
                        tx       <= 1'b0;   // start bit
                        state    <= ST_START;
                    end
                end

                ST_START: begin
                    busy <= 1'b1;

                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 32'd0;
                        tx      <= data_reg[0];
                        bit_idx <= 3'd0;
                        state   <= ST_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                ST_DATA: begin
                    busy <= 1'b1;

                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 32'd0;

                        if (bit_idx == 3'd7) begin
                            tx    <= 1'b1;  // stop bit
                            state <= ST_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                            tx      <= data_reg[bit_idx + 1'b1];
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                ST_STOP: begin
                    busy <= 1'b1;

                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 32'd0;
                        state   <= ST_DONE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                ST_DONE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
