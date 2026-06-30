`timescale 1ns / 1ps
//
// twiddle_rom.v
// 16-Point DFT Twiddle Factor ROM
//
// addr(0~15) -> 해당 회전인자(twiddle factor)의 Q15 고정소수점 (cos, -sin) 값.
// 16-bit signed, 1.15 형식 (MSB=부호, 나머지 15bit=소수부).
//

module twiddle_rom (
    input  wire [3:0]           addr,
    output reg  signed [15:0]   w_real,
    output reg  signed [15:0]   w_imag
);

    always @(*) begin
        case (addr)
            4'd0  : begin w_real = 16'sh7FFF; w_imag = 16'sh0000; end // cos(0),  -sin(0)
            4'd1  : begin w_real = 16'sh7642; w_imag = 16'shCF04; end // cos(1),  -sin(1)
            4'd2  : begin w_real = 16'sh5A82; w_imag = 16'shA57E; end // cos(2),  -sin(2)
            4'd3  : begin w_real = 16'sh30FC; w_imag = 16'sh89BE; end // cos(3),  -sin(3)
            4'd4  : begin w_real = 16'sh0000; w_imag = 16'sh8000; end // cos(4),  -sin(4)
            4'd5  : begin w_real = 16'shCF04; w_imag = 16'sh89BE; end // cos(5),  -sin(5)
            4'd6  : begin w_real = 16'shA57E; w_imag = 16'shA57E; end // cos(6),  -sin(6)
            4'd7  : begin w_real = 16'sh89BE; w_imag = 16'shCF04; end // cos(7),  -sin(7)
            4'd8  : begin w_real = 16'sh8000; w_imag = 16'sh0000; end // cos(8),  -sin(8)
            4'd9  : begin w_real = 16'sh89BE; w_imag = 16'sh30FC; end // cos(9),  -sin(9)
            4'd10 : begin w_real = 16'shA57E; w_imag = 16'sh5A82; end // cos(10), -sin(10)
            4'd11 : begin w_real = 16'shCF04; w_imag = 16'sh7642; end // cos(11), -sin(11)
            4'd12 : begin w_real = 16'sh0000; w_imag = 16'sh7FFF; end // cos(12), -sin(12)
            4'd13 : begin w_real = 16'sh30FC; w_imag = 16'sh7642; end // cos(13), -sin(13)
            4'd14 : begin w_real = 16'sh5A82; w_imag = 16'sh5A82; end // cos(14), -sin(14)
            4'd15 : begin w_real = 16'sh7642; w_imag = 16'sh30FC; end // cos(15), -sin(15)
            default: begin
                w_real = 16'sh0000;
                w_imag = 16'sh0000;
            end
        endcase
    end

endmodule