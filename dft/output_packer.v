`timescale 1ns / 1ps
//
// output_packer.v
// 8개 4비트 Bin 높이 값을 32비트 버스로 압축 패킹
//

module output_packer (
    input  wire [3:0]           scaled_bin_0, // Bin 1 (RTL index 0)
    input  wire [3:0]           scaled_bin_1, // Bin 2 (RTL index 1)
    input  wire [3:0]           scaled_bin_2, // Bin 3 (RTL index 2)
    input  wire [3:0]           scaled_bin_3, // Bin 4 (RTL index 3)
    input  wire [3:0]           scaled_bin_4, // Bin 5 (RTL index 4)
    input  wire [3:0]           scaled_bin_5, // Bin 6 (RTL index 5)
    input  wire [3:0]           scaled_bin_6, // Bin 7 (RTL index 6)
    input  wire [3:0]           scaled_bin_7, // Bin 8 (RTL index 7)

    output wire [31:0]          system_dft_wdata
);

    assign system_dft_wdata = {
        scaled_bin_7, // Bin 8 [31:28]
        scaled_bin_6, // Bin 7 [27:24]
        scaled_bin_5, // Bin 6 [23:20]
        scaled_bin_4, // Bin 5 [19:16]
        scaled_bin_3, // Bin 4 [15:12]
        scaled_bin_2, // Bin 3 [11:8]
        scaled_bin_1, // Bin 2 [7:4]
        scaled_bin_0  // Bin 1 [3:0]
    };

endmodule