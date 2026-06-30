`timescale 1ns / 1ps
//
// dft_core_top.v
// 16-Point DFT Core 최상위 통합 랩퍼
//
// 구성: pingpong_buffer(더블 버퍼) -> dft_control_fsm(마스터 제어) ->
// 8채널 dft_bin_mac 병렬 어레이(twiddle_rom 2개씩 사용) -> magnitude_scale
// (5단 파이프라인 양자화) -> output_packer(32bit 패킹)
//

module dft_core_top (
    input  wire                 clk,
    input  wire                 rst,

    input  wire                 frame_start,
    output wire                 frame_valid,

    input  wire                 input_buf_valid,
    input  wire [31:0]          input_buf_data,

    output wire [3:0]           system_dft_addr,
    output wire [31:0]          system_dft_wdata,
    output wire                 system_dft_we
);

    wire       w_clr_acc;
    wire       w_mac_en;
    wire       w_mag_start;
    wire       w_mag_valid;
    wire [2:0] w_calc_cnt;
    wire       w_fifo_prog_full;
    wire       w_bank_swap_trigger;
    wire       w_system_dft_we;

    wire signed [15:0] w_x0, w_x1;

    wire signed [15:0] r_out[0:7];
    wire signed [15:0] i_out[0:7];
    wire [3:0]         s_b[0:7];

    // 검증용 디버그 와이어 (TB에서 hierarchical reference로도 접근 가능)
    wire        w_capture_valid;
    wire [31:0] w_q_bin_pack_debug;

    wire [7:0] s_idx0 = w_calc_cnt << 1;
    wire [7:0] s_idx1 = (w_calc_cnt << 1) + 8'd1;

    // -------------------------------------------------------------------------
    // 1. 핑퐁 프레임 버퍼
    // -------------------------------------------------------------------------
    pingpong_buffer u_pingpong_buffer (
        .clk(clk),
        .rst(rst),
        .input_buf_valid(input_buf_valid),
        .input_buf_data(input_buf_data),
        .bank_swap_trigger(w_bank_swap_trigger),
        .read_addr(w_calc_cnt),
        .x_real_0(w_x0),
        .x_real_1(w_x1),
        .fifo_prog_full(w_fifo_prog_full)
    );

    // -------------------------------------------------------------------------
    // 2. 마스터 제어 FSM
    // -------------------------------------------------------------------------
    dft_control_fsm u_control_fsm (
        .clk(clk),
        .rst(rst),
        .frame_start(frame_start),
        .fifo_prog_full(w_fifo_prog_full),
        .mag_valid(w_mag_valid),
        .clr_acc(w_clr_acc),
        .mac_en(w_mac_en),
        .mag_start(w_mag_start),
        .system_dft_we(w_system_dft_we),
        .bank_swap_trigger(w_bank_swap_trigger),
        .calc_cnt(w_calc_cnt),
        .frame_valid(frame_valid)
    );

    // -------------------------------------------------------------------------
    // 3. 8채널 독립 주파수 MAC 어레이 엔진 병렬 인스턴스화
    // -------------------------------------------------------------------------
    genvar k;
    generate
        for (k = 0; k <= 7; k = k + 1) begin : gen_dft_bin_mac_array
            wire [7:0] addr0_calc = ((k + 1) * s_idx0);
            wire [7:0] addr1_calc = ((k + 1) * s_idx1);

            wire [3:0] addr0 = addr0_calc[3:0];
            wire [3:0] addr1 = addr1_calc[3:0];

            wire signed [15:0] wr0, wi0, wr1, wi1;

            twiddle_rom u_rom0 (.addr(addr0), .w_real(wr0), .w_imag(wi0));
            twiddle_rom u_rom1 (.addr(addr1), .w_real(wr1), .w_imag(wi1));

            dft_bin_mac u_bin_mac (
                .clk(clk),
                .rst(rst),
                .clr_acc(w_clr_acc),
                .mac_en(w_mac_en),
                .x_real_n(w_x0),
                .x_real_n1(w_x1),
                .w_real_n(wr0),
                .w_imag_n(wi0),
                .w_real_n1(wr1),
                .w_imag_n1(wi1),
                .s_bin_real_out(r_out[k]),
                .s_bin_imag_out(i_out[k])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // 4. 진폭 정규화 + Capture 엔진
    // -------------------------------------------------------------------------
    magnitude_scale u_mag_scale (
        .clk(clk),
        .rst(rst),
        .mag_start(w_mag_start),
        .bin_real_0(r_out[0]), .bin_imag_0(i_out[0]),
        .bin_real_1(r_out[1]), .bin_imag_1(i_out[1]),
        .bin_real_2(r_out[2]), .bin_imag_2(i_out[2]),
        .bin_real_3(r_out[3]), .bin_imag_3(i_out[3]),
        .bin_real_4(r_out[4]), .bin_imag_4(i_out[4]),
        .bin_real_5(r_out[5]), .bin_imag_5(i_out[5]),
        .bin_real_6(r_out[6]), .bin_imag_6(i_out[6]),
        .bin_real_7(r_out[7]), .bin_imag_7(i_out[7]),
        .scaled_bin_0(s_b[0]), .scaled_bin_1(s_b[1]),
        .scaled_bin_2(s_b[2]), .scaled_bin_3(s_b[3]),
        .scaled_bin_4(s_b[4]), .scaled_bin_5(s_b[5]),
        .scaled_bin_6(s_b[6]), .scaled_bin_7(s_b[7]),
        .mag_valid(w_mag_valid),
        .capture_valid(w_capture_valid),
        .q_bin_pack_debug(w_q_bin_pack_debug)
    );

    // -------------------------------------------------------------------------
    // 5. 32비트 마스터 버스 압축 패킹 출력
    // -------------------------------------------------------------------------
    output_packer u_packer (
        .scaled_bin_0(s_b[0]), .scaled_bin_1(s_b[1]),
        .scaled_bin_2(s_b[2]), .scaled_bin_3(s_b[3]),
        .scaled_bin_4(s_b[4]), .scaled_bin_5(s_b[5]),
        .scaled_bin_6(s_b[6]), .scaled_bin_7(s_b[7]),
        .system_dft_wdata(system_dft_wdata)
    );

    assign system_dft_addr = 4'd8;
    assign system_dft_we   = w_system_dft_we;

endmodule