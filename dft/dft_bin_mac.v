`timescale 1ns / 1ps
//
// dft_bin_mac.v
// 단일 주파수 Bin 독립 MAC 연산 엔진 (8개 Bin 각각 병렬 인스턴스화 대상)
//
// Timing Closure 수정본:
//   - 기존 critical path:
//       calc_cnt -> pingpong_buffer mem_array read -> twiddle_rom -> multiplier/add -> acc
//   - 수정:
//       x/twiddle 입력을 1클럭 레지스터링하고, mac_en도 1클럭 지연시켜
//       RAM/ROM read 경로와 DSP MAC 누적 경로를 분리한다.
//   - 이 변경 때문에 최종 acc/r_out 안정 시점이 기존보다 1클럭 늦어진다.
//     dft_control_fsm.v의 mag_start도 그에 맞춰 2클럭 지연으로 조정해야 한다.
//

module dft_bin_mac (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 clr_acc,
    input  wire                 mac_en,

    // 시간 영역 샘플 쌍 입력 (Real-only)
    input  wire signed [15:0]   x_real_n,    // Sample N
    input  wire signed [15:0]   x_real_n1,   // Sample N+1

    // Twiddle ROM 연동 회전인자 입력 쌍
    input  wire signed [15:0]   w_real_n,    // W_r[n]
    input  wire signed [15:0]   w_imag_n,    // W_i[n]
    input  wire signed [15:0]   w_real_n1,   // W_r[n+1]
    input  wire signed [15:0]   w_imag_n1,   // W_i[n+1]

    // 최종 Q15 포맷 복원 포화 출력 포트
    output reg  signed [15:0]   s_bin_real_out,
    output reg  signed [15:0]   s_bin_imag_out
);

    // -------------------------------------------------------------------------
    // Stage 0: 입력 샘플 + Twiddle register
    // -------------------------------------------------------------------------
    // 목적:
    //   calc_cnt/read_addr/twiddle_addr에서 바로 DSP 누적기로 이어지는 긴
    //   조합 경로를 끊는다. mac_en이 1인 사이클의 입력을 먼저 저장하고,
    //   다음 클럭에서 저장된 값을 MAC 누적에 사용한다.
    //
    // 주의:
    //   mac_en_d가 1일 때 누적하므로, 기능상 MAC 결과는 기존보다 1클럭 늦게
    //   최종 안정된다. FSM의 mag_start 지연도 같이 수정해야 한다.
    // -------------------------------------------------------------------------
    reg signed [15:0] x_real_n_r;
    reg signed [15:0] x_real_n1_r;
    reg signed [15:0] w_real_n_r;
    reg signed [15:0] w_imag_n_r;
    reg signed [15:0] w_real_n1_r;
    reg signed [15:0] w_imag_n1_r;
    reg               mac_en_d;

    always @(posedge clk) begin
        if (rst) begin
            x_real_n_r  <= 16'sd0;
            x_real_n1_r <= 16'sd0;
            w_real_n_r  <= 16'sd0;
            w_imag_n_r  <= 16'sd0;
            w_real_n1_r <= 16'sd0;
            w_imag_n1_r <= 16'sd0;
            mac_en_d    <= 1'b0;
        end else if (clr_acc) begin
            mac_en_d    <= 1'b0;
        end else begin
            mac_en_d <= mac_en;

            if (mac_en) begin
                x_real_n_r  <= x_real_n;
                x_real_n1_r <= x_real_n1;
                w_real_n_r  <= w_real_n;
                w_imag_n_r  <= w_imag_n;
                w_real_n1_r <= w_real_n1;
                w_imag_n1_r <= w_imag_n1;
            end
        end
    end

    // 1. Expression Sizing 제약 회피용 명시적 32-bit signed 확장선
    wire signed [31:0] x0_ext    = { {16{x_real_n_r[15]}},  x_real_n_r  };
    wire signed [31:0] x1_ext    = { {16{x_real_n1_r[15]}}, x_real_n1_r };
    wire signed [31:0] w0_ext    = { {16{w_real_n_r[15]}},  w_real_n_r  };
    wire signed [31:0] w1_ext    = { {16{w_real_n1_r[15]}}, w_real_n1_r };
    wire signed [31:0] w0_im_ext = { {16{w_imag_n_r[15]}},  w_imag_n_r  };
    wire signed [31:0] w1_im_ext = { {16{w_imag_n1_r[15]}}, w_imag_n1_r };

    // 2. DSP48E1 추론 유도 + 부호 확장 기반 곱셈
    (* use_dsp = "yes" *) wire signed [31:0] mult_real_0;
    (* use_dsp = "yes" *) wire signed [31:0] mult_imag_0;
    (* use_dsp = "yes" *) wire signed [31:0] mult_real_1;
    (* use_dsp = "yes" *) wire signed [31:0] mult_imag_1;

    assign mult_real_0 = x0_ext * w0_ext;
    assign mult_imag_0 = x0_ext * w0_im_ext;
    assign mult_real_1 = x1_ext * w1_ext;
    assign mult_imag_1 = x1_ext * w1_im_ext;

    // 3. 비트 캐리 소실 방지용 33비트 가산기
    wire signed [32:0] add_real;
    wire signed [32:0] add_imag;

    assign add_real = $signed(mult_real_0) + $signed(mult_real_1);
    assign add_imag = $signed(mult_imag_0) + $signed(mult_imag_1);

    // 4. 40비트 독립 복소 누적기
    reg signed [39:0] acc_real;
    reg signed [39:0] acc_imag;

    always @(posedge clk) begin
        if (rst) begin
            acc_real <= 40'sd0;
            acc_imag <= 40'sd0;
        end else if (clr_acc) begin
            acc_real <= 40'sd0;
            acc_imag <= 40'sd0;
        end else if (mac_en_d) begin
            acc_real <= acc_real + { {7{add_real[32]}}, add_real };
            acc_imag <= acc_imag + { {7{add_imag[32]}}, add_imag };
        end
    end

    wire signed [39:0] acc_real_signed = acc_real;
    wire signed [39:0] acc_imag_signed = acc_imag;

    // 5. Q15 복원(>>15)에 부합하는 포화 임계값
    localparam signed [39:0] SAT_POS = 40'sh003FFF8000;  //  32767 << 15
    localparam signed [39:0] SAT_NEG = 40'shFFC0000000;  // -32768 << 15

    always @(*) begin
        if (acc_real_signed > SAT_POS)
            s_bin_real_out = 16'h7FFF;
        else if (acc_real_signed < SAT_NEG)
            s_bin_real_out = 16'h8000;
        else
            s_bin_real_out = acc_real_signed[30:15];

        if (acc_imag_signed > SAT_POS)
            s_bin_imag_out = 16'h7FFF;
        else if (acc_imag_signed < SAT_NEG)
            s_bin_imag_out = 16'h8000;
        else
            s_bin_imag_out = acc_imag_signed[30:15];
    end

endmodule