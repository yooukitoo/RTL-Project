`timescale 1ns / 1ps
//
// dft_control_fsm.v
// DFT Core 마스터 제어 FSM
//
// 한 프레임(16-sample) 처리 흐름:
//   S_IDLE      frame_start && fifo_prog_full 대기, clr_acc로 누적기 초기화
//   S_LOAD      1클럭 정착
//   S_CALC      calc_cnt(0~7) 동안 mac_en=1, 8회 MAC 누적
//   S_HOLD      마지막 누적이 끝나길 1클럭 대기 (mac_en은 이 상태가
//               끝나는 edge에서야 0이 되므로, r_out은 다음 상태부터
//               최종값으로 안정된다)
//   S_CAPTURE   mag_start(아래 참조)로 magnitude_scale 파이프라인을
//               시작시키고, mag_valid가 뜰 때까지 대기(가변 길이)
//   S_PACK      system_dft_we 펄스로 결과 기록
//   S_FRAME_DONE  bank_swap_trigger·frame_valid 발생 후 S_IDLE로 복귀
//
// mag_start는 S_HOLD를 2클럭 레지스터로 지연시킨 신호다. dft_bin_mac 입력
// 파이프라인 때문에 마지막 누적 결과가 S_HOLD 직후 바로 안정되지 않으므로,
// magnitude_scale이 이전/중간 누적값을 캡처하지 않도록 r_out이 최종값으로
// 안정된 뒤 1클럭 펄스로 내보낸다.
//

module dft_control_fsm (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 frame_start,
    input  wire                 fifo_prog_full,
    input  wire                 mag_valid,           // magnitude_scale 파이프라인 완료 펄스

    output reg                  clr_acc,
    output reg                  mac_en,
    output wire                 mag_start,           // S_HOLD를 2클럭 지연시킨 펄스 (아래 참조)
    output reg                  system_dft_we,
    output reg                  bank_swap_trigger,
    output reg [2:0]            calc_cnt,
    output reg                  frame_valid
);

    localparam S_IDLE        = 3'h0;
    localparam S_LOAD        = 3'h1;
    localparam S_CALC        = 3'h2;
    localparam S_HOLD        = 3'h3; // 데이터 정착용 1사이클 안정화 보험 상태 + mag_start 트리거
    localparam S_CAPTURE     = 3'h4; // magnitude_scale 파이프라인 완료(mag_valid) 대기 상태
    localparam S_PACK        = 3'h5;
    localparam S_FRAME_DONE  = 3'h6;

    reg [2:0] current_state;
    reg [2:0] next_state;

    always @(posedge clk or posedge rst) begin
        if (rst)    current_state <= S_IDLE;
        else        current_state <= next_state;
    end

    // 조합 논리 기반 차기 상태 전이
    always @(*) begin
        case (current_state)
            S_IDLE: begin
                if (frame_start && fifo_prog_full) next_state = S_LOAD;
                else                               next_state = S_IDLE;
            end
            S_LOAD: begin
                next_state = S_CALC;
            end
            S_CALC: begin
                if (calc_cnt == 3'd7) next_state = S_HOLD;
                else                  next_state = S_CALC;
            end
            S_HOLD: begin
                next_state = S_CAPTURE;
            end
            S_CAPTURE: begin
                // magnitude_scale 6클럭 파이프라인이 끝나 mag_valid가 뜰 때까지 대기
                next_state = mag_valid ? S_PACK : S_CAPTURE;
            end
            S_PACK: begin
                next_state = S_FRAME_DONE;
            end
            S_FRAME_DONE: begin
                next_state = S_IDLE;
            end
            default: next_state = S_IDLE;
        endcase
    end

    // mag_start: dft_bin_mac 입력 pipeline 추가로 MAC 결과 안정 시점이
    // 기존보다 1클럭 늦어졌으므로, S_HOLD를 2클럭 지연시킨 펄스로 만든다.
    //
    // 기존 1클럭 지연:
    //   S_HOLD -> S_CAPTURE 진입 시점에 mag_start
    //
    // 수정 2클럭 지연:
    //   S_HOLD -> dft_bin_mac의 마지막 pipeline 누적 완료 -> mag_start
    //
    // 이렇게 해야 magnitude_scale이 마지막 sample pair까지 누적된 r_out을 캡처한다.
    wire mag_start_raw = (current_state == S_HOLD);
    reg  mag_start_d1;
    reg  mag_start_d2;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mag_start_d1 <= 1'b0;
            mag_start_d2 <= 1'b0;
        end else begin
            mag_start_d1 <= mag_start_raw;
            mag_start_d2 <= mag_start_d1;
        end
    end

    assign mag_start = mag_start_d2;

    // 순차 회로 제어선 출력 제어
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            calc_cnt          <= 3'd0;
            clr_acc           <= 1'b0;
            mac_en            <= 1'b0;
            system_dft_we     <= 1'b0;
            bank_swap_trigger <= 1'b0;
            frame_valid       <= 1'b0;
        end else begin
            case (current_state)
                S_IDLE: begin
                    calc_cnt          <= 3'd0;
                    clr_acc           <= 1'b1;
                    mac_en            <= 1'b0;
                    system_dft_we     <= 1'b0;
                    bank_swap_trigger <= 1'b0;
                    frame_valid       <= 1'b0;
                end
                S_LOAD: begin
                    clr_acc           <= 1'b0;
                end
                S_CALC: begin
                    mac_en            <= 1'b1;
                    calc_cnt          <= calc_cnt + 1'b1;
                end
                S_HOLD: begin
                    mac_en            <= 1'b0;
                    calc_cnt          <= 3'd0;
                    // mag_start는 위에서 조합으로 처리되므로 여기서는 별도 대입 없음
                end
                S_CAPTURE: begin
                    // mag_valid를 기다리는 동안 별도 출력 변화 없음
                end
                S_PACK: begin
                    system_dft_we     <= 1'b1;
                end
                S_FRAME_DONE: begin
                    system_dft_we     <= 1'b0;
                    bank_swap_trigger <= 1'b1;
                    frame_valid       <= 1'b1;
                end
            endcase
        end
    end

endmodule