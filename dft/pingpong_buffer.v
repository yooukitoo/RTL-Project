`timescale 1ns / 1ps
//
// pingpong_buffer.v
// 16-sample 핑퐁 프레임 버퍼 (8-word 단위 더블 버퍼링)
//
// 외부에서 32bit 워드 단위로 들어오는 샘플을 8개씩 모아 한 뱅크에 채우고,
// 다른 뱅크는 연산기가 읽는 동안 새 프레임을 받는 더블 버퍼 구조다.
//
// 설계 정책:
//   - produce_done: 생산 완료 조건(write_ptr이 7이 되는 입력)을 실재
//     wire로 노출하여, 검증 코드가 가리킬 구체적인 신호를 보장한다.
//   - pending_swap: bank_swap_trigger(소비측 요청)를 "저장"과 "소비"로
//     분리해, 생산과 동시에 요청이 들어와도 유실 없이 보존한다.
//   - bank_ready: produce_done > 신규 swap 요청 저장 > pending swap 소비
//     순으로 단일 우선순위 체인에서만 갱신해, 같은 레지스터에 대한
//     다중 NBA 대입 충돌을 구조적으로 차단한다.
//   - fifo_prog_full: "읽을 수 있는 완성된 뱅크가 존재한다"를 그대로
//     의미하도록 콤비네이션으로 고정한다.
//

module pingpong_buffer (
    input  wire                 clk,
    input  wire                 rst,

    // 외부 데이터 쓰기 인터페이스
    input  wire                 input_buf_valid,
    input  wire [31:0]          input_buf_data,

    // 내부 제어단 컨트롤 인터페이스
    input  wire                 bank_swap_trigger,
    input  wire [2:0]           read_addr,

    // 내부 연산기 분배 출력 포트 (index 0, 1 실수 샘플 쌍)
    output wire signed [15:0]   x_real_0,
    output wire signed [15:0]   x_real_1,

    // 상태 플래그
    output wire                 fifo_prog_full
);

    reg [31:0] mem_array [0:15];
    reg [3:0]  write_ptr;
    reg        write_bank;
    reg        read_bank;
    reg        bank_ready;
    reg        pending_swap;

    // 뱅크 인터페이스 물리 주소 매핑
    wire [3:0] physical_write_addr = {write_bank, write_ptr[2:0]};
    wire [3:0] physical_read_addr  = {read_bank,  read_addr};

    // 생산 완료 조건 (입력이 유효하고 그 입력이 뱅크의 마지막 워드일 때)
    wire produce_done;
    assign produce_done = input_buf_valid && (write_ptr[2:0] == 3'd7);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            write_ptr    <= 4'd0;
            write_bank   <= 1'b0;
            // read_bank를 write_bank와 동일한 값(0)으로 초기화한다.
            // (서로 다른 값(1)으로 두면 첫 프레임 완성 시 write_bank가
            //  toggle되며 read_bank와 같아져 fifo_prog_full이 거짓이
            //  되고, FSM이 S_IDLE을 벗어나지 못해 bank_swap_trigger도
            //  발생하지 않는 데드락이 생긴다.)
            read_bank    <= 1'b0;
            bank_ready   <= 1'b0;
            pending_swap <= 1'b0;
        end else begin

            // 1. 외부 데이터 입력 및 물리 적재 (조건과 무관하게 항상 수행)
            if (input_buf_valid) begin
                mem_array[physical_write_addr] <= input_buf_data;
                if (write_ptr[2:0] == 3'd7)
                    write_ptr <= 4'd0;
                else
                    write_ptr <= write_ptr + 1'b1;
            end

            // 2. write_bank는 생산 완료에만 반응 (독립 블록)
            if (produce_done) begin
                write_bank <= ~write_bank;
            end

            // 3. pending_swap 저장/소비 분리 (read_bank 전용)
            //    저장: 새 swap 요청이 들어오면 무조건 저장 (유실 방지)
            //    소비: 생산이 겹치지 않는 안전한 클럭에서만 소비
            if (bank_swap_trigger) begin
                pending_swap <= 1'b1;
            end else if (pending_swap && !produce_done) begin
                read_bank    <= ~read_bank;
                pending_swap <= 1'b0;
            end

            // 4. bank_ready 전용 우선순위 체인 (다중 writer 패턴 차단)
            //    정책: produce_done > 신규 swap 요청 저장 > pending swap 소비
            if (produce_done) begin
                bank_ready <= 1'b1;
            end else if (pending_swap && !bank_swap_trigger) begin
                bank_ready <= 1'b0;
            end
        end
    end

    // fifo_prog_full: "읽을 수 있는 완성된 뱅크가 존재한다"는 의미로 고정
    assign fifo_prog_full = bank_ready && (write_bank != read_bank);

    // 32비트 패킹 워드를 16비트 정수 샘플 쌍으로 실시간 언패킹 분배
    assign x_real_0 = $signed(mem_array[physical_read_addr][15:0]);
    assign x_real_1 = $signed(mem_array[physical_read_addr][31:16]);

    // Phase A(force 기반 unit test) 동안은 인터페이스/계약성 assertion을
    // 끄고, Phase B(자연 경로)부터 켜기 위한 게이트. 실제 assertion 본체는
    // dft_core_top_tb.v에서 본 모듈의 내부 신호를 hierarchical reference로
    // 직접 참조하여 평가한다.
    reg enable_interface_assertions;
    initial enable_interface_assertions = 1'b0;

endmodule