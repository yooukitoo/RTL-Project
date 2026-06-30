`timescale 1ns / 1ps  // 시뮬레이션 시간 단위는 1ns, 표시/계산 정밀도는 1ps로 설정

// 64비트 frame_data는 16비트씩 4개의 MAX7219에 전달됨
// cascade 구조에서는 먼저 전송한 16비트가 체인의 가장 뒤쪽 MAX7219로 이동함
// 따라서 화면 순서가 반대로 보이면 frame_data 안의 16비트 묶음 순서를 바꾸면 됨

// MAX7219는 [address 8 bit] + [data 8 bit]를 받음. ex) 0x0C 0x01
// 이 모듈은 64비트를 MSB(frame_data[63])부터 1비트씩 DIN으로 내보냄
// CLK 상승 edge에서 MAX7219가 DIN 값을 읽고, CS가 다시 1이 되는 순간 값을 latch함


module max7219_frame_sender #(             // MAX7219 4개에 64비트 frame을 보내는 송신기 모듈
    parameter CLK_DIV = 50                 // 100MHz 기준: 반주기 50클럭, 전체 주기 100클럭 => 약 1MHz
)(
    input  wire        clk,                // FPGA 내부 기준 clock, 예: 100MHz
    input  wire        reset_p,            // active-high reset, 1이면 내부 상태를 초기화

    input  wire        start,              // 1클럭 동안 1이 되면 frame_data 전송 시작
    input  wire [63:0] frame_data,         // MAX4~MAX1로 보낼 64비트 데이터 묶음

    output reg         busy,               // 1이면 현재 64비트 전송 중
    output reg         done,               // 전송 완료 순간 1클럭 동안 1이 되는 완료 pulse

    output reg         max7219_din,        // MAX7219 DIN(serial data in) 핀으로 나가는 1비트 데이터
    output reg         max7219_cs,         // MAX7219 CS/load 핀, 0일 때 shift하고 1로 올라가면 latch
    output reg         max7219_clk         // MAX7219 serial clock 핀
);

    reg [63:0] shifter;                    // 전송할 64비트 frame을 저장해 두는 레지스터
    reg [6:0]  bit_cnt;                    // 현재 보낼 bit 위치, 63부터 0까지 세기 위해 7비트 사용
    reg [15:0] div_cnt;                    // 빠른 clk를 MAX7219용 느린 clk로 나누기 위한 counter
    reg        phase;                      // 0이면 다음에 CLK 상승 edge 생성, 1이면 다음에 CLK 하강 edge 생성

    always @(posedge clk or posedge reset_p) begin  // clk 상승 edge마다 동작, reset_p는 비동기 reset
        if (reset_p) begin                 // reset_p가 1이면 어떤 상태였든 바로 초기 상태로 복귀
            shifter     <= 64'd0;          // 저장해 둔 frame 값을 0으로 초기화
            bit_cnt     <= 7'd0;           // bit 위치 counter 초기화
            div_cnt     <= 16'd0;          // clock divider counter 초기화
            phase       <= 1'b0;           // reset 후에는 상승 edge를 만들 준비 상태로 둠

            busy        <= 1'b0;           // 전송 중이 아님
            done        <= 1'b0;           // 완료 pulse도 꺼 둠

            max7219_din <= 1'b0;           // DIN 기본값 0
            max7219_cs  <= 1'b1;           // CS는 idle 상태에서 1, 즉 latch/비활성 상태
            max7219_clk <= 1'b0;           // CLK는 idle 상태에서 0
        end else begin                     // reset이 아닐 때의 정상 동작
            done <= 1'b0;                  // done은 기본적으로 0, 완료된 그 순간에만 1클럭 pulse로 만듦

            if (!busy) begin               // 전송 중이 아니면 idle 상태 유지 또는 새 전송 시작을 기다림
                max7219_clk <= 1'b0;       // idle일 때 MAX7219 CLK는 0으로 고정
                max7219_cs  <= 1'b1;       // idle일 때 CS는 1로 유지해서 latch/비활성 상태 유지
                div_cnt     <= 16'd0;      // 새 전송 시작 시 divider가 깔끔하게 0부터 세도록 초기화
                phase       <= 1'b0;       // 새 전송의 첫 edge는 상승 edge가 되도록 준비

                if (start) begin           // start가 1이면 이번 clk edge에서 전송 준비 시작
                    shifter     <= frame_data;      // 입력 frame_data를 내부 레지스터에 저장
                    bit_cnt     <= 7'd63;           // 가장 먼저 보낼 bit는 MSB인 bit 63

                    busy        <= 1'b1;            // 이제부터 전송 중 상태로 진입
                    max7219_cs  <= 1'b0;            // CS를 0으로 내려 MAX7219가 serial 입력을 받게 함
                    max7219_clk <= 1'b0;            // 첫 상승 edge를 만들기 전 CLK를 0에 둠
                    max7219_din <= frame_data[63];  // 첫 상승 edge 전에 MSB를 DIN에 미리 올려둠
                end
            end else begin                 // busy가 1이면 이미 64비트 전송이 진행 중
                if (div_cnt == CLK_DIV - 1) begin   // CLK_DIV번 clk가 지나면 MAX7219 CLK를 한 번 toggle할 시점
                    div_cnt <= 16'd0;      // toggle 시점마다 divider counter를 다시 0부터 시작

                    if (phase == 1'b0) begin         // phase 0: 이번에는 CLK 상승 edge를 만들 차례
                        /*
                            CLK rising edge
                            MAX7219는 이 순간 DIN 값을 읽음
                            따라서 DIN은 이 줄보다 이전에 이미 준비되어 있어야 함
                        */
                        max7219_clk <= 1'b1;         // MAX7219 CLK를 0에서 1로 올려 상승 edge 생성
                        phase       <= 1'b1;         // 다음 toggle 때는 하강 edge를 만들도록 phase 변경
                    end else begin                   // phase 1: 이번에는 CLK 하강 edge를 만들 차례
                        /*
                            CLK falling edge
                            MAX7219가 현재 bit를 읽은 뒤, 다음 bit를 준비하는 구간
                        */
                        max7219_clk <= 1'b0;         // MAX7219 CLK를 1에서 0으로 내려 하강 edge 생성
                        phase       <= 1'b0;         // 다음 toggle 때는 다시 상승 edge를 만들도록 phase 변경

                        if (bit_cnt == 7'd0) begin   // bit 0까지 이미 상승 edge에서 읽혔다면 전송 완료
                            busy        <= 1'b0;     // 전송 중 상태 종료
                            done        <= 1'b1;     // 상위 모듈이 알 수 있게 완료 pulse 발생
                            max7219_cs  <= 1'b1;     // CS rising edge에서 MAX7219가 받은 16비트들을 latch
                            max7219_din <= 1'b0;     // 전송이 끝났으므로 DIN을 기본값 0으로 정리
                        end else begin               // 아직 보낼 bit가 남아 있으면 다음 bit 준비
                            bit_cnt     <= bit_cnt - 1'b1;          // 다음에 읽힐 bit 번호로 1 감소
                            max7219_din <= shifter[bit_cnt - 1'b1]; // 다음 상승 edge 전에 다음 bit를 DIN에 올림
                        end
                    end
                end else begin             // 아직 CLK_DIV만큼 기다리지 않았으면
                    div_cnt <= div_cnt + 1'b1;       // divider counter를 1 증가시켜 시간 지연을 만듦
                end
            end
        end
    end

endmodule
