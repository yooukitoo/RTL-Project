`timescale 1ns / 1ps

// ============================================================================
// led_data_mapper.v
//
// DFT에서 이미 LED 표시용 height(0~8)가 만들어져서 들어오는 경우용 mapper.
//
// 목적:
//   DFT Core/Magnitude_scale 단계에서 이미 각 bin을 0~8 높이로 정규화했다면,
//   LED Data Mapper에서는 0~15 -> 0~8 재스케일링을 하면 안 된다.
//
// 이 버전은:
//   - 입력 bin 값의 하위 값만 height로 사용
//   - height가 8보다 크면 8로 clamp
//   - scale_shift는 기존 인터페이스 호환을 위해 받지만 사용하지 않음
//   - 8개 height를 32 columns로 선형 보간
//   - 파형 아래를 채운 32x8 bitmap 생성
//
// row_bitmap_flat[row*32 + col]
//   row 0 = 위쪽 row
//   row 7 = 아래쪽 row
// ============================================================================

module led_data_mapper #(
    parameter BIN_COUNT   = 8,
    parameter DATA_W      = 32,

    // 기존 인스턴스 호환용. 이번 mapper에서는 직접 사용하지 않음.
    parameter COL_PER_BIN = 4,

    parameter ROW_COUNT   = 8,
    parameter COL_COUNT   = 32,

    // 0이면 파형 아래를 채움. 1이면 선만 표시.
    parameter LINE_ONLY   = 0
)(
    input  wire [DATA_W*BIN_COUNT-1:0] magnitude_flat,
    input  wire [4:0]                  scale_shift,

    output reg  [ROW_COUNT*COL_COUNT-1:0] row_bitmap_flat
);

    integer bin;
    integer row;
    integer col;

    integer pos;
    integer left_bin;
    integer rem;
    integer h0;
    integer h1;
    integer interp_height;
    integer line_row;
    integer height_int;

    reg [DATA_W-1:0] mag;
    reg [3:0]        height_bin [0:BIN_COUNT-1]; // 각 bin의 LED height 0~8

    always @(*) begin
        row_bitmap_flat = {(ROW_COUNT*COL_COUNT){1'b0}};

        // ------------------------------------------------------------
        // 1. DFT에서 이미 들어온 height 값을 그대로 사용
        //    0~8이면 그대로 사용
        //    8보다 크면 8로 clamp
        // ------------------------------------------------------------
        for (bin = 0; bin < BIN_COUNT; bin = bin + 1) begin
            mag = {DATA_W{1'b0}};

            case (bin)
                0: mag = magnitude_flat[31:0];
                1: mag = magnitude_flat[63:32];
                2: mag = magnitude_flat[95:64];
                3: mag = magnitude_flat[127:96];
                4: mag = magnitude_flat[159:128];
                5: mag = magnitude_flat[191:160];
                6: mag = magnitude_flat[223:192];
                7: mag = magnitude_flat[255:224];
                default: mag = {DATA_W{1'b0}};
            endcase

            // scale_shift는 여기서 사용하지 않는다.
            // 이유: DFT에서 이미 LED 표시용 height로 정규화했다는 가정.
            height_int = mag;

            if (height_int > ROW_COUNT)
                height_int = ROW_COUNT;
            else if (height_int < 0)
                height_int = 0;

            height_bin[bin] = height_int[3:0];
        end

        // ------------------------------------------------------------
        // 2. 8개 height를 32개 column으로 보간해서 하나의 파형 생성
        // ------------------------------------------------------------
        for (col = 0; col < COL_COUNT; col = col + 1) begin
            pos      = col * (BIN_COUNT - 1);      // 0 ~ 31*7
            left_bin = pos / (COL_COUNT - 1);      // 0 ~ 7
            rem      = pos - left_bin * (COL_COUNT - 1);

            if (left_bin >= BIN_COUNT - 1) begin
                interp_height = height_bin[BIN_COUNT-1];
            end else begin
                h0 = height_bin[left_bin];
                h1 = height_bin[left_bin + 1];

                interp_height = (h0 * ((COL_COUNT - 1) - rem)
                               + h1 * rem
                               + ((COL_COUNT - 1) / 2))
                               / (COL_COUNT - 1);
            end

            if (interp_height > ROW_COUNT)
                interp_height = ROW_COUNT;

            if (interp_height > 0) begin
                line_row = ROW_COUNT - interp_height; // height 8 -> row0, height1 -> row7

                for (row = 0; row < ROW_COUNT; row = row + 1) begin
                    if (LINE_ONLY) begin
                        if (row == line_row)
                            row_bitmap_flat[row*COL_COUNT + col] = 1'b1;
                    end else begin
                        if (row >= line_row)
                            row_bitmap_flat[row*COL_COUNT + col] = 1'b1;
                    end
                end
            end
        end
    end

endmodule