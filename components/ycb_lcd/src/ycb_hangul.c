/**
 * @file  ycb_hangul.c
 * @brief DryerLCD::draw16Korean / draw16English / draw16String 의 LVGL canvas 포팅.
 *
 *  폰트 데이터 (컴포넌트 내부):
 *    font/malgun_gothic_32.h → U+AC00~U+D7A3 전체 32×32 맑은 고딕
 *    font/english.h    → english[128][16]   8×16 ASCII 비트맵
 *
 *  한글 조합 알고리즘:
 *    Unicode U+AC00~U+D7A3 → 초성(cho) / 중성(jung) / 종성(jong) 분리
 *    중성 벌수(4벌)·초성 벌수(8벌)·종성 벌수(4벌) 선택
 *    세 글리프를 비트OR 합성 후 픽셀 단위 렌더
 */

#include "ycb_hangul.h"
#include "font/malgun_gothic_32.h"
#include "font/english.h"
#include <string.h>

/* ─────────────────────────────────────────────────────────────────
 * 내부 픽셀 쓰기 헬퍼
 * ───────────────────────────────────────────────────────────────── */

/**
 * @brief scale 배율로 (x, y) 기준 블록 픽셀을 canvas 에 씁니다.
 *  on==true → 전경색, on==false && !transparent → 배경색
 */
static inline void _px(lv_obj_t *cv,
                       int x, int y,
                       lv_color_t fg, lv_color_t bg,
                       bool on, uint8_t scale, bool transparent)
{
    if (!on && transparent)
        return;
    lv_color_t c = on ? fg : bg;
    for (int dy = 0; dy < (int)scale; dy++)
        for (int dx = 0; dx < (int)scale; dx++)
            lv_canvas_set_px(cv, x + dx, y + dy, c, LV_OPA_COVER);
}

/* ─────────────────────────────────────────────────────────────────
 * ASCII 문자 렌더링
 *  비트맵 레이아웃: english[ch][16]
 *    [0..7]  → 전반부 8바이트 (각 바이트의 bit0=row0, bit1=row1...)
 *    [8..15] → 후반부 8바이트
 *  화면 매핑: col = byte_index(0~7), row = bit_index(0~7) + 8*(block)
 *  즉 x = _xchar + col * scale,  y = _ychar + row_of_byte * scale
 * ───────────────────────────────────────────────────────────────── */
void ycb_ascii_draw_char(lv_obj_t *canvas,
                         int x, int y,
                         lv_color_t fg, lv_color_t bg,
                         uint8_t ch,
                         uint8_t scale,
                         bool transparent)
{
    if (!canvas)
        return;
    if (ch > 0x7F)
        ch = 0x20;
    const uint8_t *font = english[ch];

    /* 배경 블록 먼저 채우기 (투명 모드가 아닐 때) */
    if (!transparent)
    {
        for (int row = 0; row < 16; row++)
            for (int col = 0; col < 8; col++)
                _px(canvas, x + col * scale, y + row * scale,
                    fg, bg, false, scale, false);
    }

    /* 두 블록(각 8바이트) 처리 */
    for (int block = 0; block < 2; block++)
    {
        for (int k = 0; k < 8; k++)
        {
            uint8_t b = font[block * 8 + k];
            for (int bit = 0; bit < 8; bit++)
            {
                if (b & (1u << bit))
                {
                    int px_x = x + k * scale;
                    int px_y = y + (block * 8 + bit) * scale;
                    _px(canvas, px_x, px_y, fg, bg, true, scale, transparent);
                }
            }
        }
    }
}

/* ─────────────────────────────────────────────────────────────────
 * 한글 문자 렌더링
 *  Unicode U+AC00~U+D7A3 분해:
 *    jong  = (code - 0xAC00) % 28
 *    first = (code - 0xAC00) / 28 / 21 + 1   (1-based: 1=ㄱ .. 19=ㅎ, 없음=1→'ㅇ' 처리)
 *    mid   = (code - 0xAC00) / 28 % 21 + 1
 *
 *  원본 DryerLCD::draw16Korean 의 벌수 테이블 그대로 사용
 * ───────────────────────────────────────────────────────────────── */
void ycb_hangul_draw_char(lv_obj_t *canvas,
                          int x, int y,
                          lv_color_t fg, lv_color_t bg,
                          uint16_t code,
                          uint8_t scale,
                          bool transparent)
{
    if (!canvas)
        return;
    if (code < 0xAC00 || code > 0xD7A3)
        return;

    /* ── 벌수 선택 룩업테이블 (원본 DryerLCD 와 동일) ── */
    /* Full Unicode Hangul table: every modern syllable is directly indexed,
       so newly added UI strings can never lose individual glyphs. */
    const uint8_t *malgun = malgun32_font[code - MALGUN32_FIRST];
    const int target = 16 * scale;
    for (int row = 0; row < target; ++row) {
        const int src_y = row * 32 / target;
        for (int col = 0; col < target; ++col) {
            const int src_x = col * 32 / target;
            const bool on = (malgun[src_y * 4 + src_x / 8] &
                             (0x80u >> (src_x & 7))) != 0;
            if (on || !transparent)
                lv_canvas_set_px(canvas, x + col, y + row,
                                 on ? fg : bg, LV_OPA_COVER);
        }
    }
}

/* ─────────────────────────────────────────────────────────────────
 * UTF-8 문자열 파서 → ycb_ascii_draw_char / ycb_hangul_draw_char 호출
 * ───────────────────────────────────────────────────────────────── */
int ycb_hangul_draw(lv_obj_t *canvas,
                    int x, int y,
                    lv_color_t fg, lv_color_t bg,
                    const char *str,
                    uint8_t scale,
                    bool transparent)
{
    if (!canvas || !str)
        return 0;
    if (scale < 1)
        scale = 1;

    int total_w = 0;
    int cx = x;

    while (*str)
    {
        uint8_t c1 = (uint8_t)*str++;

        if (c1 < 0x80u)
        {
            /* ── 1바이트 ASCII ── */
            ycb_ascii_draw_char(canvas, cx, y, fg, bg, c1, scale, transparent);
            cx += 8 * scale;
            total_w += 8 * scale;
        }
        else if ((c1 & 0xE0u) == 0xC0u)
        {
            /* ── 2바이트 UTF-8 (Latin, °기호 등) ── */
            uint8_t c2 = (uint8_t)*str++;
            uint16_t code = (uint16_t)((c1 & 0x1Fu) << 6u) | (c2 & 0x3Fu);

            if (code == 0x00B0u)
            {
                /* U+00B0 ° (도 기호): 오른쪽 위에 작은 원 표시 */
                int oy = y + 1 * scale;
                int ox = cx + 5 * scale;
                /* 4픽셀 원형 점 */
                lv_canvas_set_px(canvas, ox + 1, oy, fg, LV_OPA_COVER);
                lv_canvas_set_px(canvas, ox + 2, oy, fg, LV_OPA_COVER);
                lv_canvas_set_px(canvas, ox, oy + 1, fg, LV_OPA_COVER);
                lv_canvas_set_px(canvas, ox + 3, oy + 1, fg, LV_OPA_COVER);
                lv_canvas_set_px(canvas, ox, oy + 2, fg, LV_OPA_COVER);
                lv_canvas_set_px(canvas, ox + 3, oy + 2, fg, LV_OPA_COVER);
                lv_canvas_set_px(canvas, ox + 1, oy + 3, fg, LV_OPA_COVER);
                lv_canvas_set_px(canvas, ox + 2, oy + 3, fg, LV_OPA_COVER);
            }
            /* 그 외 2바이트 문자는 공백 폭만 차지 */
            cx += 8 * scale;
            total_w += 8 * scale;
        }
        else
        {
            /* ── 3바이트 UTF-8 (한글 포함 대부분의 CJK) ── */
            uint8_t c2 = (uint8_t)*str++;
            uint8_t c3 = (uint8_t)*str++;
            uint16_t code = (uint16_t)(((c1 & 0x0Fu) << 12u) |
                                       ((c2 & 0x3Fu) << 6u) |
                                       (c3 & 0x3Fu));

            if (code >= 0xAC00u && code <= 0xD7A3u)
            {
                /* 한글 */
                ycb_hangul_draw_char(canvas, cx, y, fg, bg, code, scale, transparent);
                cx += 16 * scale;
                total_w += 16 * scale;
            }
            else
            {
                /* 기타 3바이트 문자: 공백 */
                cx += 8 * scale;
                total_w += 8 * scale;
            }
        }
    }
    return total_w;
}

/* ─────────────────────────────────────────────────────────────────
 * 문자열 폭 측정 (그리지 않음)
 * ───────────────────────────────────────────────────────────────── */
int ycb_hangul_measure(const char *str, uint8_t scale)
{
    if (!str)
        return 0;
    if (scale < 1)
        scale = 1;

    int total = 0;
    while (*str)
    {
        uint8_t c = (uint8_t)*str++;
        if (c < 0x80u)
        {
            total += 8 * scale;
        }
        else if ((c & 0xE0u) == 0xC0u)
        {
            str++; /* 2바이트 UTF-8: 나머지 1바이트 건너뜀 */
            total += 8 * scale;
        }
        else
        {
            str++;
            str++; /* 3바이트 UTF-8: 나머지 2바이트 건너뜀 */
            total += 16 * scale;
        }
    }
    return total;
}
