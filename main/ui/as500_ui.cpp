/*
 * as500_ui.cpp
 * ─────────────────────────────────────────────────────────────
 *  청소차 자율추종 LVGL v8 대시보드 — AS500UI 구현
 *  보드 : JC1060P470C_I_W  (ESP32-P4 + 1024×600 MIPI-DSI)
 *
 *  UI 레이아웃 (1024×600)
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  ⚡ AS500 v1.0       ●L  ●R     [ FOLLOW ]      │ h=54
 *  ├──────────────┬──────────────────────┬───────────────────┤
 *  │ UWB SENSOR   │  HEADING             │ MOTOR OUTPUT      │
 *  │  ┌─arc─┐    │        ↑             │ L [████████] +800 │
 *  │  │ 123 │    │     FOLLOW           │ R [██████░░] +600 │
 *  │  └─────┘    │  AVG  145 cm         │  CAN: 500kbps OK  │
 *  │  LEFT        │  DIFF +44 cm         │                   │
 *  │  ┌─arc─┐    │                      │                   │
 *  │  │ 167 │    │                      │                   │
 *  │  └─────┘    │                      │                   │
 *  │  RIGHT       │                      │                   │
 *  ├──────────────┴──────────────────────┴───────────────────┤
 *  │  dL=123  dR=167  avg=145  diff=+44  L=+800  R=+600     │ h=72
 *  └──────────────────────────────────────────────────────────┘
 * ─────────────────────────────────────────────────────────────
 */

#include "as500_ui.h"
#include "../can/can_protocol.h"
#include "../manual_input.h"
#include "../uwb_geometry.h"
#include "as_logo.h"
#include "bsp/esp32_p4_function_ev_board.h"  /* BSP_LCD_H_RES / V_RES */
#include "ycb_hangul.h"
#include "Buzzer.hpp"
#include "as500_mp3_player.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "nvs.h"
#include "esp_log.h"
#include <algorithm>
#include <cstring>
#include <math.h>

/* ─── 디스플레이 해상도 ─────────────────────────────────── */
#define DISP_W   BSP_LCD_H_RES   /* 1024 */
#define DISP_H   BSP_LCD_V_RES   /* 600  */

/* ─── arc 게이지 색상 임계값 (cm) ──────────────────────── */
static constexpr int kStopDist   = 50;   /* 이하 → 적색 */
static constexpr int kFollowDist = 100;  /* 이하 → 황색 */
static constexpr int kLidarDisplayRadius = 135;
static constexpr int kLidarFiveMeterRadius = 225;
static constexpr int kSettingCount = 27;
/* Allows the LiDAR overlay to be isolated without removing acquisition. */
static constexpr bool kLidarOverlayEnabled = true;

/* LVGL's built-in printf formatter may be configured without float support. */
static void set_tilt_label(lv_obj_t *canvas, bool valid, float degrees);

std::atomic<control_cmd_t> AS500UI::s_pending_cmd{CMD_NONE};
std::atomic<control_cmd_t> AS500UI::s_vehicle_fnr{CMD_NONE};
std::atomic<bool> AS500UI::s_interface_cruise_pressed{false};
static ManualPress s_manual_press;
static lv_obj_t *s_touch_stop = nullptr, *s_touch_estop = nullptr;
static lv_obj_t *s_touch_cruise = nullptr;
static bool s_touch_release_required = false;
static std::atomic<bool> s_touch_cruise_pressed{false};
static bool s_physical_touch_down = false;

static control_cmd_t cruise_transition_command(bool was_pressed,
                                                bool is_pressed,
                                                control_cmd_t direction)
{
    if (was_pressed == is_pressed) return CMD_NONE;
    if (!is_pressed) return CMD_STOP;
    return direction == CMD_FORWARD || direction == CMD_REVERSE
        ? direction : CMD_STOP;
}

enum Mp3UiAction : uintptr_t {
    MP3_UI_CLOSE = 1,
    MP3_UI_MODE,
    MP3_UI_PREVIOUS_PAGE,
    MP3_UI_NEXT_PAGE,
    MP3_UI_TRACK_BASE = 100,
};
static void register_touch_button(lv_obj_t *button, control_cmd_t cmd) {
    if (cmd == CMD_STOP) s_touch_stop = button;
    if (cmd == CMD_ESTOP) s_touch_estop = button;
}
static lv_obj_t *s_settings_overlay = nullptr;
static lv_obj_t *s_row_settings_overlay = nullptr;
static AS500UI *s_ui_instance = nullptr;
static lv_obj_t *s_settings_saved_popup = nullptr;
static lv_obj_t *s_setting_rows[20] = {};
static lv_obj_t *s_setting_values[20] = {};
static lv_obj_t *s_setting_name_canvases[16] = {};
static lv_obj_t *s_settings_title_canvas = nullptr;
static lv_obj_t *s_lidar_range_canvas = nullptr;
static int s_setting_selected = 0;
static constexpr int kVisibleSettingCount = 16;
enum class SettingsPage { Control, Sensor, Analog };
static SettingsPage s_settings_page = SettingsPage::Control;
static const int s_control_setting_map[16] = {
    0, 1, 2, 3, 4, 5, 23, 24, 8, 9, 14, 15, 21, 22, 6, 7
};
static const int s_sensor_setting_map[16] = {
    16, 17, 10, 11, 12, 13, 18, 19, 20, 26, -1, -1, -1, -1, -1, -1
};
static const int s_analog_setting_map[16] = {
    0, 1, 2, 3, 4, 5, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1
};
static const char *s_control_setting_names[16] = {
    "01   정지거리", "02   최대 추정거리",
    "03   최대 추종속도", "04   근접속도",
    "05   감속시간", "06   가속시간", "07   회전 게인", "08   회전 PWM 출력",
    "09   충전경보", "10   회전기동보조시간", "11   주행회전감속",
    "12   추종조향 안쪽PWM", "13   청소기 출력", "14   누적시간 초기화",
    "15   주행 가속시간", "16   주행 감속시간"
};
static const char *s_sensor_setting_names[16] = {
    "01   UWB KALMAN RANGE R", "02   UWB KALMAN MOTION Q",
    "03   라이다 경보거리", "04   라이다 정지거리",
    "05   라이다 감지각도", "06   라이다 감지범위",
    "07   라이다 최소신뢰도", "08   라이다 유지시간",
    "09   라이다 최대감지거리", "10   음영영역 배율", "", "", "", "", "", ""};
static const char *s_analog_setting_names[16] = {
    "01   THROTTLE 1 OFFSET", "02   THROTTLE 1 MAX",
    "03   THROTTLE 2 OFFSET", "04   THROTTLE 2 MAX",
    "05   VR 1 OFFSET",       "06   VR 1 MAX",
    "07   VR 2 OFFSET",       "08   VR 2 MAX",
    "", "", "", "", "", "", "", ""
};
static const float s_setting_defaults[kSettingCount] = {
    20, 2000, 1.0f, 0.3f, 0.5f, 0.5f, 0.5f, 0.5f, 42.0f, 300,
    50, 100, 0, 120, 50, 100, 10, 150, 20, 500, 300, 80, 0, 2.0f, 100, 500, 3
};
static float s_setting_data[kSettingCount] = {
    20, 2000, 1.0f, 0.3f, 0.5f, 0.5f, 0.5f, 0.5f, 42.0f, 300,
    50, 100, 0, 120, 50, 100, 10, 150, 20, 500, 300, 80, 0, 2.0f, 100, 500, 3
};
static float s_setting_saved[kSettingCount] = {};
static const float s_setting_step[kSettingCount] = {
    5, 100, .1f, .1f, .1f, .1f, .1f, .1f, .1f, 100,
    5, 5, 1, 5, 5, 10, 5, 5, 5, 50, 50, 1, 1, .1f, 5, 100, 1
};
static const float s_setting_min[kSettingCount] = {
    10, 300, .1f, .1f, .5f, .5f, .5f, .5f, 36.0f, 100,
    10, 20, -90, 10, -100, -100, 5, 5, 0, 100, 100, 0, 0, .5f, 0, 100, 1
};
static const float s_setting_max[kSettingCount] = {
    300, 2000, 1.0f, 1.0f, 5.0f, 5.0f, 5.0f, 5.0f, 55.0f, 1000,
    500, 1000, 90, 360, 100, 100, 100, 300, 255, 2000, 600, 100, 1, 3.5f, 100, 2500, 3
};

static bool s_vacuum_runtime_reset_requested = false;

/* Interface analog input calibration, stored as OFFSET/MAX pairs for
   Throttle1, Throttle2, VR1 and VR2. */
static constexpr int kAnalogChannelCount = 4;
static uint8_t s_analog_calibration[kAnalogChannelCount][2] = {
    {0, 255}, {0, 255}, {0, 255}, {0, 255}
};
static uint8_t s_analog_calibration_on_open[kAnalogChannelCount][2] = {};
static uint8_t s_analog_raw[kAnalogChannelCount] = {};
static lv_timer_t *s_analog_settings_long_press_timer = nullptr;

static constexpr uint32_t kAnalogTextColor = 0xF4F7F5;

static lv_obj_t *create_analog_text_canvas(lv_obj_t *parent, int x, int y,
                                           int width, int height,
                                           const char *text)
{
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_obj_set_pos(canvas, x, y);
    lv_obj_set_size(canvas, width, height);
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
    void *buffer = heap_caps_calloc((size_t)width * height, 4,
                                    MALLOC_CAP_SPIRAM);
    if (!buffer) {
        ESP_LOGE("AS500UI", "Failed to allocate analog text canvas");
        lv_obj_delete(canvas);
        return nullptr;
    }
    lv_canvas_set_buffer(canvas, buffer, width, height,
                         LV_COLOR_FORMAT_ARGB8888);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_TRANSP);
    const int text_width = ycb_hangul_measure(text, 2);
    ycb_hangul_draw(canvas, std::max(0, (width - text_width) / 2),
                    std::max(0, (height - 32) / 2),
                    lv_color_hex(kAnalogTextColor), lv_color_black(),
                    text, 2, true);
    return canvas;
}

static void update_analog_text_canvas(lv_obj_t *canvas, const char *text)
{
    if (!canvas) return;
    const int width = lv_obj_get_width(canvas);
    const int height = lv_obj_get_height(canvas);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_TRANSP);
    const int text_width = ycb_hangul_measure(text, 2);
    ycb_hangul_draw(canvas, std::max(0, (width - text_width) / 2),
                    std::max(0, (height - 32) / 2),
                    lv_color_hex(kAnalogTextColor), lv_color_black(),
                    text, 2, true);
}

static void load_analog_calibration()
{
    nvs_handle_t h;
    if (nvs_open("analog_cfg", NVS_READONLY, &h) == ESP_OK) {
        size_t size = sizeof(s_analog_calibration);
        if (nvs_get_blob(h, "ranges", s_analog_calibration, &size) != ESP_OK ||
            size != sizeof(s_analog_calibration)) {
            const uint8_t defaults[kAnalogChannelCount][2] = {
                {0, 255}, {0, 255}, {0, 255}, {0, 255}
            };
            memcpy(s_analog_calibration, defaults, sizeof(defaults));
        }
        nvs_close(h);
    }
    for (int i = 0; i < kAnalogChannelCount; ++i) {
        if (s_analog_calibration[i][1] <= s_analog_calibration[i][0]) {
            s_analog_calibration[i][0] = 0;
            s_analog_calibration[i][1] = 255;
        }
    }
}

static bool save_analog_calibration()
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("analog_cfg", NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE("AS500UI", "ADC calibration NVS open failed: %s",
                 esp_err_to_name(err));
        return false;
    }
    err = nvs_set_blob(h, "ranges", s_analog_calibration,
                       sizeof(s_analog_calibration));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE("AS500UI", "ADC calibration NVS save failed: %s",
                 esp_err_to_name(err));
        return false;
    }
    return true;
}

/* AS48-compatible setup menu used by the AS500 dashboard. */
static constexpr int kAs48SettingCount = 25;
static lv_obj_t *s_as48_title = nullptr;
static lv_obj_t *s_as48_names[kAs48SettingCount] = {};
static lv_obj_t *s_tilt_settings_overlay = nullptr;
static lv_obj_t *s_tilt_settings_title = nullptr;
static lv_obj_t *s_tilt_setting_rows[2] = {};
static float s_tilt_value_on_open[3] = {};
static void canvas_fill_rect(lv_obj_t *canvas, int x, int y, int w, int h,
                             lv_color_t color);
static const char *s_as48_names_all[kAs48SettingCount] = {
    "배터리 전압", "기울기 경보 각도", "롤 경보(호환)", "기울기 경보",
    "피치 보정", "롤 보정", "최대 FET 온도", "최대 모터 온도",
    "최대 전류", "가속 시간", "감속 시간", "장애물 정지 거리",
    "장애물 감속 거리", "속도 계수", "브레이크 지연", "모터 1 역방향",
    "모터 2 역방향", "구동 보정", "모터 좌우 교환", "배터리 경보 잔량",
    "경보 지속 시간", "모터 최소 PWM", "IMU 영점", "JYME02 설정",
    "조향 역방향"
};
static float s_as48_value[kAs48SettingCount] = {
    48, 15, 15, 1, 0, 0, 90, 90, 300, 1.0f, 1.0f, 0.3f, 1.5f,
    1.0f, 200, 0, 0, 0, 0, 20, 30, 10, 0, 0, 0
};
static float s_as48_value_on_open[kAs48SettingCount] = {};
static const float s_as48_min[kAs48SettingCount] = {
    12, 1, 1, 0, -180, -180, 60, 60, 50, .1f, .1f, .1f, .5f,
    .1f, 0, 0, 0, -1, 0, 5, 10, 5, 0, 0, 0
};
static const float s_as48_max[kAs48SettingCount] = {
    60, 45, 45, 1, 180, 180, 120, 120, 600, 10, 10, 1, 3,
    5, 1000, 1, 1, 1, 1, 50, 120, 50, 1, 1, 1
};
static const float s_as48_step[kAs48SettingCount] = {
    12, 1, 1, 1, 1, 1, 5, 5, 5, .1f, .1f, .05f, .1f,
    .1f, 50, 1, 1, .1f, 1, 1, 10, 1, 1, 1, 1
};

static bool as48_is_on_off(int index)
{
    return index == 3 || index == 15 || index == 16 ||
           index == 18 || index == 24;
}

static bool as48_is_menu_item(int index)
{
    /* Tilt alarm (1/2/3), offsets (4/5), and IMU zero (22) are managed by
       the dedicated dashboard Pitch/Roll long-press screen. */
    return index > 0 && index < kAs48SettingCount &&
           index != 1 && index != 2 && index != 3 &&
           index != 4 && index != 5 && index != 22;
}

static int as48_menu_number(int index)
{
    int number = 0;
    for (int i = 1; i <= index; ++i)
        if (as48_is_menu_item(i)) ++number;
    return number;
}

static void as48_format_value(int index, char *out, size_t size)
{
    if (index < 0 || index >= kAs48SettingCount) {
        snprintf(out, size, "--");
        return;
    }
    if (as48_is_on_off(index))
        snprintf(out, size, "%s", s_as48_value[index] >= .5f ? "ON" : "OFF");
    else if (index == 22 || index == 23)
        snprintf(out, size, "%s", s_as48_value[index] >= .5f ? "준비" : "실행");
    else if (index == 9 || index == 10)
        snprintf(out, size, "%.1f 초", (double)s_as48_value[index]);
    else if (index == 11 || index == 12)
        snprintf(out, size, "%.2f m", (double)s_as48_value[index]);
    else if (index == 13 || index == 17)
        snprintf(out, size, "%.1f", (double)s_as48_value[index]);
    else if (index == 14 || index == 20)
        snprintf(out, size, "%.0f ms", (double)s_as48_value[index]);
    else if (index == 19 || index == 21)
        snprintf(out, size, "%.0f %%", (double)s_as48_value[index]);
    else if (index == 0)
        snprintf(out, size, "%.0f V", (double)s_as48_value[index]);
    else if (index >= 1 && index <= 5)
        snprintf(out, size, "%.0f 도", (double)s_as48_value[index]);
    else if (index == 6 || index == 7)
        snprintf(out, size, "%.0f C", (double)s_as48_value[index]);
    else if (index == 8)
        snprintf(out, size, "%.0f A", (double)s_as48_value[index]);
    else
        snprintf(out, size, "%.0f", (double)s_as48_value[index]);
}

static void as48_draw_row(int index)
{
    if (!as48_is_menu_item(index) || !s_as48_names[index]) return;
    const lv_color_t row_bg = lv_color_hex(0x263936);
    lv_obj_t *row = s_as48_names[index];
    char name[64], value[32];
    snprintf(name, sizeof(name), "%02d  %s", as48_menu_number(index),
             s_as48_names_all[index]);
    as48_format_value(index, value, sizeof(value));
    lv_canvas_fill_bg(row, row_bg, LV_OPA_COVER);
    ycb_hangul_draw(row, 16, 16, lv_color_hex(0xDCE8E3), row_bg,
                    name, 2, false);
    const int value_width = ycb_hangul_measure(value, 2);
    ycb_hangul_draw(row, std::max(570, 760 - value_width), 16,
                    lv_color_hex(0x9ED5B8), row_bg, value, 2, false);

    const lv_color_t minus_bg = lv_color_hex(0x526D64);
    const lv_color_t plus_bg = lv_color_hex(0x4E8974);
    canvas_fill_rect(row, 772, 6, 80, 52, minus_bg);
    canvas_fill_rect(row, 866, 6, 80, 52, plus_bg);
    if (as48_is_on_off(index)) {
        ycb_hangul_draw(row, 788, 16, lv_color_white(), minus_bg,
                        "OFF", 2, false);
        ycb_hangul_draw(row, 890, 16, lv_color_white(), plus_bg,
                        "ON", 2, false);
    } else {
        ycb_hangul_draw(row, 796, 16, lv_color_white(), minus_bg,
                        "-", 2, false);
        ycb_hangul_draw(row, 890, 16, lv_color_white(), plus_bg,
                        "+", 2, false);
    }
    lv_obj_invalidate(row);
}

static void as48_refresh()
{
    if (!s_as48_title) return;
    const lv_color_t overlay_bg = lv_color_hex(0x17211F);
    lv_canvas_fill_bg(s_as48_title, overlay_bg, LV_OPA_COVER);
    ycb_hangul_draw(s_as48_title, 0, 10, lv_color_hex(0xE8F1ED),
                    overlay_bg, "차량 설정", 2, false);
    lv_obj_invalidate(s_as48_title);
    for (int index = 1; index < kAs48SettingCount; ++index) {
        if (!as48_is_menu_item(index)) continue;
        as48_draw_row(index);
        vTaskDelay(1);
    }
}

static void as48_save_all()
{
    s_as48_value[0] = 48.0f;
    /* Keep the legacy roll threshold slot synchronized for compatibility. */
    s_as48_value[2] = s_as48_value[1];
    nvs_handle_t h;
    if (nvs_open("as48_cfg", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "values", s_as48_value, sizeof(s_as48_value));
    nvs_commit(h);
    nvs_close(h);
}

static void as48_event(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED &&
        lv_event_get_code(e) != LV_EVENT_LONG_PRESSED_REPEAT) return;
    const int action = (int)(intptr_t)lv_event_get_user_data(e);
    if (action == 12) {
        as48_save_all();
        lv_obj_add_flag(s_settings_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    } else if (action == 13) {
        std::memcpy(s_as48_value, s_as48_value_on_open,
                    sizeof(s_as48_value));
        as48_refresh();
        lv_obj_add_flag(s_settings_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    } else if (action >= 100 && action < 100 + kAs48SettingCount) {
        const int index = action - 100;
        s_as48_value[index] = fmaxf(s_as48_min[index], s_as48_value[index] - s_as48_step[index]);
    } else if (action >= 200 && action < 200 + kAs48SettingCount) {
        const int index = action - 200;
        s_as48_value[index] = fminf(s_as48_max[index], s_as48_value[index] + s_as48_step[index]);
    }
    as48_refresh();
}

static void as48_row_event(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED && code != LV_EVENT_LONG_PRESSED_REPEAT)
        return;
    const int index = (int)(intptr_t)lv_event_get_user_data(e);
    if (index < 0 || index >= kAs48SettingCount) return;
    lv_indev_t *indev = lv_event_get_indev(e);
    if (!indev) return;
    lv_point_t point;
    lv_area_t area;
    lv_indev_get_point(indev, &point);
    lv_obj_get_coords(lv_event_get_target_obj(e), &area);
    const int x = point.x - area.x1;
    if (x >= 772 && x < 852) {
        if (as48_is_on_off(index)) s_as48_value[index] = 0.0f;
        else s_as48_value[index] = fmaxf(s_as48_min[index],
            s_as48_value[index] - s_as48_step[index]);
    } else if (x >= 866 && x < 946) {
        if (as48_is_on_off(index)) s_as48_value[index] = 1.0f;
        else s_as48_value[index] = fminf(s_as48_max[index],
            s_as48_value[index] + s_as48_step[index]);
    }
    else
        return;
    as48_draw_row(index);
}

static lv_obj_t *as48_button(lv_obj_t *parent, int x, int y, int w, int h,
                             const char *text, int action, uint32_t color,
                             lv_event_cb_t event_cb = as48_event)
{
    lv_obj_t *button = lv_obj_create(parent);
    lv_obj_set_pos(button, x, y); lv_obj_set_size(button, w, h);
    lv_obj_set_style_radius(button, 12, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(color), 0);
    lv_obj_set_style_bg_color(button,
                              lv_color_darken(lv_color_hex(color), LV_OPA_20),
                              LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_width(button, 2, 0);
    lv_obj_set_style_border_color(button, lv_color_lighten(
                                  lv_color_hex(color), LV_OPA_50), 0);
    lv_obj_set_style_border_color(button, lv_color_white(),
                                  LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(button, 10, 0);
    lv_obj_set_style_shadow_offset_y(button, 3, 0);
    lv_obj_set_style_shadow_opa(button, LV_OPA_30, 0);
    lv_obj_set_style_shadow_color(button, lv_color_black(), 0);
    lv_obj_set_style_pad_all(button, 0, 0);
    lv_obj_set_style_clip_corner(button, true, 0);
    lv_obj_clear_flag(button, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(button, event_cb, LV_EVENT_ALL,
                        (void *)(intptr_t)action);
    lv_obj_t *lbl = lv_canvas_create(button);
    lv_obj_set_size(lbl, w - 8, h - 8);
    lv_obj_center(lbl);
    lv_obj_clear_flag(lbl, LV_OBJ_FLAG_CLICKABLE);
    void *label_buf = heap_caps_calloc((size_t)(w - 8) * (h - 8), 4,
                                       MALLOC_CAP_SPIRAM);
    if (label_buf) {
        lv_canvas_set_buffer(lbl, label_buf, w - 8, h - 8,
                             LV_COLOR_FORMAT_ARGB8888);
        const lv_color_t bg = lv_color_hex(color);
        lv_canvas_fill_bg(lbl, bg, LV_OPA_COVER);
        const int text_width = ycb_hangul_measure(text, 2);
        ycb_hangul_draw(lbl, std::max(0, (w - 8 - text_width) / 2),
                        std::max(0, (h - 8 - 32) / 2), lv_color_white(), bg,
                        text, 2, false);
    } else {
        lv_obj_delete(lbl);
    }
    return button;
}

static void create_as48_settings(lv_obj_t *screen)
{
    nvs_handle_t h;
    if (nvs_open("as48_cfg", NVS_READONLY, &h) == ESP_OK) {
        size_t size = sizeof(s_as48_value);
        nvs_get_blob(h, "values", s_as48_value, &size);
        nvs_close(h);
    }
    /* AS500 is a fixed 48 V vehicle; this is not a user setting. */
    s_as48_value[0] = 48.0f;
    s_settings_overlay = lv_obj_create(screen);
    lv_obj_set_pos(s_settings_overlay, 0, 0); lv_obj_set_size(s_settings_overlay, 1024, 600);
    lv_obj_set_style_bg_color(s_settings_overlay, lv_color_hex(0x17211F), 0);
    lv_obj_set_style_border_width(s_settings_overlay, 0, 0);
    lv_obj_set_style_pad_all(s_settings_overlay, 0, 0);
    lv_obj_clear_flag(s_settings_overlay, LV_OBJ_FLAG_SCROLLABLE);

    s_as48_title = lv_canvas_create(s_settings_overlay);
    lv_obj_set_pos(s_as48_title, 28, 14);
    lv_obj_set_size(s_as48_title, 540, 52);
    void *title_buf = heap_caps_calloc(540 * 52, 4, MALLOC_CAP_SPIRAM);
    if (title_buf)
        lv_canvas_set_buffer(s_as48_title, title_buf, 540, 52,
                             LV_COLOR_FORMAT_ARGB8888);
    else {
        lv_obj_delete(s_as48_title);
        s_as48_title = nullptr;
    }

    lv_obj_t *list = lv_obj_create(s_settings_overlay);
    lv_obj_set_pos(list, 16, 82);
    lv_obj_set_size(list, 992, 502);
    lv_obj_set_style_bg_color(list, lv_color_hex(0x17211F), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 8, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);

    int visible_row = 0;
    for (int index = 1; index < kAs48SettingCount; ++index) {
        if (!as48_is_menu_item(index)) continue;
        s_as48_names[index] = lv_canvas_create(list);
        lv_obj_set_pos(s_as48_names[index], 0, visible_row * 72);
        lv_obj_set_size(s_as48_names[index], 960, 64);
        void *row_buf = heap_caps_calloc(960 * 64, 4, MALLOC_CAP_SPIRAM);
        if (row_buf) {
            lv_canvas_set_buffer(s_as48_names[index], row_buf, 960, 64,
                                 LV_COLOR_FORMAT_ARGB8888);
            lv_obj_add_flag(s_as48_names[index], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_flag(s_as48_names[index], LV_OBJ_FLAG_EVENT_BUBBLE);
            lv_obj_add_event_cb(s_as48_names[index], as48_row_event,
                                LV_EVENT_ALL, (void *)(intptr_t)index);
        } else {
            lv_obj_delete(s_as48_names[index]);
            s_as48_names[index] = nullptr;
        }
        ++visible_row;
        vTaskDelay(1);
    }
    as48_button(s_settings_overlay, 590, 8, 270, 64,
                "저장하고 나가기", 12, 0x386F84);
    as48_button(s_settings_overlay, 874, 8, 130, 64,
                "나가기", 13, 0x8A554D);
    as48_refresh();
    lv_obj_add_flag(s_settings_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void tilt_settings_draw_row(int slot, int index, const char *name)
{
    if (slot < 0 || slot >= 2 || !s_tilt_setting_rows[slot]) return;
    lv_obj_t *row = s_tilt_setting_rows[slot];
    const lv_color_t row_bg = lv_color_hex(0x263936);
    char value[32];
    as48_format_value(index, value, sizeof(value));
    lv_canvas_fill_bg(row, row_bg, LV_OPA_COVER);
    ycb_hangul_draw(row, 20, 16, lv_color_hex(0xDCE8E3), row_bg,
                    name, 2, false);
    const int value_width = ycb_hangul_measure(value, 2);
    ycb_hangul_draw(row, std::max(520, 750 - value_width), 16,
                    lv_color_hex(0xFFFFFF), row_bg, value, 2, false);

    const bool on_off = index == 3;
    const lv_color_t left_bg = lv_color_hex(on_off ? 0x6A625C : 0x586D68);
    const lv_color_t right_bg = lv_color_hex(on_off ? 0x3E8061 : 0x386F84);
    canvas_fill_rect(row, 772, 4, 80, 56, left_bg);
    canvas_fill_rect(row, 866, 4, 80, 56, right_bg);
    ycb_hangul_draw(row, on_off ? 780 : 796, 16, lv_color_white(), left_bg,
                    on_off ? "OFF" : "-", 2, false);
    ycb_hangul_draw(row, on_off ? 882 : 890, 16, lv_color_white(), right_bg,
                    on_off ? "ON" : "+", 2, false);
    lv_obj_invalidate(row);
}

static void tilt_settings_refresh()
{
    if (s_tilt_settings_title) {
        const lv_color_t bg = lv_color_hex(0x17211F);
        lv_canvas_fill_bg(s_tilt_settings_title, bg, LV_OPA_COVER);
        ycb_hangul_draw(s_tilt_settings_title, 0, 10,
                        lv_color_hex(0xE8F1ED), bg,
                        "기울기 설정", 2, false);
        lv_obj_invalidate(s_tilt_settings_title);
    }
    tilt_settings_draw_row(0, 1, "기울기 경보 각도");
    tilt_settings_draw_row(1, 3, "기울기 경보 사용");
}

void AS500UI::createTiltSettingsOverlay(lv_obj_t *screen)
{
    s_tilt_settings_overlay = lv_obj_create(screen);
    lv_obj_set_pos(s_tilt_settings_overlay, 0, 0);
    lv_obj_set_size(s_tilt_settings_overlay, 1024, 600);
    lv_obj_set_style_bg_color(s_tilt_settings_overlay,
                              lv_color_hex(0x17211F), 0);
    lv_obj_set_style_border_width(s_tilt_settings_overlay, 0, 0);
    lv_obj_set_style_pad_all(s_tilt_settings_overlay, 0, 0);
    lv_obj_clear_flag(s_tilt_settings_overlay, LV_OBJ_FLAG_SCROLLABLE);

    s_tilt_settings_title = lv_canvas_create(s_tilt_settings_overlay);
    lv_obj_set_pos(s_tilt_settings_title, 28, 14);
    lv_obj_set_size(s_tilt_settings_title, 330, 52);
    void *title_buf = heap_caps_calloc(330 * 52, 4, MALLOC_CAP_SPIRAM);
    if (title_buf) {
        lv_canvas_set_buffer(s_tilt_settings_title, title_buf, 330, 52,
                             LV_COLOR_FORMAT_ARGB8888);
    } else {
        lv_obj_delete(s_tilt_settings_title);
        s_tilt_settings_title = nullptr;
    }

    as48_button(s_tilt_settings_overlay, 370, 8, 200, 64,
                "기울기 영점", 30, 0x786444, tiltSettingsEvent);
    as48_button(s_tilt_settings_overlay, 590, 8, 270, 64,
                "저장하고 나가기", 31, 0x386F84, tiltSettingsEvent);
    as48_button(s_tilt_settings_overlay, 874, 8, 130, 64,
                "나가기", 32, 0x8A554D, tiltSettingsEvent);

    const int indices[2] = {1, 3};
    for (int slot = 0; slot < 2; ++slot) {
        lv_obj_t *row = lv_canvas_create(s_tilt_settings_overlay);
        s_tilt_setting_rows[slot] = row;
        lv_obj_set_pos(row, 32, 126 + slot * 82);
        lv_obj_set_size(row, 960, 64);
        void *row_buf = heap_caps_calloc(960 * 64, 4, MALLOC_CAP_SPIRAM);
        if (row_buf) {
            lv_canvas_set_buffer(row, row_buf, 960, 64,
                                 LV_COLOR_FORMAT_ARGB8888);
            lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(row, tiltSettingRowEvent, LV_EVENT_ALL,
                                (void *)(intptr_t)indices[slot]);
        } else {
            lv_obj_delete(row);
            s_tilt_setting_rows[slot] = nullptr;
        }
    }
    tilt_settings_refresh();
    lv_obj_add_flag(s_tilt_settings_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void redrawLidarRangeCanvas()
{
    if (!s_lidar_range_canvas) return;
    lv_canvas_fill_bg(s_lidar_range_canvas, lv_color_hex(0x000000),
                      LV_OPA_TRANSP);
    /* Draw the horizontal-anchor front/rear ambiguity zone first. The canvas is
       the lowest radar layer, so vehicle, TAG and LiDAR data remain visible
       above it. Frame-buffer X/Y are reversed by the physical 180-degree LCD
       mounting: +forward is canvas-down and +vehicle-right is canvas-left. */
    constexpr int shadow_step_px = 8;
    constexpr float cm_per_px = 500.0f / kLidarFiveMeterRadius;
    for (int py = shadow_step_px / 2; py < 450; py += shadow_step_px) {
        for (int px = shadow_step_px / 2; px < 512; px += shadow_step_px) {
            const int dx = px - 256;
            const int dy = py - 225;
            if (dx * dx + dy * dy >
                kLidarFiveMeterRadius * kLidarFiveMeterRadius) continue;
            const float forward_cm = dy * cm_per_px;
            const float right_cm = -dx * cm_per_px;
            if (!UwbGeometry::ambiguityZone(forward_cm, right_cm,
                    UwbGeometry::kDw3000RangeErrorCm,
                    AS500UI::settingValue(26))) continue;
            for (int oy = -2; oy <= 2; ++oy) {
                for (int ox = -2; ox <= 2; ++ox) {
                    const int x = px + ox, y = py + oy;
                    if (x >= 0 && x < 512 && y >= 0 && y < 450)
                        lv_canvas_set_px(s_lidar_range_canvas, x, y,
                                         lv_color_hex(0xE3B341), LV_OPA_20);
                }
            }
        }
    }
    auto soft_circle = [&](int radius, uint32_t color, lv_opa_t opa) {
        for (int sample = 0; sample < 1440; ++sample) {
            const float rad = sample * 0.25f *
                              3.14159265358979323846f / 180.0f;
            for (int inset = 0; inset <= 1; ++inset) {
                const int cx = 256 + (int)lroundf(cosf(rad) * (radius - inset));
                const int cy = 225 + (int)lroundf(sinf(rad) * (radius - inset));
                if (cx >= 0 && cx < 512 && cy >= 0 && cy < 450)
                    lv_canvas_set_px(s_lidar_range_canvas, cx, cy,
                                     lv_color_hex(color), opa);
            }
        }
    };
    /* These are absolute physical references and never follow the adjustable
       maximum detection/plot distance. */
    soft_circle(kLidarFiveMeterRadius, 0x8B7ED8, LV_OPA_20);
    soft_circle(kLidarDisplayRadius, 0x5FA8A0, LV_OPA_30);
    lv_obj_invalidate(s_lidar_range_canvas);
}

void AS500UI::loadSettings()
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("sweeper_cfg", NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW("AS500UI", "NVS settings open failed: %s", esp_err_to_name(err));
        return;
    }
    for (int i = 0; i < kSettingCount; ++i) {
        char key[8];
        snprintf(key, sizeof(key), "cfg%d", i);
        size_t len = sizeof(float);
        float value = s_setting_data[i];
        if (nvs_get_blob(h, key, &value, &len) == ESP_OK &&
            len == sizeof(float) && value >= s_setting_min[i] && value <= s_setting_max[i]) {
            s_setting_data[i] = value;
        } else {
            nvs_set_blob(h, key, &s_setting_data[i], sizeof(float));
        }
    }
    // cfg16/cfg17 previously meant EWMA percentage and step gate. Migrate
    // them exactly once to Kalman measurement R and motion Q defaults.
    uint8_t kalman_schema = 0;
    if (nvs_get_u8(h, "uwb_kf_v", &kalman_schema) != ESP_OK || kalman_schema < 1) {
        s_setting_data[16] = 10.0f;
        s_setting_data[17] = 150.0f;
        nvs_set_blob(h, "cfg16", &s_setting_data[16], sizeof(float));
        nvs_set_blob(h, "cfg17", &s_setting_data[17], sizeof(float));
        nvs_set_u8(h, "uwb_kf_v", 1);
    }
    // cfg14 previously stored vehicle/anchor width. Geometry now uses the
    // fixed 80 cm installation; migrate this slot once to manual combo ratio.
    uint8_t manual_turn_schema = 0;
    if (nvs_get_u8(h, "man_turn_v", &manual_turn_schema) != ESP_OK ||
        manual_turn_schema < 1) {
        s_setting_data[14] = 50.0f;
        nvs_set_blob(h, "cfg14", &s_setting_data[14], sizeof(float));
        nvs_set_u8(h, "man_turn_v", 1);
    }
    // cfg15 previously selected UART simulation. USB simulation is detected
    // automatically; migrate this slot to the follow inner-wheel percentage.
    uint8_t follow_inner_schema = 0;
    if (nvs_get_u8(h, "fol_inner_v", &follow_inner_schema) != ESP_OK ||
        follow_inner_schema < 1) {
        s_setting_data[15] = 100.0f;
        nvs_set_blob(h, "cfg15", &s_setting_data[15], sizeof(float));
        nvs_set_u8(h, "fol_inner_v", 1);
    }
    nvs_commit(h);
    nvs_close(h);
}

float AS500UI::settingValue(int index)
{
    return (index >= 0 && index < kSettingCount) ? s_setting_data[index] : 0.0f;
}

float AS500UI::as48SettingValue(int index)
{
    if (index == 0) return 48.0f;
    return (index >= 0 && index < kAs48SettingCount) ? s_as48_value[index] : 0.0f;
}

uint8_t AS500UI::calibratedAnalogValue(int channel, uint8_t raw)
{
    if (channel < 0 || channel >= kAnalogChannelCount) return 0;
    const int offset = s_analog_calibration[channel][0];
    const int maximum = s_analog_calibration[channel][1];
    if (raw <= offset) return 0;
    if (raw >= maximum) return 255;
    return (uint8_t)(((int)raw - offset) * 255 / (maximum - offset));
}

static void save_setting_value(int index)
{
    nvs_handle_t h;
    if (index < 0 || index >= kSettingCount ||
        nvs_open("sweeper_cfg", NVS_READWRITE, &h) != ESP_OK) return;
    char key[8];
    snprintf(key, sizeof(key), "cfg%d", index);
    nvs_set_blob(h, key, &s_setting_data[index], sizeof(float));
    nvs_commit(h);
    nvs_close(h);
}

bool AS500UI::setFollowTurnRpm(float rpm)
{
    if (!isfinite(rpm) || rpm < 100 || rpm > 2500) return false;
    s_setting_data[25] = rpm;
    save_setting_value(25);
    return true;
}

bool AS500UI::setFollowSimulatorSettings(float max_speed_m_s,
                                            float stop_distance_mm,
                                            float max_estimate_distance_m,
                                            float near_speed_m_s,
                                            float decel_time_s,
                                            float accel_time_s,
                                            float turn_gain,
                                            float turn_pwm_percent)
{
    const float stop_distance_cm = stop_distance_mm * 0.1f;
    const float max_estimate_distance_cm = max_estimate_distance_m * 100.0f;
    if (max_speed_m_s < s_setting_min[2] || max_speed_m_s > s_setting_max[2] ||
        stop_distance_cm < s_setting_min[0] || stop_distance_cm > s_setting_max[0] ||
        max_estimate_distance_cm < s_setting_min[1] || max_estimate_distance_cm > s_setting_max[1] ||
        near_speed_m_s < s_setting_min[3] || near_speed_m_s > s_setting_max[3] ||
        decel_time_s < s_setting_min[4] || decel_time_s > s_setting_max[4] ||
        accel_time_s < s_setting_min[5] || accel_time_s > s_setting_max[5] ||
        turn_gain < s_setting_min[23] || turn_gain > s_setting_max[23] ||
        turn_pwm_percent < s_setting_min[24] ||
        turn_pwm_percent > s_setting_max[24]) return false;

    s_setting_data[2] = max_speed_m_s;
    s_setting_data[0] = stop_distance_cm;
    s_setting_data[1] = max_estimate_distance_cm;
    s_setting_data[3] = near_speed_m_s;
    s_setting_data[4] = decel_time_s;
    s_setting_data[5] = accel_time_s;
    s_setting_data[23] = turn_gain;
    s_setting_data[24] = turn_pwm_percent;
    save_setting_value(2);
    save_setting_value(0);
    save_setting_value(1);
    save_setting_value(3);
    save_setting_value(4);
    save_setting_value(5);
    save_setting_value(23);
    save_setting_value(24);
    return true;
}

static void refresh_setting_ui(int previous_selected = -1)
{
    const bool analog_page = s_settings_page == SettingsPage::Analog;
    const int *map = s_settings_page == SettingsPage::Control
        ? s_control_setting_map
        : analog_page ? s_analog_setting_map : s_sensor_setting_map;
    const char *const *names = s_settings_page == SettingsPage::Control
        ? s_control_setting_names
        : analog_page ? s_analog_setting_names : s_sensor_setting_names;
    for (int slot = 0; slot < kVisibleSettingCount; ++slot) {
        const int index = map[slot];
        if (s_setting_name_canvases[slot] && previous_selected < 0) {
            lv_canvas_fill_bg(s_setting_name_canvases[slot],
                              lv_color_hex(0x1E293B), LV_OPA_TRANSP);
            if (index >= 0)
                ycb_hangul_draw(s_setting_name_canvases[slot], 0, 3,
                                lv_color_hex(0xF8FAFC), lv_color_hex(0x1E293B),
                                names[slot], 2, true);
            lv_obj_invalidate(s_setting_name_canvases[slot]);
        }
        if (!s_setting_rows[slot] || !s_setting_values[slot]) continue;
        if (index >= 0)
            lv_obj_add_flag(s_setting_rows[slot], LV_OBJ_FLAG_CLICKABLE);
        else
            lv_obj_clear_flag(s_setting_rows[slot], LV_OBJ_FLAG_CLICKABLE);
        const bool selected = index >= 0 && slot == s_setting_selected;
        lv_obj_set_style_bg_color(s_setting_rows[slot],
            lv_color_hex(selected ? 0x426B9B : 0x1E293B), 0);
        lv_obj_set_style_border_color(s_setting_rows[slot],
            lv_color_hex(selected ? 0x93C5FD : (index >= 0 ? 0x334155 : 0x1E293B)), 0);
        if (index < 0 || (!analog_page && index >= kSettingCount) ||
            (analog_page && index >= kAnalogChannelCount * 2)) {
            lv_obj_add_flag(s_setting_values[slot], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(s_setting_values[slot], LV_OBJ_FLAG_HIDDEN);
        char value_text[40];
        if (analog_page) {
            const int channel = index / 2;
            const int part = index % 2;
            snprintf(value_text, sizeof(value_text), "%u   RAW %u",
                     s_analog_calibration[channel][part],
                     s_analog_raw[channel]);
        } else if (index == 1)
            snprintf(value_text, sizeof(value_text), "%.0f m", (double)(s_setting_data[index] / 100.0f));
        else if (index == 2 || index == 3)
            snprintf(value_text, sizeof(value_text), "%.1f m/s", (double)s_setting_data[index]);
        else if (index == 4 || index == 5 || index == 6 || index == 7)
            snprintf(value_text, sizeof(value_text), "%.1f s", (double)s_setting_data[index]);
        else if (index == 8)
            snprintf(value_text, sizeof(value_text), "%.1f V", (double)s_setting_data[index]);
        else if (index == 9)
            snprintf(value_text, sizeof(value_text), "%.0f ms", (double)s_setting_data[index]);
        else if (index <= 5 || index == 10 || index == 11 ||
                 index == 20)
            snprintf(value_text, sizeof(value_text), "%.0f cm", (double)s_setting_data[index]);
        else if (index == 12 || index == 13)
            snprintf(value_text, sizeof(value_text), "%.0f deg", (double)s_setting_data[index]);
        else if (index == 15)
            snprintf(value_text, sizeof(value_text), "%+.0f %%", (double)s_setting_data[index]);
        else if (index == 16)
            snprintf(value_text, sizeof(value_text), "%.0f cm", (double)s_setting_data[index]);
        else if (index == 17)
            snprintf(value_text, sizeof(value_text), "%.0f cm/s2", (double)s_setting_data[index]);
        else if (index == 14 || index == 21)
            snprintf(value_text, sizeof(value_text), "%.0f %%", (double)s_setting_data[index]);
        else if (index == 22)
            snprintf(value_text, sizeof(value_text), "%s",
                     s_setting_data[index] >= 0.5f ? "RESET" : "KEEP");
        else if (index == 18)
            snprintf(value_text, sizeof(value_text), "%.0f", (double)s_setting_data[index]);
        else if (index == 19)
            snprintf(value_text, sizeof(value_text), "%.0f ms", (double)s_setting_data[index]);
        else if (index == 23)
            snprintf(value_text, sizeof(value_text), "%.1f", (double)s_setting_data[index]);
        else if (index == 24)
            snprintf(value_text, sizeof(value_text), "%.0f %%", (double)s_setting_data[index]);
        else if (index == 26)
            snprintf(value_text, sizeof(value_text), "x%.0f", (double)s_setting_data[index]);
        else
            snprintf(value_text, sizeof(value_text), "%.0f", (double)s_setting_data[index]);
        lv_label_set_text(s_setting_values[slot], value_text);
    }
    if (s_settings_title_canvas && previous_selected < 0) {
        lv_canvas_fill_bg(s_settings_title_canvas, lv_color_hex(0x1E293B), LV_OPA_TRANSP);
        const char *title = s_settings_page == SettingsPage::Control
            ? "제어설정"
            : analog_page ? "ADC CALIBRATION" : "센서설정";
        int width = ycb_hangul_measure(title, 2);
        ycb_hangul_draw(s_settings_title_canvas, (300 - width) / 2, 4,
                        lv_color_hex(0x93C5FD), lv_color_hex(0x1E293B),
                        title, 2, true);
        lv_obj_invalidate(s_settings_title_canvas);
    }
}

static void savedPopupTimer(lv_timer_t *timer)
{
    (void)timer;
    if (s_settings_saved_popup)
        lv_obj_add_flag(s_settings_saved_popup, LV_OBJ_FLAG_HIDDEN);
    if (s_row_settings_overlay)
        lv_obj_add_flag(s_row_settings_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void settingControlEvent(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED && code != LV_EVENT_LONG_PRESSED_REPEAT) return;
    int action = (int)(intptr_t)lv_event_get_user_data(e);
    if (action == 4) { /* Cancel: discard staged values. */
        if (s_settings_page == SettingsPage::Analog) {
            memcpy(s_analog_calibration, s_analog_calibration_on_open,
                   sizeof(s_analog_calibration));
            refresh_setting_ui();
            if (s_row_settings_overlay)
                lv_obj_add_flag(s_row_settings_overlay, LV_OBJ_FLAG_HIDDEN);
            s_settings_page = SettingsPage::Control;
            return;
        }
        for (int i = 0; i < kSettingCount; ++i) s_setting_data[i] = s_setting_saved[i];
        refresh_setting_ui();
        if (s_row_settings_overlay)
            lv_obj_add_flag(s_row_settings_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (action == 5) { /* Save all values, then return to main. */
        if (s_settings_page == SettingsPage::Analog) {
            if (!save_analog_calibration()) return;
            memcpy(s_analog_calibration_on_open, s_analog_calibration,
                   sizeof(s_analog_calibration));
            if (s_row_settings_overlay)
                lv_obj_add_flag(s_row_settings_overlay, LV_OBJ_FLAG_HIDDEN);
            s_settings_page = SettingsPage::Control;
            return;
        }
        if (s_setting_data[22] >= 0.5f) {
            s_vacuum_runtime_reset_requested = true;
            s_setting_data[22] = 0;
        }
        for (int i = 0; i < kSettingCount; ++i) {
            save_setting_value(i);
            s_setting_saved[i] = s_setting_data[i];
        }
        nvs_handle_t vacuum_nvs;
        if (nvs_open("sweeper_cfg", NVS_READWRITE, &vacuum_nvs) == ESP_OK) {
            nvs_set_u8(vacuum_nvs, "vac_power",
                       (uint8_t)lroundf(s_setting_data[21]));
            nvs_commit(vacuum_nvs);
            nvs_close(vacuum_nvs);
        }
        if (s_settings_saved_popup) {
            lv_obj_clear_flag(s_settings_saved_popup, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(s_settings_saved_popup);
        }
        lv_timer_t *timer = lv_timer_create(savedPopupTimer, 2000, nullptr);
        lv_timer_set_repeat_count(timer, 1);
        return;
    }
    if (action == 6) { /* Stage factory defaults; Save persists them to NVS. */
        if (s_settings_page == SettingsPage::Analog) {
            for (int i = 0; i < kAnalogChannelCount; ++i) {
                s_analog_calibration[i][0] = 0;
                s_analog_calibration[i][1] = 255;
            }
            s_setting_selected = 0;
            refresh_setting_ui();
            return;
        }
        const int *map = s_settings_page == SettingsPage::Control
            ? s_control_setting_map : s_sensor_setting_map;
        for (int slot = 0; slot < kVisibleSettingCount; ++slot) {
            const int index = map[slot];
            if (index < 0 || index >= kSettingCount) continue;
            s_setting_data[index] = s_setting_defaults[index];
        }
        s_setting_selected = 0;
        refresh_setting_ui();
        return;
    }
    const int previous_selected = s_setting_selected;
    const int *map = s_settings_page == SettingsPage::Control
        ? s_control_setting_map
        : s_settings_page == SettingsPage::Analog
            ? s_analog_setting_map : s_sensor_setting_map;
    if (action >= 100 && action < 100 + kVisibleSettingCount) {
        const int slot = action - 100;
        if (map[slot] >= 0) {
            s_setting_selected = slot;
            refresh_setting_ui(previous_selected);
        }
        return;
    }
    const int index = map[s_setting_selected];
    if (s_settings_page == SettingsPage::Analog) {
        const int channel = index / 2;
        const int part = index % 2;
        int value = s_analog_calibration[channel][part];
        if (action == 2) --value;
        if (action == 3) ++value;
        if (part == 0)
            value = std::clamp(value, 0,
                               (int)s_analog_calibration[channel][1] - 1);
        else
            value = std::clamp(value,
                               (int)s_analog_calibration[channel][0] + 1, 255);
        s_analog_calibration[channel][part] = (uint8_t)value;
        refresh_setting_ui(previous_selected);
        return;
    }
    if (action == 2) s_setting_data[index] -= s_setting_step[index];
    if (action == 3) s_setting_data[index] += s_setting_step[index];
    if (s_setting_data[index] < s_setting_min[index]) s_setting_data[index] = s_setting_min[index];
    if (s_setting_data[index] > s_setting_max[index]) s_setting_data[index] = s_setting_max[index];
    /* Touch repeat updates only the old/new rows and current value. Redrawing all
       16 ARGB canvases on every press can starve the LVGL core and trip IDLE1. */
    refresh_setting_ui(previous_selected);
    if (index == 26) redrawLidarRangeCanvas();
}

static void lidarSettingsLongPressEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_LONG_PRESSED ||
        !s_row_settings_overlay) return;
    for (int i = 0; i < kSettingCount; ++i) s_setting_saved[i] = s_setting_data[i];
    s_settings_page = SettingsPage::Sensor;
    s_setting_selected = 0;
    refresh_setting_ui();
    lv_obj_clear_flag(s_row_settings_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_row_settings_overlay);
}

static void draw_button_korean(lv_obj_t *canvas, const char *text)
{
    if (!canvas) return;
    lv_canvas_fill_bg(canvas, lv_color_hex(0x000000), LV_OPA_TRANSP);
    int width = ycb_hangul_measure(text, 2);
    int x = (220 - width) / 2;
    if (x < 0) x = 0;
    ycb_hangul_draw(canvas, x, 4, lv_color_hex(0xFFFFFF),
                    lv_color_hex(0x000000), text, 2, true);
    lv_obj_invalidate(canvas);
}

void AS500UI::buttonEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    control_cmd_t cmd = (control_cmd_t)(uintptr_t)lv_event_get_user_data(e);
    if (cmd == CMD_STOP || cmd == CMD_ESTOP || cmd == CMD_FOLLOW_START || cmd == CMD_SETTINGS)
        s_manual_press.cancel();
    if (cmd == CMD_SETTINGS) s_pending_cmd = CMD_STOP;
    if (cmd == CMD_SETTINGS && s_settings_overlay == nullptr &&
        s_ui_instance && s_ui_instance->m_dash_can) {
        lv_label_set_text(s_ui_instance->m_dash_can, "TOUCH  OK");
        lv_obj_set_style_text_color(s_ui_instance->m_dash_can,
                                    lv_color_hex(0x9ED5B8), 0);
        ESP_LOGI("HMI_INPUT", "AS500 settings touch received");
        return;
    }
    if (cmd == CMD_SETTINGS && s_settings_overlay != nullptr) {
        if (lv_obj_has_flag(s_settings_overlay, LV_OBJ_FLAG_HIDDEN)) {
            std::memcpy(s_as48_value_on_open, s_as48_value,
                        sizeof(s_as48_value_on_open));
            as48_refresh();
            lv_obj_clear_flag(s_settings_overlay, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(s_settings_overlay);
        } else {
            std::memcpy(s_as48_value, s_as48_value_on_open,
                        sizeof(s_as48_value));
            as48_refresh();
            lv_obj_add_flag(s_settings_overlay, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }
    s_pending_cmd = cmd;
}

void AS500UI::auxiliaryButtonEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    const control_cmd_t cmd =
        (control_cmd_t)(uintptr_t)lv_event_get_user_data(e);
    if (cmd == CMD_HORN_ON) {
        if (code == LV_EVENT_PRESSED) s_pending_cmd = CMD_HORN_ON;
        else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
            s_pending_cmd = CMD_HORN_OFF;
        return;
    }
    if (code == LV_EVENT_CLICKED) s_pending_cmd = cmd;
}

void AS500UI::manualButtonEvent(lv_event_t *e)
{
    const auto code = lv_event_get_code(e);
    const auto cmd = (control_cmd_t)(uintptr_t)lv_event_get_user_data(e);
    const auto now = (unsigned long long)esp_timer_get_time();
    ManualPress::Action action{ManualPress::None, 0};
    if (code == LV_EVENT_PRESSED) {
        action = s_manual_press.begin((int)cmd, now);
    } else if (s_manual_press.direction == (int)cmd) {
        if (code == LV_EVENT_PRESSING) action = s_manual_press.update(now);
        else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
            action = s_manual_press.release(now, code == LV_EVENT_PRESS_LOST);
    }
    if (action.kind == ManualPress::Tap) {
        switch ((control_cmd_t)action.direction) {
        case CMD_FORWARD: s_pending_cmd = CMD_TAP_FORWARD; break;
        case CMD_REVERSE: s_pending_cmd = CMD_TAP_REVERSE; break;
        case CMD_LEFT: s_pending_cmd = CMD_TAP_LEFT; break;
        case CMD_RIGHT: s_pending_cmd = CMD_TAP_RIGHT; break;
        default: s_pending_cmd = CMD_STOP; break;
        }
    } else if (action.kind == ManualPress::Hold) {
        s_pending_cmd = (control_cmd_t)action.direction;
    } else if (action.kind == ManualPress::Stop) {
        s_pending_cmd = CMD_STOP;
    }
    if (action.kind != ManualPress::None)
        ESP_LOGI("HMI_INPUT", "gesture=%d direction=%d held_ms=%llu",
                 (int)action.kind, action.direction, (now - s_manual_press.since) / 1000);

}

static void canvas_fill_rect(lv_obj_t *canvas, int x, int y, int w, int h,
                             lv_color_t color)
{
    for (int py = y; py < y + h; ++py)
        for (int px = x; px < x + w; ++px)
            lv_canvas_set_px(canvas, px, py, color, LV_OPA_COVER);
}

static void draw_tilt_level(lv_obj_t *canvas, float degrees, bool valid,
                            bool vertical)
{
    if (!canvas) return;
    const lv_color_t bg = lv_color_hex(0xF8FAF8);
    const lv_color_t tube = lv_color_hex(0x667873);
    const lv_color_t liquid = lv_color_hex(0xDCEFE8);
    const lv_color_t bubble = lv_color_hex(valid ? 0xE9A23B : 0xAAB5B1);
    lv_canvas_fill_bg(canvas, bg, LV_OPA_COVER);

    auto disc = [&](int cx, int cy, int radius, lv_color_t color) {
        for (int y = -radius; y <= radius; ++y)
            for (int x = -radius; x <= radius; ++x)
                if (x * x + y * y <= radius * radius)
                    lv_canvas_set_px(canvas, cx + x, cy + y,
                                     color, LV_OPA_COVER);
    };

    const float angle = valid ? std::clamp(degrees, -30.0f, 30.0f) : 0.0f;
    if (vertical) {
        /* Vehicle front is the top. Positive/forward pitch moves the bubble
           toward the rear (bottom), opposite to the vehicle inclination. */
        canvas_fill_rect(canvas, 41, 3, 25, 39, tube);
        canvas_fill_rect(canvas, 45, 5, 17, 35, liquid);
        canvas_fill_rect(canvas, 39, 21, 29, 2, tube);
        const int bubble_y = 22 + (int)lroundf(angle * 12.0f / 30.0f);
        disc(53, bubble_y, 7, bubble);
    } else {
        /* Bubble moves opposite roll: left tilt -> bubble to the right. */
        canvas_fill_rect(canvas, 7, 10, 93, 25, tube);
        canvas_fill_rect(canvas, 9, 14, 89, 17, liquid);
        canvas_fill_rect(canvas, 52, 8, 2, 29, tube);
        const int bubble_x = 53 - (int)lroundf(angle * 35.0f / 30.0f);
        disc(bubble_x, 22, 7, bubble);
    }
    lv_obj_invalidate(canvas);
}

/* icon_type: 0=no canvas icon, 1=fuel, 2=spray liquid. */
static void draw_analog_level_gauge(lv_obj_t *canvas, int percent,
                                    int icon_type)
{
    if (!canvas) return;
    const lv_color_t bg = lv_color_hex(0xF4F6F4);
    const lv_color_t ink = lv_color_hex(0x29312F);
    const lv_color_t needle = lv_color_hex(0xE94B3C);
    lv_canvas_fill_bg(canvas, bg, LV_OPA_COVER);

    auto disc = [&](int cx, int cy, int radius, lv_color_t color) {
        for (int y = -radius; y <= radius; ++y)
            for (int x = -radius; x <= radius; ++x)
                if (x * x + y * y <= radius * radius)
                    lv_canvas_set_px(canvas, cx + x, cy + y,
                                     color, LV_OPA_COVER);
    };
    auto line = [&](int x0, int y0, int x1, int y1, int width,
                    lv_color_t color) {
        const int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        const int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int error = dx + dy;
        while (true) {
            disc(x0, y0, width / 2, color);
            if (x0 == x1 && y0 == y1) break;
            const int e2 = 2 * error;
            if (e2 >= dy) { error += dy; x0 += sx; }
            if (e2 <= dx) { error += dx; y0 += sy; }
        }
    };

    constexpr int pivot_x = 78;
    constexpr int pivot_y = 67;
    for (int i = 0; i <= 10; ++i) {
        const float tick_angle = (-120.0f - i * 12.0f) * 0.01745329252f;
        const int inner = (i % 2 == 0) ? 45 : 49;
        line(pivot_x + (int)lroundf(cosf(tick_angle) * inner),
             pivot_y + (int)lroundf(sinf(tick_angle) * inner),
             pivot_x + (int)lroundf(cosf(tick_angle) * 58),
             pivot_y + (int)lroundf(sinf(tick_angle) * 58),
             i % 2 == 0 ? 5 : 2, ink);
    }

    /* Until a real sensor value is supplied, keep the needle at the centre. */
    const float shown = percent < 0 ? 50.0f : (float)std::clamp(percent, 0, 100);
    const float needle_angle = (-240.0f + shown * 1.2f) * 0.01745329252f;
    line(pivot_x, pivot_y,
         pivot_x + (int)lroundf(cosf(needle_angle) * 52.0f),
         pivot_y + (int)lroundf(sinf(needle_angle) * 52.0f),
         5, needle);
    disc(pivot_x, pivot_y, 8, needle);

    if (icon_type == 1) {
        canvas_fill_rect(canvas, 110, 54, 17, 25, ink);
        canvas_fill_rect(canvas, 113, 57, 11, 8, bg);
        canvas_fill_rect(canvas, 108, 78, 22, 3, ink);
        line(127, 58, 134, 62, 2, ink);
        line(134, 62, 134, 75, 2, ink);
        line(134, 75, 131, 77, 2, ink);
    } else if (icon_type == 2) {
        /* Spray-liquid level: up/down arrows, waves and a level scale. */
        line(108, 61, 108, 49, 3, ink);
        line(108, 49, 103, 56, 3, ink);
        line(108, 49, 113, 56, 3, ink);
        line(120, 49, 120, 61, 3, ink);
        line(120, 61, 115, 54, 3, ink);
        line(120, 61, 125, 54, 3, ink);
        line(102, 70, 108, 67, 3, ink);
        line(108, 67, 114, 70, 3, ink);
        line(114, 70, 120, 67, 3, ink);
        line(120, 67, 126, 70, 3, ink);
        line(102, 80, 108, 77, 3, ink);
        line(108, 77, 114, 80, 3, ink);
        line(114, 80, 120, 77, 3, ink);
        line(120, 77, 126, 80, 3, ink);
        line(135, 48, 135, 84, 3, ink);
        for (int y = 52; y <= 82; y += 10)
            line(129, y, 135, y, 3, ink);
    }
    lv_obj_invalidate(canvas);
}

static void draw_dashboard_dial(lv_obj_t *canvas, int width, int height,
                                float value, float minimum, float maximum,
                                bool steering, float warning_max = -1.0f)
{
    if (!canvas) return;
    const lv_color_t bg = lv_color_hex(steering ? 0xFFFFFF : 0xF4F6F4);
    const lv_color_t ink = lv_color_hex(0x29312F);
    const lv_color_t red = lv_color_hex(0xE32626);
    lv_canvas_fill_bg(canvas, bg, LV_OPA_COVER);
    auto disc = [&](int cx, int cy, int radius, lv_color_t color) {
        for (int y = -radius; y <= radius; ++y)
            for (int x = -radius; x <= radius; ++x)
                if (x * x + y * y <= radius * radius)
                    lv_canvas_set_px(canvas, cx + x, cy + y, color,
                                     LV_OPA_COVER);
    };
    auto line = [&](int x0, int y0, int x1, int y1, int thickness,
                    lv_color_t color) {
        const int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        const int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int error = dx + dy;
        while (true) {
            disc(x0, y0, thickness / 2, color);
            if (x0 == x1 && y0 == y1) break;
            const int e2 = error * 2;
            if (e2 >= dy) { error += dy; x0 += sx; }
            if (e2 <= dx) { error += dx; y0 += sy; }
        }
    };
    const int cx = width / 2;
    const int cy = steering ? 112 : 83;
    const int radius = steering ? 76 : 52;
    for (int i = 0; i <= 10; ++i) {
        const float a = (-165.0f + i * 15.0f) * 0.01745329252f;
        const int inner = radius - (i % 2 == 0 ? 14 : 9);
        const float tick_value = minimum + (maximum - minimum) * i / 10.0f;
        const lv_color_t tick_color = (!steering && warning_max >= minimum &&
                                       tick_value <= warning_max) ? red : ink;
        line(cx + (int)lroundf(cosf(a) * inner),
             cy + (int)lroundf(sinf(a) * inner),
             cx + (int)lroundf(cosf(a) * radius),
             cy + (int)lroundf(sinf(a) * radius),
             i % 2 == 0 ? 4 : 2, tick_color);
    }
    const float normalized = maximum > minimum
        ? std::clamp((value - minimum) / (maximum - minimum), 0.0f, 1.0f)
        : 0.5f;
    const float a = (-165.0f + normalized * 150.0f) * 0.01745329252f;
    line(cx, cy,
         cx + (int)lroundf(cosf(a) * (radius - 18)),
         cy + (int)lroundf(sinf(a) * (radius - 18)), 5, red);
    disc(cx, cy, steering ? 7 : 6, red);
    lv_obj_invalidate(canvas);
}

static void draw_thermometer_icon(lv_obj_t *canvas)
{
    if (!canvas) return;
    const lv_color_t clear = lv_color_hex(0x000000);
    const lv_color_t red = lv_color_hex(0xFF2B2B);
    lv_canvas_fill_bg(canvas, clear, LV_OPA_TRANSP);
    auto disc = [&](int cx, int cy, int radius, lv_color_t color) {
        for (int y = -radius; y <= radius; ++y)
            for (int x = -radius; x <= radius; ++x)
                if (x * x + y * y <= radius * radius)
                    lv_canvas_set_px(canvas, cx + x, cy + y, color,
                                     LV_OPA_COVER);
    };
    /* Red outline, mercury column and bulb, matching the supplied icon. */
    for (int y = 4; y <= 22; ++y) {
        lv_canvas_set_px(canvas, 7, y, red, LV_OPA_COVER);
        lv_canvas_set_px(canvas, 8, y, red, LV_OPA_COVER);
        lv_canvas_set_px(canvas, 13, y, red, LV_OPA_COVER);
        lv_canvas_set_px(canvas, 14, y, red, LV_OPA_COVER);
    }
    disc(11, 4, 4, red);
    disc(11, 4, 2, clear);
    canvas_fill_rect(canvas, 10, 13, 3, 13, red);
    disc(11, 27, 7, red);
    disc(11, 27, 4, clear);
    disc(11, 27, 3, red);
    for (int i = 0; i < 4; ++i)
        canvas_fill_rect(canvas, 22, 7 + i * 6, 8, 2, red);
    lv_obj_invalidate(canvas);
}

static void draw_horn_icon(lv_obj_t *canvas)
{
    if (!canvas) return;
    const lv_color_t fg = lv_color_white();
    const lv_color_t clear = lv_color_hex(0x000000);
    lv_canvas_fill_bg(canvas, clear, LV_OPA_TRANSP);

    auto disc = [&](int cx, int cy, int radius, lv_color_t color,
                    lv_opa_t opa = LV_OPA_COVER) {
        for (int py = -radius; py <= radius; ++py)
            for (int px = -radius; px <= radius; ++px)
                if (px * px + py * py <= radius * radius)
                    lv_canvas_set_px(canvas, cx + px, cy + py, color, opa);
    };
    /* Rounded mouthpiece and straight horn tube. */
    disc(5, 29, 5, fg);
    canvas_fill_rect(canvas, 5, 24, 25, 11, fg);

    /* Smooth flared bell silhouette, matching the supplied horn image. */
    for (int x = 27; x <= 45; ++x) {
        const int spread = 5 + ((x - 27) * (x - 27)) / 32;
        canvas_fill_rect(canvas, x, 29 - spread, 1, spread * 2 + 1, fg);
    }
    canvas_fill_rect(canvas, 44, 12, 4, 35, fg);
    disc(46, 14, 3, fg);
    disc(46, 45, 3, fg);

    /* Complete lower oval grip. Fill the outer ellipse and cut out a full
       inner ellipse so both the upper and lower rounded edges stay visible. */
    constexpr int grip_cx = 23, grip_cy = 38;
    constexpr int outer_rx = 13, outer_ry = 10;
    constexpr int inner_rx = 9, inner_ry = 5;
    for (int y = grip_cy - outer_ry; y <= grip_cy + outer_ry; ++y) {
        for (int x = grip_cx - outer_rx; x <= grip_cx + outer_rx; ++x) {
            const int dx = x - grip_cx;
            const int dy = y - grip_cy;
            if (dx * dx * outer_ry * outer_ry +
                dy * dy * outer_rx * outer_rx <=
                outer_rx * outer_rx * outer_ry * outer_ry)
                lv_canvas_set_px(canvas, x, y, fg, LV_OPA_COVER);
        }
    }
    for (int y = grip_cy - inner_ry; y <= grip_cy + inner_ry; ++y) {
        for (int x = grip_cx - inner_rx; x <= grip_cx + inner_rx; ++x) {
            const int dx = x - grip_cx;
            const int dy = y - grip_cy;
            if (dx * dx * inner_ry * inner_ry +
                dy * dy * inner_rx * inner_rx <=
                inner_rx * inner_rx * inner_ry * inner_ry)
                lv_canvas_set_px(canvas, x, y, clear, LV_OPA_TRANSP);
        }
    }

    /* Three clean, rounded sound-wave arcs. */
    constexpr float pi = 3.14159265358979323846f;
    for (int radius : {8, 15, 22}) {
        for (int degree = -58; degree <= 58; degree += 2) {
            const float angle = (float)degree * pi / 180.0f;
            const int x = 46 + (int)lroundf(cosf(angle) * radius);
            const int y = 29 + (int)lroundf(sinf(angle) * radius);
            disc(x, y, 2, fg);
        }
    }
    lv_obj_invalidate(canvas);
}

/* Direct LVGL-canvas port of AS48 smLCD::draw7Number(). Segment dimensions,
   spacing, masks, and tapered line construction are intentionally identical. */
static unsigned draw7Number(lv_obj_t *canvas, long n, unsigned xLoc,
                            unsigned yLoc, char cS, lv_color_t fC,
                            lv_color_t bC, int8_t nD, uint8_t dash)
{
    unsigned num = (unsigned)labs(n), i, w, h, x, y, si = 0, j = 1, d = 0;
    unsigned S2 = 5 * cS;
    unsigned S3 = 2 * cS;
    unsigned S4 = 5 * cS;
    unsigned x1 = cS + 1;
    unsigned x2 = S3 + S2 + 1;
    unsigned y1 = yLoc + x1;
    unsigned y3 = yLoc + S3 + S4 + 1;
    unsigned seg[7][2] = {{x1, yLoc}, {x2, y1}, {x2, y3 + x1},
                          {x1, (2 * y3) - yLoc}, {0, y3 + x1},
                          {0, y1}, {x1, y3}};
    static const unsigned char nums[12] = {
        0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D,
        0x7D, 0x07, 0x7F, 0x67, 0x00, 0x40
    };
    unsigned char cnt = (unsigned char)abs(nD);
    if (cnt > 10) cnt = 10;
    if (cnt < 1) cnt = 1;
    d = S2 + (3 * S3) + 2 + cS;
    xLoc += (cnt - 1) * d;
    while (cnt > 0) {
        --cnt;
        if (num > 9) i = num % 10;
        else if (!cnt && n < 0) i = 11;
        else if (nD < 0 && !num) i = 10;
        else i = num;
        num /= 10;
        if (dash > 0) i = 11;
        for (j = 0; j < 7; ++j) {
            const lv_color_t col = (nums[i] & (1 << j)) ? fC : bC;
            if (j == 0 || j == 3 || j == 6) {
                w = S2;
                x = xLoc + seg[j][0] + cS;
                y = seg[j][1];
                const unsigned t = y + S3;
                h = y + S3 / 2;
                while (y < h) {
                    canvas_fill_rect(canvas, (int)x, (int)y, (int)w, 1, col);
                    ++y; --x; w += 2;
                }
                while (y < t) {
                    canvas_fill_rect(canvas, (int)x, (int)y, (int)w, 1, col);
                    ++y; ++x; w -= 2;
                }
            } else {
                w = S4;
                x = xLoc + seg[j][0];
                y = seg[j][1] + cS;
                const unsigned t = x + S3;
                h = x + S3 / 2;
                while (x < h) {
                    canvas_fill_rect(canvas, (int)x, (int)y, 1, (int)w, col);
                    ++x; --y; w += 2;
                }
                while (x < t) {
                    canvas_fill_rect(canvas, (int)x, (int)y, 1, (int)w, col);
                    ++x; ++y; w -= 2;
                }
            }
        }
        xLoc -= d;
        ++si;
    }
    return d * (unsigned)abs(nD);
}

static void set_tilt_label(lv_obj_t *canvas, bool valid, float degrees)
{
    if (!canvas) return;
    const lv_color_t bg = lv_color_hex(0xF8FAF8);
    const lv_color_t fg = lv_color_hex(0x20302C);
    lv_canvas_fill_bg(canvas, bg, LV_OPA_COVER);
    constexpr unsigned cs = 1;
    constexpr unsigned y = 1;

    if (!valid || !isfinite(degrees)) {
        draw7Number(canvas, 0, 29, y, cs, fg, bg, 1, 1);
        draw7Number(canvas, 0, 45, y, cs, fg, bg, 1, 1);
    } else {
        const int rounded = std::clamp((int)lroundf(degrees), -99, 99);
        const unsigned magnitude = (unsigned)abs(rounded);
        if (rounded < 0)
            draw7Number(canvas, 0, 21, y, cs, fg, bg, 1, 1);
        if (magnitude >= 10)
            draw7Number(canvas, magnitude / 10, 37, y, cs, fg, bg, 1, 0);
        draw7Number(canvas, magnitude % 10, 53, y, cs, fg, bg, 1, 0);

        /* Small seven-segment-style degree mark. */
        for (int py = 2; py <= 7; ++py)
            for (int px = 72; px <= 77; ++px)
                if (py == 2 || py == 7 || px == 72 || px == 77)
                    lv_canvas_set_px(canvas, px, py, fg, LV_OPA_COVER);
    }
    lv_obj_invalidate(canvas);
}

static void draw_speed_7segment(lv_obj_t *canvas, int speed_tenths, bool valid)
{
    if (!canvas) return;
    const lv_color_t bg = lv_color_hex(0xFFFFFF);
    const lv_color_t fg = lv_color_hex(0x20302C);
    lv_canvas_fill_bg(canvas, bg, LV_OPA_COVER);
    constexpr unsigned cs = 3;
    /* Three-digit group is centered in the full 229 px speed card. */
    constexpr unsigned startX = 59;
    constexpr unsigned y = 8;
    const unsigned digitWidth = 5 * cs + 3 * (2 * cs) + 2 + cs;
    if (!valid) {
        draw7Number(canvas, 0, startX, y, cs, fg, bg, 1, 1);
        draw7Number(canvas, 0, startX + digitWidth, y, cs, fg, bg, 1, 1);
        draw7Number(canvas, 0, startX + 2 * digitWidth + 2 * cs,
                    y, cs, fg, bg, 1, 1);
    } else {
        if (speed_tenths < 0) speed_tenths = 0;
        if (speed_tenths > 999) speed_tenths = 999;
        const int integer = speed_tenths / 10;
        draw7Number(canvas, integer >= 10 ? integer / 10 : 10,
                    startX, y, cs, integer >= 10 ? fg : bg, bg, 1, 0);
        const unsigned onesX = startX + digitWidth;
        draw7Number(canvas, integer % 10, onesX, y, cs, fg, bg, 1, 0);
        const unsigned dotCenterX = onesX + digitWidth - cs;
        const unsigned dotCenterY = y + 7 * cs - cs + 24;
        const int radius = cs + 1;
        for (int dy = -radius; dy <= radius; ++dy)
            for (int dx = -radius; dx <= radius; ++dx)
                if (dx * dx + dy * dy <= radius * radius)
                    canvas_fill_rect(canvas, (int)dotCenterX + dx,
                                     (int)dotCenterY + dy, 1, 1, fg);
        draw7Number(canvas, speed_tenths % 10,
                    onesX + digitWidth + 2 * cs, y, cs, fg, bg, 1, 0);
    }
    lv_obj_invalidate(canvas);
}

static void draw_distance_7segment(lv_obj_t *canvas, uint32_t distance_tenths)
{
    if (!canvas) return;
    const lv_color_t bg = lv_color_hex(0xFFFFFF);
    const lv_color_t fg = lv_color_hex(0x20302C);
    lv_canvas_fill_bg(canvas, bg, LV_OPA_COVER);
    distance_tenths %= 1000000u;
    constexpr unsigned cs = 1;
    constexpr unsigned digit_w = 14;
    for (int i = 0; i < 6; ++i) {
        uint32_t divisor = 1;
        for (int n = 0; n < 5 - i; ++n) divisor *= 10u;
        draw7Number(canvas, (distance_tenths / divisor) % 10u,
                    2 + i * digit_w, 1, cs, fg, bg, 1, 0);
    }
    canvas_fill_rect(canvas, 2 + 5 * digit_w - 3, 20, 3, 3, fg);
    lv_obj_invalidate(canvas);
}

bool AS500UI::setVehicleFNR(control_cmd_t direction)
{
    if (direction != CMD_NONE && direction != CMD_FORWARD &&
        direction != CMD_REVERSE) return false;
    const control_cmd_t previous = s_vehicle_fnr.exchange(direction);
    if (previous != direction && cruisePressed()) {
        s_pending_cmd = direction == CMD_FORWARD || direction == CMD_REVERSE
            ? direction : CMD_STOP;
    }
    return true;
}

void AS500UI::cruiseButtonEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    const bool was_pressed = cruisePressed();
    if (code == LV_EVENT_PRESSED) s_touch_cruise_pressed = true;
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
        s_touch_cruise_pressed = false;
    else return;
    const control_cmd_t command = cruise_transition_command(
        was_pressed, cruisePressed(), s_vehicle_fnr.load());
    if (command != CMD_NONE) s_pending_cmd = command;
}

void AS500UI::setInterfaceCruisePressed(bool pressed)
{
    const bool was_pressed = cruisePressed();
    s_interface_cruise_pressed = pressed;
    const control_cmd_t command = cruise_transition_command(
        was_pressed, cruisePressed(), s_vehicle_fnr.load());
    if (command != CMD_NONE) s_pending_cmd = command;
}

bool AS500UI::cruisePressed()
{
    return s_touch_cruise_pressed.load() ||
           s_interface_cruise_pressed.load();
}

void AS500UI::currentGaugeEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    AS500UI *ui = static_cast<AS500UI *>(lv_event_get_user_data(e));
    if (!ui) return;
    ui->m_dash_current_max_amps =
        ui->m_dash_current_max_amps == 100 ? 200 : 100;
    if (ui->m_dash_current_scale)
        lv_label_set_text_fmt(ui->m_dash_current_scale, "MAX %dA",
                              ui->m_dash_current_max_amps);
}

void AS500UI::tripResetEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    AS500UI *ui = static_cast<AS500UI *>(lv_event_get_user_data(e));
    if (!ui) return;
    ui->m_dash_trip_km = 0.0;
    draw_distance_7segment(ui->m_dash_trip, 0);
}

void AS500UI::tiltZeroEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_LONG_PRESSED) return;
    if (!s_tilt_settings_overlay) return;
    s_tilt_value_on_open[0] = s_as48_value[1];
    s_tilt_value_on_open[1] = s_as48_value[2];
    s_tilt_value_on_open[2] = s_as48_value[3];
    tilt_settings_refresh();
    lv_obj_clear_flag(s_tilt_settings_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_tilt_settings_overlay);
}

void AS500UI::tiltSettingsEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    AS500UI *ui = s_ui_instance;
    const int action = (int)(intptr_t)lv_event_get_user_data(e);
    if (action == 31) {
        s_as48_value[2] = s_as48_value[1];
        as48_save_all();
        lv_obj_add_flag(s_tilt_settings_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (action == 32) {
        s_as48_value[1] = s_tilt_value_on_open[0];
        s_as48_value[2] = s_tilt_value_on_open[1];
        s_as48_value[3] = s_tilt_value_on_open[2];
        tilt_settings_refresh();
        lv_obj_add_flag(s_tilt_settings_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (action != 30 || !ui || !ui->m_dash_imu_valid) return;

    /* Displayed angle = raw angle + saved offset. Subtracting the current
       displayed angle therefore stores the exact offset that makes it zero. */
    s_as48_value[4] = std::clamp(s_as48_value[4] - ui->m_dash_pitch_deg,
                                 s_as48_min[4], s_as48_max[4]);
    s_as48_value[5] = std::clamp(s_as48_value[5] - ui->m_dash_roll_deg,
                                 s_as48_min[5], s_as48_max[5]);
    /* Persist the zero offsets immediately without committing alarm edits
       that have not yet been accepted with Save and Exit. */
    const float pending_angle = s_as48_value[1];
    const float pending_legacy_angle = s_as48_value[2];
    const float pending_enabled = s_as48_value[3];
    s_as48_value[1] = s_tilt_value_on_open[0];
    s_as48_value[2] = s_tilt_value_on_open[1];
    s_as48_value[3] = s_tilt_value_on_open[2];
    as48_save_all();
    s_as48_value[1] = pending_angle;
    s_as48_value[2] = pending_legacy_angle;
    s_as48_value[3] = pending_enabled;
    ui->m_dash_pitch_deg = 0.0f;
    ui->m_dash_roll_deg = 0.0f;
    set_tilt_label(ui->m_dash_pitch, true, 0.0f);
    set_tilt_label(ui->m_dash_roll, true, 0.0f);
    draw_tilt_level(ui->m_dash_pitch_level, 0.0f, true, true);
    draw_tilt_level(ui->m_dash_roll_level, 0.0f, true, false);
    ESP_LOGI("AS500UI", "IMU zero saved: pitch_offset=%.1f roll_offset=%.1f",
             (double)s_as48_value[4], (double)s_as48_value[5]);
}

void AS500UI::tiltSettingRowEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED && code != LV_EVENT_LONG_PRESSED_REPEAT)
        return;
    const int index = (int)(intptr_t)lv_event_get_user_data(e);
    if (index != 1 && index != 3) return;
    lv_indev_t *indev = lv_event_get_indev(e);
    if (!indev) return;
    lv_point_t point;
    lv_area_t area;
    lv_indev_get_point(indev, &point);
    lv_obj_get_coords(lv_event_get_target_obj(e), &area);
    const int x = point.x - area.x1;
    if (x >= 772 && x < 852) {
        if (index == 3) s_as48_value[index] = 0.0f;
        else s_as48_value[index] = fmaxf(s_as48_min[index],
            s_as48_value[index] - s_as48_step[index]);
    } else if (x >= 866 && x < 946) {
        if (index == 3) s_as48_value[index] = 1.0f;
        else s_as48_value[index] = fminf(s_as48_max[index],
            s_as48_value[index] + s_as48_step[index]);
    } else {
        return;
    }
    s_as48_value[2] = s_as48_value[1];
    tilt_settings_refresh();
}

void AS500UI::refreshAnalogSettings()
{
    char text[24];
    for (int channel = 0; channel < kAnalogChannelCount; ++channel) {
        snprintf(text, sizeof(text), "%03u", m_analog_raw[channel]);
        update_analog_text_canvas(m_analog_raw_values[channel], text);
        snprintf(text, sizeof(text), "%03u / 255",
                 calibratedAnalogValue(channel, m_analog_raw[channel]));
        update_analog_text_canvas(m_analog_calibrated_values[channel], text);
        snprintf(text, sizeof(text), "%u", s_analog_calibration[channel][0]);
        update_analog_text_canvas(m_analog_setting_values[channel * 2], text);
        snprintf(text, sizeof(text), "%u", s_analog_calibration[channel][1]);
        update_analog_text_canvas(m_analog_setting_values[channel * 2 + 1],
                                  text);
    }
}

bool AS500UI::openAnalogSettings()
{
    if (s_vehicle_fnr.load() != CMD_NONE) {
        ESP_LOGW("AS500UI", "ADC calibration blocked: FNR is not neutral");
        return false;
    }
    AS500UI *ui = s_ui_instance;
    if (!ui) {
        ESP_LOGE("AS500UI", "ADC calibration unavailable: settings overlay missing");
        return false;
    }
    if (!ui->m_analog_settings) {
        ESP_LOGI("AS500UI", "ADC calibration overlay creation requested");
        ui->createAnalogSettingsOverlay(lv_scr_act());
    }
    if (!ui->m_analog_settings) {
        ESP_LOGE("AS500UI", "ADC calibration unavailable: overlay creation failed");
        return false;
    }
    update_analog_text_canvas(ui->m_analog_status, "");
    memcpy(s_analog_calibration_on_open, s_analog_calibration,
           sizeof(s_analog_calibration));
    ui->refreshAnalogSettings();
    lv_obj_clear_flag(ui->m_analog_settings, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(ui->m_analog_settings);
    ESP_LOGI("AS500UI", "ADC calibration menu opened");
    return true;
}

void AS500UI::analogSettingsTouchEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        if (s_analog_settings_long_press_timer) {
            lv_timer_delete(s_analog_settings_long_press_timer);
            s_analog_settings_long_press_timer = nullptr;
        }
        s_analog_settings_long_press_timer =
            lv_timer_create(analogSettingsLongPressTimer, 3000, nullptr);
        if (!s_analog_settings_long_press_timer) {
            ESP_LOGE("AS500UI", "Failed to create speed-limit settings timer");
        } else {
            lv_timer_set_repeat_count(s_analog_settings_long_press_timer, 1);
        }
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (s_analog_settings_long_press_timer) {
            lv_timer_delete(s_analog_settings_long_press_timer);
            s_analog_settings_long_press_timer = nullptr;
        }
    }
}

void AS500UI::analogSettingsLongPressTimer(lv_timer_t *timer)
{
    if (timer != s_analog_settings_long_press_timer) return;
    s_analog_settings_long_press_timer = nullptr;
    openAnalogSettings();
}

void AS500UI::analogSettingsEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    AS500UI *ui = static_cast<AS500UI *>(lv_event_get_user_data(e));
    if (!ui) return;
    if (code != LV_EVENT_CLICKED) return;

    const int action = (int)(intptr_t)lv_obj_get_user_data(
        lv_event_get_target_obj(e));
    if (action == 1) {
        if (!save_analog_calibration()) {
            update_analog_text_canvas(ui->m_analog_status, "NVS SAVE FAILED");
            return;
        }
        update_analog_text_canvas(ui->m_analog_status, "");
        lv_obj_add_flag(ui->m_analog_settings, LV_OBJ_FLAG_HIDDEN);
    } else if (action == 2) {
        memcpy(s_analog_calibration, s_analog_calibration_on_open,
               sizeof(s_analog_calibration));
        lv_obj_add_flag(ui->m_analog_settings, LV_OBJ_FLAG_HIDDEN);
    } else if (action == 3) {
        for (int i = 0; i < kAnalogChannelCount; ++i) {
            s_analog_calibration[i][0] = 0;
            s_analog_calibration[i][1] = 255;
        }
        update_analog_text_canvas(ui->m_analog_status, "");
        ui->refreshAnalogSettings();
    }
}

void AS500UI::analogSettingAdjustEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED && code != LV_EVENT_LONG_PRESSED_REPEAT)
        return;
    AS500UI *ui = s_ui_instance;
    const int action = (int)(intptr_t)lv_event_get_user_data(e) - 100;
    if (!ui || action < 0 || action >= kAnalogChannelCount * 4) return;
    const int channel = action / 4;
    const int part = (action % 4) / 2; /* 0=OFFSET, 1=MAX */
    const int delta = (action & 1) ? 1 : -1;
    int value = (int)s_analog_calibration[channel][part] + delta;
    if (part == 0)
        value = std::clamp(value, 0,
                           (int)s_analog_calibration[channel][1] - 1);
    else
        value = std::clamp(value,
                           (int)s_analog_calibration[channel][0] + 1, 255);
    s_analog_calibration[channel][part] = (uint8_t)value;
    update_analog_text_canvas(ui->m_analog_status, "");
    ui->refreshAnalogSettings();
}

void AS500UI::createAnalogSettingsOverlay(lv_obj_t *screen)
{
    lv_mem_monitor_t mem;
    lv_mem_monitor(&mem);
    ESP_LOGI("AS500UI", "LVGL heap before analog overlay: free=%lu largest=%lu used=%u%%",
             (unsigned long)mem.free_size,
             (unsigned long)mem.free_biggest_size, (unsigned)mem.used_pct);

    m_analog_settings = lv_obj_create(screen);
    lv_obj_set_pos(m_analog_settings, 0, 0);
    lv_obj_set_size(m_analog_settings, 1024, 600);
    lv_obj_set_style_bg_color(m_analog_settings, lv_color_hex(0x111A18), 0);
    lv_obj_set_style_border_width(m_analog_settings, 0, 0);
    lv_obj_set_style_pad_all(m_analog_settings, 24, 0);
    lv_obj_clear_flag(m_analog_settings, LV_OBJ_FLAG_SCROLLABLE);

    auto make_button = [&](lv_obj_t *parent, const char *value,
                           int x, int y, int w, int h, int text_x,
                           int action, lv_event_cb_t callback) {
        lv_obj_t *button = lv_obj_create(parent);
        lv_obj_add_flag(button, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(button, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(button, x, y);
        lv_obj_set_size(button, w, h);
        lv_obj_set_style_pad_all(button, 0, 0);
        lv_obj_set_style_bg_color(button, lv_color_hex(0x263832), 0);
        lv_obj_set_style_bg_color(
            button, lv_color_hex(0x287A67),
            LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(button, 1, 0);
        lv_obj_set_style_border_color(button, lv_color_hex(0x40564D), 0);
        lv_obj_set_style_radius(button, 8, 0);
        lv_obj_set_user_data(button, (void *)(intptr_t)action);
        lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED,
                            callback == analogSettingsEvent ? this
                                                            : (void *)(intptr_t)action);
        if (callback == analogSettingAdjustEvent) {
            lv_obj_add_event_cb(button, callback, LV_EVENT_LONG_PRESSED_REPEAT,
                                (void *)(intptr_t)action);
        }
        create_analog_text_canvas(button, text_x, (h - 32) / 2,
                                  w - text_x * 2, 32, value);
        return button;
    };

    create_analog_text_canvas(m_analog_settings, 12, 0, 976, 36,
                              "아날로그 입력 보정");
    create_analog_text_canvas(m_analog_settings, 12, 78, 180, 32, "채널");
    create_analog_text_canvas(m_analog_settings, 204, 78, 128, 32, "RAW");
    create_analog_text_canvas(m_analog_settings, 344, 78, 188, 32, "OFFSET");
    create_analog_text_canvas(m_analog_settings, 540, 78, 188, 32, "MAX");
    create_analog_text_canvas(m_analog_settings, 736, 78, 228, 32, "적용");
    ESP_LOGI("AS500UI", "Analog overlay header created");
    static const char *channel_names[kAnalogChannelCount] = {
        "스로틀 1", "스로틀 2", "속도제한", "VR 2"
    };
    for (int channel = 0; channel < kAnalogChannelCount; ++channel) {
        const int y = 128 + channel * 88;
        lv_obj_t *row = lv_obj_create(m_analog_settings);
        lv_obj_set_pos(row, 0, y);
        lv_obj_set_size(row, 976, 72);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x1A2823), 0);
        lv_obj_set_style_border_color(row, lv_color_hex(0x304039), 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_radius(row, 8, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        create_analog_text_canvas(row, 12, 20, 180, 32,
                                  channel_names[channel]);
        m_analog_raw_values[channel] = create_analog_text_canvas(
            row, 204, 20, 128, 32, "000");
        m_analog_calibrated_values[channel] = create_analog_text_canvas(
            row, 736, 20, 228, 32, "000 / 255");
        for (int part = 0; part < 2; ++part) {
            const int x = part == 0 ? 344 : 540;
            const int slot = channel * 2 + part;
            make_button(row, "-", x + 4, 9, 38, 54, 3,
                        100 + channel * 4 + part * 2,
                        analogSettingAdjustEvent);
            m_analog_setting_values[slot] = create_analog_text_canvas(
                row, x + 46, 20, 96, 32, "0");
            make_button(row, "+", x + 148, 9, 38, 54, 3,
                        101 + channel * 4 + part * 2,
                        analogSettingAdjustEvent);
        }
        lv_mem_monitor(&mem);
        ESP_LOGI("AS500UI", "Analog overlay channel %d created: LVGL free=%lu largest=%lu",
                         channel + 1, (unsigned long)mem.free_size,
                         (unsigned long)mem.free_biggest_size);
    }
    m_analog_status = create_analog_text_canvas(
        m_analog_settings, 12, 466, 952, 32, "");
    make_button(m_analog_settings, "DEFAULT", 304, 500, 150, 60, 3,
                3, analogSettingsEvent);
    make_button(m_analog_settings, "CANCEL", 464, 500, 150, 60, 3,
                2, analogSettingsEvent);
    make_button(m_analog_settings, "SAVE", 624, 500, 150, 60, 3,
                1, analogSettingsEvent);
    refreshAnalogSettings();
    lv_obj_add_flag(m_analog_settings, LV_OBJ_FLAG_HIDDEN);
    lv_mem_monitor(&mem);
    ESP_LOGI("AS500UI", "Analog overlay ready: LVGL free=%lu largest=%lu used=%u%%",
             (unsigned long)mem.free_size,
             (unsigned long)mem.free_biggest_size, (unsigned)mem.used_pct);
}

void AS500UI::vehicleDrawEvent(lv_event_t *e)
{
    AS500UI *ui = static_cast<AS500UI *>(lv_event_get_user_data(e));
    lv_layer_t *layer = lv_event_get_layer(e);
    if (!ui || !layer) return;
    lv_area_t coords;
    lv_obj_get_coords(lv_event_get_target_obj(e), &coords);
    const float a = (ui->m_vehicle_angle_tenths / 10.0f + 180.0f) * 0.01745329252f;
    const float sn = sinf(a), cs = cosf(a);
    /* Source artwork uses y=0..72. Re-center it around y=36 so the fixed
       radar origin (256,225) is the actual vehicle center. */
    auto point = [&](int x, int y) {
        const int centered_y = y - 36;
        return lv_point_precise_t{
            (lv_value_precise_t)(coords.x1 + 90 + lroundf(x * cs - centered_y * sn)),
            (lv_value_precise_t)(coords.y1 + 90 + lroundf(x * sn + centered_y * cs))};
    };
    auto triangle = [&](lv_point_precise_t p0, lv_point_precise_t p1,
                        lv_point_precise_t p2, uint32_t color) {
        lv_draw_triangle_dsc_t d;
        lv_draw_triangle_dsc_init(&d);
        d.p[0] = p0; d.p[1] = p1; d.p[2] = p2;
        d.color = lv_color_hex(color); d.opa = LV_OPA_COVER;
        lv_draw_triangle(layer, &d);
    };
    auto quad = [&](int x1, int y1, int x2, int y2, uint32_t color) {
        const auto a0 = point(x1, y1), a1 = point(x2, y1);
        const auto a2 = point(x2, y2), a3 = point(x1, y2);
        triangle(a0, a1, a2, color); triangle(a0, a2, a3, color);
    };
    auto stroke = [&](int x1, int y1, int x2, int y2) {
        lv_draw_line_dsc_t d;
        lv_draw_line_dsc_init(&d);
        d.p1 = point(x1, y1); d.p2 = point(x2, y2);
        d.color = lv_color_hex(0x2F9ED6);
        d.opa = LV_OPA_COVER; d.width = 2; d.round_start = 1; d.round_end = 1;
        lv_draw_line(layer, &d);
    };
    /* Vehicle and anchor width are the fixed 80 cm installation. */
    int half_width = (int)lroundf(UwbGeometry::kAnchorBaselineCm * 0.5f);
    if (half_width < 20) half_width = 20;
    if (half_width > 50) half_width = 50;
    int body_half = half_width - 14;
    if (body_half < 12) body_half = 12;

    /* Dark tyres have no white outline. The chassis is centered on the
       common vehicle/TAG/LiDAR display origin. */
    quad(-half_width, 10, -body_half, 32, 0x8B949E);
    quad(body_half, 10, half_width, 32, 0x8B949E);
    quad(-half_width, 45, -body_half, 68, 0x8B949E);
    quad(body_half, 45, half_width, 68, 0x8B949E);

    /* A bright arrow on every tyre shows its actual rotation direction.
       Positive motor output points toward the vehicle nose (-Y); negative
       output points toward the rear (+Y). Opposing arrows make pivot turns
       immediately recognizable. */
    auto wheel_arrow = [&](int x, int y, int direction) {
        if (direction == 0) return;
        const int tip_y = y + (direction > 0 ? -8 : 8);
        const int base_y = y + (direction > 0 ? 6 : -6);
        triangle(point(x, tip_y), point(x - 5, base_y), point(x + 5, base_y),
                 0xFFFFFF);
        const int inner_tip_y = y + (direction > 0 ? -6 : 6);
        const int inner_base_y = y + (direction > 0 ? 4 : -4);
        triangle(point(x, inner_tip_y), point(x - 3, inner_base_y),
                 point(x + 3, inner_base_y), 0xFF3B30);
    };
    const int left_x = -(half_width + body_half) / 2;
    const int right_x = (half_width + body_half) / 2;
    wheel_arrow(left_x, 21, ui->m_left_wheel_direction);
    wheel_arrow(left_x, 56, ui->m_left_wheel_direction);
    wheel_arrow(right_x, 21, ui->m_right_wheel_direction);
    wheel_arrow(right_x, 56, ui->m_right_wheel_direction);

    quad(-body_half, 0, body_half, 72, ui->m_vehicle_color);
    quad(-body_half + 6, 10, body_half - 6, 30, 0x1B2730);
    triangle(point(-body_half, 0), point(body_half, 0), point(0, -8),
             ui->m_vehicle_color);
    /* EZEV branding on the rear deck, drawn as geometry so it rotates with
       the complete vehicle instead of remaining screen-aligned. */
    const int ty = 51;
    stroke(-15, ty, -15, ty + 8); stroke(-15, ty, -11, ty);
    stroke(-15, ty + 4, -11, ty + 4); stroke(-15, ty + 8, -11, ty + 8);
    stroke(-8, ty, -2, ty); stroke(-2, ty, -8, ty + 8);
    stroke(-8, ty + 8, -2, ty + 8);
    stroke(1, ty, 1, ty + 8); stroke(1, ty, 5, ty);
    stroke(1, ty + 4, 5, ty + 4); stroke(1, ty + 8, 5, ty + 8);
    stroke(8, ty, 11, ty + 8); stroke(14, ty, 11, ty + 8);

    /* Show the exact UWB anchor coordinates used by uwb_geometry.h.
       Vehicle coordinates are +forward toward y=0 and +right toward +x:
       The vehicle reference is the rear anchor midpoint:
       A0 = rear-left (0, -width/2), A1 = rear-right (0, +width/2). */
    auto anchor_marker = [&](float forward_cm, float right_cm,
                             const char *name, uint32_t color) {
        const auto p = point((int)lroundf(right_cm),
                             36 - (int)lroundf(forward_cm));
        lv_draw_rect_dsc_t marker;
        lv_draw_rect_dsc_init(&marker);
        marker.bg_color = lv_color_hex(color);
        marker.border_color = lv_color_hex(0xFFFFFF);
        marker.border_width = 1;
        marker.radius = LV_RADIUS_CIRCLE;
        lv_area_t marker_area = {(int32_t)p.x - 4, (int32_t)p.y - 4,
                                 (int32_t)p.x + 4, (int32_t)p.y + 4};
        lv_draw_rect(layer, &marker, &marker_area);

        lv_draw_label_dsc_t text;
        lv_draw_label_dsc_init(&text);
        text.text = name;
        text.font = &lv_font_montserrat_12;
        text.color = lv_color_hex(color);
        text.align = LV_TEXT_ALIGN_CENTER;
        lv_area_t text_area = {(int32_t)p.x - 12, (int32_t)p.y - 20,
                               (int32_t)p.x + 12, (int32_t)p.y - 6};
        lv_draw_label(layer, &text, &text_area);
    };
    const float anchor_width=UwbGeometry::kAnchorBaselineCm;
    anchor_marker(UwbGeometry::kAnchorForwardCm,
                  -anchor_width * 0.5f, "A0", 0x59E1D0);
    anchor_marker(UwbGeometry::kAnchorForwardCm,
                  anchor_width * 0.5f, "A1", 0xAFA4EA);

    /* LiDAR display origin is deliberately placed at the vehicle center. */
    lv_draw_rect_dsc_t lidar;
    lv_draw_rect_dsc_init(&lidar);
    lidar.bg_color = lv_color_hex(0xE3B341);
    lidar.border_color = lv_color_hex(0xFFF2A8);
    lidar.border_width = 2; lidar.radius = LV_RADIUS_CIRCLE;
    const auto lc = point(0, 36);
    lv_area_t la = {(int32_t)lc.x - 7, (int32_t)lc.y - 7,
                    (int32_t)lc.x + 7, (int32_t)lc.y + 7};
    lv_draw_rect(layer, &lidar, &la);
}

void AS500UI::createVacuumSettingsOverlay()
{
    if (m_vacuum_settings) return;
    auto box = [](lv_obj_t *parent, int x, int y, int w, int h,
                  uint32_t bg, uint32_t border, int radius) {
        lv_obj_t *o = lv_obj_create(parent);
        lv_obj_set_pos(o, x, y); lv_obj_set_size(o, w, h);
        lv_obj_set_style_bg_color(o, lv_color_hex(bg), 0);
        lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(o, lv_color_hex(border), 0);
        lv_obj_set_style_border_width(o, 1, 0);
        lv_obj_set_style_radius(o, radius, 0);
        lv_obj_set_style_pad_all(o, 0, 0);
        lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
        return o;
    };
    auto text = [](lv_obj_t *parent, const char *value, int x, int y,
                   int w, int h, const lv_font_t *font, uint32_t color,
                   lv_text_align_t align = LV_TEXT_ALIGN_CENTER) {
        lv_obj_t *l = lv_label_create(parent);
        lv_obj_set_pos(l, x, y); lv_obj_set_size(l, w, h);
        lv_label_set_text(l, value);
        lv_obj_set_style_text_font(l, font, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
        lv_obj_set_style_text_align(l, align, 0);
        return l;
    };
    m_vacuum_settings = box(lv_scr_act(), 0, 0, 1024, 600,
                            0x0D1117, 0x30363D, 0);
    text(m_vacuum_settings, "CLEANER SETTINGS", 262, 30, 500, 42,
         &lv_font_montserrat_28, 0x58A6FF);
    text(m_vacuum_settings, "CLEANING OUTPUT", 142, 130, 300, 32,
         &lv_font_montserrat_20, 0xFFFFFF, LV_TEXT_ALIGN_LEFT);
    text(m_vacuum_settings, "0 - 100%  /  STEP 1%", 142, 172, 300, 24,
         &lv_font_montserrat_14, 0x8FA6B5, LV_TEXT_ALIGN_LEFT);
    m_vacuum_output_value = text(m_vacuum_settings, "80%", 672, 142, 190, 50,
                                 &lv_font_montserrat_32, 0x55B7FF,
                                 LV_TEXT_ALIGN_RIGHT);
    text(m_vacuum_settings, "RUN TIME", 142, 270, 220, 30,
         &lv_font_montserrat_20, 0xFFFFFF, LV_TEXT_ALIGN_LEFT);
    m_vacuum_runtime_value = text(m_vacuum_settings, "00:00:00", 642, 268,
                                  220, 40, &lv_font_montserrat_28, 0xFFFFFF,
                                  LV_TEXT_ALIGN_RIGHT);
    auto button = [&](int x, int y, int w, int h, const char *caption,
                      int action, uint32_t color, bool repeat = false) {
        lv_obj_t *b = box(m_vacuum_settings, x, y, w, h, color, 0x0D1117, 6);
        lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(b, vacuumSettingsEvent, LV_EVENT_CLICKED,
                            (void *)(intptr_t)action);
        if (repeat)
            lv_obj_add_event_cb(b, vacuumSettingsEvent,
                                LV_EVENT_LONG_PRESSED_REPEAT,
                                (void *)(intptr_t)action);
        if (action == 5) {
            lv_obj_t *cv = lv_canvas_create(b);
            lv_obj_set_size(cv, w - 20, 40);
            lv_obj_center(cv);
            void *buf = heap_caps_calloc((w - 20) * 40, 4, MALLOC_CAP_SPIRAM);
            if (buf) {
                lv_canvas_set_buffer(cv, buf, w - 20, 40,
                                     LV_COLOR_FORMAT_ARGB8888);
                lv_canvas_fill_bg(cv, lv_color_hex(color), LV_OPA_TRANSP);
                const int width = ycb_hangul_measure(caption, 2);
                ycb_hangul_draw(cv, (w - 20 - width) / 2, 4,
                                lv_color_hex(0xFFFFFF), lv_color_hex(color),
                                caption, 2, true);
            }
        } else {
            text(b, caption, 4, (h - 28) / 2, w - 8, 28,
                 &lv_font_montserrat_18, 0xFFFFFF);
        }
    };
    button(112, 370, 190, 70, "- 1%", 0, 0x6E40C9, true);
    button(312, 370, 190, 70, "+ 1%", 1, 0x6E40C9, true);
    button(512, 370, 400, 70, "RUN TIME CLEAR", 2, 0x9E6A03);
    button(12, 500, 320, 100, "EXIT", 3, 0x57606A);
    button(332, 500, 360, 100, "SAVE & EXIT", 4, 0x238636);
    button(692, 500, 320, 100, "공장초기화", 5, 0xB62324);
    Buzzer::instance().attachToTree(m_vacuum_settings);
}

void AS500UI::vacuumButtonEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        s_pending_cmd = CMD_VACUUM_TOGGLE;
    }
}

void AS500UI::vacuumSettingsEvent(lv_event_t *e)
{
    if (!s_ui_instance) return;
    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED && code != LV_EVENT_LONG_PRESSED_REPEAT) return;
    AS500UI *ui = s_ui_instance;
    const int action = (int)(intptr_t)lv_event_get_user_data(e);
    if (action == 0 && ui->m_clean_output_edit > 0) --ui->m_clean_output_edit;
    if (action == 1 && ui->m_clean_output_edit < 100) ++ui->m_clean_output_edit;
    if (action == 2) ui->m_clean_runtime_ms = 0;
    if (action == 3) { /* Exit: keep staged values, do not persist. */
        lv_obj_delete_async(ui->m_vacuum_settings);
        ui->m_vacuum_settings = nullptr;
        ui->m_vacuum_output_value = nullptr;
        ui->m_vacuum_runtime_value = nullptr;
        return;
    }
    if (action == 4) { /* Save & Exit. */
        ui->m_clean_output_percent = ui->m_clean_output_edit;
        nvs_handle_t h;
        if (nvs_open("sweeper_cfg", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_u8(h, "vac_power", ui->m_clean_output_percent);
            nvs_set_u64(h, "vac_run_min", ui->m_clean_runtime_ms / 60000);
            nvs_commit(h);
            nvs_close(h);
        }
        ui->m_last_saved_runtime_min = ui->m_clean_runtime_ms / 60000;
        lv_obj_delete_async(ui->m_vacuum_settings);
        ui->m_vacuum_settings = nullptr;
        ui->m_vacuum_output_value = nullptr;
        ui->m_vacuum_runtime_value = nullptr;
        return;
    }
    if (action == 5) { /* Restore cleaner settings to factory defaults. */
        ui->m_clean_output_percent = 80;
        ui->m_clean_output_edit = 80;
        ui->m_clean_runtime_ms = 0;
        ui->m_last_saved_runtime_min = 0;
        nvs_handle_t h;
        if (nvs_open("sweeper_cfg", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_u8(h, "vac_power", 80);
            nvs_set_u64(h, "vac_run_min", 0);
            nvs_commit(h);
            nvs_close(h);
        }
        lv_obj_delete_async(ui->m_vacuum_settings);
        ui->m_vacuum_settings = nullptr;
        ui->m_vacuum_output_value = nullptr;
        ui->m_vacuum_runtime_value = nullptr;
        return;
    }
    if (ui->m_vacuum_output_value)
        lv_label_set_text_fmt(ui->m_vacuum_output_value, "%u%%",
                              ui->m_clean_output_edit);
    if (ui->m_vacuum_runtime_value) {
        const uint64_t seconds = ui->m_clean_runtime_ms / 1000;
        lv_label_set_text_fmt(ui->m_vacuum_runtime_value, "%02llu:%02llu:%02llu",
            (unsigned long long)(seconds / 3600),
            (unsigned long long)((seconds / 60) % 60),
            (unsigned long long)(seconds % 60));
    }
}

extern "C" bool as500_touch_points(const uint16_t *x, const uint16_t *y, uint8_t count) {
    return AS500UI::touchPoints(x, y, count);
}

bool AS500UI::touchPoints(const uint16_t *x, const uint16_t *y, uint8_t count)
{
    auto hit = [](lv_obj_t *obj, int px, int py) {
        if (!obj || !lv_obj_is_valid(obj) || !lv_obj_is_visible(obj)) return false;
        lv_area_t area;
        lv_obj_get_coords(obj, &area);
        return px >= area.x1 && px <= area.x2 && py >= area.y1 && py <= area.y2;
    };
    bool stop = false, estop = false, cruise = false;
    if (count && !s_physical_touch_down) {
        s_physical_touch_down = true;
        ESP_LOGI("AS500UI", "TOUCH DOWN count=%u x=%u y=%u",
                 (unsigned)count, (unsigned)x[0], (unsigned)y[0]);
    } else if (!count && s_physical_touch_down) {
        s_physical_touch_down = false;
        ESP_LOGI("AS500UI", "TOUCH UP");
    }
    const bool vehicle_settings_open = s_settings_overlay &&
        !lv_obj_has_flag(s_settings_overlay, LV_OBJ_FLAG_HIDDEN);
    const bool row_settings_open = s_row_settings_overlay &&
        !lv_obj_has_flag(s_row_settings_overlay, LV_OBJ_FLAG_HIDDEN);
    const bool analog_settings_open = s_ui_instance &&
        s_ui_instance->m_analog_settings &&
        !lv_obj_has_flag(s_ui_instance->m_analog_settings,
                         LV_OBJ_FLAG_HIDDEN);
    const bool tilt_settings_open = s_tilt_settings_overlay &&
        !lv_obj_has_flag(s_tilt_settings_overlay, LV_OBJ_FLAG_HIDDEN);
    const bool overlay = vehicle_settings_open || row_settings_open ||
                         analog_settings_open ||
                         tilt_settings_open;
    if (!count) s_touch_release_required = false;
    for (unsigned i = 0; i < count && !overlay; ++i) {
        stop |= hit(s_touch_stop, x[i], y[i]);
        estop |= hit(s_touch_estop, x[i], y[i]);
        cruise |= hit(s_touch_cruise, x[i], y[i]);
    }
    const bool consumed = stop || estop || cruise ||
                          s_touch_cruise_pressed.load() ||
                          s_touch_release_required;
    const bool cruise_was_pressed = cruisePressed();
    s_touch_cruise_pressed = cruise;
    const control_cmd_t cruise_command = cruise_transition_command(
        cruise_was_pressed, cruisePressed(), s_vehicle_fnr.load());
    if (cruise_command != CMD_NONE) s_pending_cmd = cruise_command;
    if (stop || estop) {
        s_touch_release_required = true;
        s_pending_cmd = estop ? CMD_ESTOP : CMD_STOP;
    }
    return consumed;
}

control_cmd_t AS500UI::takeCommand()
{
    // One operation: a concurrent RELEASE/STOP must remain pending for the
    // next iteration, rather than being erased by a separate clear.
    return s_pending_cmd.exchange(CMD_NONE);
}

/* ═══════════════════════════════════════════════════════════
 *  상태 → 표현 변환
 * ═══════════════════════════════════════════════════════════ */
lv_color_t AS500UI::stateColor(robot_state_t s)
{
    switch (s) {
    case S_FOLLOW:                                   return lv_color_hex(0x2EA043);
    case S_ALIGN: case S_TURN_L: case S_TURN_R:
    case S_MAN_FWD: case S_MAN_REV:
    case S_MAN_LEFT: case S_MAN_RIGHT:                return lv_color_hex(0x58A6FF);
    case S_NEAR:                                     return lv_color_hex(0xE3B341);
    case S_ESTOP: case S_NOSIG:                      return lv_color_hex(0xDA3633);
    default:                                         return lv_color_hex(0x8B949E);
    }
}

const char *AS500UI::stateText(robot_state_t s)
{
    static const char *t[] = {
        "  IDLE  ", " FOLLOW ", "  ALIGN ",
        "  NEAR  ", " TURN L ", " TURN R ",
        " E-STOP ", "NO SIGNAL", " MAN FWD ",
        " MAN REV ", " MAN LEFT", "MAN RIGHT"
    };
    return ((unsigned)s < 12) ? t[s] : "?";
}

const char *AS500UI::stateSym(robot_state_t s)
{
    switch (s) {
    case S_FOLLOW:              return LV_SYMBOL_UP;
    case S_NEAR:                return LV_SYMBOL_STOP;
    case S_ALIGN:               return LV_SYMBOL_REFRESH;
    case S_TURN_L:              return LV_SYMBOL_LEFT;
    case S_TURN_R:              return LV_SYMBOL_RIGHT;
    case S_ESTOP: case S_NOSIG: return LV_SYMBOL_WARNING;
    case S_MAN_FWD:              return LV_SYMBOL_UP;
    case S_MAN_REV:              return LV_SYMBOL_DOWN;
    case S_MAN_LEFT:             return LV_SYMBOL_LEFT;
    case S_MAN_RIGHT:            return LV_SYMBOL_RIGHT;
    default:                    return LV_SYMBOL_PAUSE;
    }
}

/* ═══════════════════════════════════════════════════════════
 *  내부 헬퍼 : arc 게이지 생성
 * ═══════════════════════════════════════════════════════════ */
static lv_obj_t *make_arc(lv_obj_t *parent, lv_coord_t y_offset)
{
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_set_size(arc, 140, 140);
    lv_obj_align(arc, LV_ALIGN_TOP_MID, 0, y_offset);
    lv_arc_set_range(arc, 0, 300);
    lv_arc_set_value(arc, 0);
    lv_arc_set_bg_angles(arc, 135, 45);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x21262D), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 14,                     LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x2EA043), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 14,                     LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(arc,  LV_OPA_TRANSP,            LV_PART_KNOB);
    lv_obj_set_style_pad_all(arc, 0,                        LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    return arc;
}

/* 얇은 수평 구분선 */
[[maybe_unused]] static void make_divider(lv_obj_t *parent, lv_coord_t y, lv_coord_t w)
{
    lv_obj_t *d = lv_obj_create(parent);
    lv_obj_set_pos(d, (DISP_W - w) / 2, y);   /* 개략 위치; 패널 기준이면 조정 */
    lv_obj_set_size(d, w, 1);
    lv_obj_set_style_bg_color(d, lv_color_hex(0x30363D), LV_PART_MAIN);
    lv_obj_set_style_border_width(d, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(d, 0, LV_PART_MAIN);
}

/* Keep a line object's invalidation area close to the actual segment.  A
 * full-panel line object makes LVGL redraw every canvas/image behind it and
 * can keep taskLVGL inside the software renderer long enough to starve IDLE0. */
static void set_line_segment(lv_obj_t *line, lv_point_precise_t points[2],
                             int x1, int y1, int x2, int y2, int padding = 4)
{
    if (!line) return;
    const int left = (x1 < x2 ? x1 : x2) - padding;
    const int top = (y1 < y2 ? y1 : y2) - padding;
    const int right = (x1 > x2 ? x1 : x2) + padding;
    const int bottom = (y1 > y2 ? y1 : y2) + padding;
    points[0] = {(lv_value_precise_t)(x1 - left),
                 (lv_value_precise_t)(y1 - top)};
    points[1] = {(lv_value_precise_t)(x2 - left),
                 (lv_value_precise_t)(y2 - top)};
    lv_obj_set_pos(line, left, top);
    lv_obj_set_size(line, right - left + 1, bottom - top + 1);
    lv_line_set_points_mutable(line, points, 2);
}

/* ═══════════════════════════════════════════════════════════
 *  AS500UI::init()
 * ═══════════════════════════════════════════════════════════ */
static lv_obj_t *mp3_label(lv_obj_t *parent, const char *text,
                           int x, int y, int w, int h,
                           const lv_font_t *font, uint32_t color,
                           lv_text_align_t align = LV_TEXT_ALIGN_CENTER)
{
    lv_obj_t *label_obj = lv_label_create(parent);
    lv_obj_set_pos(label_obj, x, y);
    lv_obj_set_size(label_obj, w, h);
    lv_label_set_text(label_obj, text);
    lv_obj_set_style_text_font(label_obj, font, 0);
    lv_obj_set_style_text_color(label_obj, lv_color_hex(color), 0);
    lv_obj_set_style_text_align(label_obj, align, 0);
    return label_obj;
}

void AS500UI::mp3TileEvent(lv_event_t *e)
{
    auto *ui = static_cast<AS500UI *>(lv_event_get_user_data(e));
    if (!ui) return;
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_LONG_PRESSED) {
        const esp_err_t start_result = AS500Mp3Player::instance().begin();
        if (start_result != ESP_OK) {
            ESP_LOGE("AS500UI", "MP3 service start failed: %s",
                     esp_err_to_name(start_result));
        }
        ui->m_mp3_long_pressed = true;
        ui->createMp3SettingsOverlay();
    } else if (code == LV_EVENT_CLICKED) {
        if (ui->m_mp3_long_pressed) {
            ui->m_mp3_long_pressed = false;
            return;
        }
        auto &player = AS500Mp3Player::instance();
        const esp_err_t start_result = player.begin();
        if (start_result == ESP_OK) player.toggle();
        else ESP_LOGE("AS500UI", "MP3 service start failed: %s",
                      esp_err_to_name(start_result));
        ui->m_mp3_visual = -1;
    }
}

void AS500UI::mp3SettingsEvent(lv_event_t *e)
{
    auto *ui = s_ui_instance;
    if (!ui) return;
    auto &player = AS500Mp3Player::instance();
    const uintptr_t action = reinterpret_cast<uintptr_t>(lv_event_get_user_data(e));
    if (action == MP3_UI_CLOSE) {
        if (ui->m_mp3_settings) lv_obj_add_flag(ui->m_mp3_settings, LV_OBJ_FLAG_HIDDEN);
    } else if (action == MP3_UI_MODE) {
        player.setMode(player.mode() == AS500Mp3Player::PlayMode::Sequential
                           ? AS500Mp3Player::PlayMode::Random
                           : AS500Mp3Player::PlayMode::Sequential);
        ui->refreshMp3Settings();
    } else if (action == MP3_UI_PREVIOUS_PAGE) {
        ui->m_mp3_list_offset = ui->m_mp3_list_offset >= 8 ? ui->m_mp3_list_offset - 8 : 0;
        ui->refreshMp3Settings();
    } else if (action == MP3_UI_NEXT_PAGE) {
        if (ui->m_mp3_list_offset + 8 < player.trackCount()) ui->m_mp3_list_offset += 8;
        ui->refreshMp3Settings();
    } else if (action >= MP3_UI_TRACK_BASE) {
        player.selectTrack(ui->m_mp3_list_offset + action - MP3_UI_TRACK_BASE);
        ui->refreshMp3Settings();
    }
}

static void mp3_volume_event(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    AS500Mp3Player::instance().setVolume(
        static_cast<uint8_t>(lv_slider_get_value(lv_event_get_target_obj(e))));
}

void AS500UI::createMp3SettingsOverlay()
{
    if (m_mp3_settings) {
        lv_obj_clear_flag(m_mp3_settings, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(m_mp3_settings);
        refreshMp3Settings();
        return;
    }
    m_mp3_settings = lv_obj_create(lv_screen_active());
    lv_obj_set_pos(m_mp3_settings, 150, 55);
    lv_obj_set_size(m_mp3_settings, 724, 490);
    lv_obj_set_style_bg_color(m_mp3_settings, lv_color_hex(0xEEF2EF), 0);
    lv_obj_set_style_border_color(m_mp3_settings, lv_color_hex(0x526D64), 0);
    lv_obj_set_style_border_width(m_mp3_settings, 2, 0);
    lv_obj_set_style_radius(m_mp3_settings, 8, 0);
    lv_obj_set_style_pad_all(m_mp3_settings, 0, 0);
    lv_obj_clear_flag(m_mp3_settings, LV_OBJ_FLAG_SCROLLABLE);

    mp3_label(m_mp3_settings, "MP3 PLAYER", 24, 14, 250, 40,
              &lv_font_montserrat_28, 0x20302C, LV_TEXT_ALIGN_LEFT);
    m_mp3_status = mp3_label(m_mp3_settings, "USB NOT CONNECTED", 282, 19, 360, 28,
                             &lv_font_montserrat_16, 0xB03A2E, LV_TEXT_ALIGN_RIGHT);
    lv_label_set_long_mode(m_mp3_status, LV_LABEL_LONG_DOT);
    lv_obj_t *close = lv_btn_create(m_mp3_settings);
    lv_obj_set_pos(close, 660, 10); lv_obj_set_size(close, 44, 40);
    lv_obj_set_style_pad_all(close, 0, 0);
    lv_obj_add_event_cb(close, mp3SettingsEvent, LV_EVENT_CLICKED,
                        reinterpret_cast<void *>(MP3_UI_CLOSE));
    mp3_label(close, LV_SYMBOL_CLOSE, 0, 3, 44, 32, &lv_font_montserrat_24,
              0xFFFFFF, LV_TEXT_ALIGN_CENTER);

    mp3_label(m_mp3_settings, "VOLUME", 24, 70, 110, 30,
              &lv_font_montserrat_16, 0x465650, LV_TEXT_ALIGN_LEFT);
    m_mp3_volume = lv_slider_create(m_mp3_settings);
    lv_obj_set_pos(m_mp3_volume, 130, 76); lv_obj_set_size(m_mp3_volume, 310, 18);
    lv_slider_set_range(m_mp3_volume, 0, 100);
    lv_obj_add_event_cb(m_mp3_volume, mp3_volume_event, LV_EVENT_VALUE_CHANGED, nullptr);

    m_mp3_mode = lv_btn_create(m_mp3_settings);
    lv_obj_set_pos(m_mp3_mode, 470, 62); lv_obj_set_size(m_mp3_mode, 220, 44);
    lv_obj_set_style_pad_all(m_mp3_mode, 0, 0);
    lv_obj_add_event_cb(m_mp3_mode, mp3SettingsEvent, LV_EVENT_CLICKED,
                        reinterpret_cast<void *>(MP3_UI_MODE));

    mp3_label(m_mp3_settings, "MP3 LIST", 24, 118, 150, 30,
              &lv_font_montserrat_18, 0x20302C, LV_TEXT_ALIGN_LEFT);
    for (size_t i = 0; i < 8; ++i) {
        lv_obj_t *button = lv_btn_create(m_mp3_settings);
        lv_obj_set_pos(button, 24, 153 + static_cast<int>(i) * 35);
        lv_obj_set_size(button, 666, 31);
        lv_obj_set_style_radius(button, 3, 0);
        lv_obj_set_style_pad_all(button, 0, 0);
        lv_obj_add_event_cb(button, mp3SettingsEvent, LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(MP3_UI_TRACK_BASE + i));
        m_mp3_track_buttons[i] = button;
        lv_obj_t *track = lv_canvas_create(button);
        lv_obj_set_pos(track, 8, 3);
        lv_obj_set_size(track, 640, 24);
        void *track_buffer = heap_caps_calloc(640 * 24, 4, MALLOC_CAP_SPIRAM);
        if (track_buffer) {
            lv_canvas_set_buffer(track, track_buffer, 640, 24,
                                 LV_COLOR_FORMAT_ARGB8888);
            m_mp3_track_labels[i] = track;
        } else {
            lv_obj_delete(track);
        }
    }
    lv_obj_t *prev = lv_btn_create(m_mp3_settings);
    lv_obj_set_pos(prev, 250, 438); lv_obj_set_size(prev, 90, 34);
    lv_obj_set_style_pad_all(prev, 0, 0);
    lv_obj_add_event_cb(prev, mp3SettingsEvent, LV_EVENT_CLICKED,
                        reinterpret_cast<void *>(MP3_UI_PREVIOUS_PAGE));
    mp3_label(prev, LV_SYMBOL_PREV, 0, 2, 90, 27, &lv_font_montserrat_18,
              0xFFFFFF, LV_TEXT_ALIGN_CENTER);
    lv_obj_t *next = lv_btn_create(m_mp3_settings);
    lv_obj_set_pos(next, 380, 438); lv_obj_set_size(next, 90, 34);
    lv_obj_set_style_pad_all(next, 0, 0);
    lv_obj_add_event_cb(next, mp3SettingsEvent, LV_EVENT_CLICKED,
                        reinterpret_cast<void *>(MP3_UI_NEXT_PAGE));
    mp3_label(next, LV_SYMBOL_NEXT, 0, 2, 90, 27, &lv_font_montserrat_18,
              0xFFFFFF, LV_TEXT_ALIGN_CENTER);
    refreshMp3Settings();
}

void AS500UI::refreshMp3Settings()
{
    if (!m_mp3_settings) return;
    auto &player = AS500Mp3Player::instance();
    m_mp3_revision = player.revision();
    if (m_mp3_list_offset >= player.trackCount()) m_mp3_list_offset = 0;
    lv_slider_set_value(m_mp3_volume, player.volume(), LV_ANIM_OFF);
    lv_obj_clean(m_mp3_mode);
    mp3_label(m_mp3_mode,
              player.mode() == AS500Mp3Player::PlayMode::Random
                  ? LV_SYMBOL_SHUFFLE "  RANDOM"
                  : LV_SYMBOL_LIST "  SEQUENTIAL",
              0, 7, 220, 30, &lv_font_montserrat_14, 0xFFFFFF, LV_TEXT_ALIGN_CENTER);
    if (player.lastError() != ESP_OK)
        lv_label_set_text_fmt(m_mp3_status, "ERROR: %s", esp_err_to_name(player.lastError()));
    else if (player.mounted() && !player.trackCount())
        lv_label_set_text(m_mp3_status, "NO MP3 FILES");
    else if (player.mounted())
        lv_label_set_text_fmt(m_mp3_status, "%s %u/%u",
                              player.playing() ? "PLAYING" : player.enabled() ? "STARTING" : "STOPPED",
                              static_cast<unsigned>(player.currentTrack() + 1),
                              static_cast<unsigned>(player.trackCount()));
    else
        lv_label_set_text(m_mp3_status, player.enabled() ? "WAITING FOR USB" : "USB NOT CONNECTED");
    lv_obj_set_style_text_color(m_mp3_status,
        lv_color_hex(player.mounted() && player.lastError() == ESP_OK ? 0x238636 : 0xB03A2E), 0);
    for (size_t i = 0; i < 8; ++i) {
        const size_t index = m_mp3_list_offset + i;
        char track_name[AS500Mp3Player::kMaxPath];
        if (player.copyTrackName(index, track_name, sizeof(track_name))) {
            lv_obj_clear_flag(m_mp3_track_buttons[i], LV_OBJ_FLAG_HIDDEN);
            const uint32_t row_color = index == player.currentTrack() ? 0x238636 : 0x61756E;
            lv_obj_set_style_bg_color(m_mp3_track_buttons[i], lv_color_hex(row_color), 0);
            char track_text[224];
            snprintf(track_text, sizeof(track_text), "%02u  %s",
                     static_cast<unsigned>(index + 1), track_name);
            if (m_mp3_track_labels[i]) {
                lv_canvas_fill_bg(m_mp3_track_labels[i], lv_color_hex(row_color), LV_OPA_COVER);
                ycb_hangul_draw(m_mp3_track_labels[i], 0, 3, lv_color_white(),
                                lv_color_hex(row_color), track_text, 1, false);
            }
        } else {
            lv_obj_add_flag(m_mp3_track_buttons[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void AS500UI::init(bool can_ok)
{
    s_ui_instance = this;
    loadSettings();
    load_analog_calibration();
    nvs_handle_t vacuum_nvs;
    if (nvs_open("sweeper_cfg", NVS_READWRITE, &vacuum_nvs) == ESP_OK) {
        uint8_t power = 80;
        uint64_t runtime_min = 0;
        if (nvs_get_u8(vacuum_nvs, "vac_power", &power) != ESP_OK || power > 100) {
            power = 80;
            nvs_set_u8(vacuum_nvs, "vac_power", power);
        }
        if (nvs_get_u64(vacuum_nvs, "vac_run_min", &runtime_min) != ESP_OK) {
            /* One-time migration from the older millisecond-based key. */
            uint64_t legacy_runtime_ms = 0;
            if (nvs_get_u64(vacuum_nvs, "vac_runtime", &legacy_runtime_ms) == ESP_OK) {
                runtime_min = legacy_runtime_ms / 60000;
                nvs_erase_key(vacuum_nvs, "vac_runtime");
            }
            nvs_set_u64(vacuum_nvs, "vac_run_min", runtime_min);
        }
        nvs_commit(vacuum_nvs);
        nvs_close(vacuum_nvs);
        m_clean_output_percent = power;
        m_clean_output_edit = power;
        s_setting_data[21] = power;
        m_clean_runtime_ms = runtime_min * 60000;
        m_last_saved_runtime_min = runtime_min;
    }
    /* as500_layout.json layout (1024 x 600). */
    {
        lv_obj_t *scr = lv_scr_act();
        lv_obj_set_style_bg_color(scr, lv_color_hex(0x0F172A), 0);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
        lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

        auto box = [&](lv_obj_t *parent, int x, int y, int w, int h,
                       uint32_t bg, uint32_t border, int radius = 0) {
            lv_obj_t *o = lv_obj_create(parent);
            lv_obj_set_pos(o, x, y); lv_obj_set_size(o, w, h);
            lv_obj_set_style_bg_color(o, lv_color_hex(bg), 0);
            lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
            lv_obj_set_style_border_color(o, lv_color_hex(border), 0);
            lv_obj_set_style_border_width(o, 1, 0);
            lv_obj_set_style_radius(o, radius, 0);
            lv_obj_set_style_pad_all(o, 0, 0);
            lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
            return o;
        };
        auto label = [&](lv_obj_t *parent, const char *text, int x, int y,
                         int w, int h, const lv_font_t *font,
                         uint32_t color, lv_text_align_t align = LV_TEXT_ALIGN_CENTER) {
            lv_obj_t *l = lv_label_create(parent);
            lv_obj_set_pos(l, x, y); lv_obj_set_size(l, w, h);
            lv_label_set_text(l, text);
            lv_obj_set_style_text_font(l, font, 0);
            lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
            lv_obj_set_style_text_align(l, align, 0);
            return l;
        };
        auto button = [&](int x, int y, int w, int h, const char *icon,
                          const char *text, control_cmd_t cmd, uint32_t color) {
            /* Use a plain clickable object, not lv_button. The default button
               theme creates a pressed-state render layer for all child canvases;
               ARGB8888 buffers in PSRAM then keep taskLVGL in recolor_only(). */
            lv_obj_t *b = lv_obj_create(scr);
            lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_pad_all(b, 0, 0);
            lv_obj_set_pos(b, x, y); lv_obj_set_size(b, w, h);
            lv_obj_set_style_bg_color(b, lv_color_hex(color), 0);
            lv_obj_set_style_border_color(b, lv_color_hex(0x0F172A), 0);
            lv_obj_set_style_border_width(b, 0, 0);
            lv_obj_set_style_radius(b, 0, 0);
            register_touch_button(b, cmd);
            bool manual = cmd == CMD_FORWARD || cmd == CMD_REVERSE ||
                          cmd == CMD_LEFT || cmd == CMD_RIGHT;
            if (manual) {
                /* Visual feedback only: change the plain object's background.
                   Do not transform/recolor its child canvas. */
                lv_obj_set_style_bg_color(
                    b, lv_color_darken(lv_color_hex(color), LV_OPA_40),
                    LV_PART_MAIN | LV_STATE_PRESSED);
                lv_obj_set_style_border_color(
                    b, lv_color_hex(0xFFFFFF),
                    LV_PART_MAIN | LV_STATE_PRESSED);
                lv_obj_set_style_border_width(
                    b, 4, LV_PART_MAIN | LV_STATE_PRESSED);
                lv_obj_add_event_cb(b, manualButtonEvent, LV_EVENT_PRESSED,
                                    (void *)(uintptr_t)cmd);
                lv_obj_add_event_cb(b, manualButtonEvent, LV_EVENT_PRESSING,
                                    (void *)(uintptr_t)cmd);
                lv_obj_add_event_cb(b, manualButtonEvent, LV_EVENT_RELEASED,
                                    (void *)(uintptr_t)cmd);
                lv_obj_add_event_cb(b, manualButtonEvent, LV_EVENT_PRESS_LOST,
                                    (void *)(uintptr_t)cmd);
            } else if (cmd != CMD_NONE) {
                lv_obj_add_event_cb(b, buttonEvent, LV_EVENT_CLICKED,
                                    (void *)(uintptr_t)cmd);
            }
            lv_obj_t *ic = lv_label_create(b);
            lv_label_set_text(ic, icon);
            lv_obj_set_style_text_font(ic, &lv_font_montserrat_28, 0);
            lv_obj_set_style_text_color(ic, lv_color_hex(0xFFFFFF), 0);
            lv_obj_align(ic, LV_ALIGN_TOP_MID, 0, 22);
            lv_obj_t *text_canvas = lv_canvas_create(b);
            lv_obj_set_style_image_recolor_opa(text_canvas, LV_OPA_TRANSP,
                                                LV_PART_MAIN | LV_STATE_ANY);
            lv_obj_set_size(text_canvas, 220, 40);
            lv_obj_align(text_canvas, LV_ALIGN_BOTTOM_MID, 0, -18);
            void *text_buf = heap_caps_calloc(220 * 40, 4, MALLOC_CAP_SPIRAM);
            if (text_buf) {
                lv_canvas_set_buffer(text_canvas, text_buf, 220, 40,
                                     LV_COLOR_FORMAT_ARGB8888);
                draw_button_korean(text_canvas, text);
            }
            return b;
        };

        /* The former controller dashboard is retained in source for protocol
           migration, but must not be instantiated underneath the AS500
           screen: building both trees exhausts the boot-time CPU budget. */
        if (false) {
        /* Match the logo panel to the artwork's wide aspect ratio. */
        lv_obj_t *brand = box(scr, 4, 4, 251, 118, 0xFFFFFF, 0xFFFFFF, 0);
        lv_obj_set_style_border_width(brand, 0, 0);
        lv_obj_t *logo = lv_image_create(brand);
        lv_image_set_src(logo, &as_logo);
        lv_obj_center(logo);

        /* Borderless Windows-style status tiles with independent pastel hues. */
        lv_obj_t *status = box(scr, 259, 126, 251, 157, 0x0F172A, 0x0F172A, 0);
        lv_obj_set_style_border_width(status, 0, 0);
        lv_obj_t *battery_tile = box(status, 0, 0, 251, 49, 0x18352F, 0x18352F, 6);
        lv_obj_set_style_border_width(battery_tile, 0, 0);
        label(battery_tile, LV_SYMBOL_BATTERY_FULL, 9, 7, 28, 24,
              &lv_font_montserrat_20, 0x8BD5A5);
        label(battery_tile, "BATTERY", 42, 8, 70, 22,
              &lv_font_montserrat_16, 0xD7F3E1, LV_TEXT_ALIGN_LEFT);
        m_battery_voltage = label(battery_tile, "--.-V   --.-A",
                                  112, 7, 126, 24,
                                  &lv_font_montserrat_16, 0xD7F3E1,
                                  LV_TEXT_ALIGN_RIGHT);
        m_brake_status = label(battery_tile, "EMB (DRV): UNKNOWN",
                               9, 27, 230, 15,
                               &lv_font_montserrat_12, 0xFF6969,
                               LV_TEXT_ALIGN_LEFT);
        m_battery_bar = lv_bar_create(battery_tile);
        lv_obj_set_pos(m_battery_bar, 9, 44);
        lv_obj_set_size(m_battery_bar, 230, 3);
        lv_bar_set_range(m_battery_bar, 0, 100);
        lv_bar_set_value(m_battery_bar, 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(m_battery_bar, lv_color_hex(0x35564E), LV_PART_MAIN);
        lv_obj_set_style_bg_color(m_battery_bar, lv_color_hex(0x8BD5A5), LV_PART_INDICATOR);
        lv_obj_set_style_radius(m_battery_bar, 4, LV_PART_MAIN);
        lv_obj_set_style_radius(m_battery_bar, 4, LV_PART_INDICATOR);
        lv_obj_t *cleaning_tile = box(status, 0, 53, 251, 51, 0x193149, 0x193149, 6);
        lv_obj_set_style_border_width(cleaning_tile, 0, 0);
        label(cleaning_tile, LV_SYMBOL_POWER, 9, 7, 28, 24,
              &lv_font_montserrat_18, 0x88C7F5);
        label(cleaning_tile, "CLEANING", 42, 8, 92, 24,
              &lv_font_montserrat_16, 0xDCEEFE, LV_TEXT_ALIGN_LEFT);
        m_clean_value = label(cleaning_tile, "OFF", 162, 5, 76, 28,
                              &lv_font_montserrat_20, 0x88C7F5,
                              LV_TEXT_ALIGN_RIGHT);
        m_clean_bar = lv_bar_create(cleaning_tile);
        lv_obj_set_pos(m_clean_bar, 9, 37);
        lv_obj_set_size(m_clean_bar, 230, 10);
        lv_bar_set_range(m_clean_bar, 0, 100);
        lv_bar_set_value(m_clean_bar, 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(m_clean_bar, lv_color_hex(0x38536A), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(m_clean_bar, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(m_clean_bar, lv_color_hex(0x88C7F5), LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(m_clean_bar, LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_radius(m_clean_bar, 6, LV_PART_MAIN);
        lv_obj_set_style_radius(m_clean_bar, 6, LV_PART_INDICATOR);
        lv_obj_t *runtime_tile = box(status, 0, 108, 251, 49, 0x302B49, 0x302B49, 6);
        lv_obj_set_style_border_width(runtime_tile, 0, 0);
        label(runtime_tile, LV_SYMBOL_LOOP, 9, 10, 28, 24,
              &lv_font_montserrat_18, 0xC4B5FD);
        label(runtime_tile, "RUN TIME", 42, 11, 90, 24,
              &lv_font_montserrat_16, 0xEEE9FF, LV_TEXT_ALIGN_LEFT);
        m_runtime_value = label(runtime_tile, "00:00:00", 128, 7, 110, 30,
                                &lv_font_montserrat_20, 0xC4B5FD,
                                LV_TEXT_ALIGN_RIGHT);

        /* Tag/vehicle relative-position card. */
        lv_obj_t *dir = box(scr, 514, 4, 506, 442, 0x111827, 0x111827, 8);
        lv_obj_set_style_border_width(dir, 0, 0);
        m_diff_val = label(dir, "+0.0 deg", 280, 10, 140, 24,
                           &lv_font_montserrat_18, 0xFFFFFF, LV_TEXT_ALIGN_LEFT);
        m_avg_val = label(dir, "--- cm", 280, 34, 140, 24,
                          &lv_font_montserrat_18, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);
        lv_obj_add_flag(m_diff_val, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(m_avg_val, LV_OBJ_FLAG_HIDDEN);
        /* Moving TAG marker retains its original vehicle-relative position. */
        m_dir_target = box(dir, 234, 34, 44, 30, 0x1F6FEB, 0xFFFFFF, 6);
        lv_obj_clear_flag(m_dir_target, LV_OBJ_FLAG_CLICKABLE);
        label(m_dir_target, "TAG", 0, 7, 44, 18,
              &lv_font_montserrat_12, 0xFFFFFF);
        m_dir_line = lv_line_create(dir);
        set_line_segment(m_dir_line, m_dir_line_points, 256, 225, 256, 49);
        lv_obj_set_style_line_color(m_dir_line, lv_color_hex(0x8B949E), 0);
        lv_obj_set_style_line_width(m_dir_line, 2, 0);
        lv_obj_set_style_line_dash_width(m_dir_line, 6, 0);
        lv_obj_set_style_line_dash_gap(m_dir_line, 5, 0);
        lv_obj_move_to_index(m_dir_line, 0);

        if (kLidarOverlayEnabled) {
        /* Pre-render both rings once. A single canvas avoids queuing dozens
           of arc draw tasks every frame, which can exhaust LVGL draw memory. */
        lv_obj_t *range_canvas = lv_canvas_create(dir);
        lv_obj_set_pos(range_canvas, 0, 0);
        lv_obj_set_size(range_canvas, 512, 450);
        lv_obj_set_style_image_recolor_opa(range_canvas, LV_OPA_TRANSP,
                                           LV_PART_MAIN | LV_STATE_ANY);
        lv_obj_clear_flag(range_canvas, LV_OBJ_FLAG_CLICKABLE);
        void *range_buf = heap_caps_calloc(512 * 450, 4, MALLOC_CAP_SPIRAM);
        if (range_buf) {
            lv_canvas_set_buffer(range_canvas, range_buf, 512, 450,
                                 LV_COLOR_FORMAT_ARGB8888);
            s_lidar_range_canvas = range_canvas;
            redrawLidarRangeCanvas();
        } else {
            lv_obj_add_flag(range_canvas, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_move_to_index(range_canvas, 0);
        label(dir, "5 m", 458, 8, 42, 18,
              &lv_font_montserrat_12, 0xAFA4EA);
        label(dir, "3 m", 414, 8, 42, 18,
              &lv_font_montserrat_12, 0x79C9C1);

        /* Faint left/right boundaries of the configured LiDAR detection
           sector, extending from the sensor origin to the absolute 5 m ring. */
        for (int i = 0; i < 2; ++i) {
            m_lidar_range_lines[i] = lv_line_create(dir);
            set_line_segment(m_lidar_range_lines[i],
                             m_lidar_range_line_points[i],
                             256, 225, 256, 0);
            lv_obj_set_style_line_color(m_lidar_range_lines[i],
                                        lv_color_hex(0x58A6FF), 0);
            lv_obj_set_style_line_opa(m_lidar_range_lines[i], LV_OPA_30, 0);
            lv_obj_set_style_line_width(m_lidar_range_lines[i], 2, 0);
            lv_obj_move_to_index(m_lidar_range_lines[i], 1 + i);
        }

        for (int i = 0; i < 36; ++i) {
            m_lidar_points[i] = box(dir, 253, 222, 6, 6,
                                    0xE3B341, 0xE3B341, 3);
            lv_obj_set_style_border_width(m_lidar_points[i], 0, 0);
            lv_obj_clear_flag(m_lidar_points[i], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_flag(m_lidar_points[i], LV_OBJ_FLAG_HIDDEN);
        }
        /* One ray is sufficient to emphasize the nearest obstacle. */
        m_lidar_rays[0] = lv_line_create(dir);
        set_line_segment(m_lidar_rays[0], m_lidar_ray_points[0],
                         256, 225, 256, 225);
        lv_obj_set_style_line_width(m_lidar_rays[0], 3, 0);
        lv_obj_add_flag(m_lidar_rays[0], LV_OBJ_FLAG_HIDDEN);

        m_lidar_nearest = label(dir, "NO LIDAR", 344, 207, 150, 38,
                                &lv_font_montserrat_12, 0x8B949E,
                                LV_TEXT_ALIGN_LEFT);

        /* Vehicle center, LiDAR plot origin and TAG polar origin coincide. */
        m_vehicle = lv_obj_create(dir);
        lv_obj_set_pos(m_vehicle, 166, 135);
        lv_obj_set_size(m_vehicle, 180, 180);
        lv_obj_set_style_bg_color(m_vehicle, lv_color_hex(0x0D1117), 0);
        lv_obj_set_style_bg_opa(m_vehicle, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(m_vehicle, 0, 0);
        lv_obj_set_style_pad_all(m_vehicle, 0, 0);
        lv_obj_clear_flag(m_vehicle, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(m_vehicle, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(m_vehicle, vehicleDrawEvent,
                            LV_EVENT_DRAW_MAIN, this);
        m_vehicle_info = label(dir, "0 deg  |  0 cm  |  PWM 0%",
                               110, 360, 292, 24,
                               &lv_font_montserrat_14, 0xC9D1D9);
        lv_obj_add_flag(m_vehicle_info, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(m_vehicle);
        }
        /* Fixed UWB fields: titles and units never move; only numeric labels
         * are refreshed. This avoids visible horizontal jitter as digits
         * change width. */
        label(dir, "A0:", 8, 8, 34, 24,
              &lv_font_montserrat_18, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);
        m_val_dl = label(dir, " ---", 42, 8, 52, 24,
                         &lv_font_montserrat_18, 0xFFFFFF, LV_TEXT_ALIGN_RIGHT);
        label(dir, "cm", 96, 8, 36, 24,
              &lv_font_montserrat_18, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);

        label(dir, "A1:", 8, 36, 34, 24,
              &lv_font_montserrat_18, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);
        m_val_dr = label(dir, " ---", 42, 36, 52, 24,
                         &lv_font_montserrat_18, 0xFFFFFF, LV_TEXT_ALIGN_RIGHT);
        label(dir, "cm", 96, 36, 36, 24,
              &lv_font_montserrat_18, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);
        m_pose_quality = label(dir, "TAG WAIT", 8, 66, 260, 16,
                               &lv_font_montserrat_12, 0xE3B341,
                               LV_TEXT_ALIGN_LEFT);
        m_serial_command = label(dir,
                                 "UART: WAIT",
                                 8, 410, 242, 28,
                                 &lv_font_montserrat_18, 0x58D6E8,
                                 LV_TEXT_ALIGN_LEFT);
        m_tag_remote = label(dir,
                             "KEY:00  ---ms  ---dBm",
                             252, 410, 246, 28,
                             &lv_font_montserrat_18, 0xFFFFFF,
                             LV_TEXT_ALIGN_RIGHT);

        /* Transparent long-press surface for LiDAR settings.  It does not
           alter acquisition or drawing and opens the existing NVS-backed
           settings menu directly at items 11-14. */
        lv_obj_t *lidar_settings_touch = lv_obj_create(dir);
        lv_obj_set_pos(lidar_settings_touch, 0, 64);
        lv_obj_set_size(lidar_settings_touch, 512, 350);
        lv_obj_set_style_bg_opa(lidar_settings_touch, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(lidar_settings_touch, 0, 0);
        lv_obj_set_style_pad_all(lidar_settings_touch, 0, 0);
        lv_obj_clear_flag(lidar_settings_touch, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(lidar_settings_touch, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(lidar_settings_touch, lidarSettingsLongPressEvent,
                            LV_EVENT_LONG_PRESSED, nullptr);

        /* 2 x 4 control area, exactly matching the editor layout. */
        button(259,   4, 251, 118, LV_SYMBOL_SETTINGS, "설정",           CMD_SETTINGS,     0x64748B);
        m_btn_vacuum = button(4, 126, 251, 157, LV_SYMBOL_POWER,
                              "청소기 꺼짐", CMD_VACUUM_TOGGLE, 0x4F7F8C);
        m_lbl_vacuum = lv_obj_get_child(m_btn_vacuum, 1);
        lv_obj_remove_event_cb(m_btn_vacuum, buttonEvent);
        lv_obj_add_event_cb(m_btn_vacuum, vacuumButtonEvent,
                            LV_EVENT_CLICKED, this);
        m_btn_follow = button(4, 287, 251, 159, LV_SYMBOL_PLAY,
                              "추종 시작", CMD_FOLLOW_START, 0x5E7D6D);
        m_icon_follow = lv_obj_get_child(m_btn_follow, 0);
        m_lbl_follow = lv_obj_get_child(m_btn_follow, 1);
        button(  4, 450, 251, 146, LV_SYMBOL_UP,       "전진",           CMD_FORWARD,      0x5B8DEF);
        m_drive_card = box(scr, 259, 287, 251, 159, 0x1E293B, 0x1E293B, 0);
        lv_obj_set_style_border_width(m_drive_card, 0, 0);
        lv_obj_clear_flag(m_drive_card, LV_OBJ_FLAG_CLICKABLE);
        m_drive_icon = label(m_drive_card, LV_SYMBOL_PAUSE, 92, 4, 72, 48,
                             &lv_font_montserrat_48, 0x8B949E);
        /* Distance/angle captions, units and separator stay fixed.  Their
           separator uses the same x position as the PWM row below. */
        auto fixed_korean = [&](const char *caption, int x) {
            lv_obj_t *cv = lv_canvas_create(m_drive_card);
            lv_obj_set_style_image_recolor_opa(cv, LV_OPA_TRANSP,
                                                LV_PART_MAIN | LV_STATE_ANY);
            lv_obj_set_pos(cv, x, 66);
            lv_obj_set_size(cv, 34, 26);
            void *buf = heap_caps_calloc(34 * 26, 4, MALLOC_CAP_SPIRAM);
            if (buf) {
                lv_canvas_set_buffer(cv, buf, 34, 26,
                                     LV_COLOR_FORMAT_ARGB8888);
                lv_canvas_fill_bg(cv, lv_color_hex(0x161B22), LV_OPA_TRANSP);
                ycb_hangul_draw(cv, 0, 3, lv_color_hex(0xC9D1D9),
                                lv_color_hex(0x161B22), caption, 1, true);
            }
        };
        fixed_korean("거리", 12);
        m_drive_distance = label(m_drive_card, "--", 44, 66, 60, 26,
                                 &lv_font_montserrat_18, 0xFFFFFF,
                                 LV_TEXT_ALIGN_RIGHT);
        lv_label_set_long_mode(m_drive_distance, LV_LABEL_LONG_CLIP);
        label(m_drive_card, "cm", 106, 69, 24, 22,
              &lv_font_montserrat_12, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);
        label(m_drive_card, "|", 119, 66, 18, 26,
              &lv_font_montserrat_18, 0xC9D1D9, LV_TEXT_ALIGN_CENTER);
        m_drive_status = label(m_drive_card, "--", 135, 66, 57, 26,
                               &lv_font_montserrat_18, 0xFFFFFF,
                               LV_TEXT_ALIGN_RIGHT);
        lv_label_set_long_mode(m_drive_status, LV_LABEL_LONG_CLIP);
        label(m_drive_card, "°", 192, 66, 13, 26,
              &lv_font_montserrat_18, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);
        fixed_korean("조향", 204);
        /* Keep the captions and separators static. Only the two numeric
           labels are refreshed, avoiding unnecessary redraws. */
        label(m_drive_card, "L", 12, 108, 18, 30,
              &lv_font_montserrat_24, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);
        m_drive_l_pwm = label(m_drive_card, "+0", 30, 108, 60, 30,
                              &lv_font_montserrat_24, 0xC9D1D9,
                              LV_TEXT_ALIGN_RIGHT);
        label(m_drive_card, "%", 92, 108, 24, 30,
              &lv_font_montserrat_24, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);
        label(m_drive_card, "|", 119, 108, 18, 30,
              &lv_font_montserrat_24, 0xC9D1D9, LV_TEXT_ALIGN_CENTER);
        m_drive_r_pwm = label(m_drive_card, "+0", 140, 108, 60, 30,
                              &lv_font_montserrat_24, 0xC9D1D9,
                              LV_TEXT_ALIGN_RIGHT);
        label(m_drive_card, "%", 202, 108, 24, 30,
              &lv_font_montserrat_24, 0xC9D1D9, LV_TEXT_ALIGN_LEFT);
        label(m_drive_card, "R", 228, 108, 18, 30,
              &lv_font_montserrat_24, 0xC9D1D9, LV_TEXT_ALIGN_RIGHT);

        /* Motor RPM feedback is unavailable in CAN 0x5A0. Numeric columns use the
           same x coordinates as the L/R percentage row above. */
        /* A single fixed-width row saves six LVGL objects and leaves enough
           internal RAM for LVGL's glyph draw buffer. */
        m_drive_m1_rpm = label(m_drive_card,
                               "M1   --- rpm  |  --- rpm   M2",
                               4, 134, 243, 22,
                               &lv_font_montserrat_12, 0xFFFFFF,
                               LV_TEXT_ALIGN_CENTER);
        button(259, 450, 251, 146, LV_SYMBOL_DOWN,     "후진",           CMD_REVERSE,      0x5B8DEF);
        button(514, 450, 251, 146, LV_SYMBOL_LEFT,     "좌회전",         CMD_LEFT,         0x8B7ED8);
        button(769, 450, 251, 146, LV_SYMBOL_RIGHT,    "우회전",         CMD_RIGHT,        0x8B7ED8);

        /* Centered directly above the four manual-drive cards. */
        m_estop_popup = box(scr, 259, 382, 506, 64, 0xB62324, 0xFF6B6B, 10);
        lv_obj_set_style_border_width(m_estop_popup, 3, 0);
        lv_obj_clear_flag(m_estop_popup, LV_OBJ_FLAG_CLICKABLE);
        /* This overlay is hidden during normal operation. Keep its static
           heading as a normal LVGL label instead of another ARGB8888 canvas;
           an extra Hangul canvas pushed the software renderer into a
           continuous glyph-allocation loop during the first screen draw. */
        m_estop_source = label(m_estop_popup, "E-STOP     GPIO2",
                               12, 14, 482, 36,
                               &lv_font_montserrat_24, 0xFFFFFF,
                               LV_TEXT_ALIGN_CENTER);
        lv_obj_add_flag(m_estop_popup, LV_OBJ_FLAG_HIDDEN);

        /* Settings overlay: 20 items in two columns, as ui_seting.json. */
        s_settings_overlay = box(scr, 0, 0, 1024, 600, 0x0F172A, 0x0F172A);
        s_row_settings_overlay = s_settings_overlay;
        lv_obj_t *title_panel = box(s_settings_overlay, 8, 8, 408, 84,
                                    0x1E293B, 0x1E293B, 10);
        lv_obj_set_style_border_width(title_panel, 0, 0);
        s_settings_title_canvas = lv_canvas_create(title_panel);
        lv_obj_set_style_image_recolor_opa(s_settings_title_canvas, LV_OPA_TRANSP,
                                            LV_PART_MAIN | LV_STATE_ANY);
        lv_obj_set_size(s_settings_title_canvas, 300, 40);
        lv_obj_center(s_settings_title_canvas);
        void *title_buf = heap_caps_calloc(300 * 40, 4, MALLOC_CAP_SPIRAM);
        if (title_buf) {
            lv_canvas_set_buffer(s_settings_title_canvas, title_buf, 300, 40,
                                 LV_COLOR_FORMAT_ARGB8888);
            lv_canvas_fill_bg(s_settings_title_canvas,
                              lv_color_hex(0x1E293B), LV_OPA_TRANSP);
        }
        auto setting_header_button = [&](int x, const char *text, int action,
                                         uint32_t color) {
            lv_obj_t *b = lv_obj_create(s_settings_overlay);
            lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_pad_all(b, 0, 0);
            lv_obj_set_pos(b, x + 6, 8); lv_obj_set_size(b, 188, 84);
            lv_obj_set_style_bg_color(b, lv_color_hex(color), 0);
            lv_obj_set_style_bg_color(
                b, lv_color_darken(lv_color_hex(color), LV_OPA_50),
                LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_border_color(
                b, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_border_width(
                b, 3, LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_border_width(b, 0, 0);
            lv_obj_set_style_radius(b, 10, 0);
            lv_obj_add_event_cb(b, settingControlEvent, LV_EVENT_CLICKED,
                                (void *)(intptr_t)action);
            lv_obj_t *cv = lv_canvas_create(b);
            lv_obj_set_style_image_recolor_opa(cv, LV_OPA_TRANSP,
                                                LV_PART_MAIN | LV_STATE_ANY);
            lv_obj_set_size(cv, 160, 40); lv_obj_center(cv);
            void *buf = heap_caps_calloc(160 * 40, 4, MALLOC_CAP_SPIRAM);
            if (buf) {
                lv_canvas_set_buffer(cv, buf, 160, 40, LV_COLOR_FORMAT_ARGB8888);
                lv_canvas_fill_bg(cv, lv_color_hex(color), LV_OPA_TRANSP);
                int width = ycb_hangul_measure(text, 2);
                ycb_hangul_draw(cv, (160 - width) / 2, 4,
                                lv_color_hex(0xFFFFFF), lv_color_hex(color),
                                text, 2, true);
            }
        };
        setting_header_button(424, "취소", 4, 0x64748B);
        setting_header_button(624, "저장", 5, 0x5E8C72);
        setting_header_button(824, "공장초기화", 6, 0xB66A7C);
        for (int i = 0; i < kVisibleSettingCount; ++i) {
            int col = i / 8;
            int row = i % 8;
            lv_obj_t *r = box(s_settings_overlay, 8 + col * 508, 100 + row * 43,
                              500, 41, i == 0 ? 0x426B9B : 0x1E293B,
                              i == 0 ? 0x93C5FD : 0x334155, 6);
            s_setting_rows[i] = r;
            lv_obj_add_flag(r, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(r, settingControlEvent, LV_EVENT_CLICKED,
                                (void *)(intptr_t)(100 + i));
            lv_obj_t *cv = lv_canvas_create(r);
            s_setting_name_canvases[i] = cv;
            lv_obj_add_flag(cv, LV_OBJ_FLAG_EVENT_BUBBLE);
            lv_obj_set_style_image_recolor_opa(cv, LV_OPA_TRANSP,
                                                LV_PART_MAIN | LV_STATE_ANY);
            lv_obj_set_pos(cv, 8, 1); lv_obj_set_size(cv, 320, 39);
            void *buf = heap_caps_calloc(320 * 39, 4, MALLOC_CAP_SPIRAM);
            if (buf) {
                lv_canvas_set_buffer(cv, buf, 320, 39, LV_COLOR_FORMAT_ARGB8888);
                lv_canvas_fill_bg(cv, lv_color_hex(0x1E293B), LV_OPA_TRANSP);
            }
            lv_obj_t *value_box = box(r, 330, 4, 164, 33,
                                      0x0D1117, 0x30363D, 3);
            lv_obj_add_flag(value_box, LV_OBJ_FLAG_EVENT_BUBBLE);
            s_setting_values[i] = label(value_box, "0", 4, 6, 154, 22,
                &lv_font_montserrat_18, 0xFFFFFF, LV_TEXT_ALIGN_RIGHT);
            lv_obj_add_flag(s_setting_values[i], LV_OBJ_FLAG_EVENT_BUBBLE);
        }
        lv_obj_t *help_canvas = lv_canvas_create(s_settings_overlay);
        lv_obj_set_style_image_recolor_opa(help_canvas, LV_OPA_TRANSP,
                                            LV_PART_MAIN | LV_STATE_ANY);
        lv_obj_set_pos(help_canvas, 262, 463);
        lv_obj_set_size(help_canvas, 500, 24);
        void *help_buf = heap_caps_calloc(500 * 24, 4, MALLOC_CAP_SPIRAM);
        if (help_buf) {
            lv_canvas_set_buffer(help_canvas, help_buf, 500, 24,
                                 LV_COLOR_FORMAT_ARGB8888);
            lv_canvas_fill_bg(help_canvas, lv_color_hex(0x0D1117), LV_OPA_TRANSP);
            const char *help_text = "항목을 직접 터치하고  - / + 로 값을 조정하세요";
            int help_width = ycb_hangul_measure(help_text, 1);
            ycb_hangul_draw(help_canvas, (500 - help_width) / 2, 4,
                            lv_color_hex(0x8B949E), lv_color_hex(0x0D1117),
                            help_text, 1, true);
        }
        static const char *control_text[2] = { "-", "+" };
        static const uint32_t control_color[2] = { 0x8B7ED8, 0x5B8DEF };
        for (int i = 0; i < 2; ++i) {
            lv_obj_t *b = lv_obj_create(s_settings_overlay);
            lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_pad_all(b, 0, 0);
            lv_obj_set_pos(b, i * 512, 500); lv_obj_set_size(b, 512, 100);
            lv_obj_set_style_bg_color(b, lv_color_hex(control_color[i]), 0);
            lv_obj_set_style_bg_color(
                b, lv_color_darken(lv_color_hex(control_color[i]), LV_OPA_50),
                LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_border_color(b, lv_color_hex(0x0D1117), 0);
            lv_obj_set_style_border_color(
                b, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_border_width(b, 1, 0);
            lv_obj_set_style_border_width(
                b, 4, LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_radius(b, 0, 0);
            lv_obj_add_event_cb(b, settingControlEvent, LV_EVENT_CLICKED,
                                (void *)(intptr_t)(i + 2));
            lv_obj_add_event_cb(b, settingControlEvent, LV_EVENT_LONG_PRESSED_REPEAT,
                                (void *)(intptr_t)(i + 2));
            lv_obj_t *bl = lv_label_create(b);
            lv_label_set_text(bl, control_text[i]);
            lv_obj_set_style_text_font(bl, &lv_font_montserrat_48, 0);
            lv_obj_set_style_text_color(bl, lv_color_hex(0xFFFFFF), 0);
            lv_obj_center(bl);
        }
        s_settings_saved_popup = box(s_settings_overlay, 312, 230, 400, 140,
                                     0x21262D, 0x58A6FF, 10);
        lv_obj_set_style_border_width(s_settings_saved_popup, 3, 0);
        lv_obj_set_style_shadow_width(s_settings_saved_popup, 24, 0);
        lv_obj_set_style_shadow_color(s_settings_saved_popup,
                                      lv_color_hex(0x000000), 0);
        lv_obj_t *saved_canvas = lv_canvas_create(s_settings_saved_popup);
        lv_obj_set_style_image_recolor_opa(saved_canvas, LV_OPA_TRANSP,
                                            LV_PART_MAIN | LV_STATE_ANY);
        lv_obj_set_size(saved_canvas, 340, 48);
        lv_obj_center(saved_canvas);
        void *saved_buf = heap_caps_calloc(340 * 48, 4, MALLOC_CAP_SPIRAM);
        if (saved_buf) {
            lv_canvas_set_buffer(saved_canvas, saved_buf, 340, 48,
                                 LV_COLOR_FORMAT_ARGB8888);
            lv_canvas_fill_bg(saved_canvas, lv_color_hex(0x21262D), LV_OPA_TRANSP);
            int width = ycb_hangul_measure("저장하였습니다", 2);
            ycb_hangul_draw(saved_canvas, (340 - width) / 2, 8,
                            lv_color_hex(0xFFFFFF), lv_color_hex(0x21262D),
                            "저장하였습니다", 2, true);
        }
        lv_obj_add_flag(s_settings_saved_popup, LV_OBJ_FLAG_HIDDEN);
        refresh_setting_ui();
        lv_obj_add_flag(s_settings_overlay, LV_OBJ_FLAG_HIDDEN);
        }

          /* AS500-HMI main screen. Keep the proven controller widgets below
             this layer until their 48 V protocol replacements are complete. */
          m_sprayer_dashboard = box(scr, 0, 0, 1024, 600,
                            0xEEF1ED, 0xEEF1ED, 0);
          lv_obj_set_style_border_width(m_sprayer_dashboard, 0, 0);
          lv_obj_clear_flag(m_sprayer_dashboard, LV_OBJ_FLAG_CLICKABLE);

          lv_obj_t *header = box(m_sprayer_dashboard, 0, 0, 1024, 120,
                         0x263936, 0x263936, 0);
          lv_obj_set_style_border_width(header, 0, 0);
          lv_obj_t *as_header_logo = lv_image_create(header);
          lv_image_set_src(as_header_logo, &as_logo);
          lv_obj_set_pos(as_header_logo, 16, 0);
          lv_obj_set_style_image_recolor(as_header_logo, lv_color_white(), 0);
          lv_obj_set_style_image_recolor_opa(as_header_logo, LV_OPA_COVER, 0);
          label(header, "ASS-500S", 367, 31, 330, 58,
                &lv_font_montserrat_48, 0xFFFFFF, LV_TEXT_ALIGN_LEFT);
          m_dash_can = label(header, can_ok ? "CAN  ONLINE" : "CAN  FAIL",
                       861, 93, 163, 28, &lv_font_montserrat_18,
                       can_ok ? 0x9ED5B8 : 0xF2A7A0, LV_TEXT_ALIGN_LEFT);
          lv_label_set_long_mode(m_dash_can, LV_LABEL_LONG_CLIP);
          /* Physical FNR selector received from AS500-interface over 0x501. */
          lv_obj_t *fnr_box = box(header, 723, 12, 125, 75,
                                  0xFFFFFF, 0x64748B, 4);
          m_dash_fnr_box = fnr_box;
          m_dash_fnr_title = label(fnr_box, "FNR", 6, 3, 113, 20,
                &lv_font_montserrat_16, 0x20302C, LV_TEXT_ALIGN_CENTER);
          m_dash_fnr = lv_canvas_create(fnr_box);
          lv_obj_set_pos(m_dash_fnr, 0, 29);
          lv_obj_set_size(m_dash_fnr, 125, 40);
          void *fnr_buf = heap_caps_calloc(125 * 40, 4, MALLOC_CAP_SPIRAM);
          if (fnr_buf) {
            lv_canvas_set_buffer(m_dash_fnr, fnr_buf, 125, 40,
                                 LV_COLOR_FORMAT_ARGB8888);
            lv_canvas_fill_bg(m_dash_fnr, lv_color_hex(0xFFFFFF), LV_OPA_COVER);
          } else {
            lv_obj_add_flag(m_dash_fnr, LV_OBJ_FLAG_HIDDEN);
            m_dash_fnr = nullptr;
          }
          m_dash_cruise = box(header, 861, 10, 121, 78,
                              0x526D64, 0xFFFFFF, 4);
          s_touch_cruise = m_dash_cruise;
          lv_obj_set_style_border_width(m_dash_cruise, 2, 0);
          lv_obj_add_flag(m_dash_cruise, LV_OBJ_FLAG_CLICKABLE);
          lv_obj_add_event_cb(m_dash_cruise, cruiseButtonEvent,
                              LV_EVENT_ALL, nullptr);
          m_dash_cruise_label = lv_canvas_create(m_dash_cruise);
          lv_obj_set_pos(m_dash_cruise_label, 2, 19);
          lv_obj_set_size(m_dash_cruise_label, 117, 40);
          void *cruise_buf = heap_caps_calloc(117 * 40, 4, MALLOC_CAP_SPIRAM);
          if (cruise_buf) {
            lv_canvas_set_buffer(m_dash_cruise_label, cruise_buf, 117, 40,
                                 LV_COLOR_FORMAT_ARGB8888);
          } else {
            lv_obj_add_flag(m_dash_cruise_label, LV_OBJ_FLAG_HIDDEN);
            m_dash_cruise_label = nullptr;
          }
          ESP_LOGI("AS500UI", "AS500 header ready");

          lv_obj_t *direction = box(m_sprayer_dashboard, 16, 136, 288, 448,
                            0x243633, 0x243633, 6);
          lv_obj_set_style_border_width(direction, 0, 0);
          int aux_index = 0;
          auto direction_button = [&](int x, int y, const char *symbol,
                                      const char *caption,
                                      control_cmd_t command) {
            lv_obj_t *button = box(direction, x, y, 124, 136,
                                   0x314A45, 0x58736B, 5);
            lv_obj_add_flag(button, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(button, LV_OBJ_FLAG_SCROLLABLE);
            m_aux_buttons[aux_index] = button;
            if (command == CMD_REVERSE) {
              lv_obj_t *horn = lv_canvas_create(button);
              lv_obj_set_pos(horn, 22, 9);
              lv_obj_set_size(horn, 80, 60);
              void *horn_buf = heap_caps_calloc(80 * 60, 4, MALLOC_CAP_SPIRAM);
              if (horn_buf) {
                lv_canvas_set_buffer(horn, horn_buf, 80, 60,
                                     LV_COLOR_FORMAT_ARGB8888);
                draw_horn_icon(horn);
              } else {
                lv_obj_delete(horn);
                label(button, symbol, 36, 14, 52, 52,
                      &lv_font_montserrat_32, 0xFFFFFF, LV_TEXT_ALIGN_CENTER);
              }
            } else {
              label(button, symbol, 36, 14, 52, 52,
                    &lv_font_montserrat_32, 0xFFFFFF, LV_TEXT_ALIGN_CENTER);
            }
            lv_obj_t *text = lv_canvas_create(button);
            lv_obj_set_pos(text, 2, 82);
            lv_obj_set_size(text, 120, 40);
            void *text_buf = heap_caps_calloc(120 * 40, 4, MALLOC_CAP_SPIRAM);
            if (text_buf) {
              lv_canvas_set_buffer(text, text_buf, 120, 40,
                                   LV_COLOR_FORMAT_ARGB8888);
              lv_canvas_fill_bg(text, lv_color_hex(0x314A45), LV_OPA_COVER);
              const int text_w = ycb_hangul_measure(caption, 2);
              ycb_hangul_draw(text, (120 - text_w) / 2, 4,
                              lv_color_white(), lv_color_hex(0x314A45),
                              caption, 2, false);
            }
            m_aux_labels[aux_index++] = text;

            /* Keep artwork from becoming the hit target. Route input through
               a single topmost surface so the command event always fires. */
            lv_obj_t *touch = lv_obj_create(button);
            lv_obj_set_pos(touch, 0, 0);
            lv_obj_set_size(touch, 124, 136);
            lv_obj_set_style_bg_opa(touch, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(touch, 0, 0);
            lv_obj_set_style_pad_all(touch, 0, 0);
            lv_obj_clear_flag(touch, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(touch, LV_OBJ_FLAG_CLICKABLE);
            if (command == CMD_HORN_ON) {
              lv_obj_add_event_cb(touch, auxiliaryButtonEvent,
                                  LV_EVENT_PRESSED,
                                  (void *)(uintptr_t)command);
              lv_obj_add_event_cb(touch, auxiliaryButtonEvent,
                                  LV_EVENT_RELEASED,
                                  (void *)(uintptr_t)command);
              lv_obj_add_event_cb(touch, auxiliaryButtonEvent,
                                  LV_EVENT_PRESS_LOST,
                                  (void *)(uintptr_t)command);
            } else {
              lv_obj_add_event_cb(touch, auxiliaryButtonEvent,
                                  LV_EVENT_CLICKED,
                                  (void *)(uintptr_t)command);
            }
          };
          direction_button(12, 14, LV_SYMBOL_WARNING, "비상등", CMD_HAZARD_TOGGLE);
          direction_button(152, 14, LV_SYMBOL_AUDIO, "경적", CMD_HORN_ON);
          direction_button(12, 156, LV_SYMBOL_TINT, "펌프", CMD_PUMP_TOGGLE);
          direction_button(152, 156, LV_SYMBOL_REFRESH, "송풍", CMD_BLOWER_TOGGLE);
          direction_button(12, 298, LV_SYMBOL_EYE_OPEN, "전조등", CMD_HEADLIGHT_TOGGLE);
          direction_button(152, 298, LV_SYMBOL_EYE_CLOSE, "미등", CMD_PARKING_LIGHT_TOGGLE);
          ESP_LOGI("AS500UI", "AS500 direction ready");

          lv_obj_t *speed = box(m_sprayer_dashboard, 320, 136, 229, 186,
                        0xFFFFFF, 0xD6DDD8, 6);
          m_dash_speed = lv_canvas_create(speed);
          lv_obj_set_pos(m_dash_speed, 0, 8);
          lv_obj_set_size(m_dash_speed, 229, 72);
          void *speed_buf = heap_caps_calloc(229 * 72, 4, MALLOC_CAP_SPIRAM);
          if (speed_buf) {
            lv_canvas_set_buffer(m_dash_speed, speed_buf, 229, 72,
                                 LV_COLOR_FORMAT_ARGB8888);
            draw_speed_7segment(m_dash_speed, 0, false);
          } else {
            lv_obj_add_flag(m_dash_speed, LV_OBJ_FLAG_HIDDEN);
            m_dash_speed = nullptr;
          }
          label(speed, "km/h", 145, 73, 72, 18,
              &lv_font_montserrat_16, 0x71807B, LV_TEXT_ALIGN_RIGHT);
          auto distance_row = [&](int y, const char *name, lv_obj_t **canvas,
                                  bool clearable) {
            lv_obj_t *row = box(speed, 4, y, 221, 46, 0xFFFFFF, 0xE2E7E4, 2);
            label(row, name, 6, 11, 38, 20, &lv_font_montserrat_12,
                  0x71807B, LV_TEXT_ALIGN_LEFT);
            *canvas = lv_canvas_create(row);
            lv_obj_set_pos(*canvas, 45, 8);
            lv_obj_set_size(*canvas, 90, 28);
            void *buf = heap_caps_calloc(90 * 28, 4, MALLOC_CAP_SPIRAM);
            if (buf) {
              lv_canvas_set_buffer(*canvas, buf, 90, 28,
                                   LV_COLOR_FORMAT_ARGB8888);
              draw_distance_7segment(*canvas, 0);
            } else {
              lv_obj_delete(*canvas);
              *canvas = nullptr;
            }
            label(row, "km", 139, 11, 28, 20, &lv_font_montserrat_12,
                  0x71807B, LV_TEXT_ALIGN_LEFT);
            if (clearable) {
              label(row, "RESET", 169, 11, 46, 20, &lv_font_montserrat_12,
                    0xD45A4A, LV_TEXT_ALIGN_RIGHT);
              lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
              lv_obj_add_event_cb(row, tripResetEvent, LV_EVENT_CLICKED, this);
            }
          };
          distance_row(92, "ODO", &m_dash_odometer, false);
          distance_row(140, "TRIP", &m_dash_trip, true);
          ESP_LOGI("AS500UI", "AS500 speed ready");

          auto korean_gauge_title = [&](lv_obj_t *panel, const char *text) {
            lv_obj_t *title_canvas = lv_canvas_create(panel);
            lv_obj_set_pos(title_canvas, 8, 0);
            lv_obj_set_size(title_canvas, 151, 24);
            void *title_buf = heap_caps_calloc(151 * 24, 4,
                                               MALLOC_CAP_SPIRAM);
            if (!title_buf) {
              lv_obj_delete(title_canvas);
              return;
            }
            lv_canvas_set_buffer(title_canvas, title_buf, 151, 24,
                                 LV_COLOR_FORMAT_ARGB8888);
            const lv_color_t bg = lv_color_hex(0xF4F6F4);
            lv_canvas_fill_bg(title_canvas, bg, LV_OPA_TRANSP);
            const int title_width = ycb_hangul_measure(text, 1);
            ycb_hangul_draw(title_canvas, std::max(0, 151 - title_width), 4,
                            lv_color_hex(0x52635E), bg, text, 1, true);
          };

          auto level_gauge = [&](int x, int icon_type, const char *title,
                                 lv_obj_t **gauge, lv_obj_t **value) {
            lv_obj_t *panel = box(m_sprayer_dashboard, x, 326, 167, 126,
                                  0xF4F6F4, 0xD6DDD8, 0);
            *gauge = lv_canvas_create(panel);
            lv_obj_set_pos(*gauge, 0, 0);
            lv_obj_set_size(*gauge, 167, 126);
            void *gauge_buf = heap_caps_calloc(167 * 126, 4,
                                                MALLOC_CAP_SPIRAM);
            if (gauge_buf) {
              lv_canvas_set_buffer(*gauge, gauge_buf, 167, 126,
                                   LV_COLOR_FORMAT_ARGB8888);
              draw_analog_level_gauge(*gauge, -1, icon_type);
            } else {
              lv_obj_delete(*gauge);
              *gauge = nullptr;
            }
            korean_gauge_title(panel, title);
            if (icon_type == 0) {
              *value = label(panel, "--%", 105, 48, 54, 38,
                             &lv_font_montserrat_20, 0x20302C,
                             LV_TEXT_ALIGN_CENTER);
              lv_obj_set_style_border_width(*value, 2, 0);
              lv_obj_set_style_border_color(*value,
                                            lv_color_hex(0x52635E), 0);
              lv_obj_set_style_radius(*value, 2, 0);
              lv_obj_set_style_pad_top(*value, 7, 0);
            } else {
              *value = label(panel, "--%", 103, 98, 56, 22,
                             &lv_font_montserrat_14, 0x20302C,
                             LV_TEXT_ALIGN_RIGHT);
            }
            return panel;
          };
          m_dash_speed_limit_tile = level_gauge(
              324, 0, "속도제한", &m_dash_speed_limit_gauge,
              &m_dash_speed_limit_value);
          level_gauge(837, 2, "방제액", &m_dash_spray_gauge,
                      &m_dash_spray_value);
          if (m_dash_speed_limit_tile) {
            /* The gauge canvas and value label cover the tile, so use one
               transparent topmost hit surface for a reliable long press. */
            lv_obj_t *analog_settings_touch = lv_obj_create(
                m_dash_speed_limit_tile);
            lv_obj_set_pos(analog_settings_touch, 0, 0);
            lv_obj_set_size(analog_settings_touch, 167, 126);
            lv_obj_set_style_bg_opa(analog_settings_touch, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(analog_settings_touch, 0, 0);
            lv_obj_set_style_pad_all(analog_settings_touch, 0, 0);
            lv_obj_clear_flag(analog_settings_touch, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(analog_settings_touch, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_set_style_border_width(analog_settings_touch, 0, 0);
            lv_obj_set_style_border_color(
                analog_settings_touch, lv_color_hex(0x5BC0DE),
                LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_border_width(
                analog_settings_touch, 3, LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_add_event_cb(analog_settings_touch,
                                analogSettingsTouchEvent,
                                LV_EVENT_PRESSED, this);
            lv_obj_add_event_cb(analog_settings_touch,
                                analogSettingsTouchEvent,
                                LV_EVENT_RELEASED, this);
            lv_obj_add_event_cb(analog_settings_touch,
                                analogSettingsTouchEvent,
                                LV_EVENT_PRESS_LOST, this);
          }
          ESP_LOGI("AS500UI", "AS500 limit/liquid gauges ready");

          lv_obj_t *telemetry = box(m_sprayer_dashboard, 549, 136, 229, 186,
                            0xFFFFFF, 0xD6DDD8, 6);
          lv_obj_t *steering = box(telemetry, 0, 0, 229, 186,
                                   0xFFFFFF, 0xD6DDD8, 0);
          m_dash_rpm_gauge = lv_canvas_create(steering);
          lv_obj_set_pos(m_dash_rpm_gauge, 0, 0);
          lv_obj_set_size(m_dash_rpm_gauge, 229, 150);
          void *rpm_buf = heap_caps_calloc(229 * 150, 4, MALLOC_CAP_SPIRAM);
          if (rpm_buf) {
            lv_canvas_set_buffer(m_dash_rpm_gauge, rpm_buf, 229, 150,
                                 LV_COLOR_FORMAT_ARGB8888);
            draw_dashboard_dial(m_dash_rpm_gauge, 229, 150, 0, 0, 5000, true);
          } else {
            lv_obj_delete(m_dash_rpm_gauge);
            m_dash_rpm_gauge = nullptr;
          }
          label(steering, "RPM", 85, 5, 60, 22,
                &lv_font_montserrat_14, 0x52635E, LV_TEXT_ALIGN_CENTER);
          label(steering, "0", 4, 112, 54, 24,
                &lv_font_montserrat_16, 0x29312F, LV_TEXT_ALIGN_CENTER);
          label(steering, "5000", 167, 112, 58, 24,
                &lv_font_montserrat_16, 0x29312F, LV_TEXT_ALIGN_CENTER);
          m_dash_rpm_value = label(steering, "---- rpm", 69, 82, 92, 22,
                                   &lv_font_montserrat_14, 0x29312F,
                                   LV_TEXT_ALIGN_CENTER);
          m_dash_parking_brake = lv_obj_create(steering);
          lv_obj_set_pos(m_dash_parking_brake, 56, 140);
          lv_obj_set_size(m_dash_parking_brake, 36, 36);
          lv_obj_set_style_radius(m_dash_parking_brake, LV_RADIUS_CIRCLE, 0);
          lv_obj_set_style_bg_opa(m_dash_parking_brake, LV_OPA_TRANSP, 0);
          lv_obj_set_style_border_width(m_dash_parking_brake, 3, 0);
          lv_obj_set_style_border_color(m_dash_parking_brake,
                                        lv_color_hex(0x9AA6A1), 0);
          lv_obj_set_style_pad_all(m_dash_parking_brake, 0, 0);
          lv_obj_clear_flag(m_dash_parking_brake, LV_OBJ_FLAG_SCROLLABLE);
          lv_obj_t *parking_p = lv_label_create(m_dash_parking_brake);
          lv_label_set_text(parking_p, "P");
          lv_obj_set_style_text_font(parking_p, &lv_font_montserrat_20, 0);
          lv_obj_set_style_text_color(parking_p, lv_color_hex(0x9AA6A1), 0);
          lv_obj_center(parking_p);
          lv_obj_t *temp_icon = lv_canvas_create(steering);
          lv_obj_set_pos(temp_icon, 129, 140);
          lv_obj_set_size(temp_icon, 32, 36);
          void *temp_icon_buf = heap_caps_calloc(32 * 36, 4, MALLOC_CAP_SPIRAM);
          if (temp_icon_buf) {
            lv_canvas_set_buffer(temp_icon, temp_icon_buf, 32, 36,
                                 LV_COLOR_FORMAT_ARGB8888);
            draw_thermometer_icon(temp_icon);
          } else {
            lv_obj_delete(temp_icon);
          }
          m_dash_temperature = label(steering, "-- °C", 165, 147, 64, 24,
                                     &lv_font_montserrat_14, 0x6E637D,
                                     LV_TEXT_ALIGN_LEFT);

          lv_obj_t *incline = box(m_sprayer_dashboard, 778, 136, 230, 186,
                                  0xFFFFFF, 0xD6DDD8, 6);
          auto tilt_half = [&](int x, const char *name, lv_obj_t **value,
                               lv_obj_t **level, bool vertical) {
            lv_obj_t *tile = box(incline, x, 0, 115, 93,
                                 0xF8FAF8, 0xD6DDD8, 0);
            label(tile, name, 8, 2, 99, 18, &lv_font_montserrat_12,
                  0x52635E, LV_TEXT_ALIGN_CENTER);
            *level = lv_canvas_create(tile);
            lv_obj_set_pos(*level, 4, 19);
            lv_obj_set_size(*level, 107, 45);
            void *level_buf = heap_caps_calloc(107 * 45, 4,
                                               MALLOC_CAP_SPIRAM);
            if (level_buf) {
              lv_canvas_set_buffer(*level, level_buf, 107, 45,
                                   LV_COLOR_FORMAT_ARGB8888);
              draw_tilt_level(*level, 0.0f, false, vertical);
            } else {
              lv_obj_delete(*level);
              *level = nullptr;
            }
            *value = lv_canvas_create(tile);
            lv_obj_set_pos(*value, 4, 68);
            lv_obj_set_size(*value, 107, 21);
            void *value_buf = heap_caps_calloc(107 * 21, 4,
                                               MALLOC_CAP_SPIRAM);
            if (value_buf) {
              lv_canvas_set_buffer(*value, value_buf, 107, 21,
                                   LV_COLOR_FORMAT_ARGB8888);
              set_tilt_label(*value, false, 0.0f);
            } else {
              lv_obj_delete(*value);
              *value = nullptr;
            }
          };
          tilt_half(0, "PITCH", &m_dash_pitch, &m_dash_pitch_level, true);
          tilt_half(115, "ROLL", &m_dash_roll, &m_dash_roll_level, false);
          lv_obj_t *tilt_zero_button = lv_obj_create(incline);
          lv_obj_set_pos(tilt_zero_button, 0, 0);
          lv_obj_set_size(tilt_zero_button, 230, 93);
          lv_obj_set_style_bg_opa(tilt_zero_button, LV_OPA_TRANSP, 0);
          lv_obj_set_style_border_width(tilt_zero_button, 0, 0);
          lv_obj_set_style_pad_all(tilt_zero_button, 0, 0);
          lv_obj_clear_flag(tilt_zero_button, LV_OBJ_FLAG_SCROLLABLE);
          lv_obj_add_flag(tilt_zero_button, LV_OBJ_FLAG_CLICKABLE);
          lv_obj_add_event_cb(tilt_zero_button, tiltZeroEvent,
                              LV_EVENT_LONG_PRESSED, this);

          lv_obj_t *turn_signals = box(incline, 0, 93, 230, 93,
                                       0xF4F6F4, 0xD6DDD8, 0);
          m_dash_steering_gauge = lv_canvas_create(turn_signals);
          lv_obj_set_pos(m_dash_steering_gauge, 0, 0);
          lv_obj_set_size(m_dash_steering_gauge, 230, 93);
          void *small_steering_buf = heap_caps_calloc(230 * 93, 4,
                                                      MALLOC_CAP_SPIRAM);
          if (small_steering_buf) {
            lv_canvas_set_buffer(m_dash_steering_gauge, small_steering_buf,
                                 230, 93, LV_COLOR_FORMAT_ARGB8888);
            draw_dashboard_dial(m_dash_steering_gauge, 230, 93,
                                0, -90, 90, false);
          } else {
            lv_obj_delete(m_dash_steering_gauge);
            m_dash_steering_gauge = nullptr;
          }
          label(turn_signals, "STEERING", 55, 4, 120, 20,
                &lv_font_montserrat_12, 0x52635E, LV_TEXT_ALIGN_CENTER);
          m_dash_turn_left = label(turn_signals, LV_SYMBOL_LEFT,
                                   5, 0, 48, 36,
                                   &lv_font_montserrat_32, 0xAAB5B1,
                                   LV_TEXT_ALIGN_CENTER);
          m_dash_turn_right = label(turn_signals, LV_SYMBOL_RIGHT,
                                    177, 0, 48, 36,
                                    &lv_font_montserrat_32, 0xAAB5B1,
                                    LV_TEXT_ALIGN_CENTER);

          auto meter = [&](int x, const char *title, const char *initial,
                           lv_obj_t **gauge, lv_obj_t **value) {
            lv_obj_t *panel = box(m_sprayer_dashboard, x, 326, 167, 126,
                                  0xF4F6F4, 0xD6DDD8, 0);
            *gauge = lv_canvas_create(panel);
            lv_obj_set_pos(*gauge, 0, 0);
            lv_obj_set_size(*gauge, 167, 101);
            void *buf = heap_caps_calloc(167 * 101, 4, MALLOC_CAP_SPIRAM);
            if (buf) {
              lv_canvas_set_buffer(*gauge, buf, 167, 101,
                                   LV_COLOR_FORMAT_ARGB8888);
              draw_dashboard_dial(*gauge, 167, 101, 0, 0, 100, false);
            } else {
              lv_obj_delete(*gauge);
              *gauge = nullptr;
            }
            korean_gauge_title(panel, title);
            *value = label(panel, initial, 61, 101, 96, 22,
                           &lv_font_montserrat_14, 0x20302C,
                           LV_TEXT_ALIGN_RIGHT);
          };
          meter(495, "배터리", "--%  --.-V", &m_dash_battery_gauge,
                &m_dash_battery_voltage_value);
          lv_obj_t *battery_panel = m_dash_battery_gauge
              ? lv_obj_get_parent(m_dash_battery_gauge) : nullptr;
          if (battery_panel) {
            m_dash_battery_value = nullptr;
            label(battery_panel, "0", 4, 72, 30, 18,
                  &lv_font_montserrat_12, 0x52635E, LV_TEXT_ALIGN_CENTER);
            label(battery_panel, "100%", 123, 72, 40, 18,
                  &lv_font_montserrat_12, 0x52635E, LV_TEXT_ALIGN_CENTER);
            lv_obj_set_pos(m_dash_battery_voltage_value, 8, 101);
            lv_obj_set_size(m_dash_battery_voltage_value, 151, 22);
            lv_obj_set_style_text_align(m_dash_battery_voltage_value,
                                        LV_TEXT_ALIGN_RIGHT, 0);
          }
          meter(666, "전류", "--.- A", &m_dash_current_gauge,
                &m_dash_current_value);
          lv_obj_t *current_panel = m_dash_current_gauge
              ? lv_obj_get_parent(m_dash_current_gauge) : nullptr;
          if (current_panel) {
            lv_obj_add_flag(current_panel, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(current_panel, currentGaugeEvent,
                                LV_EVENT_CLICKED, this);
            m_dash_current_scale = label(current_panel, "MAX 100A", 8, 6, 76, 22,
                                         &lv_font_montserrat_12, 0x71807B,
                                         LV_TEXT_ALIGN_LEFT);
          }
          ESP_LOGI("AS500UI", "AS500 telemetry ready");

          lv_obj_t *status_bar = box(m_sprayer_dashboard, 320, 464, 688, 120,
                             0xDDE4E0, 0xD1D9D4, 6);
          auto status_item = [&](int index, const char *symbol, const char *name,
                         lv_obj_t **value) {
            const int x = 8 + index * 136;
            lv_obj_t *item = box(status_bar, x, 8, 128, 104,
                           0xF7F9F7, 0xCED7D1, 4);
            *value = label(item, symbol, 40, 10, 48, 46,
                       &lv_font_montserrat_32, 0x83918C,
                       LV_TEXT_ALIGN_CENTER);
            lv_obj_t *caption = lv_canvas_create(item);
            lv_obj_set_pos(caption, 0, 60);
            lv_obj_set_size(caption, 128, 40);
            void *caption_buf = heap_caps_calloc(128 * 40, 4, MALLOC_CAP_SPIRAM);
            if (caption_buf) {
              lv_canvas_set_buffer(caption, caption_buf, 128, 40,
                                   LV_COLOR_FORMAT_ARGB8888);
              lv_canvas_fill_bg(caption, lv_color_hex(0xF7F9F7), LV_OPA_COVER);
              const int caption_w = ycb_hangul_measure(name, 2);
              ycb_hangul_draw(caption, (128 - caption_w) / 2, 4,
                              lv_color_hex(0x465650), lv_color_hex(0xF7F9F7),
                              name, 2, false);
            }
          };
          status_item(0, LV_SYMBOL_WARNING, "비상정지", &m_dash_brake);
          m_dash_brake_tile = lv_obj_get_parent(m_dash_brake);
          m_dash_brake_caption = m_dash_brake_tile
              ? lv_obj_get_child(m_dash_brake_tile, 1) : nullptr;
          status_item(1, LV_SYMBOL_SHUFFLE, "경사", &m_dash_tilt);
          m_dash_tilt_tile = lv_obj_get_parent(m_dash_tilt);
          m_dash_tilt_caption = m_dash_tilt_tile
              ? lv_obj_get_child(m_dash_tilt_tile, 1) : nullptr;
          status_item(2, LV_SYMBOL_LOOP, "엔진팬", &m_dash_ros);
          m_dash_engine_fan_tile = lv_obj_get_parent(m_dash_ros);
          m_dash_engine_fan_caption = m_dash_engine_fan_tile
              ? lv_obj_get_child(m_dash_engine_fan_tile, 1) : nullptr;
          if (m_dash_engine_fan_tile) {
            lv_obj_add_flag(m_dash_engine_fan_tile, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_t *fan_touch = lv_obj_create(m_dash_engine_fan_tile);
            lv_obj_set_pos(fan_touch, 0, 0);
            lv_obj_set_size(fan_touch, 128, 104);
            lv_obj_set_style_bg_opa(fan_touch, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(fan_touch, 0, 0);
            lv_obj_set_style_pad_all(fan_touch, 0, 0);
            lv_obj_clear_flag(fan_touch, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(fan_touch, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(fan_touch, buttonEvent, LV_EVENT_CLICKED,
                                (void *)(uintptr_t)CMD_ENGINE_FAN_TOGGLE);
          }
          status_item(3, LV_SYMBOL_AUDIO, "MP3", &m_dash_wireless);
          m_mp3_tile = lv_obj_get_parent(m_dash_wireless);
          lv_obj_add_flag(m_mp3_tile, LV_OBJ_FLAG_CLICKABLE);
          lv_obj_add_event_cb(m_mp3_tile, mp3TileEvent, LV_EVENT_CLICKED, this);
          lv_obj_add_event_cb(m_mp3_tile, mp3TileEvent, LV_EVENT_LONG_PRESSED, this);

          lv_obj_t *settings = box(status_bar, 552, 8, 128, 104,
                           0x526D64, 0x526D64, 4);
          lv_obj_add_flag(settings, LV_OBJ_FLAG_CLICKABLE);
          label(settings, LV_SYMBOL_SETTINGS, 40, 10, 48, 46,
              &lv_font_montserrat_32, 0xFFFFFF, LV_TEXT_ALIGN_CENTER);
          lv_obj_t *settings_caption = lv_canvas_create(settings);
          lv_obj_set_pos(settings_caption, 0, 60);
          lv_obj_set_size(settings_caption, 128, 40);
          void *settings_caption_buf = heap_caps_calloc(128 * 40, 4,
                                                        MALLOC_CAP_SPIRAM);
          if (settings_caption_buf) {
            lv_canvas_set_buffer(settings_caption, settings_caption_buf,
                                 128, 40, LV_COLOR_FORMAT_ARGB8888);
            lv_canvas_fill_bg(settings_caption, lv_color_hex(0x526D64),
                              LV_OPA_COVER);
            const char *settings_text = "설정";
            const int settings_w = ycb_hangul_measure(settings_text, 2);
            ycb_hangul_draw(settings_caption, (128 - settings_w) / 2, 4,
                            lv_color_white(), lv_color_hex(0x526D64),
                            settings_text, 2, false);
          }
          lv_obj_t *settings_touch = lv_obj_create(settings);
          lv_obj_set_pos(settings_touch, 0, 0);
          lv_obj_set_size(settings_touch, 128, 104);
          lv_obj_set_style_bg_opa(settings_touch, LV_OPA_TRANSP, 0);
          lv_obj_set_style_border_width(settings_touch, 0, 0);
          lv_obj_set_style_pad_all(settings_touch, 0, 0);
          lv_obj_clear_flag(settings_touch, LV_OBJ_FLAG_SCROLLABLE);
          lv_obj_add_flag(settings_touch, LV_OBJ_FLAG_CLICKABLE);
          lv_obj_add_event_cb(settings_touch, buttonEvent, LV_EVENT_CLICKED,
                              (void *)(uintptr_t)CMD_SETTINGS);

          create_as48_settings(scr);
          createTiltSettingsOverlay(scr);
          lv_obj_move_foreground(m_sprayer_dashboard);
          ESP_LOGI("AS500UI", "AS500 dashboard ready");

        /* Compatibility objects used by update(); hidden in this layout. */
        m_led_l = lv_led_create(scr); lv_obj_add_flag(m_led_l, LV_OBJ_FLAG_HIDDEN);
        m_led_r = lv_led_create(scr); lv_obj_add_flag(m_led_r, LV_OBJ_FLAG_HIDDEN);
        m_badge = lv_label_create(scr); lv_obj_add_flag(m_badge, LV_OBJ_FLAG_HIDDEN);
        m_log = lv_label_create(scr); lv_obj_add_flag(m_log, LV_OBJ_FLAG_HIDDEN);
        m_arrow = lv_label_create(scr); lv_obj_add_flag(m_arrow, LV_OBJ_FLAG_HIDDEN);
        m_dir_txt = lv_label_create(scr); lv_obj_add_flag(m_dir_txt, LV_OBJ_FLAG_HIDDEN);
        m_arc_l = make_arc(scr, 0); lv_obj_add_flag(m_arc_l, LV_OBJ_FLAG_HIDDEN);
        m_arc_r = make_arc(scr, 0); lv_obj_add_flag(m_arc_r, LV_OBJ_FLAG_HIDDEN);
        Buzzer::instance().initialize(GPIO_NUM_1);
        Buzzer::instance().attachToTree(scr);
    }
    return;

    /* ── 스크린 배경 (다크 네이비) ──────────────────────── */
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0D1117), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* ══════════════════════════════════════════════════════
     *  헤더 바  (y=0, h=54)
     * ══════════════════════════════════════════════════════ */
    lv_obj_t *hdr = lv_obj_create(scr);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_size(hdr, DISP_W, 54);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(0x161B22), LV_PART_MAIN);
    lv_obj_set_style_border_width(hdr, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(hdr, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(hdr, 16, LV_PART_MAIN);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    /* 상단 안전 노랑 강조선 */
    lv_obj_t *hline = lv_obj_create(hdr);
    lv_obj_set_pos(hline, 0, 0);
    lv_obj_set_size(hline, DISP_W, 3);
    lv_obj_set_style_bg_color(hline, lv_color_hex(0xFFD700), LV_PART_MAIN);
    lv_obj_set_style_border_width(hline, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(hline, 0, LV_PART_MAIN);

    /* 타이틀 */
    lv_obj_t *title = lv_label_create(hdr);
    lv_label_set_text(title, LV_SYMBOL_SHUFFLE "  AS500  v1.0");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFD700), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 0, 2);

    /* UWB 라벨 */
    lv_obj_t *uwb_lbl = lv_label_create(hdr);
    lv_label_set_text(uwb_lbl, "UWB");
    lv_obj_set_style_text_color(uwb_lbl, lv_color_hex(0x8B949E), LV_PART_MAIN);
    lv_obj_set_style_text_font(uwb_lbl, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(uwb_lbl, LV_ALIGN_CENTER, -44, 0);

    /* UWB LED — L */
    m_led_l = lv_led_create(hdr);
    lv_obj_set_size(m_led_l, 14, 14);
    lv_obj_align(m_led_l, LV_ALIGN_CENTER, -12, 0);
    lv_led_set_color(m_led_l, lv_color_hex(0x2EA043));
    lv_led_off(m_led_l);

    lv_obj_t *ll = lv_label_create(hdr);
    lv_label_set_text(ll, "L");
    lv_obj_set_style_text_color(ll, lv_color_hex(0x8B949E), LV_PART_MAIN);
    lv_obj_set_style_text_font(ll, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(ll, LV_ALIGN_CENTER, 2, 0);

    /* UWB LED — R */
    m_led_r = lv_led_create(hdr);
    lv_obj_set_size(m_led_r, 14, 14);
    lv_obj_align(m_led_r, LV_ALIGN_CENTER, 22, 0);
    lv_led_set_color(m_led_r, lv_color_hex(0x2EA043));
    lv_led_off(m_led_r);

    lv_obj_t *rl = lv_label_create(hdr);
    lv_label_set_text(rl, "R");
    lv_obj_set_style_text_color(rl, lv_color_hex(0x8B949E), LV_PART_MAIN);
    lv_obj_set_style_text_font(rl, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(rl, LV_ALIGN_CENTER, 36, 0);

    /* 상태 배지 */
    m_badge = lv_label_create(hdr);
    lv_label_set_text(m_badge, "[  IDLE  ]");
    lv_obj_set_style_text_color(m_badge, lv_color_hex(0x8B949E), LV_PART_MAIN);
    lv_obj_set_style_text_font(m_badge, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_align(m_badge, LV_ALIGN_RIGHT_MID, 0, 2);

    /* ══════════════════════════════════════════════════════
     *  패널 공통 치수
     *  x : [8] pL(268) [8] pC(450) [8] pR(274) [8]  = 1024
     *  y : 62 ~ 528  (h=466)
     * ══════════════════════════════════════════════════════ */
    const lv_coord_t PY = 62, PH = 466;

    /* ── 패널 공통 스타일 적용 람다 (C++11 inline helper) ── */
    auto make_panel = [&](lv_coord_t x, lv_coord_t w, const char *title_text) -> lv_obj_t * {
        lv_obj_t *p = lv_obj_create(scr);
        lv_obj_set_pos(p, x, PY);
        lv_obj_set_size(p, w, PH);
        lv_obj_set_style_bg_color(p, lv_color_hex(0x161B22), LV_PART_MAIN);
        lv_obj_set_style_border_color(p, lv_color_hex(0x30363D), LV_PART_MAIN);
        lv_obj_set_style_border_width(p, 1, LV_PART_MAIN);
        lv_obj_set_style_radius(p, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_all(p, 0, LV_PART_MAIN);
        lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *t = lv_label_create(p);
        lv_label_set_text(t, title_text);
        lv_obj_set_style_text_color(t, lv_color_hex(0x8B949E), LV_PART_MAIN);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 10);
        return p;
    };

    /* ══════════════════════════════════════════════════════
     *  좌측 패널 — UWB SENSOR  (x=8, w=268)
     * ══════════════════════════════════════════════════════ */
    lv_obj_t *pL = make_panel(8, 268, "UWB SENSOR");

    m_arc_l = make_arc(pL, 28);
    m_val_dl = lv_label_create(pL);
    lv_label_set_text(m_val_dl, "A0: --- cm");
    lv_obj_set_style_text_color(m_val_dl, lv_color_hex(0xC9D1D9), LV_PART_MAIN);
    lv_obj_set_style_text_font(m_val_dl, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(m_val_dl, LV_ALIGN_TOP_MID, 0, 176);

    lv_obj_t *l_tag = lv_label_create(pL);
    lv_label_set_text(l_tag, "LEFT");
    lv_obj_set_style_text_color(l_tag, lv_color_hex(0x58A6FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(l_tag, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(l_tag, LV_ALIGN_TOP_MID, 0, 200);

    lv_obj_t *div_l = lv_obj_create(pL);
    lv_obj_set_pos(div_l, 16, 225); lv_obj_set_size(div_l, 236, 1);
    lv_obj_set_style_bg_color(div_l, lv_color_hex(0x30363D), LV_PART_MAIN);
    lv_obj_set_style_border_width(div_l, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(div_l, 0, LV_PART_MAIN);

    m_arc_r = make_arc(pL, 236);
    m_val_dr = lv_label_create(pL);
    lv_label_set_text(m_val_dr, "A1: --- cm");
    lv_obj_set_style_text_color(m_val_dr, lv_color_hex(0xC9D1D9), LV_PART_MAIN);
    lv_obj_set_style_text_font(m_val_dr, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(m_val_dr, LV_ALIGN_TOP_MID, 0, 384);

    lv_obj_t *r_tag = lv_label_create(pL);
    lv_label_set_text(r_tag, "RIGHT");
    lv_obj_set_style_text_color(r_tag, lv_color_hex(0x58A6FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(r_tag, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(r_tag, LV_ALIGN_TOP_MID, 0, 408);

    /* ══════════════════════════════════════════════════════
     *  중앙 패널 — HEADING  (x=284, w=450)
     * ══════════════════════════════════════════════════════ */
    lv_obj_t *pC = make_panel(284, 450, "HEADING");

    m_arrow = lv_label_create(pC);
    lv_label_set_text(m_arrow, LV_SYMBOL_PAUSE);
    lv_obj_set_style_text_color(m_arrow, lv_color_hex(0x8B949E), LV_PART_MAIN);
    lv_obj_set_style_text_font(m_arrow, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_align(m_arrow, LV_ALIGN_CENTER, 0, -70);

    m_dir_txt = lv_label_create(pC);
    lv_label_set_text(m_dir_txt, "IDLE");
    lv_obj_set_style_text_color(m_dir_txt, lv_color_hex(0x8B949E), LV_PART_MAIN);
    lv_obj_set_style_text_font(m_dir_txt, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(m_dir_txt, LV_ALIGN_CENTER, 0, -18);

    lv_obj_t *div_c = lv_obj_create(pC);
    lv_obj_set_pos(div_c, 20, 270); lv_obj_set_size(div_c, 410, 1);
    lv_obj_set_style_bg_color(div_c, lv_color_hex(0x30363D), LV_PART_MAIN);
    lv_obj_set_style_border_width(div_c, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(div_c, 0, LV_PART_MAIN);

    lv_obj_t *avg_t = lv_label_create(pC);
    lv_label_set_text(avg_t, "AVG DIST");
    lv_obj_set_style_text_color(avg_t, lv_color_hex(0x8B949E), LV_PART_MAIN);
    lv_obj_set_style_text_font(avg_t, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(avg_t, LV_ALIGN_TOP_LEFT, 36, 286);

    m_avg_val = lv_label_create(pC);
    lv_label_set_text(m_avg_val, "--- cm");
    lv_obj_set_style_text_color(m_avg_val, lv_color_hex(0xC9D1D9), LV_PART_MAIN);
    lv_obj_set_style_text_font(m_avg_val, &lv_font_montserrat_32, LV_PART_MAIN);
    lv_obj_align(m_avg_val, LV_ALIGN_TOP_LEFT, 36, 308);

    lv_obj_t *diff_t = lv_label_create(pC);
    lv_label_set_text(diff_t, "DIFF (R-L)");
    lv_obj_set_style_text_color(diff_t, lv_color_hex(0x8B949E), LV_PART_MAIN);
    lv_obj_set_style_text_font(diff_t, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(diff_t, LV_ALIGN_TOP_RIGHT, -36, 286);

    m_diff_val = lv_label_create(pC);
    lv_label_set_text(m_diff_val, "0 cm");
    lv_obj_set_style_text_color(m_diff_val, lv_color_hex(0xC9D1D9), LV_PART_MAIN);
    lv_obj_set_style_text_font(m_diff_val, &lv_font_montserrat_32, LV_PART_MAIN);
    lv_obj_align(m_diff_val, LV_ALIGN_TOP_RIGHT, -36, 308);

    /* ══════════════════════════════════════════════════════
     *  우측 패널 — MOTOR OUTPUT  (x=742, w=274)
     * ══════════════════════════════════════════════════════ */
    lv_obj_t *pR = make_panel(742, 274, "MOTOR OUTPUT");

    /* 좌측 모터 */
    lv_obj_t *lm = lv_label_create(pR);
    lv_label_set_text(lm, "L");
    lv_obj_set_style_text_color(lm, lv_color_hex(0x58A6FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(lm, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(lm, LV_ALIGN_TOP_LEFT, 14, 46);

    lv_obj_t *legacy_bar_l = lv_bar_create(pR);
    lv_obj_set_size(legacy_bar_l, 196, 22);
    lv_obj_align(legacy_bar_l, LV_ALIGN_TOP_MID, 0, 48);
    lv_bar_set_range(legacy_bar_l, 0, 1000);
    lv_bar_set_value(legacy_bar_l, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(legacy_bar_l, lv_color_hex(0x21262D), LV_PART_MAIN);
    lv_obj_set_style_bg_color(legacy_bar_l, lv_color_hex(0x2EA043), LV_PART_INDICATOR);
    lv_obj_set_style_radius(legacy_bar_l, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(legacy_bar_l, 4, LV_PART_INDICATOR);

    lv_obj_t *legacy_spd_l = lv_label_create(pR);
    lv_label_set_text(legacy_spd_l, "0");
    lv_obj_set_style_text_color(legacy_spd_l, lv_color_hex(0xC9D1D9), LV_PART_MAIN);
    lv_obj_set_style_text_font(legacy_spd_l, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(legacy_spd_l, LV_ALIGN_TOP_MID, 0, 78);

    /* 우측 모터 */
    lv_obj_t *rm = lv_label_create(pR);
    lv_label_set_text(rm, "R");
    lv_obj_set_style_text_color(rm, lv_color_hex(0x58A6FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(rm, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(rm, LV_ALIGN_TOP_LEFT, 14, 148);

    lv_obj_t *legacy_bar_r = lv_bar_create(pR);
    lv_obj_set_size(legacy_bar_r, 196, 22);
    lv_obj_align(legacy_bar_r, LV_ALIGN_TOP_MID, 0, 150);
    lv_bar_set_range(legacy_bar_r, 0, 1000);
    lv_bar_set_value(legacy_bar_r, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(legacy_bar_r, lv_color_hex(0x21262D), LV_PART_MAIN);
    lv_obj_set_style_bg_color(legacy_bar_r, lv_color_hex(0x2EA043), LV_PART_INDICATOR);
    lv_obj_set_style_radius(legacy_bar_r, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(legacy_bar_r, 4, LV_PART_INDICATOR);

    lv_obj_t *legacy_spd_r = lv_label_create(pR);
    lv_label_set_text(legacy_spd_r, "0");
    lv_obj_set_style_text_color(legacy_spd_r, lv_color_hex(0xC9D1D9), LV_PART_MAIN);
    lv_obj_set_style_text_font(legacy_spd_r, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(legacy_spd_r, LV_ALIGN_TOP_MID, 0, 180);

    /* CAN 상태 */
    lv_obj_t *div_r = lv_obj_create(pR);
    lv_obj_set_pos(div_r, 14, 228); lv_obj_set_size(div_r, 246, 1);
    lv_obj_set_style_bg_color(div_r, lv_color_hex(0x30363D), LV_PART_MAIN);
    lv_obj_set_style_border_width(div_r, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(div_r, 0, LV_PART_MAIN);

    lv_obj_t *can_t = lv_label_create(pR);
    lv_label_set_text(can_t, "CAN BUS");
    lv_obj_set_style_text_color(can_t, lv_color_hex(0x8B949E), LV_PART_MAIN);
    lv_obj_set_style_text_font(can_t, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(can_t, LV_ALIGN_TOP_MID, 0, 240);

    lv_obj_t *can_v = lv_label_create(pR);
    lv_label_set_text(can_v, can_ok ? "500 kbps  OK" : "INIT FAILED");
    lv_obj_set_style_text_color(can_v,
        can_ok ? lv_color_hex(0x2EA043) : lv_color_hex(0xDA3633), LV_PART_MAIN);
    lv_obj_set_style_text_font(can_v, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(can_v, LV_ALIGN_TOP_MID, 0, 260);

    /* HMI drive controls. E-STOP spans the full panel width. */
    auto make_button = [&](lv_coord_t x, lv_coord_t y, lv_coord_t w,
                           const char *text, control_cmd_t cmd,
                           uint32_t color) {
        lv_obj_t *btn = lv_obj_create(pR);
        register_touch_button(btn, cmd);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(btn, 0, 0);
        lv_obj_set_pos(btn, x, y);
        lv_obj_set_size(btn, w, 34);
        lv_obj_set_style_bg_color(btn, lv_color_hex(color), LV_PART_MAIN);
        lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);
        lv_obj_add_event_cb(btn, buttonEvent, LV_EVENT_CLICKED,
                            (void *)(uintptr_t)cmd);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, text);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_center(lbl);
    };

    make_button(14, 288, 246, "EMERGENCY STOP", CMD_ESTOP,       0xB62324);
    make_button(14, 328, 118, "FOLLOW START",  CMD_FOLLOW_START, 0x238636);
    make_button(142,328, 118, "STOP",          CMD_STOP,         0x9E6A03);
    make_button(14, 368, 118, LV_SYMBOL_UP "  FWD",  CMD_FORWARD, 0x1F6FEB);
    make_button(142,368, 118, LV_SYMBOL_DOWN "  REV", CMD_REVERSE, 0x1F6FEB);
    make_button(14, 408, 118, LV_SYMBOL_LEFT "  LEFT", CMD_LEFT,  0x6E40C9);
    make_button(142,408, 118, "RIGHT  " LV_SYMBOL_RIGHT, CMD_RIGHT, 0x6E40C9);

    /* ══════════════════════════════════════════════════════
     *  하단 로그 바  (y=536, h=64)
     * ══════════════════════════════════════════════════════ */
    lv_obj_t *btm = lv_obj_create(scr);
    lv_obj_set_pos(btm, 0, 536);
    lv_obj_set_size(btm, DISP_W, 64);
    lv_obj_set_style_bg_color(btm, lv_color_hex(0x161B22), LV_PART_MAIN);
    lv_obj_set_style_radius(btm, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(btm, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(btm, lv_color_hex(0x30363D), LV_PART_MAIN);
    lv_obj_set_style_border_side(btm, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_border_width(btm, 1, LV_PART_MAIN);
    lv_obj_clear_flag(btm, LV_OBJ_FLAG_SCROLLABLE);

    m_log = lv_label_create(btm);
    lv_label_set_text(m_log, LV_SYMBOL_WIFI "  Waiting for UWB data...");
    lv_obj_set_style_text_color(m_log, lv_color_hex(0x58A6FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(m_log, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(m_log, LV_ALIGN_LEFT_MID, 16, 0);
}

/* ═══════════════════════════════════════════════════════════
 *  AS500UI::update()
 * ═══════════════════════════════════════════════════════════ */
void AS500UI::update(const robot_t &d)
{
    auto &mp3 = AS500Mp3Player::instance();
    if (m_mp3_settings && !lv_obj_has_flag(m_mp3_settings, LV_OBJ_FLAG_HIDDEN) &&
        m_mp3_revision != mp3.revision())
        refreshMp3Settings();
    const int8_t mp3_visual = mp3.enabled() ? (mp3.playing() ? 2 : 1) : 0;
    if (m_mp3_tile && m_dash_wireless && m_mp3_visual != mp3_visual) {
        const uint32_t bg = mp3_visual == 2 ? 0x238636 :
                            mp3_visual == 1 ? 0x9A7B24 : 0xF7F9F7;
        const uint32_t fg = mp3_visual ? 0xFFFFFF : 0x83918C;
        lv_obj_set_style_bg_color(m_mp3_tile, lv_color_hex(bg), 0);
        lv_obj_set_style_text_color(m_dash_wireless, lv_color_hex(fg), 0);
        lv_label_set_text(m_dash_wireless,
                          mp3_visual == 2 ? LV_SYMBOL_PAUSE : LV_SYMBOL_AUDIO);
        m_mp3_visual = mp3_visual;
    }
    lv_color_t sc = stateColor(d.state);

    /* Match esp32_as_interface_v3_48V BatteryInfo_update(): sample the
       0x5A0 voltage every 200 ms, apply EMA alpha=0.1, then map the nominal
       battery voltage from 85% (empty) to 105% (full). */
    const uint32_t battery_now_ms = lv_tick_get();
    if (!d.can_rx_ok || d.can_voltage_decivolts < 10u) {
        m_battery_filter_valid = false;
    } else if (!m_battery_filter_valid) {
        m_battery_filtered_decivolts = (float)d.can_voltage_decivolts;
        m_battery_filter_tick_ms = battery_now_ms;
        m_battery_filter_valid = true;
    } else if ((uint32_t)(battery_now_ms - m_battery_filter_tick_ms) >= 200u) {
        m_battery_filtered_decivolts =
            0.1f * (float)d.can_voltage_decivolts +
            0.9f * m_battery_filtered_decivolts;
        m_battery_filter_tick_ms = battery_now_ms;
    }

    uint16_t battery_display_decivolts = 0;
    int battery_soc = 0;
    if (m_battery_filter_valid) {
        battery_display_decivolts =
            (uint16_t)lroundf(m_battery_filtered_decivolts);
        /* AS48 algorithm with 48 V nominal: 40.8 V=0%, 50.4 V=100%. */
        const float soc = (m_battery_filtered_decivolts - 408.0f) /
                          (504.0f - 408.0f) * 100.0f;
        battery_soc = (int)lroundf(soc);
        if (battery_soc < 0) battery_soc = 0;
        if (battery_soc > 100) battery_soc = 100;
    }

    /* The AS500 dashboard is a standalone tree. Do not touch the uncreated
       legacy widgets when this layout is active. */
    if (m_sprayer_dashboard) {
        const control_cmd_t fnr = s_vehicle_fnr.load();
        if (m_dash_fnr && fnr != m_dash_fnr_visual) {
            const char *text = fnr == CMD_FORWARD ? "전진" :
                               fnr == CMD_REVERSE ? "후진" : "정지";
            const uint32_t bg = fnr == CMD_FORWARD ? 0x2E7D32 :
                                fnr == CMD_REVERSE ? 0xC62828 : 0xFFFFFF;
            const uint32_t fg = fnr == CMD_NONE ? 0x20302C : 0xFFFFFF;
            if (m_dash_fnr_box) {
                lv_obj_set_style_bg_color(m_dash_fnr_box,
                                          lv_color_hex(bg), 0);
                lv_obj_set_style_border_color(m_dash_fnr_box,
                    lv_color_hex(fnr == CMD_NONE ? 0x64748B : bg), 0);
            }
            if (m_dash_fnr_title)
                lv_obj_set_style_text_color(m_dash_fnr_title,
                                            lv_color_hex(fg), 0);
            lv_canvas_fill_bg(m_dash_fnr, lv_color_hex(bg), LV_OPA_COVER);
            const int width = ycb_hangul_measure(text, 2);
            ycb_hangul_draw(m_dash_fnr, (125 - width) / 2, 4,
                            lv_color_hex(fg), lv_color_hex(bg),
                            text, 2, false);
            lv_obj_invalidate(m_dash_fnr);
            m_dash_fnr_visual = fnr;
        }
        const bool cruise_pressed = cruisePressed();
        const bool cruise_active = cruise_pressed;
        const int8_t cruise = cruise_pressed ? 1 : 0;
        if (m_dash_cruise && cruise != m_dash_cruise_visual) {
            const uint32_t bg = cruise_pressed ? 0xEC0909 : 0x526D64;
            lv_obj_set_style_bg_color(m_dash_cruise,
                lv_color_hex(bg), 0);
            if (m_dash_cruise_label) {
                const char *text = cruise_active ? "주행" : "정지";
                lv_canvas_fill_bg(m_dash_cruise_label, lv_color_hex(bg),
                                  LV_OPA_COVER);
                const int width = ycb_hangul_measure(text, 2);
                ycb_hangul_draw(m_dash_cruise_label, (117 - width) / 2, 4,
                                lv_color_white(), lv_color_hex(bg),
                                text, 2, false);
                lv_obj_invalidate(m_dash_cruise_label);
            }
            m_dash_cruise_visual = cruise;
        }
        const bool driver_status_valid = d.can_rx_ok && d.can_rx_age_ms <= 500;
        const bool pump_response = driver_status_valid &&
            (d.can_relay_status & AS500_RELAY_PUMP_MASK) != 0;
        const bool blower_response = driver_status_valid &&
            (d.can_relay_status & AS500_RELAY_BLOWER_MASK) != 0;
        const bool engine_fan_response = driver_status_valid &&
            (d.can_relay_status & AS500_RELAY_ENGINE_FAN_MASK) != 0;
        const uint8_t aux = (d.hazard_on ? 1u : 0u) |
                            (d.horn_on ? 2u : 0u) |
                            (pump_response ? 4u : 0u) |
                            (blower_response ? 8u : 0u) |
                            (d.headlight_on ? 16u : 0u) |
                            (d.parking_light_on ? 32u : 0u);
        if (aux != m_aux_visual) {
            static const char *names[6] = {
                "비상등", "경적", "펌프", "송풍", "전조등", "미등"
            };
            for (int i = 0; i < 6; ++i) {
                const bool on = (aux & (1u << i)) != 0;
                const uint32_t bg = on ? 0xEC0909 : 0x314A45;
                if (m_aux_buttons[i])
                    lv_obj_set_style_bg_color(m_aux_buttons[i], lv_color_hex(bg), 0);
                if (m_aux_labels[i]) {
                    lv_canvas_fill_bg(m_aux_labels[i], lv_color_hex(bg), LV_OPA_COVER);
                    const int width = ycb_hangul_measure(names[i], 2);
                    ycb_hangul_draw(m_aux_labels[i], (120 - width) / 2, 4,
                                    lv_color_white(), lv_color_hex(bg),
                                    names[i], 2, false);
                    lv_obj_invalidate(m_aux_labels[i]);
                }
            }
            m_aux_visual = aux;
        }
        const int8_t engine_fan_visual = engine_fan_response ? 1 : 0;
        if (m_dash_engine_fan_tile && m_dash_ros &&
            m_dash_engine_fan_visual != engine_fan_visual) {
            const lv_color_t bg = lv_color_hex(
                engine_fan_response ? 0xD64545 : 0xF7F9F7);
            lv_obj_set_style_bg_color(m_dash_engine_fan_tile, bg, 0);
            lv_obj_set_style_border_color(m_dash_engine_fan_tile,
                lv_color_hex(engine_fan_response ? 0xA62F2F : 0xCED7D1), 0);
            lv_obj_set_style_text_color(m_dash_ros,
                lv_color_hex(engine_fan_response ? 0xFFFFFF : 0x83918C), 0);
            if (m_dash_engine_fan_caption) {
                lv_canvas_fill_bg(m_dash_engine_fan_caption, bg, LV_OPA_COVER);
                const char *caption = "엔진팬";
                const int caption_w = ycb_hangul_measure(caption, 2);
                ycb_hangul_draw(m_dash_engine_fan_caption,
                                (128 - caption_w) / 2, 4,
                                lv_color_hex(engine_fan_response
                                    ? 0xFFFFFF : 0x465650),
                                bg, caption, 2, false);
                lv_obj_invalidate(m_dash_engine_fan_caption);
            }
            m_dash_engine_fan_visual = engine_fan_visual;
        }
        if (m_dash_can) {
            lv_label_set_text(m_dash_can, d.can_rx_ok ? "CAN  ONLINE" : "CAN  FAIL");
            lv_obj_set_style_text_color(m_dash_can,
                lv_color_hex(d.can_rx_ok ? 0x9ED5B8 : 0xF2A7A0), 0);
        }
        if (m_dash_speed) {
            if (d.can_motion_valid) {
                /* Same calculation previously used by AS500-interface.  The
                   P4 receives packed RPM directly from STM32 ID 0x5B8. */
                const float avg_rpm =
                    (fabsf((float)d.can_motor1_rpm) +
                     fabsf((float)d.can_motor2_rpm)) * 0.5f;
                float speed_kmh = avg_rpm * 3.14159265358979323846f * 0.25f /
                                   10.0f * 60.0f / 1000.0f *
                                   as48SettingValue(13);
                if (d.can_motion_brake_applied) speed_kmh = 0.0f;
                const int speed_tenths = (int)lroundf(speed_kmh * 10.0f);
                draw_speed_7segment(m_dash_speed, speed_tenths, true);
                const uint32_t distance_now = lv_tick_get();
                if (m_dash_distance_tick_ms != 0) {
                    const uint32_t elapsed_ms = std::min<uint32_t>(
                        distance_now - m_dash_distance_tick_ms, 1000u);
                    const double delta_km = (double)speed_kmh *
                                            (double)elapsed_ms / 3600000.0;
                    m_dash_odometer_km += delta_km;
                    m_dash_trip_km += delta_km;
                }
                m_dash_distance_tick_ms = distance_now;
            } else {
                draw_speed_7segment(m_dash_speed, 0, false);
                m_dash_distance_tick_ms = lv_tick_get();
            }
        }
        draw_distance_7segment(m_dash_odometer,
            (uint32_t)lround(m_dash_odometer_km * 10.0));
        draw_distance_7segment(m_dash_trip,
            (uint32_t)lround(m_dash_trip_km * 10.0));
        float steering_degrees = d.steering_angle_deg;
        while (steering_degrees > 180.0f) steering_degrees -= 360.0f;
        while (steering_degrees < -180.0f) steering_degrees += 360.0f;
        if (!d.steering_valid) {
            steering_degrees =
                ((float)d.can_command_right - (float)d.can_command_left) *
                90.0f / 2000.0f;
        }
        steering_degrees = std::clamp(steering_degrees, -90.0f, 90.0f);
        if (m_dash_steering_gauge)
            draw_dashboard_dial(m_dash_steering_gauge, 230, 93,
                                steering_degrees, -90, 90, false);
        const int average_rpm = d.can_motion_valid
            ? std::clamp((abs((int)d.can_motor1_rpm) +
                          abs((int)d.can_motor2_rpm)) / 2, 0, 5000)
            : 0;
        if (m_dash_rpm_gauge)
            draw_dashboard_dial(m_dash_rpm_gauge, 229, 150,
                                average_rpm, 0, 5000, true);
        if (m_dash_rpm_value) {
            if (d.can_motion_valid)
                lv_label_set_text_fmt(m_dash_rpm_value, "%d rpm", average_rpm);
            else
                lv_label_set_text(m_dash_rpm_value, "---- rpm");
        }
        const int battery_gauge_soc = m_battery_filter_valid ? battery_soc : -1;
        const int battery_warning_soc = std::clamp(
            (int)lroundf(as48SettingValue(19)), 0, 100);
        if (m_dash_battery_gauge &&
            (m_dash_battery_gauge_soc != battery_gauge_soc ||
             m_dash_battery_gauge_warning != battery_warning_soc)) {
            draw_dashboard_dial(m_dash_battery_gauge, 167, 101,
                                battery_gauge_soc >= 0 ? battery_gauge_soc : 0,
                                0, 100, false, battery_warning_soc);
            m_dash_battery_gauge_soc = (int16_t)battery_gauge_soc;
            m_dash_battery_gauge_warning = (int16_t)battery_warning_soc;
        }
        const uint16_t battery_text_voltage = m_battery_filter_valid
            ? battery_display_decivolts : 0xFFFFu;
        if (m_dash_battery_voltage_value &&
            (m_dash_battery_text_soc != battery_gauge_soc ||
             m_dash_battery_text_voltage != battery_text_voltage)) {
            if (m_battery_filter_valid)
                lv_label_set_text_fmt(m_dash_battery_voltage_value,
                                      "%d%%  %u.%uV", battery_soc,
                                      battery_display_decivolts / 10,
                                      battery_display_decivolts % 10);
            else
                lv_label_set_text(m_dash_battery_voltage_value, "--%  --.-V");
            m_dash_battery_text_soc = (int16_t)battery_gauge_soc;
            m_dash_battery_text_voltage = battery_text_voltage;
        }
        if (m_dash_current_gauge)
            draw_dashboard_dial(m_dash_current_gauge, 167, 101,
                                d.can_rx_ok ? d.can_current_deciamps : 0,
                                0, m_dash_current_max_amps * 10, false);
        if (m_dash_current_value) {
            if (d.can_rx_ok)
                lv_label_set_text_fmt(m_dash_current_value, "%u.%u A",
                    d.can_current_deciamps / 10, d.can_current_deciamps % 10);
            else
                lv_label_set_text(m_dash_current_value, "--.- A");
        }
        if (m_dash_temperature) {
            if (d.can_rx_ok)
                lv_label_set_text_fmt(m_dash_temperature, "%d °C",
                                      (int)d.can_fet_temperature_c);
            else
                lv_label_set_text(m_dash_temperature, "-- °C");
        }
        const int8_t parking_brake_visual =
            (d.can_motion_valid && d.can_motion_brake_applied) ? 1 : 0;
        if (m_dash_parking_brake &&
            m_dash_parking_brake_visual != parking_brake_visual) {
            const lv_color_t brake_color = lv_color_hex(
                parking_brake_visual ? 0xE32626 : 0x9AA6A1);
            lv_obj_set_style_border_color(m_dash_parking_brake,
                                          brake_color, 0);
            lv_obj_t *parking_p = lv_obj_get_child(m_dash_parking_brake, 0);
            if (parking_p)
                lv_obj_set_style_text_color(parking_p, brake_color, 0);
            m_dash_parking_brake_visual = parking_brake_visual;
        }
        m_dash_imu_valid = d.imu_valid;
        m_dash_pitch_deg = d.imu_pitch_deg;
        m_dash_roll_deg = d.imu_roll_deg;
        set_tilt_label(m_dash_pitch, d.imu_valid, d.imu_pitch_deg);
        set_tilt_label(m_dash_roll, d.imu_valid, d.imu_roll_deg);
        draw_tilt_level(m_dash_pitch_level, d.imu_pitch_deg,
                        d.imu_valid, true);
        draw_tilt_level(m_dash_roll_level, d.imu_roll_deg,
                        d.imu_valid, false);
        const uint8_t new_analog_raw[kAnalogChannelCount] = {
            d.interface_throttle1, d.interface_throttle2,
            d.interface_vr1, d.interface_vr2
        };
        const bool analog_raw_changed =
            memcmp(s_analog_raw, new_analog_raw, sizeof(s_analog_raw)) != 0;
        memcpy(s_analog_raw, new_analog_raw, sizeof(s_analog_raw));
        memcpy(m_analog_raw, new_analog_raw, sizeof(m_analog_raw));
        if (analog_raw_changed && m_analog_settings &&
            !lv_obj_has_flag(m_analog_settings, LV_OBJ_FLAG_HIDDEN))
            refreshAnalogSettings();

        const bool speed_limit_valid = d.interface_analog_valid;
        const uint8_t calibrated_vr1 = calibratedAnalogValue(2,
                                                              d.interface_vr1);
        const int speed_limit_percent = speed_limit_valid
            ? ((int)calibrated_vr1 * 100 + 127) / 255 : 0;
        draw_analog_level_gauge(m_dash_speed_limit_gauge,
                                speed_limit_valid ? speed_limit_percent : -1,
                                0);
        if (m_dash_speed_limit_value) {
            if (speed_limit_valid)
                lv_label_set_text_fmt(m_dash_speed_limit_value, "%d%%",
                                      speed_limit_percent);
            else
                lv_label_set_text(m_dash_speed_limit_value, "--%");
        }
        const uint8_t calibrated_vr2 = calibratedAnalogValue(3,
                                                              d.interface_vr2);
        const int spray_percent = d.interface_analog_valid
            ? ((int)calibrated_vr2 * 100 + 127) / 255 : 0;
        draw_analog_level_gauge(m_dash_spray_gauge,
                                d.interface_analog_valid ? spray_percent : -1,
                                2);
        if (m_dash_spray_value) {
            if (d.interface_analog_valid)
                lv_label_set_text_fmt(m_dash_spray_value, "%d%%", spray_percent);
            else
                lv_label_set_text(m_dash_spray_value, "--%");
        }
        const bool output_valid = d.interface_analog_valid;
        const bool left_signal = output_valid &&
            (d.interface_gpio_output_state &
             AS500_IO_OUTPUT_LEFT_SIGNAL_MASK) != 0;
        const bool right_signal = output_valid &&
            (d.interface_gpio_output_state &
             AS500_IO_OUTPUT_RIGHT_SIGNAL_MASK) != 0;
        if (m_dash_turn_left)
            lv_obj_set_style_text_color(m_dash_turn_left,
                lv_color_hex(left_signal ? 0x20B85A : 0xAAB5B1), 0);
        if (m_dash_turn_right)
            lv_obj_set_style_text_color(m_dash_turn_right,
                lv_color_hex(right_signal ? 0x20B85A : 0xAAB5B1), 0);
        if (m_dash_tilt_tile && m_dash_tilt_alarm_visual != (int8_t)d.tilt_alarm) {
            const bool alarm = d.tilt_alarm != 0;
            const lv_color_t bg = lv_color_hex(alarm ? 0xD64545 : 0xF7F9F7);
            const lv_color_t fg = lv_color_hex(alarm ? 0xFFFFFF : 0x83918C);
            lv_obj_set_style_bg_color(m_dash_tilt_tile, bg, 0);
            lv_obj_set_style_border_color(m_dash_tilt_tile,
                                          lv_color_hex(alarm ? 0xA62F2F : 0xCED7D1), 0);
            lv_obj_set_style_text_color(m_dash_tilt, fg, 0);
            if (m_dash_tilt_caption) {
                lv_canvas_fill_bg(m_dash_tilt_caption, bg, LV_OPA_COVER);
                const char *caption = "경사";
                const int caption_w = ycb_hangul_measure(caption, 2);
                ycb_hangul_draw(m_dash_tilt_caption, (128 - caption_w) / 2, 4,
                                lv_color_hex(alarm ? 0xFFFFFF : 0x465650), bg,
                                caption, 2, false);
                lv_obj_invalidate(m_dash_tilt_caption);
            }
            m_dash_tilt_alarm_visual = (int8_t)d.tilt_alarm;
        }
        if (m_dash_brake_tile && m_dash_brake) {
            const bool emergency = d.can_emergency || d.gpio_emergency;
            const bool blink_on = ((esp_timer_get_time() / 500000) & 1) == 0;
            const int8_t visual = emergency ? (blink_on ? 1 : 2) : 0;
            if (m_dash_brake_visual != visual) {
                const bool alarm_on = visual == 1;
                const lv_color_t bg = lv_color_hex(alarm_on ? 0xD64545 : 0xF7F9F7);
                lv_obj_set_style_bg_color(m_dash_brake_tile, bg, 0);
                lv_obj_set_style_border_color(m_dash_brake_tile,
                    lv_color_hex(alarm_on ? 0xA62F2F : 0xCED7D1), 0);
                lv_obj_set_style_text_color(m_dash_brake,
                    lv_color_hex(alarm_on ? 0xFFFFFF : 0x83918C), 0);
                if (m_dash_brake_caption) {
                    lv_canvas_fill_bg(m_dash_brake_caption, bg, LV_OPA_COVER);
                    const char *caption = "비상정지";
                    const int caption_w = ycb_hangul_measure(caption, 2);
                    ycb_hangul_draw(m_dash_brake_caption,
                                    (128 - caption_w) / 2, 4,
                                    lv_color_hex(alarm_on ? 0xFFFFFF : 0x465650),
                                    bg, caption, 2, false);
                    lv_obj_invalidate(m_dash_brake_caption);
                }
                m_dash_brake_visual = visual;
            }
        }
        if (m_drive_icon) {
            const int left = d.can_command_left;
            const int right = d.can_command_right;
            const char *symbol = LV_SYMBOL_PAUSE;
            if (left > 50 && right > 50) symbol = LV_SYMBOL_UP;
            else if (left < -50 && right < -50) symbol = LV_SYMBOL_DOWN;
            else if (right > left + 50) symbol = LV_SYMBOL_LEFT;
            else if (left > right + 50) symbol = LV_SYMBOL_RIGHT;
            lv_label_set_text(m_drive_icon, symbol);
        }
        return;
    }

    // CAN reports the driver's relay command, not physical brake feedback.
    const int brake_state = (!d.can_rx_ok || d.can_rx_age_ms > 500) ? 0 :
                            (d.can_brake_applied ? 1 : 2);
    if (m_brake_status && m_brake_visual_state != brake_state) {
        lv_label_set_text(m_brake_status, brake_state == 1 ? "EMB (DRV): APPLIED" :
                         brake_state == 2 ? "EMB (DRV): RELEASED" : "EMB (DRV): UNKNOWN");
        lv_obj_set_style_text_color(m_brake_status, lv_color_hex(
            brake_state == 1 ? 0xFFD166 : brake_state == 2 ? 0x8BD5A5 : 0xFF6969), 0);
        m_brake_visual_state = brake_state;
    }

    const bool battery_changed = !m_battery_visual_initialized ||
                                 m_battery_visual_valid != m_battery_filter_valid ||
                                 (m_battery_filter_valid &&
                                  (m_battery_visual_voltage !=
                                       battery_display_decivolts ||
                                   m_battery_visual_current !=
                                       d.can_current_deciamps));
    if (m_battery_voltage && m_battery_bar && battery_changed) {
        if (m_battery_filter_valid) {
            lv_label_set_text_fmt(m_battery_voltage,
                                  "%u.%uV   %u.%uA",
                                  battery_display_decivolts / 10,
                                  battery_display_decivolts % 10,
                                  d.can_current_deciamps / 10,
                                  d.can_current_deciamps % 10);
            lv_bar_set_value(m_battery_bar, battery_soc, LV_ANIM_OFF);
            lv_obj_set_style_text_color(m_battery_voltage,
                                         lv_color_hex(0xD7F3E1), 0);
        } else {
            lv_label_set_text(m_battery_voltage, "--.-V   --.-A");
            lv_bar_set_value(m_battery_bar, 0, LV_ANIM_OFF);
            lv_obj_set_style_text_color(m_battery_voltage,
                                         lv_color_hex(0xFF6969), 0);
        }
        m_battery_visual_initialized = true;
        m_battery_visual_valid = m_battery_filter_valid;
        m_battery_visual_voltage = battery_display_decivolts;
        m_battery_visual_current = d.can_current_deciamps;
    }

    /* UWB LED */
    (d.mask & 0x01) ? lv_led_on(m_led_l) : lv_led_off(m_led_l);
    (d.mask & 0x02) ? lv_led_on(m_led_r) : lv_led_off(m_led_r);

    /* 상태 배지 */
    lv_label_set_text_fmt(m_badge, "[ %s ]", stateText(d.state));
    lv_obj_set_style_text_color(m_badge, sc, LV_PART_MAIN);

    /* LEFT arc */
    {
        int v = d.dL > 300 ? 300 : d.dL;
        lv_arc_set_value(m_arc_l, v);
        lv_color_t c = d.dL <= kStopDist   ? lv_color_hex(0xDA3633) :
                        d.dL <= kFollowDist  ? lv_color_hex(0xE3B341) :
                                               lv_color_hex(0x2EA043);
        lv_obj_set_style_arc_color(m_arc_l, c, LV_PART_INDICATOR);
        lv_label_set_text_fmt(m_val_dl, "%4d", d.dL);
        lv_obj_set_style_text_color(m_val_dl, lv_color_hex(
            (d.mask & 1) && d.remote_age_ms <= 300 ? 0xFFFFFF : 0xE3B341), 0);
    }

    /* RIGHT arc */
    {
        int v = d.dR > 300 ? 300 : d.dR;
        lv_arc_set_value(m_arc_r, v);
        lv_color_t c = d.dR <= kStopDist   ? lv_color_hex(0xDA3633) :
                        d.dR <= kFollowDist  ? lv_color_hex(0xE3B341) :
                                               lv_color_hex(0x2EA043);
        lv_obj_set_style_arc_color(m_arc_r, c, LV_PART_INDICATOR);
        lv_label_set_text_fmt(m_val_dr, "%4d", d.dR);
        lv_obj_set_style_text_color(m_val_dr, lv_color_hex(
            (d.mask & 2) && d.remote_age_ms <= 300 ? 0xFFFFFF : 0xE3B341), 0);
    }

    // The display uses the exact filtered estimate used by the drive controller.
    int diff = d.dR - d.dL;
    float center_dist = d.target_distance_cm;
    float angle_deg = d.target_angle_deg;
    int avg = (int)lroundf(center_dist);
    const bool pose_available = d.target_pose_valid &&
        (d.mask & 3) == 3 && d.remote_age_ms <= 300;
    const bool pose_fresh = pose_available && !d.target_ranges_held;
    const int64_t pose_now_us = esp_timer_get_time();
    if (pose_available) {
        m_pose_display_us = pose_now_us - (int64_t)d.remote_age_ms * 1000;
        m_pose_display_distance = center_dist;
        m_pose_display_angle = angle_deg;
    }
    const bool pose_held = pose_available && d.target_ranges_held;
    const bool uwb_position_valid = pose_available;
    if (m_pose_quality) {
        const char *reason = d.target_range_pair_error ? "RANGE PAIR ERROR" :
            d.target_geometry_shadow ? "FRONT VALIDITY LIMIT" :
            pose_fresh ? (d.target_pose_corrected ? "TAG CORRECTED" : "TAG OK") :
            d.target_ranges_held ? "PREVIOUS PAIR" :
            d.remote_age_ms > 300 ? "TAG TIMEOUT" :
            (d.mask & 3) == 0 ? "A0 / A1 LOST" :
            !(d.mask & 1) ? "A0 LOST" :
            !(d.mask & 2) ? "A1 LOST" : "RANGE GEOMETRY ERROR";
        lv_label_set_text_fmt(m_pose_quality, "%s%s", reason,
                              pose_held ? " (HOLD)" : "");
        lv_obj_set_style_text_color(m_pose_quality,
            lv_color_hex(pose_fresh ? 0x79C9C1 : 0xE3B341), 0);
    }
    if (pose_held) {
        center_dist = m_pose_display_distance;
        angle_deg = m_pose_display_angle;
    }
    if (m_tag_distance && m_tag_angle) {
        if (uwb_position_valid) {
            lv_label_set_text_fmt(m_tag_distance, "%4d", (int)lroundf(center_dist));
            char angle_text_fixed[16];
            snprintf(angle_text_fixed, sizeof(angle_text_fixed), "%+5.1f",
                     (double)angle_deg);
            lv_label_set_text(m_tag_angle, angle_text_fixed);
        } else {
            lv_label_set_text(m_tag_distance, " ---");
            lv_label_set_text(m_tag_angle, "  ---");
        }
    }
    if (m_serial_command) {
        if (d.uart_command_valid) {
            if (d.uart_command_type == 2) {
                lv_label_set_text_fmt(m_serial_command, "UART D,%d,%d,%u",
                                      d.uart_command_left,
                                      d.uart_command_right,
                                      (unsigned)d.uart_command_mask);
            } else {
                lv_label_set_text_fmt(m_serial_command, "UART M,%d,%d,%u",
                                      d.uart_command_left,
                                      d.uart_command_right,
                                      d.uart_command_enable ? 1u : 0u);
            }
            lv_obj_set_style_text_color(m_serial_command,
                                        lv_color_hex(0x58D6E8), 0);
        } else {
            lv_label_set_text(m_serial_command, "UART: WAIT");
            lv_obj_set_style_text_color(m_serial_command,
                                        lv_color_hex(0x8B949E), 0);
        }
    }
    if (m_tag_remote) {
        lv_label_set_text_fmt(m_tag_remote,
            "KEY:%02X  %ums  %ddBm",
            d.remote_keys, d.remote_age_ms, d.remote_rssi);
        lv_obj_set_style_text_color(m_tag_remote, lv_color_hex(0xFFFFFF), 0);
    }
    char distance_text[32];
    char angle_text[32];
    snprintf(distance_text, sizeof(distance_text), "%.0f +/-%.0f cm",
             (double)center_dist,(double)d.target_distance_error_cm);
    snprintf(angle_text, sizeof(angle_text), "%+.1f +/-%.1f deg",
             (double)angle_deg,(double)d.target_angle_error_deg);
    lv_label_set_text(m_avg_val, uwb_position_valid ? distance_text : "-- cm");
    lv_label_set_text(m_diff_val, uwb_position_valid ? angle_text : "-- deg");

    /* Keep the TAG marker at its previous dynamic polar position. */
    if (m_dir_target) {

        int radius = (int)lroundf(fminf(center_dist, 500.0f) * 225.0f / 500.0f);
        float target_angle = (angle_deg + 180.0f) * 0.01745329252f;
        // Frame-buffer X is reversed by the physically 180-degree mounted LCD.
        // This makes +bearing appear on the vehicle's right, matching the
        // simulator and the steering calculation.
        int target_x = 256 + (int)lroundf(sinf(target_angle) * radius);
        int target_y = 225 - (int)lroundf(cosf(target_angle) * radius);
        int marker_x = target_x - 22;
        int marker_y = target_y - 15;
        // Polar radius already fits the radar; keep the 180-degree transform symmetric.
        lv_obj_set_pos(m_dir_target, marker_x, marker_y);
        if (uwb_position_valid) lv_obj_clear_flag(m_dir_target, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(m_dir_target, LV_OBJ_FLAG_HIDDEN);
        if (m_dir_line) {
            if (uwb_position_valid) lv_obj_clear_flag(m_dir_line, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(m_dir_line, LV_OBJ_FLAG_HIDDEN);
            set_line_segment(m_dir_line, m_dir_line_points,
                             256, 225, marker_x + 22, marker_y + 15);
        }
    }

    /* LiDAR refresh remains independent of the TAG marker. */
    {
        constexpr int lidar_x = 256;
        constexpr int lidar_y = 225;
        constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
        const int threshold_a = (int)lroundf(settingValue(10) * 10.0f);
        const int threshold_b = (int)lroundf(settingValue(11) * 10.0f);
        const int stop_mm = threshold_a < threshold_b ? threshold_a : threshold_b;
        const int warning_mm = threshold_a > threshold_b ? threshold_a : threshold_b;
        const int lidar_min_confidence = (int)lroundf(settingValue(18));
        const float detect_center = settingValue(12);
        const float detect_half_width = settingValue(13) * 0.5f;
        const int lidar_max_distance_mm =
            (int)lroundf(settingValue(20) * 10.0f);
        const float vehicle_angle = m_vehicle_angle_tenths / 10.0f;
        const float visible_center = vehicle_angle + detect_center;
        auto angle_distance = [](float a, float b) {
            float delta = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
            return fabsf(delta);
        };
        /* The configured detection sector is mounted to the vehicle and
           therefore follows its rotation. LiDAR points themselves remain in
           the scan/display frame and are only clipped by this sector. */
        for (int side = 0; side < 2; ++side) {
            if (!m_lidar_range_lines[side]) continue;
            const float boundary_angle =
                (visible_center +
                 (side == 0 ? -detect_half_width : detect_half_width) + 180.0f) *
                kDegToRad;
            const int line_x = lidar_x +
                (int)lroundf(sinf(boundary_angle) * kLidarFiveMeterRadius);
            const int line_y = lidar_y -
                (int)lroundf(cosf(boundary_angle) * kLidarFiveMeterRadius);
            set_line_segment(m_lidar_range_lines[side],
                             m_lidar_range_line_points[side],
                             lidar_x, lidar_y, line_x, line_y);
        }
        for (int i = 0; i < 36; ++i) {
            lv_obj_t *point = m_lidar_points[i];
            if (!point) continue;
            uint16_t distance_mm = d.lidar_distance_mm[i];
            float sector_angle = i * 10.0f + 5.0f;
            const float angle_delta = angle_distance(sector_angle,
                                                     visible_center);
            if (!d.lidar_ok || !distance_mm ||
                d.lidar_confidence[i] < lidar_min_confidence ||
                angle_delta > detect_half_width) {
                lv_obj_add_flag(point, LV_OBJ_FLAG_HIDDEN);
                continue;
            }
            /* The rings use an absolute scale: 3 m = 135 px, 5 m = 225 px.
               Measurements beyond the configured maximum remain visible but
               are pinned to that maximum-distance position. */
            const int plotted_distance_mm = distance_mm > lidar_max_distance_mm
                ? lidar_max_distance_mm : distance_mm;
            int radius = plotted_distance_mm * kLidarFiveMeterRadius / 5000;
            if (radius < 30) radius = 30;
            float angle = (sector_angle + 180.0f) * kDegToRad;
            int px = lidar_x + (int)lroundf(sinf(angle) * radius) - 3;
            int py = lidar_y - (int)lroundf(cosf(angle) * radius) - 3;
            lv_obj_set_pos(point, px, py);
            uint32_t color = distance_mm <= stop_mm ? 0xDA3633 :
                             distance_mm <= warning_mm ? 0xE3B341 : 0x58A6FF;
            lv_obj_set_style_bg_color(point, lv_color_hex(color), 0);
            lv_obj_clear_flag(point, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_t *nearest_ray = m_lidar_rays[0];
        const bool nearest_in_sector =
            angle_distance((float)d.lidar_nearest_angle_deg,
                           visible_center) <= detect_half_width;
        if (nearest_ray && d.lidar_ok && d.lidar_nearest_mm &&
            d.lidar_nearest_mm <= lidar_max_distance_mm &&
            nearest_in_sector) {
            int radius = 30 + d.lidar_nearest_mm *
                (kLidarDisplayRadius - 30) / lidar_max_distance_mm;
            float angle = (d.lidar_nearest_angle_deg + 180.0f) * kDegToRad;
            const int ray_x = lidar_x + (int)lroundf(sinf(angle) * radius);
            const int ray_y = lidar_y - (int)lroundf(cosf(angle) * radius);
            set_line_segment(nearest_ray, m_lidar_ray_points[0],
                             lidar_x, lidar_y, ray_x, ray_y);
            uint32_t color = d.lidar_nearest_mm <= stop_mm ? 0xDA3633 :
                             d.lidar_nearest_mm <= warning_mm ? 0xE3B341 : 0x58A6FF;
            lv_obj_set_style_line_color(nearest_ray, lv_color_hex(color), 0);
            lv_obj_set_style_line_opa(nearest_ray, LV_OPA_70, 0);
            lv_obj_clear_flag(nearest_ray, LV_OBJ_FLAG_HIDDEN);
        } else if (nearest_ray) {
            lv_obj_add_flag(nearest_ray, LV_OBJ_FLAG_HIDDEN);
        }
        if (m_lidar_nearest) {
            int text_x = 344;
            int text_y = 207;
            lv_obj_set_pos(m_lidar_nearest, text_x, text_y);
            if (d.lidar_ok && d.lidar_nearest_mm && nearest_in_sector) {
                int angle = -d.lidar_nearest_angle_deg;
                if (angle < -180) angle += 360;
                lv_label_set_text_fmt(m_lidar_nearest, "%u cm\n%+d deg",
                                      d.lidar_nearest_mm / 10, angle);
                lv_obj_set_style_text_color(m_lidar_nearest,
                    lv_color_hex(d.lidar_nearest_mm <= stop_mm ? 0xDA3633 :
                                 d.lidar_nearest_mm <= warning_mm ? 0xE3B341 :
                                                                   0x58A6FF), 0);
            } else {
                lv_label_set_text(m_lidar_nearest, "NO LIDAR");
                lv_obj_set_style_text_color(m_lidar_nearest,
                                             lv_color_hex(0x8B949E), 0);
            }
        }
    }

    const char *dir_symbol = LV_SYMBOL_WARNING;
    const char *dir_text = "NO UWB";
    lv_color_t dir_color = lv_color_hex(0xDA3633);
    if (uwb_position_valid) {
        if (angle_deg < -2.0f) {
            dir_symbol = LV_SYMBOL_LEFT;
            dir_text = "TARGET LEFT";
            dir_color = lv_color_hex(0x58A6FF);
        } else if (angle_deg > 2.0f) {
            dir_symbol = LV_SYMBOL_RIGHT;
            dir_text = "TARGET RIGHT";
            dir_color = lv_color_hex(0x58A6FF);
        } else {
            dir_symbol = LV_SYMBOL_UP;
            dir_text = "TARGET CENTER";
            dir_color = lv_color_hex(0x2EA043);
        }
    }
    lv_label_set_text(m_arrow, dir_symbol);
    lv_obj_set_style_text_color(m_arrow, dir_color, LV_PART_MAIN);
    lv_label_set_text(m_dir_txt, dir_text);
    lv_obj_set_style_text_color(m_dir_txt, dir_color, LV_PART_MAIN);

    /* Display the normalized target values sent by ESP32-P4 over 0x701. */
    int pct_l = d.can_command_left / 10;
    int pct_r = d.can_command_right / 10;
    if (pct_l > 100) pct_l = 100;
    if (pct_l < -100) pct_l = -100;
    if (pct_r > 100) pct_r = 100;
    if (pct_r < -100) pct_r = -100;
    const int drive_pct = (pct_l + pct_r) / 2;
    const int turn_pct = (pct_r - pct_l) / 2;
    const char *drive_symbol = LV_SYMBOL_PAUSE;
    uint32_t drive_color = 0x8B949E;
    if (abs(turn_pct) > abs(drive_pct) && abs(turn_pct) > 5) {
        drive_symbol = turn_pct > 0 ? LV_SYMBOL_LEFT : LV_SYMBOL_RIGHT;
        drive_color = 0xB78CFF;
    } else if (drive_pct > 5) {
        drive_symbol = LV_SYMBOL_UP;
        drive_color = 0x49C779;
    } else if (drive_pct < -5) {
        drive_symbol = LV_SYMBOL_DOWN;
        drive_color = 0xFF5A5F;
    }
    if (m_drive_status) {
        lv_label_set_text(m_drive_icon, drive_symbol);
        lv_obj_set_style_text_color(m_drive_icon, lv_color_hex(drive_color), 0);
        const uint32_t pose_color = uwb_position_valid ? 0xFFFFFF : 0x8B949E;
        lv_obj_set_style_text_color(m_drive_distance, lv_color_hex(pose_color), 0);
        lv_obj_set_style_text_color(m_drive_status, lv_color_hex(pose_color), 0);
        lv_obj_set_style_border_color(m_drive_card, lv_color_hex(drive_color), 0);
        lv_label_set_text_fmt(m_drive_l_pwm, "%+d", pct_l);
        lv_label_set_text_fmt(m_drive_r_pwm, "%+d", pct_r);
    }

    // Refresh target telemetry independently of the vehicle artwork.
    if (m_drive_distance && m_drive_status) {
        char drive_distance_text[16];
        char drive_angle_text[16];
        if (uwb_position_valid)
            snprintf(drive_distance_text, sizeof(drive_distance_text), "%.0f",
                     (double)center_dist);
        else
            snprintf(drive_distance_text, sizeof(drive_distance_text), "--");
        snprintf(drive_angle_text, sizeof(drive_angle_text), "%+d",
                 (int)lroundf(angle_deg));
        if (!uwb_position_valid) snprintf(drive_angle_text, sizeof(drive_angle_text), "--");
        lv_label_set_text(m_drive_distance, drive_distance_text);
        lv_label_set_text(m_drive_status, drive_angle_text);
    }

    /* FOLLOW uses the triangulated UWB target angle. Manual operation uses
       the actual motor command differential to derive display angle/speed. */
    if (m_vehicle) {
        m_vehicle_angle_tenths = 0; // Vehicle-relative radar, mounting rotation is applied when drawing.
        /* Rotate every chassis segment around the fixed LiDAR/vehicle axis.
           This turns the complete vehicle without LVGL transform layers. */
        const uint32_t vehicle_color = drive_pct > 5 ? 0x238636 :
                                       drive_pct < -5 ? 0xDA3633 :
                                       abs(turn_pct) > 5 ? 0x6E40C9 : 0x57606A;
        m_vehicle_color = vehicle_color;
        m_left_wheel_direction = pct_l > 2 ? 1 : (pct_l < -2 ? -1 : 0);
        m_right_wheel_direction = pct_r > 2 ? 1 : (pct_r < -2 ? -1 : 0);
        lv_obj_invalidate(m_vehicle);
        lv_obj_move_foreground(m_vehicle);

        // Vehicle < TAG < LiDAR ray < LiDAR distance points.
        if (m_dir_line) lv_obj_move_background(m_dir_line);
        if (m_dir_target) lv_obj_move_foreground(m_dir_target);
        if (m_lidar_rays[0]) lv_obj_move_foreground(m_lidar_rays[0]);
        for (lv_obj_t *point : m_lidar_points) {
            if (point && !lv_obj_has_flag(point, LV_OBJ_FLAG_HIDDEN))
                lv_obj_move_foreground(point);
        }
    }

    m_clean_output_percent = (uint8_t)lroundf(s_setting_data[21]);
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if (s_vacuum_runtime_reset_requested) {
        m_clean_runtime_ms = 0;
        m_last_saved_runtime_min = 0;
        m_last_runtime_tick_ms = now_ms;
        nvs_handle_t h;
        if (nvs_open("sweeper_cfg", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_u64(h, "vac_run_min", 0);
            nvs_commit(h);
            nvs_close(h);
        }
        s_vacuum_runtime_reset_requested = false;
    }
    if (m_last_runtime_tick_ms && d.vacuum_on)
        m_clean_runtime_ms += (uint64_t)(now_ms - m_last_runtime_tick_ms);
    m_last_runtime_tick_ms = now_ms;
    if (m_clean_bar && m_clean_value) {
        lv_bar_set_value(m_clean_bar,
                         d.vacuum_on ? m_clean_output_percent : 0, LV_ANIM_OFF);
        if (d.vacuum_on)
            lv_label_set_text_fmt(m_clean_value, "%u%%", m_clean_output_percent);
        else
            lv_label_set_text(m_clean_value, "OFF");
        lv_obj_set_style_text_color(m_clean_value, lv_color_hex(0x88C7F5), 0);
    }
    if (m_runtime_value) {
        const uint64_t seconds = m_clean_runtime_ms / 1000;
        lv_label_set_text_fmt(m_runtime_value, "%02llu:%02llu:%02llu",
            (unsigned long long)(seconds / 3600),
            (unsigned long long)((seconds / 60) % 60),
            (unsigned long long)(seconds % 60));
    }
    const uint64_t runtime_min = m_clean_runtime_ms / 60000;
    if (d.vacuum_on && runtime_min != m_last_saved_runtime_min) {
        nvs_handle_t h;
        if (nvs_open("sweeper_cfg", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_u64(h, "vac_run_min", runtime_min);
            nvs_commit(h);
            nvs_close(h);
            m_last_saved_runtime_min = runtime_min;
        }
    }

    /* Toggle cards: gray when inactive, green when active. */
    const int8_t follow_visual = d.follow_enabled ? 1 : 0;
    if (m_btn_follow && m_lbl_follow &&
        m_follow_visual != follow_visual) {
        lv_obj_set_style_bg_color(m_btn_follow,
            lv_color_hex(d.follow_enabled ? 0x238636 : 0x57606A), LV_PART_MAIN);
        draw_button_korean(m_lbl_follow,
            d.follow_enabled ? "추종 정지" : "추종 시작");
        if (m_icon_follow)
            lv_label_set_text(m_icon_follow,
                d.follow_enabled ? LV_SYMBOL_STOP : LV_SYMBOL_PLAY);
        m_follow_visual = follow_visual;
    }
    const int8_t vacuum_visual = d.vacuum_on ? 1 : 0;
    if (m_btn_vacuum && m_lbl_vacuum &&
        m_vacuum_visual != vacuum_visual) {
        lv_obj_set_style_bg_color(m_btn_vacuum,
            lv_color_hex(d.vacuum_on ? 0x238636 : 0x57606A), LV_PART_MAIN);
        draw_button_korean(m_lbl_vacuum,
            d.vacuum_on ? "청소기 켜짐" : "청소기 꺼짐");
        m_vacuum_visual = vacuum_visual;
    }

    const bool rpm_changed = !m_rpm_visual_initialized ||
                             m_rpm_visual_valid != d.can_rx_ok ||
                             (d.can_rx_ok &&
                              (m_rpm_visual_m1 != d.can_motor1_rpm ||
                               m_rpm_visual_m2 != d.can_motor2_rpm));
    if (m_drive_m1_rpm && rpm_changed) {
        if (d.can_rx_ok) {
            lv_label_set_text_fmt(m_drive_m1_rpm,
                                  "M1 %+5d rpm  |  %+5d rpm M2",
                                  d.can_motor1_rpm, d.can_motor2_rpm);
        } else {
            lv_label_set_text(m_drive_m1_rpm,
                              "M1   --- rpm  |  --- rpm   M2");
        }
        m_rpm_visual_initialized = true;
        m_rpm_visual_valid = d.can_rx_ok;
        m_rpm_visual_m1 = d.can_motor1_rpm;
        m_rpm_visual_m2 = d.can_motor2_rpm;
    }

    if (m_estop_popup) {
        const bool show_estop = d.gpio_emergency || d.can_emergency ||
                                d.state == S_ESTOP;
        if (show_estop) {
            const int8_t source_code = d.gpio_emergency && d.can_emergency ? 3 :
                                       d.gpio_emergency ? 1 :
                                       d.can_emergency ? 2 : 0;
            if (m_estop_source && m_estop_visual_source != source_code) {
                const char *source = source_code == 3 ? "GPIO2 + CAN" :
                                     source_code == 1 ? "GPIO2 LOW" :
                                     source_code == 2 ? "CAN RX" : "HMI";
                lv_label_set_text_fmt(m_estop_source, "E-STOP     %s", source);
                m_estop_visual_source = source_code;
            }
            if (!m_estop_visual_visible) {
                lv_obj_clear_flag(m_estop_popup, LV_OBJ_FLAG_HIDDEN);
                m_estop_visual_visible = true;
            }
        } else if (m_estop_visual_visible) {
            lv_obj_add_flag(m_estop_popup, LV_OBJ_FLAG_HIDDEN);
            m_estop_visual_visible = false;
            m_estop_visual_source = -1;
        }
    }

    /* 로그 바 */
    if (d.state == S_ESTOP || d.state == S_NOSIG) {
        lv_label_set_text_fmt(m_log,
            LV_SYMBOL_WARNING "  %s",
            d.state == S_NOSIG ? "No UWB signal — motors stopped"
                               : "Emergency Stop — check system");
        lv_obj_set_style_text_color(m_log, lv_color_hex(0xDA3633), LV_PART_MAIN);
    } else {
        lv_label_set_text_fmt(m_log,
            LV_SYMBOL_LOOP "  dL=%d  dR=%d  avg=%d  diff=%+d  L=%+d  R=%+d",
            d.dL, d.dR, avg, diff, d.sL, d.sR);
        lv_obj_set_style_text_color(m_log, lv_color_hex(0x58A6FF), LV_PART_MAIN);
    }
}
