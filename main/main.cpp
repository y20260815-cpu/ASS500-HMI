/*
 * ============================================================
 *  main.cpp  —  ESP32-P4 청소차 자율추종 컨트롤러  (ESP-IDF v5.x)
 *
 *  환경  : ESP-IDF v5.x + LVGL v8 + BSP (JC1060P470C_I_W)
 *  보드  : JC1060P470C_I_W  (ESP32-P4 v1.x + 1024×600 MIPI-DSI)
 *
 *  클래스 구조
 *    ui/as500_ui       — LVGL v8 대시보드  (robot_t, robot_state_t 포함)
 *    can/as500_can   — TWAI 드라이브/E-STOP 송신
 *    uart/as500_uart — A0 앵커 UART 수신 & 파싱
 *
 *  main.cpp 담당
 *    - 추종 알고리즘 (calc_drive)
 *    - robot_task (FreeRTOS, Core 1)
 *    - app_main   (초기화 + LVGL 타이머 등록)
 * ============================================================
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "nvs_flash.h"
#include "nvs.h"
extern "C" {
#include "esp_hosted_ota.h"
}

#include "bsp/esp-bsp.h"

#include "ui/as500_ui.h"        /* robot_t, robot_state_t, AS500UI  */
#include "can/as500_can.h"      /* AS500Can                            */
#include "can/drive_timing.h"
#include "manual_input.h"
#include "remote_drive.h"
#include "follow_startup.h"
#include "follow_filter.h"
#include "uwb_geometry.h"
#include "uwb_kalman.h"
#include "uwb_motion_history.h"
#include "uwb_divergence.h"
#include "follow_stop.h"
#include "follow_turn_limit.h"
#include "follow_steering.h"
#include <atomic>
#include "follow_direction.h"
#include "uart/as500_uart.h"    /* AS500Uart, AS500Uart::Frame         */
#include "lidar/lidar.h"
#include "espnow/as500_espnow.h"
#include "class/pca9555_driver/Pca9555Driver.h"
#include "class/mpu6050/mpu6050.h"

static const char *TAG = "as500";
static constexpr bool kLogPeriodicControlPose = false;
static esp_reset_reason_t g_boot_reset_reason = ESP_RST_UNKNOWN;

static const char *reset_reason_text(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXTERNAL";
    case ESP_RST_SW:       return "SOFTWARE";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "OTHER_WDT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
    }
}

/* ════════════════════════════════════════════════════════════
 *  핀 / 포트 상수
 * ════════════════════════════════════════════════════════════ */
#define UART_RX_GPIO    45          /* FPC3 pin-3 : A0 TX → P4 RX  */
#define UART_TX_GPIO    46          /* FPC3 pin-4 : A0 RX ← P4 TX  */
#define CAN_TX_GPIO     32          /* Actual board wiring -> CAN transceiver TXD */
#define CAN_RX_GPIO     33          /* Actual board wiring <- CAN transceiver RXD */
#define ESTOP_GPIO       2          /* Active-low emergency-stop input */
#define VACUUM_GPIO     47          /* Cleaner relay/control, active HIGH */
#define LIDAR_TXD_GPIO   3          /* LD06 TXD -> ESP32-P4 UART RX */
#define LIDAR_PWM_GPIO   4          /* ESP32-P4 PWM -> LD06 PWM     */

/* ════════════════════════════════════════════════════════════
 *  추종 파라미터
 * ════════════════════════════════════════════════════════════ */
#define FOLLOW_DIST     100     /* cm : 이 거리 초과 → 전진        */
#define STOP_DIST       20      /* cm : 테스트 목표 도달 정지      */
#define MAX_DIST        250     /* cm : 최고속 포화 거리            */
#define MOTOR_RPM_PER_MPS 2500.0f  /* 1.0 m/s = 2500 rpm = CAN 1000 */
#define MOTOR_MAX_RPM     2500
#define CAN_COMMAND_MAX   1000
#define SPD_INPLACE_RPM    500
#define TURN_K          12      /* 조향 게인 (speed/cm)             */
#define DIFF_DZ         5       /* 방향 데드존 (cm)                 */
#define DATA_TO_MS      300     /* ESP-NOW 수신 타임아웃 → E-STOP */
#define UWB_LOSS_TO_MS  500     /* A0 or A1 zero/missing continuous timeout */

/* ════════════════════════════════════════════════════════════
 *  전역 인스턴스
 * ════════════════════════════════════════════════════════════ */
#define MONITOR_MS      500
#define MANUAL_SPD      300

static AS500Can        g_can;
static AS500Uart       g_uart;
static AS500UI         g_ui;

/* The ESP-Hosted transport resets the whole P4 when the onboard C6 does not
 * answer.  Keep the HMI operational on boards with an unavailable/unflashed
 * C6; enable this again after the relay firmware/SDIO link is confirmed. */
static constexpr bool kEnableHostedEspNow = false;
static Lidar             g_lidar;
static AS500EspNow     g_espnow;
static Pca9555Driver   g_pca9555(0x21);
static MPU6050         g_mpu6050(0x68);
static bool            g_mpu6050_ready = false;

static robot_t           g_bot  = { .state = S_IDLE };
static SemaphoreHandle_t g_lock = NULL;
static bool              g_follow_enabled = false;
static ManualJog g_manual_jog;
static ManualStartup g_manual_startup;
static FollowStartup g_follow_startup;
static bool g_follow_startup_clearance = false;
static int64_t g_follow_geometry_rx_us = 0;
static int64_t g_last_complete_ranges_us = 0;
static bool g_range_sample_substituted = false;
static bool g_raw_shadow_crossed = false;
static bool g_raw_balance_valid = false;
static float g_previous_raw_balance = 0.0f;
static float g_previous_raw_left = 0.0f;
static float g_previous_raw_right = 0.0f;
static float g_previous_raw_balance_delta = 0.0f;
static bool g_jog_force_zero = false;
static bool g_manual_stop_pending = false;
static bool              g_remote_estop_latched = false;
static UwbKalman::ConstantVelocity g_range_kalman_a0;
static UwbKalman::ConstantVelocity g_range_kalman_a1;
static UwbKalman::Angle g_angle_kalman;
static bool              g_usb_serial_ready = false;
static bool              g_estop_gpio_ready = false;
static bool              g_vacuum_gpio_ready = false;

static constexpr const char *kAuxNvsNamespace = "as500_aux";
static constexpr const char *kAuxNvsStateKey = "relay_state";

static uint8_t auxiliary_state_bits(const robot_t &state)
{
    return (state.pump_on ? AS500_RELAY_PUMP_MASK : 0u) |
           (state.blower_on ? AS500_RELAY_BLOWER_MASK : 0u) |
           (state.engine_fan_on ? AS500_RELAY_ENGINE_FAN_MASK : 0u);
}

static void save_auxiliary_state(const robot_t &state)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(kAuxNvsNamespace, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, kAuxNvsStateKey, auxiliary_state_bits(state));
        if (err == ESP_OK) err = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Auxiliary state save failed: %s", esp_err_to_name(err));
}

static void restore_auxiliary_state()
{
    uint8_t state = 0;
    nvs_handle_t nvs = 0;
    const esp_err_t open_err = nvs_open(kAuxNvsNamespace, NVS_READONLY, &nvs);
    if (open_err == ESP_OK) {
        if (nvs_get_u8(nvs, kAuxNvsStateKey, &state) != ESP_OK) state = 0;
        nvs_close(nvs);
    }
    state &= AS500_RELAY_P4_COMMAND_MASK;
    g_bot.pump_on = (state & AS500_RELAY_PUMP_MASK) != 0;
    g_bot.blower_on = (state & AS500_RELAY_BLOWER_MASK) != 0;
    g_bot.engine_fan_on = (state & AS500_RELAY_ENGINE_FAN_MASK) != 0;
    ESP_LOGI(TAG, "Auxiliary state restored: pump=%u blower=%u fan=%u",
             g_bot.pump_on ? 1u : 0u, g_bot.blower_on ? 1u : 0u,
             g_bot.engine_fan_on ? 1u : 0u);
}

static void set_vacuum_output(bool on)
{
    xSemaphoreTake(g_lock, portMAX_DELAY);
    g_bot.vacuum_on = on;
    xSemaphoreGive(g_lock);

    if (!g_vacuum_gpio_ready) {
        ESP_LOGW(TAG, "Vacuum command ignored: GPIO%d is not ready", VACUUM_GPIO);
        return;
    }

    /* Re-assert output mode in case a peripheral/BSP initialization changed
       the GPIO matrix after boot. GPIO47 is active HIGH. */
    gpio_set_direction((gpio_num_t)VACUUM_GPIO, GPIO_MODE_OUTPUT);
    const esp_err_t err = gpio_set_level((gpio_num_t)VACUUM_GPIO, on ? 1 : 0);
    ESP_LOGI(TAG, "Vacuum GPIO%d=%d (readback=%d, %s)",
             VACUUM_GPIO, on ? 1 : 0,
             gpio_get_level((gpio_num_t)VACUUM_GPIO), esp_err_to_name(err));
}

/* USB Serial/JTAG console command decoder.  The same M/D line format is
   accepted here and on the product UART1 pins (GPIO45/46). */
static AS500Uart::Frame poll_usb_console()
{
    static char line[64] = {};
    static size_t index = 0;
    AS500Uart::Frame result;
    if (!g_usb_serial_ready) return result;
    uint8_t bytes[64];
    const int count = usb_serial_jtag_read_bytes(bytes, sizeof(bytes), 0);
    if (count <= 0) return result;

    for (int i = 0; i < count; ++i) {
        const uint8_t ch = bytes[i];
        if (ch == '\n') {
            line[index] = '\0';
            AS500Uart::Frame parsed;
            if (AS500Uart::parseLine(line, parsed)) result = parsed;
            index = 0;
        } else if (ch != '\r') {
            if (index < sizeof(line) - 1) line[index++] = (char)ch;
            else index = 0;
        }
    }
    return result;
}

static int64_t g_last_valid_ranges_us = 0;
static int16_t g_display_range_l = 0, g_display_range_r = 0;
static UwbGeometry::Ranges g_last_geometry_pair{0,0,false,false};
static int64_t g_last_geometry_pair_us = 0;
static bool g_range_pair_error = false;
static UwbMotion::Divergence g_uwb_divergence;
static int64_t g_uwb_correction_hold_until = 0;
static std::atomic_bool g_uwb_restart_front{false};
static std::atomic_bool g_follow_control_reset{false};
static std::atomic_bool g_follow_translating_steer{false};
static bool g_uwb_geometry_shadow=false;
static uint8_t valid_range_mask(uint8_t mask, int left, int right) {
    if (left <= 0) mask &= ~1u;
    if (right <= 0) mask &= ~2u;
    return mask;
}

static uint8_t filter_uwb_ranges(int16_t raw_l, int16_t raw_r, uint8_t mask,
                              int16_t &filtered_l, int16_t &filtered_r)
{
    const int64_t now = esp_timer_get_time();
    const float baseline = UwbGeometry::kAnchorBaselineCm;
    const uint8_t raw_valid_mask=valid_range_mask(mask,raw_l,raw_r);
    if (g_uwb_restart_front.exchange(false)) {
        g_range_kalman_a0.reset();g_range_kalman_a1.reset();g_angle_kalman.reset();
        g_last_valid_ranges_us = 0;
        g_last_complete_ranges_us=0;g_range_sample_substituted=false;
        g_range_pair_error=false;
        g_raw_shadow_crossed=false;g_raw_balance_valid=false;
        g_previous_raw_left=g_previous_raw_right=0.0f;
        g_previous_raw_balance_delta=0.0f;
        g_follow_geometry_rx_us = 0; g_uwb_divergence = {};
        g_display_range_l = g_display_range_r = 0;
        g_last_geometry_pair={0,0,false,false};g_last_geometry_pair_us=0;
        g_uwb_correction_hold_until=0;
    }
    if(raw_valid_mask==3)g_last_complete_ranges_us=now;

    // Signal processing order is deliberately fixed: independent Kalman
    // filters first, then pair geometry, then the bearing Kalman in
    // update_target_pose(). No median or EWMA stage remains in this path.
    const float kRangeMeasurementStdCm=std::clamp(
        AS500UI::settingValue(16),5.0f,100.0f);
    const float kRangeAccelerationStdCmS2=std::clamp(
        AS500UI::settingValue(17),5.0f,300.0f);
    if(raw_valid_mask&1)g_range_kalman_a0.update(
        raw_l,now,kRangeMeasurementStdCm,kRangeAccelerationStdCmS2);
    else if(g_range_kalman_a0.ready())g_range_kalman_a0.predict(
        now,kRangeAccelerationStdCmS2);
    if(raw_valid_mask&2)g_range_kalman_a1.update(
        raw_r,now,kRangeMeasurementStdCm,kRangeAccelerationStdCmS2);
    else if(g_range_kalman_a1.ready())g_range_kalman_a1.predict(
        now,kRangeAccelerationStdCmS2);

    const bool both_filtered=g_range_kalman_a0.ready()&&g_range_kalman_a1.ready();
    const float kalman_l=both_filtered?g_range_kalman_a0.value():0.0f;
    const float kalman_r=both_filtered?g_range_kalman_a1.value():0.0f;
    const auto pair=both_filtered
        ?UwbGeometry::reconcile(kalman_l,kalman_r,baseline,3)
        :UwbGeometry::Ranges{kalman_l,kalman_r,false,false};
    const bool missing=raw_valid_mask!=3;
    g_range_pair_error=!missing&&!pair.valid;
    if(!missing&&pair.valid){
        // Shadow crossing classification also uses only Kalman-filtered
        // distances. Interpolation catches a strip crossed between packets.
        const float balance=UwbGeometry::branchBalance(pair.left,pair.right,baseline);
        if(std::isfinite(balance)){
            const float band=std::clamp(AS500UI::settingValue(26),1.0f,3.0f)*
                UwbGeometry::kDw3000RangeErrorCm;
            bool crossed=fabsf(balance)<=band;
            if(g_raw_balance_valid){
                for(int i=1;i<8&&!crossed;++i){
                    const float t=i/8.0f;
                    const float il=g_previous_raw_left+(pair.left-g_previous_raw_left)*t;
                    const float ir=g_previous_raw_right+(pair.right-g_previous_raw_right)*t;
                    const float ib=UwbGeometry::branchBalance(il,ir,baseline);
                    crossed=std::isfinite(ib)&&fabsf(ib)<=band;
                }
                crossed=crossed||
                    (g_previous_raw_balance*balance<0.0f&&
                     fabsf(g_previous_raw_balance)>band&&fabsf(balance)>band);
                const float magnitude_delta=fabsf(balance)-fabsf(g_previous_raw_balance);
                crossed=crossed||
                    (g_previous_raw_balance_delta<0.0f&&magnitude_delta>0.0f&&
                     fabsf(g_previous_raw_balance)<=band);
                g_previous_raw_balance_delta=magnitude_delta;
            }
            if(crossed)g_raw_shadow_crossed=true;
            g_previous_raw_balance=balance;g_raw_balance_valid=true;
            g_previous_raw_left=pair.left;g_previous_raw_right=pair.right;
        }
        g_last_geometry_pair=pair;g_last_geometry_pair_us=now;
        g_range_sample_substituted=false;
        g_last_valid_ranges_us=now;
        filtered_l=g_display_range_l=(int16_t)lroundf(pair.left);
        filtered_r=g_display_range_r=(int16_t)lroundf(pair.right);
        return 3;
    }

    // A missing sensor or an impossible filtered pair may briefly use the
    // last valid geometry. Repeated missing data still reaches the existing
    // 500 ms motor stop; pair errors expire after 300 ms.
    const int64_t hold_limit_us=missing?UWB_LOSS_TO_MS*1000LL:300000LL;
    const bool can_hold=g_last_geometry_pair.valid&&now>=g_last_geometry_pair_us&&
        now-g_last_geometry_pair_us<=hold_limit_us;
    if(can_hold){
        g_range_sample_substituted=true;
        filtered_l=g_display_range_l=(int16_t)lroundf(g_last_geometry_pair.left);
        filtered_r=g_display_range_r=(int16_t)lroundf(g_last_geometry_pair.right);
        return 3;
    }
    g_range_sample_substituted=false;
    filtered_l=filtered_r=0;
    return 0;
}

static bool simulation_enabled()
{
    /* Browser simulation is detected from USB Serial frames. The former menu
       switch is now the automatic-follow inner-wheel percentage. */
    return false;
}

static void write_serial_response(bool via_usb, const char *line)
{
    if (via_usb) {
        printf("%s", line);
        fflush(stdout);
    } else {
        g_uart.writeLine(line);
    }
}

static void send_follow_config(bool via_usb)
{
    char line[96];
    const float max_speed_m_s = AS500UI::settingValue(2);
    const float stop_distance_mm = AS500UI::settingValue(0) * 10.0f;
    snprintf(line, sizeof(line),
             "CFG,%.1f,%.0f,%.0f,%.1f,%.1f,%.1f,%.1f,%.0f\n",
             (double)max_speed_m_s, (double)stop_distance_mm,
             (double)(AS500UI::settingValue(1) / 100.0f),
             (double)AS500UI::settingValue(3),
             (double)AS500UI::settingValue(4),
             (double)AS500UI::settingValue(5),
             (double)AS500UI::settingValue(23),
             (double)AS500UI::settingValue(24));
    write_serial_response(via_usb, line);
    snprintf(line, sizeof(line), "GEOM,%.1f,%.1f,%.2f,%.1f,%.0f\n",
             (double)UwbGeometry::kAnchorBaselineCm, (double)UwbGeometry::kLongitudinalGapCm,
             (double)AS500UI::settingValue(16), (double)AS500UI::settingValue(17),
             (double)AS500UI::settingValue(26));
    write_serial_response(via_usb, line);
    snprintf(line, sizeof(line), "TURN,%.0f\n", (double)AS500UI::settingValue(25));
    write_serial_response(via_usb, line);
}

/* Publish control/UWB state without overwriting the independently updated
 * LiDAR scan.  Whole-structure assignment caused fresh scans to disappear
 * whenever robot_ctrl committed an older snapshot. */
static void store_control_state(const robot_t &src)
{
    xSemaphoreTake(g_lock, portMAX_DELAY);
    g_bot.dL = src.dL;
    g_bot.dR = src.dR;
    g_bot.mask = src.mask;
    g_bot.target_distance_cm = src.target_distance_cm;
    g_bot.target_angle_deg = src.target_angle_deg;
    g_bot.target_distance_error_cm = src.target_distance_error_cm;
    g_bot.target_angle_error_deg = src.target_angle_error_deg;
    g_bot.target_pose_valid = src.target_pose_valid;
    g_bot.target_ranges_held = src.target_ranges_held;
    g_bot.target_geometry_shadow = src.target_geometry_shadow;
    g_bot.target_range_pair_error = src.target_range_pair_error;
    g_bot.target_pose_corrected = src.target_pose_corrected;
    g_bot.tag_adc0 = src.tag_adc0;
    g_bot.tag_adc1 = src.tag_adc1;
    g_bot.sL = src.sL;
    g_bot.sR = src.sR;
    g_bot.state = src.state;
    g_bot.uart_command_valid = src.uart_command_valid;
    g_bot.uart_command_type = src.uart_command_type;
    g_bot.uart_command_left = src.uart_command_left;
    g_bot.uart_command_right = src.uart_command_right;
    g_bot.uart_command_mask = src.uart_command_mask;
    g_bot.uart_command_enable = src.uart_command_enable;
    g_bot.follow_enabled = src.follow_enabled;
    /* vacuum_on is owned by set_vacuum_output(). The control task has a
       long-lived snapshot and must not overwrite a newer UI/remote toggle. */
    g_bot.remote_age_ms = src.remote_age_ms;
    g_bot.remote_keys = src.remote_keys;
    g_bot.remote_rssi = src.remote_rssi;
    xSemaphoreGive(g_lock);
}

/* ════════════════════════════════════════════════════════════
 *  LVGL 타이머 콜백  (10 Hz)
 *  공유 데이터 스냅샷 → g_ui.update()
 * ════════════════════════════════════════════════════════════ */
static void lvgl_tick_cb(lv_timer_t *t)
{
    (void)t;
    robot_t snap;
    if (xSemaphoreTake(g_lock, 0) == pdTRUE) {
        snap = g_bot;
        xSemaphoreGive(g_lock);
        g_ui.update(snap);
    }
}

/* ════════════════════════════════════════════════════════════
 *  추종 주행 계산 (스키드 스티어링)
 *
 *  diff = dR - dL
 *    > 0 : 타겟이 로봇 왼쪽  → 좌회전
 *    < 0 : 타겟이 로봇 오른쪽 → 우회전
 * ════════════════════════════════════════════════════════════ */
static void update_target_pose(robot_t *b)
{
    b->target_ranges_held = g_range_sample_substituted;
    b->target_geometry_shadow = false;
    b->target_range_pair_error = g_range_pair_error;
    if (g_range_sample_substituted && b->target_pose_valid) return;
    b->target_pose_valid = false;
    b->target_pose_corrected = false;
    if ((b->mask & 3) != 3 || b->dL <= 0 || b->dR <= 0) return;
    const int64_t now = esp_timer_get_time();
    // The installation guarantees that TAG stays in front of the A0-A1 line.
    // Therefore the positive-normal circle intersection is the only valid pose.
    const float baseline = UwbGeometry::kAnchorBaselineCm;
    const auto pair = UwbGeometry::reconcile(b->dL, b->dR, baseline, b->mask);
    if (!pair.valid || g_range_sample_substituted) return;
    b->target_pose_corrected = pair.corrected;
    // Horizontal rear anchors: A0=left (0,-width/2), A1=right
    // (0,+width/2). Their midpoint is the vehicle ranging/control origin.
    const float along=(pair.left*pair.left+baseline*baseline-pair.right*pair.right)/(2.0f*baseline);
    const float normal=sqrtf(std::max(0.0f,pair.left*pair.left-along*along));
    const float forward=UwbGeometry::kAnchorForwardCm+normal;
    const float lateral=-baseline*.5f+along;
    g_uwb_geometry_shadow=normal<
        std::clamp(AS500UI::settingValue(26),1.0f,3.0f)*UwbGeometry::kDw3000RangeErrorCm||
        g_raw_shadow_crossed;
    g_raw_shadow_crossed=false;
    b->target_geometry_shadow=g_uwb_geometry_shadow;
    g_follow_geometry_rx_us = now;
    const float center_dist = hypotf(forward, lateral);
    const float raw_target_angle_rad = atan2f(lateral, forward);
    const auto uncertainty=UwbGeometry::poseUncertainty(pair.left,pair.right,forward,lateral,
        UwbGeometry::kDw3000RangeErrorCm,baseline);
    b->target_distance_cm = center_dist;
    const float angle_measurement_std_rad=std::max(2.0f,uncertainty.angle_deg)*
        UwbKalman::kPi/180.0f;
    b->target_angle_deg = g_angle_kalman.update(raw_target_angle_rad,now,
        angle_measurement_std_rad,UwbKalman::kPi)*57.2957795f;
    b->target_distance_error_cm = uncertainty.distance_cm;
    b->target_angle_error_deg = uncertainty.angle_deg;
    b->target_pose_valid = true;
}

static void calc_drive(robot_t *b)
{
    g_follow_startup_clearance = false;
    g_follow_translating_steer.store(false);

    /* Keep alignment hysteresis separate from the UI drive state.  S_TURN_L/R
       is also used for ordinary forward steering and must not re-enter pivot
       mode merely because the previous command contained steering. */
    static bool aligning_in_place = false;
    static bool initial_alignment_pending = false;
    static int terminal_turn_sign = 0;
    static int64_t terminal_align_cooldown_until = 0;
    static bool steering_active = false;
    if(g_follow_control_reset.exchange(false)){
        aligning_in_place=false;terminal_turn_sign=0;
        initial_alignment_pending=true;
        terminal_align_cooldown_until=0;steering_active=false;
    }
    const int stop_dist = (int)lroundf(AS500UI::settingValue(0));
    const int max_dist = (int)lroundf(AS500UI::settingValue(1));
    const float max_mps = AS500UI::settingValue(2);
    const float min_mps = std::min(AS500UI::settingValue(3), max_mps);
    const float decel_time_s = AS500UI::settingValue(4);
    const int speed_max = (int)lroundf(max_mps * MOTOR_RPM_PER_MPS);
    const int speed_min = (int)lroundf(min_mps * MOTOR_RPM_PER_MPS);
    /* 앵커 유효성 */
    if (!b->target_pose_valid || (b->mask & 0x03) != 0x03 || b->dL == 0 || b->dR == 0) {
        const int64_t range_now=esp_timer_get_time();
        const bool ranges_within_500ms=g_last_complete_ranges_us>0&&
            range_now-g_last_complete_ranges_us<=UWB_LOSS_TO_MS*1000LL;
        if (ranges_within_500ms &&
            (b->state == S_FOLLOW || b->state == S_ALIGN ||
             b->state == S_TURN_L || b->state == S_TURN_R)) return;
        aligning_in_place = false;
        steering_active = false;
        b->sL = b->sR = 0;
        b->state = S_NOSIG;
        return;
    }

    const float center_dist = b->target_distance_cm;
    const float target_angle_deg = b->target_angle_deg;
    const int avg = (int)lroundf(center_dist);
    constexpr int travel_sign=1;
    if(esp_timer_get_time()<g_uwb_correction_hold_until){
        b->sL=b->sR=0;aligning_in_place=false;steering_active=false;b->state=S_FOLLOW;return;
    }
    const float tracking_angle = target_angle_deg;


    /* Outside the configured estimate range, do not drive on an extrapolated
       UWB position. */
    if (avg > max_dist) {
        aligning_in_place = false;
        steering_active = false;
        b->sL = b->sR = 0;
        b->state = S_NOSIG;
        return;
    }

    // Once the requested distance is reached, do not chase noisy close-range
    // bearing measurements. The approach alignment below has already faced
    // the vehicle toward TAG before entering this stop zone.
    if (center_dist <= (float)stop_dist) {
        aligning_in_place = false;
        steering_active = false;
        const int stop_step = std::max(1, (int)lroundf(
            speed_max * 0.1f / std::max(decel_time_s, 0.1f)));
        auto ramp_zero = [stop_step](int16_t value) -> int16_t {
            if (value > 0) return (int16_t)std::max(0, (int)value - stop_step);
            if (value < 0) return (int16_t)std::min(0, (int)value + stop_step);
            return 0;
        };
        b->sL = ramp_zero(b->sL);
        b->sR = ramp_zero(b->sR);
        b->state = S_NEAR;
        return;
    }

    // Do not let startup assistance defeat the near-target braking envelope.
    const float startup_margin_cm = std::max(50.0f,
        50.0f * max_mps * std::max(decel_time_s, 0.1f));
    g_follow_startup_clearance = avg - stop_dist > startup_margin_cm;

    int base = 0, turn = 0;

    /* Cruise at maximum speed, then limit it with the time-based braking
       profile near the stop distance. */
    if (avg > stop_dist) {
        base = speed_max;
        /* Smooth time-based approach curve.  The near-speed term controls
           the low-speed end of the curve but is multiplied by x, so the
           target remains continuous and reaches zero at the stop distance. */
        const float remaining_m = (float)(avg - stop_dist) * 0.01f;
        const float braking_distance_m =
            0.5f * max_mps * std::max(decel_time_s, 0.1f);
        const float x = std::clamp(remaining_m / braking_distance_m,
                                   0.0f, 1.0f);
        const float smooth_x = x * x * (3.0f - 2.0f * x);
        const float limited_approach_mps =
            min_mps * x + (max_mps - min_mps) * smooth_x;
        const int approach_rpm = (int)lroundf(
            limited_approach_mps * MOTOR_RPM_PER_MPS);
        if (base > approach_rpm) base = approach_rpm;
        if (base > speed_max) base = speed_max;
    }

    /* 조향 */
    const float steering_angle = std::clamp(tracking_angle, -45.0f, 45.0f);
    const float turn_gain = AS500UI::settingValue(23);
    const float curvature_ratio=UwbGeometry::kAnchorBaselineCm /
        std::max(center_dist,20.0f)*sinf(steering_angle*0.01745329252f)*turn_gain;
    turn=(int)lroundf(-curvature_ratio*std::max(base,1));
    const float steering_error = fabsf(tracking_angle);
    // Normal steering keeps its distance-scaled band. Geometry uncertainty is
    // used for final alignment; allowing a weak far-field solution to widen
    // this band could suppress the turn needed to approach the target.
    const float steering_enter=FollowFilter::steeringEntry(center_dist);
    const float steering_exit=FollowFilter::steeringExit(center_dist);
    if (steering_error >= steering_enter) steering_active = true;
    else if (steering_error <= steering_exit) steering_active = false;
    if (!steering_active) turn = 0;

    int lS, rS;
    bool in_place_alignment_output = false;
    const float abs_target_angle = fabsf(tracking_angle);
    /* Near the TAG, forward steering has a very small turn radius and can orbit
       forever. Enter a pivot before the stop boundary, then resume the final
       straight approach only after a much smaller exit angle. */
    const float terminal_zone_cm = (float)stop_dist + std::max(
        50.0f, 2.0f * b->target_distance_error_cm);
    const bool allow_pre_arrival_pivot = center_dist <= terminal_zone_cm;
    const int64_t control_now=esp_timer_get_time();
    /* Align once before the first translation, then retain the existing
       near-target alignment. Both stages use the same entry/exit hysteresis
       so a noisy bearing cannot alternate the pivot direction. */
    if (initial_alignment_pending) {
        if (!aligning_in_place) {
            if (abs_target_angle >= 25.0f) {
                aligning_in_place = true;
                terminal_turn_sign = tracking_angle > 0.0f ? -1 : 1;
            } else {
                initial_alignment_pending = false;
            }
        } else if (abs_target_angle <= 10.0f ||
                   tracking_angle * (terminal_turn_sign < 0 ? 1.0f : -1.0f) <= 0.0f) {
            aligning_in_place = false;
            initial_alignment_pending = false;
            terminal_turn_sign = 0;
            terminal_align_cooldown_until=control_now+1000000;
        }
    } else {
        if (!allow_pre_arrival_pivot)
            aligning_in_place = false;
        else if (!aligning_in_place && control_now>=terminal_align_cooldown_until &&
                 abs_target_angle >= 25.0f) {
            aligning_in_place = true;
            terminal_turn_sign = tracking_angle > 0.0f ? -1 : 1;
        } else if (aligning_in_place &&
                   (abs_target_angle <= 10.0f ||
                    tracking_angle * (terminal_turn_sign < 0 ? 1.0f : -1.0f) <= 0.0f)) {
            aligning_in_place = false;
            terminal_turn_sign = 0;
            terminal_align_cooldown_until=control_now+1000000;
        }
    }

    if (aligning_in_place &&
        (initial_alignment_pending || allow_pre_arrival_pivot)) {
        in_place_alignment_output = true;
        /* Face the target before translating. Keeping forward speed during a
           large heading error creates a circular orbit around the tag. */
        // Separate sustained pivot drive from translational speed and remove
        // the previous turn/4 attenuation. Taper near alignment to avoid hunting.
        const int t4 = followPivotCommand(terminal_turn_sign, abs_target_angle,
            (int)lroundf(AS500UI::settingValue(25)));
        lS = -t4;
        rS = t4;
        b->state = (terminal_turn_sign > 0) ? S_TURN_L : S_TURN_R;
    } else {
        /* 전진 + 조향 */
        if (abs_target_angle > 10.0f) {
            const float heading_scale = std::clamp(
                1.0f - 0.60f * (abs_target_angle - 10.0f) / 35.0f,
                0.65f, 1.0f);
            base = (int)lroundf((float)base * heading_scale);
        }
        /* Keep both sides in the selected travel direction until the stop
           distance is reached.  Steering remains differential, while final
           heading alignment is handled by the arrival branch above. */
        const int max_forward_turn = (int)lroundf((float)base * 0.75f);
        turn = std::clamp(turn, -max_forward_turn, max_forward_turn);
        // Setting 08 scales the differential steering component as well as
        // limiting an in-place pivot. 100% preserves the previous response.
        turn = FollowTurnLimit::scale(turn, AS500UI::settingValue(24));
        lS = travel_sign * base - turn;
        rS = travel_sign * base + turn;
        /* Derive the ratio from commands after their physical speed cap, so
           the outer-wheel reference is the value that CAN can really send. */
        lS = std::clamp(lS, -speed_max, speed_max);
        rS = std::clamp(rS, -speed_max, speed_max);
        const FollowSteering::MotorPair adjusted =
            FollowSteering::applyInnerPercent(lS, rS,
                                              AS500UI::settingValue(15));
        lS = adjusted.left;
        rS = adjusted.right;
        g_follow_translating_steer.store(turn != 0);
        b->state = turn != 0
                    ? ((turn > 0) ? S_TURN_L : S_TURN_R)
                    : S_FOLLOW;
    }

    // Only the explicit alignment branch is an in-place turn. A negative
    // inner wheel from travelling steering keeps the translational limit.
    const int configured_turn_limit = FollowTurnLimit::rpm(
        AS500UI::settingValue(24),MOTOR_MAX_RPM);
    const int output_limit = in_place_alignment_output ?
                             configured_turn_limit : speed_max;
    lS = std::clamp(lS, -output_limit, output_limit);
    rS = std::clamp(rS, -output_limit, output_limit);

    // Send the target once; STM32 owns acceleration through startup/cruise.
    // The near-target approach envelope above still limits the target.
    b->sL = (int16_t)lS;
    b->sR = (int16_t)rS;
}

static void apply_hmi_command(control_cmd_t cmd)
{
    if (cmd == CMD_NONE) return;
    if (cmd == CMD_STOP || cmd == CMD_ESTOP) g_manual_stop_pending = true;
    bool tap = false;
    switch (cmd) {
    case CMD_TAP_FORWARD: cmd = CMD_FORWARD; tap = true; break;
    case CMD_TAP_REVERSE: cmd = CMD_REVERSE; tap = true; break;
    case CMD_TAP_LEFT: cmd = CMD_LEFT; tap = true; break;
    case CMD_TAP_RIGHT: cmd = CMD_RIGHT; tap = true; break;
    default: break;
    }
    const bool auxiliary_command = cmd == CMD_VACUUM_TOGGLE ||
        cmd == CMD_HAZARD_TOGGLE || cmd == CMD_HORN_ON || cmd == CMD_HORN_OFF ||
        cmd == CMD_PUMP_TOGGLE || cmd == CMD_BLOWER_TOGGLE ||
        cmd == CMD_ENGINE_FAN_TOGGLE ||
        cmd == CMD_HEADLIGHT_TOGGLE || cmd == CMD_PARKING_LIGHT_TOGGLE;
    if (!auxiliary_command) {
        if (g_manual_jog.active()) g_jog_force_zero = true;
        g_manual_jog.cancel();
        g_manual_startup.cancel();
    }

    robot_t local;
    xSemaphoreTake(g_lock, portMAX_DELAY);
    local = g_bot;
    xSemaphoreGive(g_lock);
    if (tap && (local.gpio_emergency || local.can_emergency || local.state == S_ESTOP))
        return;
    if (tap) g_manual_jog.begin();
    const bool held_direction = !tap && (cmd == CMD_FORWARD || cmd == CMD_REVERSE ||
                                         cmd == CMD_LEFT || cmd == CMD_RIGHT);
    if (held_direction && !local.gpio_emergency && !local.can_emergency &&
        local.state != S_ESTOP) {
        g_manual_startup.begin();
        const int dl = (cmd == CMD_REVERSE || cmd == CMD_LEFT) ? -1 : 1;
        const int dr = (cmd == CMD_REVERSE || cmd == CMD_RIGHT) ? -1 : 1;
        g_manual_startup.evidence.reset(local.can_current_deciamps,
            local.can_motor1_rpm, local.can_motor2_rpm, dl, dr,
            local.can_rx_ok && local.can_rx_age_ms <= 150);
    }
    int manual_full_rpm = (int)lroundf(AS500UI::settingValue(2) *
                                       MOTOR_RPM_PER_MPS);
    if (manual_full_rpm < 1) manual_full_rpm = 1;
    if (manual_full_rpm > 32767) manual_full_rpm = 32767;
    if (tap) manual_full_rpm = std::min(manual_full_rpm,
        (cmd == CMD_LEFT || cmd == CMD_RIGHT) ? ManualJog::TurnRpm : ManualJog::Rpm);

    bool vacuum_changed = false;
    bool auxiliary_changed = false;
    bool persistent_auxiliary_changed = false;
    switch (cmd) {
    case CMD_ESTOP:
        g_follow_enabled = false;
        local.follow_enabled = false;
        local.sL = local.sR = 0;
        local.state = S_ESTOP;
        break;
    case CMD_FOLLOW_START:
        g_follow_enabled = !g_follow_enabled;
        if(g_follow_enabled){
            // A0-A1 front is the only installed operating region. Reset the
            // filters on each follow start; no rear hypothesis is generated.
            g_uwb_restart_front.store(true);
            g_follow_control_reset.store(true);
        }
        local.follow_enabled = g_follow_enabled;
        local.sL = local.sR = 0;
        local.state = S_IDLE;
        break;
    case CMD_STOP:
        g_remote_estop_latched = false;
        g_follow_enabled = false;
        local.follow_enabled = false;
        local.sL = local.sR = 0;
        local.state = S_IDLE;
        break;
    case CMD_VACUUM_TOGGLE:
        local.vacuum_on = !local.vacuum_on;
        vacuum_changed = true;
        break;
    case CMD_HAZARD_TOGGLE: local.hazard_on = !local.hazard_on; auxiliary_changed = true; break;
    case CMD_HORN_ON:       local.horn_on = true; auxiliary_changed = true; break;
    case CMD_HORN_OFF:      local.horn_on = false; auxiliary_changed = true; break;
    case CMD_PUMP_TOGGLE:   local.pump_on = !local.pump_on; auxiliary_changed = true; persistent_auxiliary_changed = true; break;
    case CMD_BLOWER_TOGGLE: local.blower_on = !local.blower_on; auxiliary_changed = true; persistent_auxiliary_changed = true; break;
    case CMD_ENGINE_FAN_TOGGLE: local.engine_fan_on = !local.engine_fan_on; auxiliary_changed = true; persistent_auxiliary_changed = true; break;
    case CMD_HEADLIGHT_TOGGLE: local.headlight_on = !local.headlight_on; auxiliary_changed = true; break;
    case CMD_PARKING_LIGHT_TOGGLE: local.parking_light_on = !local.parking_light_on; auxiliary_changed = true; break;
    case CMD_FORWARD:
        /* Do not insert a zero command here. If the same direction is pressed
           again while release deceleration is still in progress, the 100 ms
           output ramp continues from its current value. Direction changes
           are handled by ramp_motor_command(), which crosses zero safely. */
        g_follow_enabled = false;
        local.follow_enabled = false;
        local.sL = local.sR = (int16_t)manual_full_rpm;
        local.state = S_MAN_FWD;
        break;
    case CMD_REVERSE:
        g_follow_enabled = false;
        local.follow_enabled = false;
        local.sL = local.sR = (int16_t)-manual_full_rpm;
        local.state = S_MAN_REV;
        break;
    case CMD_LEFT:
        g_follow_enabled = false;
        local.follow_enabled = false;
        local.sL = (int16_t)-manual_full_rpm;
        local.sR = (int16_t)manual_full_rpm;
        local.state = S_MAN_LEFT;
        break;
    case CMD_RIGHT:
        g_follow_enabled = false;
        local.follow_enabled = false;
        local.sL = (int16_t)manual_full_rpm;
        local.sR = (int16_t)-manual_full_rpm;
        local.state = S_MAN_RIGHT;
        break;
    default:
        return;
    }

    store_control_state(local);
    if (auxiliary_changed) {
        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_bot.hazard_on = local.hazard_on;
        g_bot.horn_on = local.horn_on;
        g_bot.pump_on = local.pump_on;
        g_bot.blower_on = local.blower_on;
        g_bot.engine_fan_on = local.engine_fan_on;
        g_bot.headlight_on = local.headlight_on;
        g_bot.parking_light_on = local.parking_light_on;
        xSemaphoreGive(g_lock);
    }
    if (persistent_auxiliary_changed) save_auxiliary_state(local);
    if (vacuum_changed) set_vacuum_output(local.vacuum_on);
}

/* ════════════════════════════════════════════════════════════
 *  robot_task  —  UART 수신 + 추종 로직  (Core 1)
 * ════════════════════════════════════════════════════════════ */
enum : uint8_t {
    REMOTE_RELEASE   = 0x00,
    REMOTE_FRONT     = 0x01,
    REMOTE_REAR      = 0x02,
    REMOTE_RIGHT     = 0x08,
    REMOTE_LEFT      = 0x04,
    REMOTE_FOLLOW    = 0x10,
    REMOTE_VACUUM    = 0x20,
};

static bool is_remote_direction(uint8_t code)
{
    return (code & 0x0F) != 0;
}

static bool is_manual_drive_state(robot_state_t state)
{
    return state == S_MAN_FWD || state == S_MAN_REV ||
           state == S_MAN_LEFT || state == S_MAN_RIGHT;
}

/* Apply one 100 ms slew-rate step to an individual motor.  A sign change
   always decelerates to zero before accelerating in the opposite direction. */
static int16_t ramp_motor_command(int16_t current, int16_t target,
                                  int accel_step, int decel_step)
{
    accel_step = std::max(accel_step, 1);
    decel_step = std::max(decel_step, 1);

    if ((current > 0 && target < 0) || (current < 0 && target > 0)) {
        if (current > 0) return (int16_t)std::max(0, current - decel_step);
        return (int16_t)std::min(0, current + decel_step);
    }

    const int step = abs((int)target) > abs((int)current)
                         ? accel_step : decel_step;
    if (target > current)
        return (int16_t)std::min((int)target, (int)current + step);
    if (target < current)
        return (int16_t)std::max((int)target, (int)current - step);
    return current;
}

/* Controller targets are motor RPM. Convert with the common vehicle scale:
   1.0 m/s = 2500 rpm = CAN 1000 (100%). Keep whole-percent CAN steps so a
   displayed 0% always corresponds to an exact zero command. */
static int16_t rpm_to_can_command(int16_t rpm)
{
    const int limited_rpm = std::clamp((int)rpm, -MOTOR_MAX_RPM,
                                       MOTOR_MAX_RPM);
    const int command = (int)lroundf((float)limited_rpm * CAN_COMMAND_MAX /
                                     MOTOR_RPM_PER_MPS);
    const int percent = command >= 0 ? (command + 5) / 10
                                     : (command - 5) / 10;
    return (int16_t)(percent * 10);
}

/* Codes are edge-triggered here. Repeated ESP-NOW packets while a key is held
   must not repeatedly toggle FOLLOW/VACUUM or restart a manual command. */
static void apply_remote_code(uint8_t code, uint8_t previous_code)
{
    const unsigned direction = code & 15;
    if (direction) {
        const int rpm = std::clamp((int)lroundf(AS500UI::settingValue(2) *
                                              MOTOR_RPM_PER_MPS), 1, MOTOR_MAX_RPM);
        const auto mixed = RemoteDrive::mix(direction, rpm,
            (int)lroundf(AS500UI::settingValue(14)));
        if (!mixed.valid) { apply_hmi_command(CMD_STOP); return; }
        const control_cmd_t base = (direction & REMOTE_FRONT) ? CMD_FORWARD :
            (direction & REMOTE_REAR) ? CMD_REVERSE :
            (direction & REMOTE_LEFT) ? CMD_LEFT : CMD_RIGHT;
        robot_t local;
        xSemaphoreTake(g_lock, portMAX_DELAY);
        local = g_bot;
        xSemaphoreGive(g_lock);
        if (local.gpio_emergency || local.can_emergency || local.state == S_ESTOP) return;
        const robot_state_t desired = base == CMD_FORWARD ? S_MAN_FWD :
            base == CMD_REVERSE ? S_MAN_REV : base == CMD_LEFT ? S_MAN_LEFT : S_MAN_RIGHT;
        // Adding/removing steering while travelling does not restart startup assist.
        if (local.state != desired) apply_hmi_command(base);
        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_bot.sL = mixed.left;
        g_bot.sR = mixed.right;
        xSemaphoreGive(g_lock);
        return;
    }
    if (is_remote_direction(previous_code)) apply_hmi_command(CMD_STOP);
    if ((code & REMOTE_FOLLOW) && !(previous_code & REMOTE_FOLLOW))
        apply_hmi_command(CMD_FOLLOW_START);
    if ((code & REMOTE_VACUUM) && !(previous_code & REMOTE_VACUUM))
        apply_hmi_command(CMD_VACUUM_TOGGLE);
}

static void robot_task(void *arg)
{
    int64_t last_monitor = 0;
    int64_t last_rx = 0;
    uint8_t last_remote_code = REMOTE_RELEASE;
    uint16_t last_sequence = 0;
    bool have_sequence = false;
    bool remote_motion_active = false;
    bool remote_release_required = false;
    bool last_sim_enabled = simulation_enabled();
    int64_t last_uart_motor_rx = 0;
    bool uart_motor_active = false;
    bool uart_timeout_reported = false;
    int64_t last_can_status_rx = 0;
    int64_t last_can_command_tx = esp_timer_get_time();
    int64_t last_io_command_tx = 0;
    int64_t tilt_over_since_us = 0;
    uint8_t tilt_alarm = 0;
    uint8_t interface_fnr = AS500_FNR_NEUTRAL;
    bool interface_fnr_valid = false;
    bool interface_emergency_armed = false;
    bool throttle_override_active = false;
    int16_t can_output_left = 0;
    int16_t can_output_right = 0;
    bool manual_ramp_active = false;
    float published_follow_config[8] = {
        -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f
    };
    bool config_usb_subscribed = false;
    bool config_uart_subscribed = false;

    while (1) {
        const control_cmd_t hmi_command = AS500UI::takeCommand();
        uint8_t touch_direction = 0;
        switch (hmi_command) {
        case CMD_FORWARD: touch_direction = 1; break;
        case CMD_REVERSE: touch_direction = 2; break;
        case CMD_LEFT: touch_direction = 4; break;
        case CMD_RIGHT: touch_direction = 8; break;
        case CMD_FORWARD_LEFT: touch_direction = 5; break;
        case CMD_FORWARD_RIGHT: touch_direction = 9; break;
        case CMD_REVERSE_LEFT: touch_direction = 6; break;
        case CMD_REVERSE_RIGHT: touch_direction = 10; break;
        default: break;
        }
        if (touch_direction) apply_remote_code(touch_direction, 0);
        else apply_hmi_command(hmi_command);
        if (hmi_command != CMD_NONE) remote_motion_active = false;
        if (hmi_command != CMD_NONE)
            ESP_LOGI("HMI_INPUT", "APPLIED cmd=%d t_ms=%lld", (int)hmi_command,
                     (long long)(esp_timer_get_time() / 1000));

        const bool gpio_emergency = g_estop_gpio_ready &&
                                    gpio_get_level((gpio_num_t)ESTOP_GPIO) == 0;
        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_bot.gpio_emergency = gpio_emergency;
        if (gpio_emergency) {
            g_follow_enabled = false;
            g_bot.follow_enabled = false;
            g_bot.sL = g_bot.sR = 0;
            g_bot.state = S_ESTOP;
        }
        xSemaphoreGive(g_lock);

        const AS500Can::Status can_status = g_can.pollStatus();
        interface_fnr = can_status.interface_fnr;
        interface_fnr_valid = can_status.interface_fnr_valid;
        /* The Interface input image is zero until its PCA9555 scan is ready.
           IOI_4 is an NC E-stop input, so accepting that startup zero would
           send a false E-stop which the STM32 intentionally latches. Arm the
           input only after observing the normal/high state at least once. */
        if (can_status.interface_fnr_valid &&
            (can_status.interface_gpio_input_state &
             AS500_IO_INPUT_EMERGENCY_NORMAL_MASK) != 0) {
            interface_emergency_armed = true;
        }
        AS500UI::setVehicleFNR(
            interface_fnr_valid && interface_fnr == AS500_FNR_FORWARD ? CMD_FORWARD :
            interface_fnr_valid && interface_fnr == AS500_FNR_REVERSE ? CMD_REVERSE :
            CMD_NONE);
        AS500UI::setInterfaceCruisePressed(
            can_status.interface_cruise_pressed);
        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_bot.steering_angle_raw = can_status.steering_angle_raw;
        g_bot.steering_angle_deg = can_status.steering_angle_deg;
        g_bot.steering_speed_dps = can_status.steering_speed_dps;
        g_bot.steering_turns = can_status.steering_turns;
        g_bot.steering_temperature_c = can_status.steering_temperature_c;
        g_bot.steering_age_ms = can_status.steering_age_ms;
        g_bot.steering_valid = can_status.steering_valid;
        g_bot.can_motor1_rpm = can_status.motor1_rpm;
        g_bot.can_motor2_rpm = can_status.motor2_rpm;
        g_bot.can_motion_age_ms = can_status.motion_age_ms;
        g_bot.can_motion_valid = can_status.motion_valid;
        g_bot.can_motion_brake_applied = can_status.motion_brake_applied;
        g_bot.can_emergency = can_status.motion_valid &&
                              can_status.motion_emergency;
        g_bot.interface_gpio_input_state = can_status.interface_gpio_input_state;
        g_bot.interface_gpio_output_state = can_status.interface_gpio_output_state;
        g_bot.interface_throttle1 = can_status.interface_throttle1;
        g_bot.interface_throttle2 = can_status.interface_throttle2;
        g_bot.interface_vr1 = can_status.interface_vr1;
        g_bot.interface_vr2 = can_status.interface_vr2;
        g_bot.interface_analog_age_ms = can_status.interface_analog_age_ms;
        g_bot.interface_analog_valid = can_status.interface_analog_valid;
        g_bot.imu_valid = can_status.interface_imu_valid;
        if (can_status.interface_imu_valid) {
            g_bot.imu_pitch_deg = can_status.interface_pitch_deg +
                                  AS500UI::as48SettingValue(4);
            g_bot.imu_roll_deg = can_status.interface_roll_deg +
                                 AS500UI::as48SettingValue(5);
        }
        xSemaphoreGive(g_lock);

        /* A non-zero pedal value has priority over cruise. Start/maintain
           manual travel from FNR while either throttle is present. When both
           return to zero, release pedal control; cruise may then keep the
           selected direction active while its button remains held. */
        const uint8_t interface_throttle = can_status.interface_analog_valid
            ? std::max(AS500UI::calibratedAnalogValue(
                           0, can_status.interface_throttle1),
                       AS500UI::calibratedAnalogValue(
                           1, can_status.interface_throttle2))
            : 0u;
        const bool throttle_present = interface_throttle != 0u;
        const control_cmd_t throttle_direction =
            interface_fnr_valid && interface_fnr == AS500_FNR_FORWARD
                ? CMD_FORWARD
                : interface_fnr_valid && interface_fnr == AS500_FNR_REVERSE
                    ? CMD_REVERSE : CMD_NONE;
        if (throttle_present && throttle_direction != CMD_NONE) {
            robot_state_t current_state;
            bool emergency_active;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            current_state = g_bot.state;
            emergency_active = g_bot.gpio_emergency || g_bot.can_emergency ||
                               current_state == S_ESTOP;
            xSemaphoreGive(g_lock);
            const robot_state_t desired_state = throttle_direction == CMD_FORWARD
                ? S_MAN_FWD : S_MAN_REV;
            if (!emergency_active && current_state != desired_state)
                apply_hmi_command(throttle_direction);
        } else if (throttle_present) {
            /* Pedal input with neutral/invalid FNR must never fall through to
               cruise. */
            apply_hmi_command(CMD_STOP);
        } else if (throttle_override_active && !AS500UI::cruisePressed()) {
            apply_hmi_command(CMD_STOP);
        }
        throttle_override_active = throttle_present;

        /* Match the former Interface alarm policy, but use P4 settings:
           an enabled threshold must be exceeded continuously for 3 seconds. */
        const int64_t tilt_now_us = esp_timer_get_time();
        const bool tilt_enabled = AS500UI::as48SettingValue(3) >= 0.5f;
        const float pitch_abs = fabsf(can_status.interface_pitch_deg +
                                      AS500UI::as48SettingValue(4));
        const float roll_abs = fabsf(can_status.interface_roll_deg +
                                     AS500UI::as48SettingValue(5));
        const bool pitch_over = can_status.interface_imu_valid && tilt_enabled &&
                                pitch_abs >= AS500UI::as48SettingValue(1);
        const bool roll_over = can_status.interface_imu_valid && tilt_enabled &&
                               roll_abs >= AS500UI::as48SettingValue(1);
        if (pitch_over || roll_over) {
            if (tilt_over_since_us == 0) tilt_over_since_us = tilt_now_us;
            if (tilt_now_us - tilt_over_since_us >= 3000000)
                tilt_alarm = pitch_over ? 1u : 2u;
        } else {
            tilt_over_since_us = 0;
            tilt_alarm = 0;
        }
        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_bot.tilt_alarm = tilt_alarm;
        xSemaphoreGive(g_lock);

        if (last_io_command_tx == 0 ||
            tilt_now_us - last_io_command_tx >=
                (int64_t)AS500_CAN_IO_COMMAND_PERIOD_MS * 1000) {
            bool hazard_requested = false;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            hazard_requested = g_bot.hazard_on;
            xSemaphoreGive(g_lock);

            const bool left_requested = can_status.interface_fnr_valid &&
                (can_status.interface_gpio_input_state &
                 AS500_IO_INPUT_LEFT_MASK) != 0;
            const bool right_requested = can_status.interface_fnr_valid &&
                (can_status.interface_gpio_input_state &
                 AS500_IO_INPUT_RIGHT_MASK) != 0;
            /* 500 ms ON / 500 ms OFF. 0x501 reports the physical relay
               state back so the dashboard arrows follow the actual output. */
            const bool signal_on = ((tilt_now_us / 500000) & 1) == 0;
            uint8_t output_control = tilt_alarm
                ? AS500_IO_CONTROL_BUZZER_MASK : 0u;
            /* IOO_3 (RELAY4) is the brake-light output on the Interface
               board. Follow the STM32-reported EMB state so the relay
               changes only when the brake is actually applied/released. */
            if (can_status.motion_valid && can_status.motion_brake_applied)
                output_control |= AS500_IO_CONTROL_BRAKE_LIGHT_MASK;
            if (signal_on) {
                if (hazard_requested || left_requested)
                    output_control |= AS500_IO_CONTROL_LEFT_SIGNAL_MASK;
                if (hazard_requested || right_requested)
                    output_control |= AS500_IO_CONTROL_RIGHT_SIGNAL_MASK;
            }
            g_can.sendInterfaceOutputCommand(output_control);
            last_io_command_tx = tilt_now_us;
        }
        if (can_status.valid) {
            last_can_status_rx = esp_timer_get_time();
            if (!gpio_emergency &&
                g_manual_startup.observe(last_can_status_rx, can_status.current_received_us,
                    can_status.current_deciamps, can_status.motor1_rpm, can_status.motor2_rpm)) {
                ESP_LOGI("STARTUP", "ESTIMATED peak_dA=%d current_dA=%u rpm=%d/%d",
                    g_manual_startup.evidence.peak, can_status.current_deciamps,
                    can_status.motor1_rpm, can_status.motor2_rpm);
            }
            if (!gpio_emergency &&
                g_follow_startup.assist.observe(last_can_status_rx,
                    can_status.current_received_us, can_status.current_deciamps,
                    can_status.motor1_rpm, can_status.motor2_rpm)) {
                ESP_LOGI("FOLLOW_STARTUP", "ESTIMATED peak_dA=%d current_dA=%u rpm=%d/%d",
                    g_follow_startup.assist.evidence.peak, can_status.current_deciamps,
                    can_status.motor1_rpm, can_status.motor2_rpm);
            }
            xSemaphoreTake(g_lock, portMAX_DELAY);
            g_bot.can_voltage_decivolts = can_status.battery_decivolts;
            g_bot.can_current_deciamps = can_status.current_deciamps;
            g_bot.can_fet_temperature_c = can_status.fet_temperature_c;
            g_bot.can_relay_status = can_status.relay_status;
            g_bot.can_switch_status = can_status.switch_status;
            g_bot.can_error_code = can_status.error_code;
            g_bot.can_brake_applied = can_status.brake_applied;
            g_bot.can_rx_ok = true;
            g_bot.can_rx_age_ms = 0;
            xSemaphoreGive(g_lock);
        }
        const int64_t can_age_ms = last_can_status_rx == 0 ? 65535 :
            (esp_timer_get_time() - last_can_status_rx) / 1000;
        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_bot.can_rx_age_ms = can_age_ms > 65535 ? 65535 : (uint16_t)can_age_ms;
        /* CAN display/telemetry is valid only while ID 0x5A0 is fresh. */
        g_bot.can_rx_ok = last_can_status_rx != 0 && can_age_ms <= 300;
        xSemaphoreGive(g_lock);

        /* Setting 15 changes only the ranging input source:
           SIMULATION = UART D frames, REAL = ESP-NOW Tag frames. */
        const bool sim_enabled = simulation_enabled();
        if (sim_enabled != last_sim_enabled) {
            g_range_kalman_a0.reset();g_range_kalman_a1.reset();g_angle_kalman.reset();
            last_rx = 0;
            g_last_valid_ranges_us = 0;
            g_last_complete_ranges_us = 0;
            g_range_sample_substituted = false;
            g_range_pair_error = false;
            g_raw_shadow_crossed = false;
            g_raw_balance_valid = false;
            g_previous_raw_left = g_previous_raw_right = 0.0f;
            g_previous_raw_balance_delta = 0.0f;
            g_last_geometry_pair={0,0,false,false};g_last_geometry_pair_us=0;
            g_uwb_divergence = {};
            g_display_range_l = g_display_range_r = 0;
            g_follow_geometry_rx_us = 0;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            g_bot.sL = g_bot.sR = 0;
            g_bot.mask = 0;
            g_bot.target_pose_valid = false;
            g_bot.target_ranges_held = false;
            g_bot.target_geometry_shadow = false;
            g_bot.target_range_pair_error = false;
            g_bot.state = g_follow_enabled ? S_NOSIG : S_IDLE;
            xSemaphoreGive(g_lock);
            last_sim_enabled = sim_enabled;
            ESP_LOGI(TAG, "Follow input changed to %s",
                     sim_enabled ? "UART_SIMULATOR" : "ESPNOW_TAG");
        }

        AS500Uart::Frame uart_frame = poll_usb_console();
        const bool uart_from_usb = uart_frame.valid;
        if (!uart_frame.valid) uart_frame = g_uart.poll(0);
        if (uart_frame.valid &&
            uart_frame.type == AS500Uart::Frame::Type::ConfigGet) {
            if (uart_from_usb) config_usb_subscribed = true;
            else config_uart_subscribed = true;
            send_follow_config(uart_from_usb);
        }
        if (uart_frame.valid && uart_frame.type == AS500Uart::Frame::Type::TurnSet) {
            AS500UI::setFollowTurnRpm(uart_frame.turn_rpm);
            send_follow_config(uart_from_usb);
        }
        if (uart_frame.valid &&
            uart_frame.type == AS500Uart::Frame::Type::ConfigSet) {
            if (uart_from_usb) config_usb_subscribed = true;
            else config_uart_subscribed = true;
            const bool saved = AS500UI::setFollowSimulatorSettings(
                uart_frame.config_max_speed_m_s,
                uart_frame.config_stop_distance_mm,
                uart_frame.config_max_estimate_distance_m,
                uart_frame.config_near_speed_m_s,
                uart_frame.config_decel_time_s,
                uart_frame.config_accel_time_s,
                uart_frame.config_turn_gain,
                uart_frame.config_turn_pwm_percent);
            if (saved) {
                write_serial_response(uart_from_usb, "ACK,CFG,SAVED\n");
                send_follow_config(uart_from_usb);
            } else {
                write_serial_response(uart_from_usb, "ERR,CFG,RANGE\n");
            }
        }

        /* Push settings changed from the EZEV menu to both serial paths. */
        const float current_follow_config[8] = {
            AS500UI::settingValue(2), AS500UI::settingValue(0),
            AS500UI::settingValue(1), AS500UI::settingValue(3),
            AS500UI::settingValue(4), AS500UI::settingValue(5),
            AS500UI::settingValue(23),
            AS500UI::settingValue(24)
        };
        bool follow_config_changed = false;
        for (int i = 0; i < 8; ++i) {
            if (current_follow_config[i] != published_follow_config[i]) {
                follow_config_changed = true;
                published_follow_config[i] = current_follow_config[i];
            }
        }
        if (follow_config_changed) {
            if (config_usb_subscribed) send_follow_config(true);
            if (config_uart_subscribed) send_follow_config(false);
        }
        if (uart_frame.valid &&
            uart_frame.type == AS500Uart::Frame::Type::Distance) {
            char ack[64];
            snprintf(ack, sizeof(ack), "ACK,D,%d,%d,%u\n",
                     uart_frame.dL, uart_frame.dR,
                     (unsigned)uart_frame.mask);
            if (uart_from_usb) {
                printf("%s", ack);
                fflush(stdout);
            } else {
                g_uart.writeLine(ack);
            }

            robot_t local;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            local = g_bot;
            xSemaphoreGive(g_lock);
            local.uart_command_valid = true;
            local.uart_command_type = 2;
            local.uart_command_left = uart_frame.dL;
            local.uart_command_right = uart_frame.dR;
            local.uart_command_mask = uart_frame.mask;

            /* Web Serial arrives through USB Serial/JTAG and is always the
               simulator source. Product UART1 remains selectable by setting
               15 so changing to a real Tag only changes the input source. */
            const bool use_uart_distance = uart_from_usb || sim_enabled;
            if (use_uart_distance) {
                last_rx = esp_timer_get_time();
                int16_t filtered_l = uart_frame.dL;
                int16_t filtered_r = uart_frame.dR;
                const uint8_t filtered_mask = filter_uwb_ranges(uart_frame.dL, uart_frame.dR,
                                  uart_frame.mask, filtered_l, filtered_r);
                local.dL = filtered_l;
                local.dR = filtered_r;
                local.mask = filtered_mask;
                local.tag_adc0 = uart_frame.adc0;
                local.tag_adc1 = uart_frame.adc1;
                local.remote_age_ms = 0;
                update_target_pose(&local);
                if (g_follow_enabled) calc_drive(&local);
                local.follow_enabled = g_follow_enabled;

                /* Output feedback is published by the periodic CAN producer. */
            }
            store_control_state(local);
        }
        if (uart_frame.valid &&
            uart_frame.type == AS500Uart::Frame::Type::Motor) {
            uart_timeout_reported = false;
            char ack[48];
            snprintf(ack, sizeof(ack), "ACK,M,%d,%d,%u\n",
                     uart_frame.motor_left, uart_frame.motor_right,
                     uart_frame.motor_enable ? 1u : 0u);
            if (uart_from_usb) {
                printf("%s", ack);
                fflush(stdout);
            } else {
                g_uart.writeLine(ack);
            }

            robot_t local;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            local = g_bot;
            xSemaphoreGive(g_lock);
            local.uart_command_valid = true;
            local.uart_command_type = 1;
            local.uart_command_left = uart_frame.motor_left;
            local.uart_command_right = uart_frame.motor_right;
            local.uart_command_mask = 0;
            local.uart_command_enable = uart_frame.motor_enable;

            /* Direct simulator motor values are allowed only while FOLLOW is
               armed. Receiving an M frame must never turn FOLLOW back off. */
            if (g_follow_enabled) {
                last_uart_motor_rx = esp_timer_get_time();
                last_rx = last_uart_motor_rx;
                uart_motor_active = uart_frame.motor_enable;
                local.sL = uart_frame.motor_enable ? uart_frame.motor_left : 0;
                local.sR = uart_frame.motor_enable ? uart_frame.motor_right : 0;
                local.state = uart_frame.motor_enable ? S_FOLLOW : S_IDLE;
            } else {
                uart_motor_active = false;
            }
            local.follow_enabled = g_follow_enabled;
            store_control_state(local);
        }

        if (uart_motor_active &&
            esp_timer_get_time() - last_uart_motor_rx > DATA_TO_MS * 1000LL) {
            uart_motor_active = false;
            if (!uart_timeout_reported) {
                ESP_LOGW(TAG, "UART motor command timeout - E-STOP");
                uart_timeout_reported = true;
            }
            xSemaphoreTake(g_lock, portMAX_DELAY);
            g_bot.sL = g_bot.sR = 0;
            g_bot.state = S_NOSIG;
            xSemaphoreGive(g_lock);
        }

        /* ── UART 폴링 (AS500Uart가 버퍼 관리) ──────── */
        AS500EspNow::Frame frame = g_espnow.poll(pdMS_TO_TICKS(10));

        if (!sim_enabled && frame.valid &&
            (!have_sequence || frame.packet.sequence != last_sequence)) {
            last_rx = esp_timer_get_time();

            const uint16_t sequence_delta =
                (uint16_t)(frame.packet.sequence - last_sequence);
            const bool new_sequence = !have_sequence || sequence_delta != 0;
            last_sequence = frame.packet.sequence;
            have_sequence = true;
            int16_t frame_filtered_l = frame.packet.distance_a0_cm;
            int16_t frame_filtered_r = frame.packet.distance_a1_cm;
            const uint8_t filtered_mask = filter_uwb_ranges(frame.packet.distance_a0_cm,
                              frame.packet.distance_a1_cm,
                              frame.packet.valid_mask,
                              frame_filtered_l, frame_filtered_r);
            static int64_t last_range_diagnostic = 0;
            if (last_rx - last_range_diagnostic >= 500000) {
                last_range_diagnostic = last_rx;
                ESP_LOGI("UWB_RANGE", "raw=%d,%d mask=0x%02X kalman=%d,%d valid=%u held=%u pair_error=%u corrected=%u",
                    frame.packet.distance_a0_cm, frame.packet.distance_a1_cm,
                    frame.packet.valid_mask, frame_filtered_l, frame_filtered_r,
                    filtered_mask == 3, g_range_sample_substituted,
                    g_range_pair_error,
                    filtered_mask == 3 && g_last_geometry_pair.corrected);
            }

            /* 로컬 복사 후 계산 */
            robot_t local;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            local = g_bot;
            xSemaphoreGive(g_lock);

            local.dL = frame_filtered_l;
            local.dR = frame_filtered_r;
            local.mask = filtered_mask;
            local.tag_adc0 = frame.packet.adc0_raw;
            local.tag_adc1 = frame.packet.adc1_raw;
            local.remote_age_ms = 0;
            local.remote_keys = frame.packet.key_state;
            local.remote_rssi = frame.rssi;

            const uint8_t remote_code = frame.packet.key_state;
            if ((remote_code & 15) == 0) remote_release_required = false;
            if (!remote_release_required && new_sequence && remote_code != last_remote_code) {
                ESP_LOGI(TAG,
                         "TAG RX mac=%02X:%02X:%02X:%02X:%02X:%02X rssi=%d seq=%u keys=0x%02X A0=%dcm A1=%dcm valid=0x%02X",
                         frame.source_mac[0], frame.source_mac[1], frame.source_mac[2],
                         frame.source_mac[3], frame.source_mac[4], frame.source_mac[5],
                         frame.rssi, frame.packet.sequence, remote_code,
                         frame.packet.distance_a0_cm, frame.packet.distance_a1_cm,
                         frame.packet.valid_mask);
                const uint8_t previous_code = last_remote_code;
                last_remote_code = remote_code;
                apply_remote_code(remote_code, previous_code);
                remote_motion_active = RemoteDrive::mix(remote_code, 1).valid;
                xSemaphoreTake(g_lock, portMAX_DELAY);
                local = g_bot;
                xSemaphoreGive(g_lock);
                local.dL = frame_filtered_l;
                local.dR = frame_filtered_r;
                local.mask = filtered_mask;
                local.tag_adc0 = frame.packet.adc0_raw;
                local.tag_adc1 = frame.packet.adc1_raw;
            }
            update_target_pose(&local);
            if (g_follow_enabled) {
                calc_drive(&local);
                /* Reaching the stop distance stops only the motors (S_NEAR).
                   FOLLOW stays armed and resumes when the Tag moves away;
                   only an explicit remote/touch command may toggle it. */
                local.follow_enabled = g_follow_enabled;
            }

            /* 결과 저장 */
            store_control_state(local);

        }

        /* Active input-source dead-man timeout. Manual HMI motion is stopped
           by the button release/STOP event, not by UWB freshness. */
        int64_t elapsed_ms = last_rx == 0 ? DATA_TO_MS + 1LL :
                             (esp_timer_get_time() - last_rx) / 1000;
        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_bot.remote_age_ms = elapsed_ms > 65535 ? 65535 : (uint16_t)elapsed_ms;
        xSemaphoreGive(g_lock);
        if (remote_motion_active && elapsed_ms > DATA_TO_MS) {
            apply_hmi_command(CMD_STOP);
            remote_motion_active = false;
            remote_release_required = true;
            last_remote_code = REMOTE_RELEASE;
            ESP_LOGW(TAG, "Remote timeout: stopped; release direction keys to rearm");
        }
        const int64_t follow_now=esp_timer_get_time();
        const bool uwb_ranges_lost=g_last_complete_ranges_us==0||
            follow_now-g_last_complete_ranges_us>UWB_LOSS_TO_MS*1000LL;
        if (g_follow_enabled && uwb_ranges_lost) {
            xSemaphoreTake(g_lock, portMAX_DELAY);
            if (g_bot.state != S_ESTOP && g_bot.state != S_NOSIG) {
                g_bot.sL    = g_bot.sR = 0;
                g_bot.mask = 0;
                g_bot.target_pose_valid = false;
                g_bot.state = S_NOSIG;
                g_bot.follow_enabled = true;
                g_uwb_restart_front.store(true);
                g_follow_control_reset.store(true);
                ESP_LOGW(TAG, "%s timeout (%lld ms) - FOLLOW remains armed",
                         sim_enabled ? "UART simulator" : "active input",
                         elapsed_ms);
            }
            xSemaphoreGive(g_lock);
        }

        /* Calculate and transmit the ESP32-P4-owned 0x701 drive command. */
        const int64_t can_command_now = esp_timer_get_time();
        if (g_manual_stop_pending ||
            can_command_now - last_can_command_tx >=
                (int64_t)AS500_CAN_DRIVE_COMMAND_PERIOD_MS * 1000) {
            int16_t command_left;
            int16_t command_right;
            robot_state_t command_state;
            bool command_gpio_emergency;
            bool command_can_emergency;
            robot_t drive_snapshot;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            drive_snapshot = g_bot;
            command_left = g_bot.sL;
            command_right = g_bot.sR;
            command_state = g_bot.state;
            command_gpio_emergency = g_bot.gpio_emergency;
            command_can_emergency = g_bot.can_emergency;
            xSemaphoreGive(g_lock);
            const bool command_interface_emergency =
                interface_emergency_armed && can_status.interface_fnr_valid &&
                (can_status.interface_gpio_input_state &
                 AS500_IO_INPUT_EMERGENCY_NORMAL_MASK) == 0;
            /* E-stop is carried only by 0x701 byte6 bit4. Byte7 remains the
               auxiliary-relay command port. */
            const bool command_emergency = command_state == S_ESTOP ||
                                           command_gpio_emergency ||
                                           command_can_emergency ||
                                           command_interface_emergency;
            const bool manual_command = is_manual_drive_state(command_state);
            const bool manual_pivot = command_state == S_MAN_LEFT ||
                                      command_state == S_MAN_RIGHT;

            if (!manual_command || command_emergency || g_manual_stop_pending ||
                g_jog_force_zero || g_manual_jog.active()) g_manual_startup.cancel();
            const bool startup_was_running = g_manual_startup.running;
            const bool startup_command = g_manual_startup.update(can_command_now);
            if (startup_was_running && !startup_command && !g_manual_startup.estimated)
                ESP_LOGI("STARTUP", "TIMEOUT_UNCONFIRMED peak_dA=%d",
                         g_manual_startup.evidence.peak);

            const bool follow_command = g_follow_enabled &&
                (command_state == S_FOLLOW || command_state == S_ALIGN ||
                 command_state == S_TURN_L || command_state == S_TURN_R);
            const bool lidar_blocks_assist = drive_snapshot.lidar_ok &&
                drive_snapshot.lidar_nearest_mm > 0 &&
                drive_snapshot.lidar_nearest_mm <= AS500UI::settingValue(11) * 10.0f;
            const bool follow_assist_allowed = follow_command &&
                !command_emergency && !g_manual_stop_pending && !g_jog_force_zero &&
                drive_snapshot.can_rx_ok && drive_snapshot.can_rx_age_ms <= 150 &&
                (drive_snapshot.mask & 3) == 3 &&
                g_follow_startup_clearance && !lidar_blocks_assist &&
                g_follow_geometry_rx_us != 0 &&
                can_command_now - g_follow_geometry_rx_us <= 500000 &&
                /* A negative inner wheel is still travelling steering. Do
                   not misclassify it as an in-place pivot startup. */
                !(g_follow_translating_steer.load() &&
                  FollowTurnLimit::pivot(command_left, command_right));
            const bool follow_was_running = g_follow_startup.assist.running;
            const bool follow_assist = g_follow_startup.update(can_command_now,
                command_left, command_right, follow_assist_allowed,
                drive_snapshot.can_current_deciamps, drive_snapshot.can_motor1_rpm,
                drive_snapshot.can_motor2_rpm,
                (unsigned long long)lroundf(AS500UI::settingValue(9)) * 1000ULL);
            if (follow_was_running && !follow_assist && follow_assist_allowed &&
                !g_follow_startup.assist.estimated &&
                can_command_now - g_follow_startup.assist.since >= g_follow_startup.assist.window)
                ESP_LOGI("FOLLOW_STARTUP", "TIMEOUT_UNCONFIRMED peak_dA=%d",
                         g_follow_startup.assist.evidence.peak);

            if (command_emergency) {
                g_manual_jog.cancel();
                g_jog_force_zero = false;
                can_output_left = 0;
                can_output_right = 0;
                manual_ramp_active = false;
            } else if (g_manual_stop_pending || g_jog_force_zero) {
                can_output_left = can_output_right = 0;
                manual_ramp_active = false;
                g_jog_force_zero = false;
            } else if (g_manual_jog.active()) {
                const bool pulse = g_manual_jog.output(can_command_now);
                can_output_left = pulse ? command_left : 0;
                can_output_right = pulse ? command_right : 0;
                manual_ramp_active = false;
                if (!g_manual_jog.active()) {
                    xSemaphoreTake(g_lock, portMAX_DELAY);
                    g_bot.sL = g_bot.sR = 0;
                    g_bot.state = S_IDLE;
                    xSemaphoreGive(g_lock);
                    command_state = S_IDLE;
                }
            } else if (startup_command) {
                // STM32 performs the smooth PWM ramp. Cruise speed must not
                // reduce the initial drive command needed to overcome load.
                can_output_left = (int16_t)std::clamp(
                    RemoteDrive::assisted(command_left, command_left, command_right,
                        manual_pivot ? ManualStartup::PivotCommand : ManualStartup::StraightCommand),
                    -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
                can_output_right = (int16_t)std::clamp(
                    RemoteDrive::assisted(command_right, command_left, command_right,
                        manual_pivot ? ManualStartup::PivotCommand : ManualStartup::StraightCommand),
                    -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
                manual_ramp_active = true;
            } else if (follow_assist) {
                can_output_left = (int16_t)std::clamp(
                    FollowStartup::command(command_left, command_left, command_right),
                    -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
                can_output_right = (int16_t)std::clamp(
                    FollowStartup::command(command_right, command_left, command_right),
                    -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
                g_follow_startup.noteCommand(can_output_left, can_output_right);
                manual_ramp_active = false;
            } else if (manual_command || follow_command) {
                // Keep one ramp for all manual directions, including handover.
                // STM32 handles smooth PWM and direction-change
                // deceleration/settling. Avoid retargeting its curve every 100 ms.
                can_output_left = (int16_t)std::clamp((int)command_left,
                                                     -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
                can_output_right = (int16_t)std::clamp((int)command_right,
                                                      -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
                manual_ramp_active = true;
            } else if (manual_command || manual_ramp_active) {
                const int accel_step = (int)lroundf(
                    MOTOR_MAX_RPM * 0.1f /
                    std::max(AS500UI::settingValue(6), 0.1f));
                const int decel_step = (int)lroundf(
                    MOTOR_MAX_RPM * 0.1f /
                    std::max(AS500UI::settingValue(7), 0.1f));
                const int16_t target_left = manual_command
                    ? (int16_t)std::clamp((int)command_left,
                                           -MOTOR_MAX_RPM, MOTOR_MAX_RPM)
                    : 0;
                const int16_t target_right = manual_command
                    ? (int16_t)std::clamp((int)command_right,
                                           -MOTOR_MAX_RPM, MOTOR_MAX_RPM)
                    : 0;
                can_output_left = ramp_motor_command(
                    can_output_left, target_left, accel_step, decel_step);
                can_output_right = ramp_motor_command(
                    can_output_right, target_right, accel_step, decel_step);
                manual_ramp_active = manual_command || can_output_left != 0 ||
                                     can_output_right != 0;
            } else {
                can_output_left = (int16_t)std::clamp(
                    (int)command_left, -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
                can_output_right = (int16_t)std::clamp(
                    (int)command_right, -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
            }

            /* Startup assistance must obey the same user-set follow pivot
               limit. Apply it after every command path so no helper can raise
               an opposing-wheel command beyond setting 08. */
            const bool follow_pivot_command = follow_command &&
                FollowTurnLimit::pivot(command_left,command_right);
            if (follow_pivot_command) {
                can_output_left = (int16_t)FollowTurnLimit::apply(can_output_left,
                    AS500UI::settingValue(24),MOTOR_MAX_RPM);
                can_output_right = (int16_t)FollowTurnLimit::apply(can_output_right,
                    AS500UI::settingValue(24),MOTOR_MAX_RPM);
            }

            /* A pedal value has priority. Cruise supplies a full-scale demand
               only while both Throttle1 and Throttle2 are zero. */
            const uint8_t throttle_raw = drive_snapshot.interface_analog_valid
                ? std::max(AS500UI::calibratedAnalogValue(
                               0, drive_snapshot.interface_throttle1),
                           AS500UI::calibratedAnalogValue(
                               1, drive_snapshot.interface_throttle2))
                : 0u;
            const uint8_t drive_demand_raw = throttle_raw != 0u
                ? throttle_raw
                : (AS500UI::cruisePressed() ? 255u : 0u);
            can_output_left = (int16_t)(((int32_t)can_output_left *
                                         drive_demand_raw) / 255);
            can_output_right = (int16_t)(((int32_t)can_output_right *
                                          drive_demand_raw) / 255);

            const uint8_t speed_limit_raw = drive_snapshot.interface_analog_valid
                ? AS500UI::calibratedAnalogValue(
                      2, drive_snapshot.interface_vr1) : 0u;
            can_output_left = (int16_t)(((int32_t)can_output_left *
                                         speed_limit_raw) / 255);
            can_output_right = (int16_t)(((int32_t)can_output_right *
                                          speed_limit_raw) / 255);

            int16_t tx_left = rpm_to_can_command(can_output_left);
            int16_t tx_right = rpm_to_can_command(can_output_right);
            const uint8_t tx_fnr = interface_fnr_valid
                ? interface_fnr : AS500_FNR_NEUTRAL;
            if (tx_fnr == AS500_FNR_NEUTRAL) {
                tx_left = tx_right = 0;
            } else {
                /* FNR owns longitudinal direction. Preserve the left/right
                   steering ratio, but reverse the complete pair when its
                   current travel direction disagrees with the selector. */
                const int pair_sum = (int)tx_left + (int)tx_right;
                if ((tx_fnr == AS500_FNR_FORWARD && pair_sum < 0) ||
                    (tx_fnr == AS500_FNR_REVERSE && pair_sum > 0)) {
                    tx_left = (int16_t)-tx_left;
                    tx_right = (int16_t)-tx_right;
                }
            }
            xSemaphoreTake(g_lock, portMAX_DELAY);
            g_bot.can_command_left = tx_left;
            g_bot.can_command_right = tx_right;
            xSemaphoreGive(g_lock);
            const bool timing_follow = follow_command;
            const uint8_t accel_raw = (uint8_t)DriveTiming::encode(
                AS500UI::settingValue(DriveTiming::accelIndex(timing_follow)));
            const uint8_t decel_raw = (uint8_t)DriveTiming::encode(
                AS500UI::settingValue(DriveTiming::decelIndex(timing_follow)));
            uint8_t input_port = (uint8_t)
                (drive_snapshot.interface_gpio_input_state & 0xFFu);
            /* Interface IOI_4 is NC (1=normal, 0=E-stop). Normalize the
               forwarded 0x701 bit to 1=E-stop and avoid a boot-time trip. */
            input_port &= (uint8_t)~AS500_DRIVE_INPUT_ESTOP_MASK;
            if (command_interface_emergency)
                input_port |= AS500_DRIVE_INPUT_ESTOP_MASK;
            /* Cruise is momentary. Either physical Interface switch or the
               P4 touch button may assert it; releasing both clears it. */
            input_port &= (uint8_t)~AS500_DRIVE_INPUT_CRUISE_MASK;
            if (AS500UI::cruisePressed())
                input_port |= AS500_DRIVE_INPUT_CRUISE_MASK;
            /* Byte 7 is a command, not a copy of the 0x5A0 feedback byte.
               RY1/RY2/RY3 belong exclusively to the STM32. */
            uint8_t relay_port = 0U;
            if (drive_snapshot.pump_on)
                relay_port |= AS500_RELAY_PUMP_MASK;
            if (drive_snapshot.blower_on)
                relay_port |= AS500_RELAY_BLOWER_MASK;
            if (drive_snapshot.engine_fan_on)
                relay_port |= AS500_RELAY_ENGINE_FAN_MASK;
            const bool tx_ok = g_can.sendDriveCommand(
                tx_left, tx_right, accel_raw, decel_raw, tx_fnr,
                command_emergency, input_port, relay_port);
            robot_t tx_snapshot;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            tx_snapshot = g_bot;
            xSemaphoreGive(g_lock);
            static uint32_t tx_sequence = 0;
            char ctrl[160];
            // Report the exact values transmitted by the AS500 HMI.
            snprintf(ctrl, sizeof(ctrl),
                     "CTRL,%d,%d,%d,%d,%u,%s,%u,%u,%u,%lu,%u,%u\n",
                     tx_left, tx_right, tx_snapshot.can_motor1_rpm,
                     tx_snapshot.can_motor2_rpm, tx_snapshot.can_rx_ok ? 1u : 0u,
                     AS500UI::stateText(command_state),
                     tx_snapshot.can_voltage_decivolts, tx_snapshot.can_current_deciamps,
                     tx_snapshot.can_rx_age_ms, (unsigned long)++tx_sequence,
                     tx_ok ? 1u : 0u, tx_snapshot.can_brake_applied ? 1u : 0u);
            if (kLogPeriodicControlPose) {
                g_uart.writeLine(ctrl);
                if (g_usb_serial_ready) write_serial_response(true, ctrl);
            }
            static float published_width = -1, published_alpha = -1, published_gate = -1,
                         published_shadow_multiplier = -1;
            const float width = UwbGeometry::kAnchorBaselineCm;
            const float alpha = AS500UI::settingValue(16);
            const float gate = AS500UI::settingValue(17);
            const float shadow_multiplier = AS500UI::settingValue(26);
            if (width != published_width || alpha != published_alpha || gate != published_gate ||
                shadow_multiplier != published_shadow_multiplier) {
                char geometry[96];
                snprintf(geometry, sizeof(geometry), "GEOM,%.1f,%.1f,%.2f,%.1f,%.0f\n",
                    (double)width,
                    (double)UwbGeometry::kLongitudinalGapCm,
                    (double)alpha, (double)gate, (double)shadow_multiplier);
                g_uart.writeLine(geometry);
                if (g_usb_serial_ready) write_serial_response(true, geometry);
                published_width = width; published_alpha = alpha; published_gate = gate;
                published_shadow_multiplier = shadow_multiplier;
            }
            char pose[112];
            const bool pose_valid = tx_snapshot.target_pose_valid &&
                (tx_snapshot.mask & 3) == 3 &&g_last_complete_ranges_us>0&&
                can_command_now-g_last_complete_ranges_us<=UWB_LOSS_TO_MS*1000LL;
            snprintf(pose, sizeof(pose), "POSE,%u,%.1f,%.2f,%d,%d\n",
                pose_valid ? 1u : 0u,
                (double)(tx_snapshot.target_distance_cm * 10.0f),
                (double)tx_snapshot.target_angle_deg, tx_snapshot.dL, tx_snapshot.dR);
            if (kLogPeriodicControlPose) {
                g_uart.writeLine(pose);
                if (g_usb_serial_ready) write_serial_response(true, pose);
            }
            g_manual_stop_pending = false;
            last_can_command_tx = can_command_now;
        }

        /* Print a compact status line twice per second. */
        int64_t now = esp_timer_get_time();
        if ((now - last_monitor) / 1000 >= MONITOR_MS) {
            robot_t snap;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            snap = g_bot;
            xSemaphoreGive(g_lock);

            int64_t rx_age_ms = last_rx == 0 ? -1 : (now - last_rx) / 1000;
            bool uwb_ok = rx_age_ms <= DATA_TO_MS &&
                          (snap.mask & 0x03) == 0x03;
            const AS500Can::Diagnostics can_diag = g_can.diagnostics();

            printf("[STATUS] t=%8lldms state=%-9s UWB=%s age=%4lldms "
                   "mask=0x%02X dL=%4dcm dR=%4dcm avg=%4dcm diff=%+4dcm ADC0=%4u ADC1=%4u "
                   "motorL=%+5d motorR=%+5d CAN=%s st=%d txq=%lu TEC=%lu REC=%lu fail=%lu buserr=%lu\n",
                   (long long)(now / 1000),
                   AS500UI::stateText(snap.state),
                   uwb_ok ? "OK" : "LOST",
                   (long long)rx_age_ms,
                   (unsigned)snap.mask,
                   snap.dL, snap.dR,
                   (snap.dL + snap.dR) / 2,
                   snap.dR - snap.dL,
                   snap.tag_adc0, snap.tag_adc1,
                   snap.can_command_left, snap.can_command_right,
                   snap.can_ok ? "OK" : "FAIL",
                   can_diag.state, (unsigned long)can_diag.tx_queue,
                   (unsigned long)can_diag.tx_errors,
                   (unsigned long)can_diag.rx_errors,
                   (unsigned long)can_diag.tx_failed,
                   (unsigned long)can_diag.bus_errors);
            fflush(stdout);
            char uart_status[128];
            snprintf(uart_status, sizeof(uart_status),
                     "STATUS,%s,%d,%d,%d,%d,%u,%u,%u,%s,%u\n",
                     AS500UI::stateText(snap.state), snap.dL, snap.dR,
                     snap.can_command_left, snap.can_command_right,
                     snap.can_ok ? 1u : 0u,
                     snap.can_voltage_decivolts, snap.can_rx_ok ? 1u : 0u,
                     reset_reason_text(g_boot_reset_reason), snap.can_current_deciamps);
            g_uart.writeLine(uart_status);
            /* Web Serial uses USB Serial/JTAG, while a real tag uses UART1.
               Publish the same actual CAN command values on both paths. */
            if (g_usb_serial_ready) {
                printf("%s", uart_status);
                fflush(stdout);
            }
            last_monitor = now;
        }
        /* pollStatus() and all other inputs above are non-blocking. Block for
           one RTOS tick so CPU1's idle task can run and feed the task WDT. */
        vTaskDelay(1);
    }
}

/* ════════════════════════════════════════════════════════════
 *  app_main
 * ════════════════════════════════════════════════════════════ */
/* LD06 receive and 360-degree obstacle monitor. Motor-stop policy is kept
 * outside the driver so it can be tuned after installation tests. */
static void lidar_task(void *arg)
{
    (void)arg;
    bool was_detected = false;
    int64_t last_report_us = 0;
    while (1) {
        Lidar::Frame frame = g_lidar.poll(pdMS_TO_TICKS(20));
        if (!frame.valid) continue;

        uint16_t scan_distance[36];
        uint8_t scan_confidence[36];
        const uint8_t min_confidence = (uint8_t)lroundf(AS500UI::settingValue(18));
        const uint32_t max_age_ms = (uint32_t)lroundf(AS500UI::settingValue(19));
        const float detect_center_deg = AS500UI::settingValue(12);
        const float detect_half_width_deg = AS500UI::settingValue(13) * 0.5f;
        const uint16_t lidar_max_distance_mm = (uint16_t)lroundf(
            AS500UI::settingValue(20) * 10.0f);
        int scan_count = g_lidar.copyScan(scan_distance, scan_confidence, 36, max_age_ms);
        Lidar::Obstacle nearest = g_lidar.closestObstacle(
            detect_center_deg, detect_half_width_deg, lidar_max_distance_mm,
            min_confidence, max_age_ms);
        const uint16_t stop_mm = (uint16_t)lroundf(
            AS500UI::settingValue(11) * 10.0f);
        Lidar::Obstacle obstacle = g_lidar.closestObstacle(
            detect_center_deg, detect_half_width_deg, stop_mm,
            min_confidence, max_age_ms);

        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_bot.lidar_ok = scan_count > 0;
        for (int i = 0; i < 36; ++i) {
            g_bot.lidar_distance_mm[i] = scan_distance[i];
            g_bot.lidar_confidence[i] = scan_confidence[i];
        }
        g_bot.lidar_nearest_mm = nearest.detected ? nearest.distance_mm : 0;
        g_bot.lidar_nearest_angle_deg = nearest.detected
            ? (int16_t)lroundf(nearest.angle_deg) : 0;
        xSemaphoreGive(g_lock);
        int64_t now_us = esp_timer_get_time();
        if (obstacle.detected && (!was_detected || now_us - last_report_us >= 500000)) {
            ESP_LOGW(TAG, "LIDAR obstacle: %u mm at %.0f deg (confidence %u)",
                     obstacle.distance_mm, (double)obstacle.angle_deg,
                     obstacle.confidence);
            last_report_us = now_us;
        } else if (!obstacle.detected && was_detected) {
            ESP_LOGI(TAG, "LIDAR obstacle cleared");
        }
        was_detected = obstacle.detected;
    }
}

extern "C" void app_main(void)
{
    const esp_reset_reason_t boot_reset_reason = esp_reset_reason();
    g_boot_reset_reason = boot_reset_reason;
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    const esp_err_t usb_serial_err =
        usb_serial_jtag_driver_install(&usb_serial_config);
    if (usb_serial_err == ESP_OK) {
        usb_serial_jtag_vfs_use_driver();
        g_usb_serial_ready = true;
        printf("USB_RX_READY\n");
        printf("AS500_BUILD_LCD_DIAG_20260928_1\n");
        fflush(stdout);
    } else {
        ESP_LOGE(TAG, "USB Serial/JTAG driver initialization failed: %s",
                 esp_err_to_name(usb_serial_err));
    }
    /* ── NVS ─────────────────────────────────────────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* ── 공유 Mutex ──────────────────────────────────── */
    g_lock = xSemaphoreCreateMutex();
    assert(g_lock != NULL);
    restore_auxiliary_state();

    /* ESP-NOW receiver: onboard ESP32-C6 through ESP-Hosted/SDIO. */
    if (kEnableHostedEspNow) {
        if (!g_espnow.init()) {
            ESP_LOGE(TAG, "ESP-NOW receiver initialization failed");
        }
    } else {
        ESP_LOGW(TAG, "Hosted ESP-NOW disabled; continuing without C6 relay");
    }

    /* One-time update of the onboard C6 to the EZ ESP-NOW relay image. */
    nvs_handle_t ota_nvs = 0;
    uint8_t relay_installed = 0;
    if (nvs_open("ez_relay", NVS_READWRITE, &ota_nvs) == ESP_OK) {
        nvs_get_u8(ota_nvs, "installed", &relay_installed);
        if (!relay_installed && g_espnow.isReady()) {
            extern const uint8_t c6_relay_start[] asm("_binary_c6_espnow_relay_bin_start");
            extern const uint8_t c6_relay_end[] asm("_binary_c6_espnow_relay_bin_end");
            const size_t relay_size = c6_relay_end - c6_relay_start;
            ESP_LOGI(TAG, "Updating onboard C6 relay (%u bytes)", (unsigned)relay_size);
            esp_err_t ota_err = esp_hosted_slave_ota_begin();
            for (size_t offset = 0; ota_err == ESP_OK && offset < relay_size; offset += 4096) {
                const size_t chunk = std::min<size_t>(4096, relay_size - offset);
                ota_err = esp_hosted_slave_ota_write(
                    const_cast<uint8_t *>(c6_relay_start + offset), (uint32_t)chunk);
            }
            if (ota_err == ESP_OK) ota_err = esp_hosted_slave_ota_end();
            if (ota_err == ESP_OK) {
                nvs_set_u8(ota_nvs, "installed", 1);
                nvs_commit(ota_nvs);
                ESP_LOGI(TAG, "C6 relay OTA complete; activating image");
                esp_hosted_slave_ota_activate();
            } else {
                ESP_LOGE(TAG, "C6 relay OTA failed: %s", esp_err_to_name(ota_err));
            }
        }
        nvs_close(ota_nvs);
    }

    /* UART1 is occupied by the UWB anchor, so LD06 uses UART2. */
    if (!g_uart.init(UART_RX_GPIO, UART_TX_GPIO, 115200, UART_NUM_1)) {
        ESP_LOGE(TAG, "Simulator/UWB UART initialization failed");
    }

    const gpio_config_t vacuum_gpio_config = {
        .pin_bit_mask = 1ULL << VACUUM_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&vacuum_gpio_config) == ESP_OK) {
        gpio_set_level((gpio_num_t)VACUUM_GPIO, 0);
        g_vacuum_gpio_ready = true;
        ESP_LOGI(TAG, "Vacuum output ready: GPIO%d, active HIGH", VACUUM_GPIO);
    } else {
        ESP_LOGE(TAG, "Vacuum GPIO%d initialization failed", VACUUM_GPIO);
    }

    const gpio_config_t estop_gpio_config = {
        .pin_bit_mask = 1ULL << ESTOP_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&estop_gpio_config) == ESP_OK) {
        g_estop_gpio_ready = true;
        ESP_LOGI(TAG, "E-STOP input ready: GPIO%d, active LOW", ESTOP_GPIO);
    } else {
        ESP_LOGE(TAG, "E-STOP GPIO%d initialization failed", ESTOP_GPIO);
    }
    char boot_status[48];
    snprintf(boot_status, sizeof(boot_status), "BOOT,%s,%d\n",
             reset_reason_text(boot_reset_reason), (int)boot_reset_reason);
    g_uart.writeLine(boot_status);
    if (!g_lidar.init(LIDAR_TXD_GPIO, LIDAR_PWM_GPIO)) {
        ESP_LOGE(TAG, "LD06 initialization failed");
    }

    /* ── CAN / TWAI 초기화 ───────────────────────────── */
    g_bot.can_ok = g_can.init(CAN_TX_GPIO, CAN_RX_GPIO);
    if (!g_bot.can_ok) {
        ESP_LOGE(TAG, "CAN initialization failed");
    }

    /* ── 디스플레이 + LVGL (BSP) ─────────────────────── */
    /* Match the vendor's working JC1060P470C_I_W_Y LVGL v9 setup. */
    bsp_display_cfg_t disp_cfg = {
        .lvgl_port_cfg = {
            .task_priority = 4,
            .task_stack = 16384,
            .task_affinity = -1,
            .task_max_sleep_ms = 500,
            .task_stack_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DEFAULT,
            .timer_period_ms = 5,
        },
        .buffer_size   = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma    = false,
            .buff_spiram = false,
            .sw_rotate   = true,
        },
    };
    printf("LCD_DIAG: init begin\n");
    fflush(stdout);
    lv_display_t *disp = bsp_display_start_with_config(&disp_cfg);
    if (disp == NULL) {
        ESP_LOGE(TAG, "Display init FAILED");
        printf("LCD_DIAG: init FAILED\n");
        fflush(stdout);
        return;
    }
    printf("LCD_DIAG: init OK\n");
    fflush(stdout);
    ESP_LOGI(TAG, "Display %dx%d ready", BSP_LCD_H_RES, BSP_LCD_V_RES);

    /* ES_I2C is the BSP bus on GPIO7(SDA)/GPIO8(SCL). Reuse it; never
       install a second driver on the codec/display-owned bus. */
    i2c_master_bus_handle_t es_i2c=bsp_i2c_get_handle();
    if(es_i2c){
        esp_err_t pca_err=g_pca9555.begin(es_i2c,0x21);
        ESP_LOGI(TAG,"PCA9555 0x21: %s",esp_err_to_name(pca_err));
        g_mpu6050_ready=g_mpu6050.begin(es_i2c);
        if(g_mpu6050_ready)g_mpu6050_ready=g_mpu6050.calibrate(500);
        ESP_LOGI(TAG,"MPU6050 0x68: %s",g_mpu6050_ready?"ready":"not found");
    }else ESP_LOGE(TAG,"ES_I2C BSP handle unavailable");

    /* ── UI 생성 ─────────────────────────────────────── */
    if (!bsp_display_lock(portMAX_DELAY)) {
        ESP_LOGE(TAG, "LVGL lock FAILED");
        return;
    }
    g_ui.init(g_bot.can_ok);
    /* Push the first frame before leaving initialization.  This also makes
       the boot screen deterministic if the LVGL task has not run yet. */
    lv_obj_invalidate(lv_screen_active());
    lv_refr_now(disp);
    printf("LCD_DIAG: first frame rendered\n");
    const esp_err_t backlight_err = bsp_display_backlight_on();
    printf("LCD_DIAG: backlight after first frame %s\n",
           esp_err_to_name(backlight_err));
    fflush(stdout);
    ESP_LOGI(TAG, "Heap after UI: internal=%lu largest=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    lv_timer_create(lvgl_tick_cb, 200, NULL);   /* 5 Hz UI; control remains 50 ms */
    bsp_display_unlock();

    /* ── 로봇 제어 태스크 (Core 1) ───────────────────── */
    /* robot_t now contains the full LiDAR scan.  The control path also uses
     * floating-point geometry and printf, so 4 KiB can overflow as soon as
     * FOLLOW enables that deeper call chain. */
    BaseType_t robot_task_ok = xTaskCreatePinnedToCore(
        robot_task, "robot_ctrl", 8192, NULL, 5, NULL, 1);
    if (robot_task_ok != pdPASS) {
        ESP_LOGE(TAG, "robot_ctrl task creation failed");
    }
    if (g_lidar.isReady()) {
        BaseType_t lidar_task_ok = xTaskCreatePinnedToCore(
            lidar_task, "lidar_rx", 6144, NULL, 5, NULL, 1);
        if (lidar_task_ok != pdPASS) {
            ESP_LOGE(TAG, "lidar_rx task creation failed");
        }
    }

    ESP_LOGI(TAG, "=== AS500 ready ===");
}
