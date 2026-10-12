/*
 * as500_ui.h
 * ─────────────────────────────────────────────────────────────
 *  청소차 자율추종 LVGL v8 대시보드 — AS500UI 클래스 선언
 *  보드 : JC1060P470C_I_W  (ESP32-P4 + 1024×600 MIPI-DSI)
 *
 *  사용법 (main.cpp 측):
 *    1. bsp_display_lock(0) 안에서 g_ui.init(can_ok) 호출
 *    2. lv_timer 콜백 (LVGL 태스크) 에서 g_ui.update(snapshot) 호출
 *    3. 추가 잠금 불필요 — 항상 LVGL 태스크 컨텍스트에서 실행
 * ─────────────────────────────────────────────────────────────
 */
#pragma once
#include "lvgl.h"
#include <atomic>

/* ═══════════════════════════════════════════════════════════
 *  로봇 상태 열거형  (main.cpp / as500_ui.cpp 공유)
 * ═══════════════════════════════════════════════════════════ */
typedef enum {
    S_IDLE   = 0,   /* 대기                   */
    S_FOLLOW,       /* 전진 추종              */
    S_ALIGN,        /* 제자리 정렬            */
    S_NEAR,         /* 근접 정지              */
    S_TURN_L,       /* 좌회전                 */
    S_TURN_R,       /* 우회전                 */
    S_ESTOP,        /* 비상정지               */
    S_NOSIG,        /* UWB 신호 없음          */
    S_MAN_FWD,
    S_MAN_REV,
    S_MAN_LEFT,
    S_MAN_RIGHT,
} robot_state_t;

typedef enum {
    CMD_NONE = 0,
    CMD_ESTOP,
    CMD_FOLLOW_START,
    CMD_STOP,
    CMD_VACUUM_TOGGLE,
    CMD_FORWARD,
    CMD_REVERSE,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_SETTINGS,
    CMD_TAP_FORWARD, CMD_TAP_REVERSE, CMD_TAP_LEFT, CMD_TAP_RIGHT,
    CMD_FORWARD_LEFT, CMD_FORWARD_RIGHT, CMD_REVERSE_LEFT, CMD_REVERSE_RIGHT,
    CMD_HAZARD_TOGGLE, CMD_HORN_ON, CMD_HORN_OFF,
    CMD_PUMP_TOGGLE, CMD_BLOWER_TOGGLE, CMD_ENGINE_FAN_TOGGLE,
    CMD_HEADLIGHT_TOGGLE, CMD_PARKING_LIGHT_TOGGLE,
} control_cmd_t;

/* ═══════════════════════════════════════════════════════════
 *  공유 데이터 구조체  (RTOS 태스크 ↔ LVGL 타이머)
 * ═══════════════════════════════════════════════════════════ */
typedef struct {
    int16_t       dL, dR;    /* UWB 좌/우 거리 (cm)          */
    uint8_t       mask;      /* 앵커 유효 비트 (bit0=L, 1=R) */
    float target_distance_cm, target_angle_deg;
    float target_distance_error_cm, target_angle_error_deg;
    bool target_pose_valid;
    bool target_pose_corrected;
    bool target_ranges_held;
    bool target_geometry_shadow; /* direction line: branch selected by history */
    bool target_range_pair_error; /* filtered A0/A1 fail circle geometry */
    uint16_t      tag_adc0, tag_adc1; /* Tag ADC raw (GPIO35/36) */
    int16_t       sL, sR;    /* 좌/우 모터 목표 RPM (-2500~+2500) */
    int16_t       can_command_left;  /* calculated target only, 10 = 1% */
    int16_t       can_command_right; /* calculated target only, 10 = 1% */
    robot_state_t state;
    bool          can_ok;
    bool          can_rx_ok;             /* fresh STM32 0x5A0 response */
    uint16_t      can_voltage_decivolts; /* battery voltage in 0.1 V */
    uint16_t      can_current_deciamps;  /* battery current in 0.1 A */
    uint16_t      can_rx_age_ms;
    int16_t       can_motor1_rpm;
    int16_t       can_motor2_rpm;
    uint16_t      can_motion_age_ms;
    bool          can_motion_valid;
    bool          can_motion_brake_applied;
    uint16_t      interface_gpio_input_state;
    uint8_t       interface_gpio_output_state;
    uint8_t       interface_throttle1;
    uint8_t       interface_throttle2;
    uint8_t       interface_vr1;
    uint8_t       interface_vr2;
    uint16_t      interface_analog_age_ms;
    bool          interface_analog_valid;
    uint8_t       can_motor_temperature_c; /* unavailable in 0x5A0 */
    int8_t        can_fet_temperature_c;
    uint8_t       can_relay_status;
    uint8_t       can_switch_status;
    uint8_t       can_error_code;
    bool          can_brake_applied; // Driver-reported relay command
    bool          can_emergency;     // 0x5A0 has no E-stop bit
    uint16_t      steering_angle_raw;
    float         steering_angle_deg;
    float         steering_speed_dps;
    int16_t       steering_turns;
    float         steering_temperature_c;
    uint16_t      steering_age_ms;
    bool          steering_valid;
    bool          gpio_emergency;
    bool          uart_command_valid;
    uint8_t       uart_command_type; /* 1=M motor, 2=D distance */
    int16_t       uart_command_left;
    int16_t       uart_command_right;
    uint8_t       uart_command_mask;
    bool          uart_command_enable;
    bool          follow_enabled;
    bool          vacuum_on;
    bool          hazard_on;
    bool          horn_on;
    bool          pump_on;
    bool          blower_on;
    bool          engine_fan_on;
    bool          headlight_on;
    bool          parking_light_on;
    bool          lidar_ok;
    uint16_t      lidar_distance_mm[36]; /* 10-degree sectors, 0 = no data */
    uint8_t       lidar_confidence[36];
    uint16_t      lidar_nearest_mm;
    int16_t       lidar_nearest_angle_deg;
    uint16_t      remote_age_ms;       /* Last Tag ESP-NOW packet age */
    uint8_t       remote_keys;         /* F/R/L/R/STOP/ESTOP bits */
    int8_t        remote_rssi;
    bool          imu_valid;
    float         imu_pitch_deg;
    float         imu_roll_deg;
    uint8_t       tilt_alarm; /* 0=normal, 1=pitch, 2=roll */
} robot_t;

/* ═══════════════════════════════════════════════════════════
 *  AS500UI
 *
 *  LVGL 대시보드 위젯 전체를 캡슐화.
 *  생성/갱신 메서드만 외부에 공개하고 모든 핸들은 private.
 * ═══════════════════════════════════════════════════════════ */
class AS500UI {
public:
    AS500UI()  = default;
    ~AS500UI() = default;

    /**
     * @brief LVGL 위젯 전체 생성 및 초기 레이아웃 구성
     *        반드시 bsp_display_lock() 내에서 호출
     * @param can_ok  CAN 초기화 성공 여부 (우측 패널 상태 표시)
     */
    void init(bool can_ok = false);

    /**
     * @brief 로봇 상태로 전체 위젯 값 갱신
     *        LVGL 타이머 콜백(LVGL 태스크 컨텍스트)에서만 호출
     * @param d  갱신할 로봇 상태 스냅샷 (const 참조)
     */
    void update(const robot_t &d);

    /** Return and clear the latest HMI control command. */
    static control_cmd_t takeCommand();
    /** Apply the physical FNR selector received from AS500-interface. */
    static bool setVehicleFNR(control_cmd_t direction);
    /** Apply the momentary cruise key received from AS500-interface. */
    static void setInterfaceCruisePressed(bool pressed);
    /** True while either the Interface or P4 cruise key is held. */
    static bool cruisePressed();
    static bool touchPoints(const uint16_t *x, const uint16_t *y, uint8_t count);
    static void loadSettings();
    static float settingValue(int index);
    static float as48SettingValue(int index);
    /** Apply the saved OFFSET/MAX calibration to Interface analog channel
        0=Throttle1, 1=Throttle2, 2=VR1, 3=VR2. */
    static uint8_t calibratedAnalogValue(int channel, uint8_t raw);
    static bool setFollowTurnRpm(float rpm);
    static bool setFollowSimulatorSettings(float max_speed_m_s,
                                           float stop_distance_mm,
                                           float max_estimate_distance_m,
                                           float near_speed_m_s,
                                           float decel_time_s,
                                           float accel_time_s,
                                           float turn_gain,
                                           float turn_pwm_percent);

    /* ── 상태 → 표현 변환 유틸리티 (외부 사용 가능) ────── */
    static lv_color_t  stateColor(robot_state_t s);  /* 상태별 강조 색       */
    static const char *stateText (robot_state_t s);  /* 상태 문자열          */
    static const char *stateSym  (robot_state_t s);  /* LVGL 방향 심볼       */

private:
    static void buttonEvent(lv_event_t *e);
    static void auxiliaryButtonEvent(lv_event_t *e);
    static void manualButtonEvent(lv_event_t *e);
    static void cruiseButtonEvent(lv_event_t *e);
    static void currentGaugeEvent(lv_event_t *e);
    static void tripResetEvent(lv_event_t *e);
    static void tiltZeroEvent(lv_event_t *e);
    static void tiltSettingsEvent(lv_event_t *e);
    static void tiltSettingRowEvent(lv_event_t *e);
    void createTiltSettingsOverlay(lv_obj_t *screen);
    static void analogSettingsTouchEvent(lv_event_t *e);
    static void analogSettingsLongPressTimer(lv_timer_t *timer);
    static bool openAnalogSettings();
    static void analogSettingsEvent(lv_event_t *e);
    static void analogSettingAdjustEvent(lv_event_t *e);
    void createAnalogSettingsOverlay(lv_obj_t *screen);
    void refreshAnalogSettings();
    static void mp3TileEvent(lv_event_t *e);
    static void mp3SettingsEvent(lv_event_t *e);
    void createMp3SettingsOverlay();
    void refreshMp3Settings();
    static void vacuumButtonEvent(lv_event_t *e);
    static void vacuumSettingsEvent(lv_event_t *e);
    static void vehicleDrawEvent(lv_event_t *e);
    void createVacuumSettingsOverlay();
    static std::atomic<control_cmd_t> s_pending_cmd;
    static std::atomic<control_cmd_t> s_vehicle_fnr;
    static std::atomic<bool> s_interface_cruise_pressed;
    /* ── 헤더 ──────────────────────── */
    lv_obj_t *m_led_l    = nullptr;
    lv_obj_t *m_led_r    = nullptr;
    lv_obj_t *m_badge    = nullptr;

    /* ── 좌측 패널 : UWB SENSOR ───── */
    lv_obj_t *m_arc_l    = nullptr;
    lv_obj_t *m_arc_r    = nullptr;
    lv_obj_t *m_val_dl   = nullptr;
    lv_obj_t *m_val_dr   = nullptr;
    lv_obj_t *m_tag_distance = nullptr;
    lv_obj_t *m_tag_angle = nullptr;
    int64_t m_pose_display_us = 0;
    float m_pose_display_distance = 0;
    float m_pose_display_angle = 0;

    /* ── 중앙 패널 : HEADING ────────── */
    lv_obj_t *m_arrow    = nullptr;
    lv_obj_t *m_dir_txt  = nullptr;
    lv_obj_t *m_avg_val  = nullptr;
    lv_obj_t *m_diff_val = nullptr;
    lv_obj_t *m_dir_target = nullptr;
    lv_obj_t *m_dir_line = nullptr;
    lv_point_precise_t m_dir_line_points[2] = {};
    lv_obj_t *m_lidar_nearest = nullptr;
    lv_obj_t *m_vehicle = nullptr;
    uint32_t m_vehicle_color = 0x57606A;
    int8_t m_left_wheel_direction = 0;
    int8_t m_right_wheel_direction = 0;
    lv_obj_t *m_vehicle_info = nullptr;
    int16_t m_vehicle_angle_tenths = 0;
    lv_obj_t *m_serial_command = nullptr;
    lv_obj_t *m_tag_remote = nullptr;
    lv_obj_t *m_lidar_points[36] = {};
    lv_obj_t *m_lidar_rays[36] = {};
    lv_point_precise_t m_lidar_ray_points[36][2] = {};
    lv_obj_t *m_lidar_range_lines[2] = {};
    lv_point_precise_t m_lidar_range_line_points[2][2] = {};

    /* Main-screen telemetry and drive-state widgets. */
    lv_obj_t *m_clean_bar = nullptr;
    lv_obj_t *m_battery_bar = nullptr;
    lv_obj_t *m_battery_voltage = nullptr;
    lv_obj_t *m_brake_status = nullptr;
    int m_brake_visual_state = -1;
    lv_obj_t *m_clean_value = nullptr;
    lv_obj_t *m_runtime_value = nullptr;
    lv_obj_t *m_drive_card = nullptr;
    lv_obj_t *m_drive_icon = nullptr;
    lv_obj_t *m_drive_distance = nullptr;
    lv_obj_t *m_drive_status = nullptr;
    lv_obj_t *m_pose_quality = nullptr;
    lv_obj_t *m_drive_l_pwm = nullptr;
    lv_obj_t *m_drive_r_pwm = nullptr;
    lv_obj_t *m_drive_m1_rpm = nullptr;
    lv_obj_t *m_estop_popup = nullptr;
    lv_obj_t *m_estop_source = nullptr;
    int8_t    m_follow_visual = -1;
    int8_t    m_vacuum_visual = -1;
    bool      m_rpm_visual_initialized = false;
    bool      m_rpm_visual_valid = false;
    int16_t   m_rpm_visual_m1 = 0;
    int16_t   m_rpm_visual_m2 = 0;
    bool      m_estop_visual_visible = false;
    int8_t    m_estop_visual_source = -1;
    bool      m_battery_visual_initialized = false;
    bool      m_battery_visual_valid = false;
    uint16_t  m_battery_visual_voltage = 0;
    uint16_t  m_battery_visual_current = 0;
    bool      m_battery_filter_valid = false;
    float     m_battery_filtered_decivolts = 0.0f;
    uint32_t  m_battery_filter_tick_ms = 0;
    lv_obj_t *m_vacuum_settings = nullptr;
    lv_obj_t *m_vacuum_output_value = nullptr;
    lv_obj_t *m_vacuum_runtime_value = nullptr;
    uint8_t  m_clean_output_percent = 80;
    uint8_t  m_clean_output_edit = 80;
    uint64_t m_clean_runtime_ms = 0;
    uint64_t m_last_saved_runtime_min = 0;
    int64_t  m_last_runtime_tick_ms = 0;
    bool     m_vacuum_long_pressed = false;

    /* ── 하단 로그 바 ───────────────── */
    lv_obj_t *m_log      = nullptr;
    lv_obj_t *m_btn_follow = nullptr;
    lv_obj_t *m_btn_vacuum = nullptr;
    lv_obj_t *m_lbl_follow = nullptr;
    lv_obj_t *m_icon_follow = nullptr;
    lv_obj_t *m_lbl_vacuum = nullptr;

    /* AS500 sprayer dashboard. Fields not yet supplied by the 48 V CAN
       protocol remain explicitly unavailable instead of showing estimates. */
    lv_obj_t *m_sprayer_dashboard = nullptr;
    lv_obj_t *m_dash_direction = nullptr;
    lv_obj_t *m_dash_speed = nullptr;
    lv_obj_t *m_dash_odometer = nullptr;
    lv_obj_t *m_dash_trip = nullptr;
    double m_dash_odometer_km = 0.0;
    double m_dash_trip_km = 0.0;
    uint32_t m_dash_distance_tick_ms = 0;
    lv_obj_t *m_dash_spray_gauge = nullptr;
    lv_obj_t *m_dash_speed_limit_gauge = nullptr;
    lv_obj_t *m_dash_speed_limit_tile = nullptr;
    lv_obj_t *m_dash_steering_gauge = nullptr;
    lv_obj_t *m_dash_rpm_gauge = nullptr;
    lv_obj_t *m_dash_rpm_value = nullptr;
    lv_obj_t *m_dash_parking_brake = nullptr;
    int8_t m_dash_parking_brake_visual = -1;
    lv_obj_t *m_dash_battery_gauge = nullptr;
    lv_obj_t *m_dash_current_gauge = nullptr;
    lv_obj_t *m_dash_battery_value = nullptr;
    lv_obj_t *m_dash_battery_voltage_value = nullptr;
    int16_t m_dash_battery_gauge_soc = -2;
    int16_t m_dash_battery_gauge_warning = -1;
    int16_t m_dash_battery_text_soc = -2;
    uint16_t m_dash_battery_text_voltage = 0xFFFFu;
    lv_obj_t *m_dash_current_value = nullptr;
    lv_obj_t *m_dash_current_scale = nullptr;
    lv_obj_t *m_dash_temperature = nullptr;
    lv_obj_t *m_dash_pitch = nullptr;
    lv_obj_t *m_dash_roll = nullptr;
    lv_obj_t *m_dash_pitch_level = nullptr;
    lv_obj_t *m_dash_roll_level = nullptr;
    bool m_dash_imu_valid = false;
    float m_dash_pitch_deg = 0.0f;
    float m_dash_roll_deg = 0.0f;
    lv_obj_t *m_dash_speed_limit_value = nullptr;
    lv_obj_t *m_dash_spray_value = nullptr;
    lv_obj_t *m_analog_settings = nullptr;
    lv_obj_t *m_analog_setting_values[8] = {};
    lv_obj_t *m_analog_raw_values[4] = {};
    lv_obj_t *m_analog_calibrated_values[4] = {};
    lv_obj_t *m_analog_status = nullptr;
    uint8_t m_analog_raw[4] = {};
    int m_dash_current_max_amps = 100;
    lv_obj_t *m_dash_can = nullptr;
    lv_obj_t *m_dash_fnr_box = nullptr;
    lv_obj_t *m_dash_fnr_title = nullptr;
    lv_obj_t *m_dash_fnr = nullptr;
    lv_obj_t *m_dash_cruise = nullptr;
    lv_obj_t *m_dash_cruise_label = nullptr;
    control_cmd_t m_dash_fnr_visual = CMD_SETTINGS; /* force first redraw */
    int8_t m_dash_cruise_visual = -1;
    lv_obj_t *m_aux_buttons[6] = {};
    lv_obj_t *m_aux_labels[6] = {};
    uint8_t m_aux_visual = 0xFF;
    lv_obj_t *m_dash_turn_left = nullptr;
    lv_obj_t *m_dash_turn_right = nullptr;
    lv_obj_t *m_dash_brake = nullptr;
    lv_obj_t *m_dash_brake_tile = nullptr;
    lv_obj_t *m_dash_brake_caption = nullptr;
    int8_t m_dash_brake_visual = -1;
    lv_obj_t *m_dash_obstacle = nullptr;
    lv_obj_t *m_dash_tilt = nullptr;
    lv_obj_t *m_dash_tilt_tile = nullptr;
    lv_obj_t *m_dash_tilt_caption = nullptr;
    int8_t m_dash_tilt_alarm_visual = -1;
    lv_obj_t *m_dash_spray = nullptr;
    lv_obj_t *m_dash_wired = nullptr;
    lv_obj_t *m_dash_wireless = nullptr;
    lv_obj_t *m_mp3_tile = nullptr;
    lv_obj_t *m_mp3_settings = nullptr;
    lv_obj_t *m_mp3_volume = nullptr;
    lv_obj_t *m_mp3_mode = nullptr;
    lv_obj_t *m_mp3_status = nullptr;
    lv_obj_t *m_mp3_track_buttons[8] = {};
    lv_obj_t *m_mp3_track_labels[8] = {};
    size_t m_mp3_list_offset = 0;
    bool m_mp3_long_pressed = false;
    int8_t m_mp3_visual = -1;
    lv_obj_t *m_dash_ros = nullptr;
    lv_obj_t *m_dash_engine_fan_tile = nullptr;
    lv_obj_t *m_dash_engine_fan_caption = nullptr;
    int8_t m_dash_engine_fan_visual = -1;
};
