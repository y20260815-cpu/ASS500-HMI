/*
 * ============================================================
 *  esp32p4_controller.ino
 *  역할 : ESP32-P4 (7" LCD-HMI) 메인 추종 컨트롤러
 *
 *  입력  : UART1 ← A0(ESP32 DW3000) "D,{dL},{dR},{mask}\n"
 *  출력  : CAN bus → 4WD 모터 컨트롤러 (좌/우 속도 명령)
 *  표시  : 7" LCD (거리 / 방향 / 속도 상태)
 *
 *  ┌─────────────────────────────────────────────────┐
 *  │  CAN 프레임 설계 (브로드캐스트 방식)             │
 *  │                                                  │
 *  │  ID 0x100 : 드라이브 커맨드                      │
 *  │    [0-1] int16_t left_speed  (-1000 ~ +1000)    │
 *  │    [2-3] int16_t right_speed (-1000 ~ +1000)    │
 *  │    [4]   uint8_t flags                           │
 *  │             bit0 = 1:활성화  0:정지              │
 *  │             bit1 = 1:브레이크 (급정지)           │
 *  │    [5-7] reserved (0x00)                         │
 *  │                                                  │
 *  │  ID 0x000 : 비상정지 (E-Stop)                   │
 *  │    [0] = 0xFF                                    │
 *  │                                                  │
 *  │  속도 매핑                                       │
 *  │    +1000 = 전진 100%                             │
 *  │       0  = 정지                                  │
 *  │    -1000 = 후진 100%                             │
 *  │                                                  │
 *  │  4WD 차동 조향 (스키드 스티어링)                 │
 *  │    왼쪽 바퀴(FL+RL) = left_speed                 │
 *  │    오른쪽 바퀴(FR+RR) = right_speed              │
 *  └─────────────────────────────────────────────────┘
 *
 *  하드웨어 배선 (JC1060P470C_I_W Expand IO 기준)
 *   UART1  : RX=GPIO45(FPC3-3) ← A0 TX , TX=GPIO46(FPC3-4) → A0 RX
 *   CAN TX : GPIO32 → SN65HVD230 TXD  (실제 보드 배선)
 *   CAN RX : GPIO33 ← SN65HVD230 RXD
 *   CAN 종단저항 : 버스 양 끝 각 120Ω (총 2개)
 *   CAN 속도  : 500 kbps (변경 시 TWAI_TIMING_CONFIG 수정)
 *
 *  GPIO27 = 디스플레이 RST (전용) — CAN으로 사용 불가
 *  GPIO10/11 = I2S 오디오 코덱 (하드와이어드) — UART로 사용 불가
 *
 *  보드 설정 : Arduino IDE → "ESP32P4 Dev Module"
 * ============================================================
 */

#include "driver/twai.h"

// ─────────────────────────────────────────
//  UART (A0 연결) 핀/설정
// ─────────────────────────────────────────
#define UART_A0_RX      45          // FPC3 pin-3 : A0 TX → P4 RX
#define UART_A0_TX      46          // FPC3 pin-4 : A0 RX ← P4 TX (예비)
#define UART_A0_BAUD    115200

// ─────────────────────────────────────────
//  CAN bus (TWAI) 핀/설정
// ─────────────────────────────────────────
#define CAN_TX_PIN      32          // 실제 보드 → CAN 확장보드 SN65HVD230 TXD
#define CAN_RX_PIN      33          // 실제 보드 ← CAN 확장보드 SN65HVD230 RXD

#define CAN_ID_DRIVE    0x100       // 드라이브 커맨드
#define CAN_ID_ESTOP    0x000       // 비상정지

// ─────────────────────────────────────────
//  추종 파라미터 (cm / 속도 단위)
// ─────────────────────────────────────────
#define FOLLOW_DIST_CM  100         // 이 거리보다 멀면 전진
#define STOP_DIST_CM    50          // 이 거리 이하면 정지
#define MAX_DIST_CM     250         // 속도 포화 거리
#define MAX_SPEED       1000        // CAN 속도 최대 (단위: -1000 ~ +1000)
#define MIN_SPEED       300         // 기동 최소 속도
#define IN_PLACE_SPEED  200         // 제자리 회전 속도
#define TURN_GAIN       12          // 조향 게인 (speed/cm)
#define DIFF_DEADZONE   5           // 방향 오차 허용 (cm)
#define DATA_TIMEOUT_MS 2000        // 수신 타임아웃 → 비상정지

// ─────────────────────────────────────────
//  UART 수신 버퍼
// ─────────────────────────────────────────
#define UART_BUF_LEN    64
static char     uart_buf[UART_BUF_LEN];
static uint8_t  uart_idx = 0;

// ─────────────────────────────────────────
//  전역 상태
// ─────────────────────────────────────────
static int16_t  g_d_left     = 0;
static int16_t  g_d_right    = 0;
static uint8_t  g_mask       = 0;
static int16_t  g_left_spd   = 0;
static int16_t  g_right_spd  = 0;
static uint32_t last_data_ms = 0;
static bool     can_ready    = false;

// ─────────────────────────────────────────
//  함수 선언
// ─────────────────────────────────────────
static bool  twai_init(void);
static void  send_drive_cmd(int16_t left_spd, int16_t right_spd, bool enable);
static void  send_estop(void);
static bool  parse_uart_line(const char *line);
static void  calc_and_drive(void);
static void  lcd_update(void);         // LCD 표시 (LVGL 연동 위치)

// ═════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════
void setup()
{
    // ── 디버그 시리얼 ────────────────────
    Serial.begin(115200);
    Serial.println("[P4] ESP32-P4 Controller Starting...");

    // ── A0 통신 UART1 ───────────────────
    Serial1.begin(UART_A0_BAUD, SERIAL_8N1, UART_A0_RX, UART_A0_TX);
    Serial.println("[P4] UART1 ready (A0 link)");

    // ── CAN (TWAI) 초기화 ────────────────
    can_ready = twai_init();
    if (can_ready) {
        Serial.println("[P4] CAN 500kbps ready");
    } else {
        Serial.println("[P4] CAN INIT FAILED — check transceiver wiring");
    }

    // ── LCD 초기화 (LVGL 사용 시 여기서) ─
    // lv_init();
    // lv_display_create(...);
    // ui_init();
    Serial.println("[P4] Ready. Waiting for UWB data...");

    last_data_ms = millis();
}

// ═════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════
void loop()
{
    // ── UART 수신 처리 ───────────────────
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == '\n') {
            uart_buf[uart_idx] = '\0';
            if (parse_uart_line(uart_buf)) {
                last_data_ms = millis();
                calc_and_drive();
                lcd_update();
            }
            uart_idx = 0;
        } else if (uart_idx < UART_BUF_LEN - 1) {
            uart_buf[uart_idx++] = c;
        } else {
            // 버퍼 오버플로 → 리셋
            uart_idx = 0;
        }
    }

    // ── 수신 타임아웃 → 비상정지 ────────
    if (millis() - last_data_ms > DATA_TIMEOUT_MS) {
        if (g_left_spd != 0 || g_right_spd != 0) {
            Serial.println("[P4] Data timeout — E-STOP");
            send_estop();
            g_left_spd  = 0;
            g_right_spd = 0;
            lcd_update();
        }
    }
}

// ═════════════════════════════════════════
//  UART 수신 파싱
//  형식: "D,{dL},{dR},{mask}\n"
//  예:   "D,123,145,3\n"
// ═════════════════════════════════════════
static bool parse_uart_line(const char *line)
{
    if (line[0] != 'D' || line[1] != ',') return false;

    int dL, dR, mask;
    int n = sscanf(line + 2, "%d,%d,%d", &dL, &dR, &mask);
    if (n != 3) return false;

    g_d_left  = (int16_t)constrain(dL,   0, 3000);
    g_d_right = (int16_t)constrain(dR,   0, 3000);
    g_mask    = (uint8_t)(mask & 0xFF);
    return true;
}

// ═════════════════════════════════════════
//  추종 주행 계산 → CAN 송신
//
//  diff = d_right - d_left
//    > 0 : 타겟이 로봇 왼쪽  → 좌회전
//    < 0 : 타겟이 로봇 오른쪽 → 우회전
//    ≈ 0 : 정면 → 직진
// ═════════════════════════════════════════
static void calc_and_drive()
{
    // 두 앵커 모두 유효한지 확인
    if ((g_mask & 0x03) != 0x03 || g_d_left == 0 || g_d_right == 0) {
        send_estop();
        g_left_spd = g_right_spd = 0;
        Serial.println("[P4] Mask invalid — stop");
        return;
    }

    int avg  = (g_d_left + g_d_right) / 2;
    int diff = g_d_right - g_d_left;    // 양수 = 타겟이 왼쪽

    // ── 근접 정지 ───────────────────────
    if (avg <= STOP_DIST_CM) {
        send_drive_cmd(0, 0, false);
        g_left_spd = g_right_spd = 0;
        Serial.printf("[P4] STOP  avg=%d cm\n", avg);
        return;
    }

    int base = 0, turn = 0;

    // ── 전진 속도 계산 ──────────────────
    if (avg > FOLLOW_DIST_CM) {
        base = map(avg, FOLLOW_DIST_CM, MAX_DIST_CM, MIN_SPEED, MAX_SPEED);
        base = constrain(base, MIN_SPEED, MAX_SPEED);
    }

    // ── 조향 계산 ───────────────────────
    if (abs(diff) > DIFF_DEADZONE) {
        turn = constrain(diff * TURN_GAIN, -MAX_SPEED, MAX_SPEED);
    }

    int lSpd, rSpd;

    if (avg <= FOLLOW_DIST_CM) {
        // 적정 거리 → 제자리 방향 조정
        if (abs(diff) > DIFF_DEADZONE) {
            lSpd = -constrain( turn / 4, -IN_PLACE_SPEED, IN_PLACE_SPEED);
            rSpd =  constrain( turn / 4, -IN_PLACE_SPEED, IN_PLACE_SPEED);
        } else {
            lSpd = rSpd = 0;   // 이미 정면 → 정지
        }
    } else {
        // 전진하면서 조향
        // diff > 0 (타겟 왼쪽) → 왼쪽 감속·오른쪽 가속 → 왼쪽 선회
        lSpd = base - turn;
        rSpd = base + turn;
    }

    lSpd = constrain(lSpd, -MAX_SPEED, MAX_SPEED);
    rSpd = constrain(rSpd, -MAX_SPEED, MAX_SPEED);

    g_left_spd  = (int16_t)lSpd;
    g_right_spd = (int16_t)rSpd;

    send_drive_cmd(g_left_spd, g_right_spd, true);

    Serial.printf("[P4] dL=%3d dR=%3d avg=%3d diff=%4d  L=%5d R=%5d\n",
                  g_d_left, g_d_right, avg, diff, lSpd, rSpd);
}

// ═════════════════════════════════════════
//  CAN: 드라이브 커맨드 송신
//  ID=0x100, DLC=8
//  Data: [L_H][L_L][R_H][R_L][flags][0][0][0]
// ═════════════════════════════════════════
static void send_drive_cmd(int16_t left_spd, int16_t right_spd, bool enable)
{
    if (!can_ready) return;

    twai_message_t msg;
    msg.identifier         = CAN_ID_DRIVE;
    msg.extd               = 0;   // 표준 프레임 (11-bit ID)
    msg.rtr                = 0;
    msg.data_length_code   = 8;

    // int16_t → Big-Endian 2바이트
    msg.data[0] = (uint8_t)((left_spd  >> 8) & 0xFF);
    msg.data[1] = (uint8_t)( left_spd        & 0xFF);
    msg.data[2] = (uint8_t)((right_spd >> 8) & 0xFF);
    msg.data[3] = (uint8_t)( right_spd       & 0xFF);
    msg.data[4] = enable ? 0x01 : 0x00;  // bit0: enable
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(5));
    if (ret != ESP_OK) {
        Serial.printf("[P4] CAN TX error: 0x%X\n", ret);
    }
}

// ═════════════════════════════════════════
//  CAN: 비상정지 송신
//  ID=0x000, DLC=1, Data=[0xFF]
// ═════════════════════════════════════════
static void send_estop()
{
    if (!can_ready) return;

    twai_message_t msg;
    msg.identifier       = CAN_ID_ESTOP;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 1;
    msg.data[0]          = 0xFF;

    twai_transmit(&msg, pdMS_TO_TICKS(5));
}

// ═════════════════════════════════════════
//  CAN (TWAI) 초기화
//  500 kbps, Normal 모드, 모든 프레임 수신
// ═════════════════════════════════════════
static bool twai_init()
{
    twai_general_config_t g_cfg = {
        .mode           = TWAI_MODE_NORMAL,
        .tx_io          = (gpio_num_t)CAN_TX_PIN,
        .rx_io          = (gpio_num_t)CAN_RX_PIN,
        .clkout_io      = TWAI_IO_UNUSED,
        .bus_off_io     = TWAI_IO_UNUSED,
        .tx_queue_len   = 10,
        .rx_queue_len   = 10,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1
    };
    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_cfg, &t_cfg, &f_cfg) != ESP_OK) return false;
    if (twai_start() != ESP_OK) return false;
    return true;
}

// ═════════════════════════════════════════
//  LCD 표시 업데이트
//
//  LVGL 사용 시 이 함수 안에서 lv_label_set_text() 등 호출
//  현재는 Serial 출력으로 대체
// ═════════════════════════════════════════
static void lcd_update()
{
    // ─────────────────────────────────────
    //  TODO: LVGL 위젯 업데이트
    //
    //  lv_label_set_text_fmt(lbl_dL,    "Left  : %d cm", g_d_left);
    //  lv_label_set_text_fmt(lbl_dR,    "Right : %d cm", g_d_right);
    //  lv_label_set_text_fmt(lbl_speed, "L=%d  R=%d",    g_left_spd, g_right_spd);
    //  lv_bar_set_value(bar_left,  abs(g_left_spd),  LV_ANIM_OFF);
    //  lv_bar_set_value(bar_right, abs(g_right_spd), LV_ANIM_OFF);
    //  lv_timer_handler();   // LVGL 루프 구동
    // ─────────────────────────────────────
    (void)0;  // 위 주석 해제 시 삭제
}
