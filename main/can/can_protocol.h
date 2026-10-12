/*
 * AS500 vehicle CAN protocol - single source of truth
 *
 * Keep this file C-compatible so the same definition can be copied to the
 * STM32 and ESP32 projects. Multi-byte values are transmitted explicitly;
 * do not cast CAN payloads to structs.
 */
#pragma once

#include <stdint.h>

#define AS500_CAN_PROTOCOL_VERSION       1u
#define AS500_CAN_BITRATE_KBPS           500u
#define AS500_CAN_CLASSIC_MAX_DLC        8u

/* Active frames. */
#define AS500_CAN_ID_STEERING_ENCODER    0x050u /* JY-ME02 -> AS500-HMI */
#define AS500_CAN_ID_DRIVER_STATUS       0x5A0u /* STM32 -> AS500-HMI */
#define AS500_CAN_ID_DRIVE_COMMAND       0x701u /* AS500-HMI -> STM32 */

/* AS500-interface telemetry. */
#define AS500_CAN_ID_IO_STATE            0x501u /* ESP32 -> AS500-HMI */
#define AS500_CAN_ID_IO_TILT             0x502u /* ESP32 -> AS500-HMI */
/* Reserved for the coordinated three-board architecture. */
#define AS500_CAN_ID_DRIVER_MOTION       0x5A1u /* STM32 -> AS500-HMI */
#define AS500_CAN_ID_IO_COMMAND          0x601u /* AS500-HMI -> ESP32 */
#define AS500_CAN_ID_MAIN_HEARTBEAT      0x710u /* AS500-HMI -> all */

/* Deprecated: must not be used by new code. */
#define AS500_CAN_ID_LEGACY_CONFIG_FIRST 0x5B0u
#define AS500_CAN_ID_LEGACY_CONFIG_LAST  0x5B7u
#define AS500_CAN_ID_LEGACY_STATUS       0x5B8u
#define AS500_CAN_ID_LEGACY_CURRENT      0x5B9u

/* 0x5B8 legacy motion/status frame still emitted by the STM32 driver.
 * RPM1/RPM2 are two packed 12-bit values spanning bytes 0..2. */
#define AS500_LEGACY_STATUS_DLC              8u
#define AS500_LEGACY_STATUS_EMB_APPLIED_MASK (1u << 3)
#define AS500_LEGACY_STATUS_ESTOP_MASK       (1u << 2)
#define AS500_CAN_MOTION_TIMEOUT_MS           300u

/* Common timing and fail-safe limits. */
#define AS500_CAN_DRIVE_COMMAND_PERIOD_MS  20u
#define AS500_CAN_DRIVE_COMMAND_TIMEOUT_MS 150u
#define AS500_CAN_DRIVER_STATUS_PERIOD_MS  100u
#define AS500_CAN_DRIVER_STATUS_TIMEOUT_MS 300u
#define AS500_CAN_IO_PERIOD_MS            100u
#define AS500_CAN_IO_TIMEOUT_MS            300u
#define AS500_CAN_STEERING_TIMEOUT_MS       500u
#define AS500_CAN_HEARTBEAT_PERIOD_MS       100u
#define AS500_CAN_HEARTBEAT_TIMEOUT_MS      300u
#define AS500_CAN_IO_COMMAND_PERIOD_MS       100u
#define AS500_CAN_IO_COMMAND_TIMEOUT_MS      300u

/* 0x501: Interface GPIO state and ADC, DLC=8, no marker/version. */
#define AS500_IO_STATE_DLC                  8u
#define AS500_IO_BYTE_INPUT_STATE_LOW       0u
#define AS500_IO_BYTE_INPUT_STATE_HIGH      1u
#define AS500_IO_BYTE_OUTPUT_STATE          2u
#define AS500_IO_INPUT_STATE_MASK            0x03FFu
#define AS500_IO_OUTPUT_STATE_MASK           0x3Fu
#define AS500_IO_OUTPUT_BUZZER_MASK          (1u << 0)
#define AS500_IO_OUTPUT_LEFT_SIGNAL_MASK     (1u << 1)
#define AS500_IO_OUTPUT_RIGHT_SIGNAL_MASK    (1u << 2)
#define AS500_IO_BYTE_THROTTLE1             3u
#define AS500_IO_BYTE_THROTTLE2             4u
#define AS500_IO_BYTE_VR1                   5u
#define AS500_IO_BYTE_VR2                   6u
#define AS500_IO_BYTE_RESERVED              7u
#define AS500_IO_INPUT_FORWARD_MASK         (1u << 0)
#define AS500_IO_INPUT_REVERSE_MASK         (1u << 1)
#define AS500_IO_INPUT_LEFT_MASK            (1u << 2)
#define AS500_IO_INPUT_RIGHT_MASK           (1u << 3)
#define AS500_IO_INPUT_EMERGENCY_NORMAL_MASK (1u << 4) /* NC: 1=normal */
#define AS500_IO_INPUT_CRUISE_MASK          (1u << 5)

/* 0x701 Byte 6 uses the same bit position. P4 ORs the Interface switch
   and its own touch button, so this bit is set only while either is held. */
#define AS500_DRIVE_INPUT_CRUISE_MASK       AS500_IO_INPUT_CRUISE_MASK
#define AS500_IO_CONTROL_LEFT_SIGNAL_MASK    (1u << 0)
#define AS500_IO_CONTROL_RIGHT_SIGNAL_MASK   (1u << 1)
#define AS500_IO_CONTROL_HORN_MASK           (1u << 2)
#define AS500_IO_CONTROL_HEADLIGHT_MASK      (1u << 3)
#define AS500_IO_CONTROL_BRAKE_LIGHT_MASK    (1u << 4)
#define AS500_IO_CONTROL_BUZZER_MASK         (1u << 5)

/* 0x601: P4 -> Interface output-control bitmap, no marker/version. */
#define AS500_IO_COMMAND_DLC                  1u
#define AS500_IO_COMMAND_BYTE_CONTROL         0u

#define AS500_FNR_NEUTRAL                   0u
#define AS500_FNR_FORWARD                   1u
#define AS500_FNR_REVERSE                   2u

/* 0x5A0: STM32 motor-driver status, DLC=8, big-endian numeric fields. */
#define AS500_DRIVER_STATUS_DLC             8u
#define AS500_DRIVER_STATUS_BYTE_INPUTS     0u
#define AS500_DRIVER_STATUS_BYTE_RELAYS     1u
#define AS500_DRIVER_STATUS_BYTE_IO_ERROR   2u
#define AS500_DRIVER_STATUS_BYTE_FET_TEMP   3u
#define AS500_DRIVER_STATUS_BYTE_VOLTAGE_H  4u
#define AS500_DRIVER_STATUS_BYTE_VOLTAGE_L  5u
#define AS500_DRIVER_STATUS_BYTE_CURRENT_H  6u
#define AS500_DRIVER_STATUS_BYTE_CURRENT_L  7u

#define AS500_RELAY_RY1_MASK                (1u << 0)
#define AS500_RELAY_RY2_MASK                (1u << 1)
#define AS500_RELAY_RY3_EMB_MASK            (1u << 2)
#define AS500_RELAY_RY4_MASK                (1u << 3)
#define AS500_RELAY_RY5_MASK                (1u << 4)
#define AS500_RELAY_RY6_MASK                (1u << 5)
#define AS500_RELAY_WUP_MASK                (1u << 6)
#define AS500_RELAY_WDN_MASK                (1u << 7)

/* Byte 0 / drive Byte 6: A+/A-/B+/B-/C+/C-/D+/D- in bit order. */
#define AS500_INPUT_AP_MASK                 (1u << 0)
#define AS500_INPUT_AN_MASK                 (1u << 1)
#define AS500_INPUT_BP_MASK                 (1u << 2)
#define AS500_INPUT_BN_MASK                 (1u << 3)
#define AS500_INPUT_CP_MASK                 (1u << 4)
#define AS500_INPUT_CN_MASK                 (1u << 5)
#define AS500_INPUT_DP_MASK                 (1u << 6)
#define AS500_INPUT_DN_MASK                 (1u << 7)
#define AS500_DRIVE_INPUT_ESTOP_MASK        AS500_INPUT_CP_MASK /* 1=E-stop */

#define AS500_RELAY_PUMP_MASK               AS500_RELAY_RY4_MASK
#define AS500_RELAY_BLOWER_MASK             AS500_RELAY_RY5_MASK
#define AS500_RELAY_ENGINE_FAN_MASK         AS500_RELAY_RY6_MASK
/* P4 may command only the STM32 auxiliary relays. RY1/RY2/RY3 are local. */
#define AS500_RELAY_P4_COMMAND_MASK          (AS500_RELAY_PUMP_MASK | \
                                              AS500_RELAY_BLOWER_MASK | \
                                              AS500_RELAY_ENGINE_FAN_MASK)

#define AS500_DRIVER_IO_AP_MASK             (1u << 7)
#define AS500_DRIVER_IO_AN_MASK             (1u << 6)
#define AS500_DRIVER_IO_BP_MASK             (1u << 5)
#define AS500_DRIVER_IO_BN_MASK             (1u << 4)
#define AS500_DRIVER_ERROR_MASK             0x0Fu

/* 0x502: pitch/roll only, no marker/version. */
#define AS500_IO_TILT_DLC                    4u
#define AS500_IO_TILT_BYTE_PITCH_H           0u
#define AS500_IO_TILT_BYTE_ROLL_H            2u

/* 0x50: JY-ME02, DLC=8, little-endian numeric fields. */
#define AS500_STEERING_HEADER                0x55u
#define AS500_STEERING_TYPE_MEASUREMENT      0x55u
#define AS500_STEERING_TYPE_TEMPERATURE      0x56u
#define AS500_STEERING_DLC                   8u

/* 0x701: motor command, DLC=8, big-endian signed motor commands. */
#define AS500_DRIVE_COMMAND_DLC              8u
#define AS500_DRIVE_BYTE_LEFT_H              0u
#define AS500_DRIVE_BYTE_LEFT_L              1u
#define AS500_DRIVE_BYTE_BRAKE_DELAY         2u /* raw x 5 ms */
#define AS500_DRIVE_BYTE_BATTERY             3u
#define AS500_DRIVE_BYTE_RIGHT_H             4u
#define AS500_DRIVE_BYTE_RIGHT_L             5u
#define AS500_DRIVE_BYTE_INPUT_PORT          6u
#define AS500_DRIVE_BYTE_RELAY_PORT          7u

static inline uint16_t as500_can_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static inline uint16_t as500_can_u16_le(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static inline int16_t as500_can_i16_be(const uint8_t *p)
{
    return (int16_t)as500_can_u16_be(p);
}

static inline int16_t as500_can_i16_le(const uint8_t *p)
{
    return (int16_t)as500_can_u16_le(p);
}

