# AS500 차량 CAN 통신 규격

- 규격 버전: 1
- CAN: Classical CAN 2.0A, 표준 11-bit ID
- 속도: 500 kbit/s
- Payload: 최대 8 byte
- 기준 헤더: `main/can/can_protocol.h`

이 문서와 `can_protocol.h`가 서로 다르면 `can_protocol.h`를 우선합니다. 멀티바이트 값은 프레임별로 명시된 byte order를 사용하며 C/C++ 구조체를 payload에 직접 복사하지 않습니다.

## 보드별 책임

| 노드 | 책임 |
|---|---|
| AS500-HMI | 메인 상태 머신, 운전 판단, 명령 생성, 표시 및 설정 |
| STM32 2026ydriver | 모터/브레이크/릴레이 구동, 전류·온도 보호, 명령 timeout 안전정지 |
| ESP32-AS_interface | FNR, VR1/VR2, 기울기, 좌우 제한 등 I/O 수집과 보조 출력 |
| JY-ME02 | 조향 엔코더 측정값 송신 |

모터 명령 `0x701`의 정상 송신자는 AS500-HMI 하나만 둡니다. 비상정지와 STM32 자체 보호는 AS500 상태와 무관하게 동작해야 합니다.

## CAN ID 목록

| ID | 상태 | 송신 → 수신 | 주기 | 내용 |
|---:|---|---|---:|---|
| `0x050` | ACTIVE | JY-ME02 → AS500 | 센서 설정값 | 조향 측정/온도 |
| `0x501` | IO_STATE | ESP32 → AS500 | 100 ms | GPIO 입력/출력 상태, ADC 4채널 통합 |
| `0x502` | IO_TILT | ESP32 → AS500 | 100 ms | Pitch/Roll 전용, DLC 4 |
| `0x503` | UNUSED | - | - | 사용하지 않음 |
| `0x5A0` | ACTIVE | STM32 → AS500 | 100 ms | 드라이버 I/O·전압·전류·온도 |
| `0x5A1` | RESERVED | STM32 → AS500 | 20~50 ms | 좌우 RPM 및 운전 상태 |
| `0x601` | IO_COMMAND | P4 → ESP32 | 100 ms / 300 ms | Interface 출력 제어 bitmap |
| `0x701` | ACTIVE/이전대상 | 메인 제어기 → STM32 | 20 ms | 좌우 모터 명령 |
| `0x703` | ACTIVE | 모든 제어기 → STM32 | 이벤트 반복 | 비상정지 |
| `0x710` | RESERVED | AS500 → 전체 | 100 ms | 메인 heartbeat |
| `0x5B0~0x5B9` | DEPRECATED | 기존 장치 | - | 신규 코드에서 사용 금지 |

`ACTIVE/이전대상`은 STM32 수신 규격은 현재 활성 상태지만 송신 주체를 AS500-HMI 하나로 이전해야 한다는 의미입니다.

## 0x050 조향 엔코더

표준 ID, DLC 8, 멀티바이트 값은 little-endian입니다.

측정 프레임:

| Byte | 내용 |
|---:|---|
| 0 | `0x55` |
| 1 | `0x55` 측정 프레임 |
| 2~3 | 각도 raw, `uint16` |
| 4~5 | 각속도 raw, `int16` |
| 6~7 | 회전수, `int16` |

- 각도: `raw / 32768 × 360°`
- 각속도: 현재 센서 주기 100 ms 기준 `raw / 32768 × 3600°/s`
- AS500 유효 timeout: 500 ms

온도 프레임은 Byte 0=`0x55`, Byte 1=`0x56`, Byte 2~3=`int16` 온도이며 단위는 `0.01°C/LSB`입니다.

## 0x5A0 STM32 드라이버 상태

표준 ID, DLC 8, 100 ms, 멀티바이트 값은 big-endian입니다.

| Byte | 내용 |
|---:|---|
| 0 | 고정 marker `0xA0` |
| 1 | bit0~7 = RY1, RY2, RY3, RY4, RY5, RY6, WUP, WDN 실제 출력 상태 |
| 2 | bit7~4 = A+, A-, B+, B- 입력, bit3~0 = 오류 코드 |
| 3 | FET 온도 `int8`, `1°C/LSB` |
| 4~5 | 배터리 전압 `uint16`, `0.1V/LSB` |
| 6~7 | 전류 `uint16`, `0.1A/LSB` |

RY3가 OFF이면 스프링 브레이크가 체결된 것으로 해석합니다. AS500은 300 ms 이상 수신되지 않으면 드라이버 상태를 무효로 처리해야 합니다.

## 0x701 모터 명령

표준 ID, DLC 8, 목표 주기 20 ms입니다. 좌우 명령은 big-endian `int16`입니다.

| Byte | 내용 |
|---:|---|
| 0~1 | 좌측 모터 명령 |
| 2 | 브레이크 지연 raw, `5 ms/LSB` |
| 3 | 기존 battery/보조 필드(추후 의미 고정 필요) |
| 4~5 | 우측 모터 명령 |
| 6 | 기존 toggle bit field |
| 7 | 기존 button bit field |

STM32는 마지막 정상 명령 후 150 ms가 지나면 좌우 명령을 0으로 만들고 안전 브레이크 상태로 전환해야 합니다. Byte 3, 6, 7은 기존 STM32 동작을 보존하기 위해 남겨두며, 세부 bit 정의를 확정하기 전에는 신규 의미를 추가하지 않습니다.

## 0x703 비상정지

- DLC: 1
- Byte 0: `0xFF`
- 비상정지가 해제되어 안전 상태가 확인될 때까지 반복 송신합니다.
- STM32는 다른 명령보다 `0x703`을 우선 처리합니다.

## RESERVED 프레임 적용 규칙

### `0x501` Interface GPIO/ADC 통합 프레임

| Byte | 내용 |
|---:|---|
| 0 | Interface 입력 bit0~7 (`uint8`) |
| 1 | Interface 입력 bit8~9 (`uint8`, 상위 6-bit는 `0`) |
| 2 | Interface 출력 상태 bit0~5 (`uint8`, 상위 2-bit는 `0`) |
| 3 | Throttle1 (`0~255`) |
| 4 | Throttle2 (`0~255`) |
| 5 | VR1 (`0~255`) |
| 6 | VR2 (`0~255`) |
| 7 | Reserved (`0`) |

CAN ID 자체로 프레임을 구분하므로 marker와 version byte를 사용하지 않습니다. ADC 전용 프레임은 없으며 네 ADC 값은 `0x501`에 통합됩니다.

P4는 VR1(Byte 5)의 `0~255`를 좌우 모터 최종 속도 제한 `0~100%`로 적용합니다. 대시보드 경사 패널은 상단에 Pitch/Roll을 좌우로 표시하고, 하단의 좌우 프로그레스바에 동일한 VR1 속도 제한 비율을 표시합니다. `0x501`이 유효하지 않으면 속도 제한을 0%로 처리합니다.

P4는 마지막 정상 프레임으로부터 300 ms가 지나면 네 입력을 무효로 처리합니다.

### `0x502` Interface Pitch/Roll

| Byte | 내용 |
|---:|---|
| 0..1 | Pitch, `int16` big-endian, `0.01°/LSB` |
| 2..3 | Roll, `int16` big-endian, `0.01°/LSB` |

DLC는 4이며 marker, version, 상태 flag를 사용하지 않습니다. P4는 마지막 정상 프레임으로부터 300 ms가 지나면 Pitch/Roll을 무효로 처리합니다. `0x503`은 송수신하지 않습니다.

### `0x601` P4 → Interface 출력 명령

| Byte | 내용 |
|---:|---|
| 0 | 출력 제어 bitmap: bit0 좌측 방향등, bit1 우측 방향등, bit2 혼/외부 부저, bit3 전조등, bit4 브레이크등, bit5 PCB 부저 |

DLC는 1이며 marker와 version을 사용하지 않습니다. P4는 100 ms마다 명령을 전송합니다. Interface는 명령을 수동적으로 출력에 적용하고 실제 I²C 출력 상태 6-bit는 `0x501` byte 2로 회신합니다. 300 ms 동안 새 명령이 없으면 모든 출력을 OFF 처리합니다. 기울기 경보는 P4 설정의 Pitch/Roll 임계값, 경보 사용 여부, Pitch/Roll offset을 사용하며 임계값이 3초 연속 초과될 때 발생합니다.

`0x5A1`, `0x710`은 ID와 방향만 예약했습니다. 실제 byte layout은 STM32 RPM 범위와 제어 요구사항을 확인한 후 별도 버전으로 확정합니다. layout 확정 전에는 해당 ID를 송신하지 않습니다.

## 변경 관리

1. ID, byte 위치, 단위 또는 signed 여부 변경 시 `AS500_CAN_PROTOCOL_VERSION`을 올립니다.
2. 세 프로젝트가 같은 `can_protocol.h` 사본을 사용해야 합니다.
3. 송신자는 DLC와 reserved byte를 항상 고정값으로 채웁니다.
4. 수신자는 ID뿐 아니라 DLC, marker/version, 값 범위와 timeout을 검사합니다.
5. `0x5B0~0x5B9`는 신규 기능에 재사용하지 않습니다.

