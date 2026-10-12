# AS500-HMI

## CAN protocol

세 프로젝트(`AS500-HMI`, `AS500-interface`, `AS500-FND Driver`)의 전체 구성과
차량 CAN ID, payload, 단위, timeout, 보드별 책임은
[공통 CAN_PROTOCOL.md](../CAN_PROTOCOL.md)에 정리되어 있습니다.

`main/can/can_protocol.h`는 HMI 코드에서 사용하는 정의이며, 현재 구현과 헤더 간
차이는 통합 문서의 "현재 구현상 주의사항"을 확인하십시오.

ESP32-P4 기반 7인치(1024x600) HMI 프로젝트입니다.

## 기준 프로젝트

- 하드웨어, GPIO, LCD/BSP: `UWB-DW3000/esp32p4_controller`
- 애플리케이션 로직: `prj_ESPN32/esp32_as_interface_v3_48V`
- 보드: JC1060P470C_I_W, ESP32-P4 v1.x

DW3000 controller의 현재 작업 트리를 기준으로 복제했으며, 빌드 폴더와 원본 Git 이력은 포함하지 않았습니다. 기존 AS500-HMI 골격은 동일 상위 폴더의 `AS500-HMI_preclone_20260926_105412`에 보관되어 있습니다.

## 이식 원칙

P4 BSP, LCD/터치 초기화, GPIO 설정은 기준 프로젝트 구조를 유지합니다. DW3000 청소차 전용 로직은 단계적으로 분리하고, 기존 48V 인터페이스 프로젝트의 CAN, UART, ESP-NOW, 배터리, 릴레이 및 설정 로직으로 교체합니다.
