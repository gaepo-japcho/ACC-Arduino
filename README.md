# ACC_ARDUINO

ACC (Adaptive Cruise Control) 프로젝트의 아두이노 펌웨어.
ECU(MPC5606B)와 CAN으로 통신하며 4개의 DC 엔코더 모터를 구동합니다.

## 아키텍처

핀 충돌을 피하기 위해 아두이노 2대로 기능을 분리한 구조.

```
┌──────┐         ┌───────────────────────┐         ┌────────────────────┐
│ ECU  │══CAN════│  Arduino #1 (CAN)     │══I2C═══│  Arduino #2 (모터) │
│ MPC  │ 500k    │  Uno + CAN BUS Shield │ A4/A5  │  Uno + 모터쉴드×2  │
│ 5606 │         │  (acc_can_node)       │        │  (acc_motor_node)  │
└──────┘         └───────────────────────┘         └─────────┬──────────┘
                                                              │
                                                     ┌────────┴────────┐
                                                     │ 왼쪽쌍  오른쪽쌍  │
                                                     │ 모터 2개 모터 2개│
                                                     └─────────────────┘
                                                       + 엔코더 1개
```

- **acc_can_node/** — Arduino #1: CAN ↔ I2C 게이트웨이
- **acc_motor_node/** — Arduino #2: 모터 4개 구동 + 엔코더
- **acc_motor_node/acc_motor_front/** — (DEPRECATED, 구 버전 참고용)
- **acc_motor_node/acc_motor_rear/** — (DEPRECATED, 구 버전 참고용)

## 하드웨어

| 항목 | 수량 | 비고 |
|---|---|---|
| Arduino Uno | 2 | |
| DFRobot CAN BUS Shield | 1 | Arduino #1에 장착 |
| Arduino Motor Shield R3 | 2 | Arduino #2에 스택 |
| DC 엔코더 모터 | 4 | |
| 모터용 배터리 | 1 | 7~12V 권장 |

## 배선 요약

### Arduino #1 (CAN 노드)
- **CAN BUS Shield**: 우노 위에 그대로 스택
- **CAN 버스**: 쉴드 터미널(CAN_H, CAN_L, GND) → ECU
- **I2C**: A4, A5, GND → Arduino #2의 A4, A5, GND
- **사용 핀**: D2(CAN INT), D10~D13(SPI), A4(SDA), A5(SCL)

### Arduino #2 (모터 노드)
- **모터 쉴드 2장 스택**: 우노 위에 쉴드1 → 쉴드1 위에 쉴드2
- **Vin 점퍼**: 아래쪽 쉴드 제거(선택), 위쪽 쉴드 유지해서 배터리 연결
- **모터 배선**:
  - 쉴드1 A(+/-) → 앞 왼쪽 (극성 뒤집기)
  - 쉴드2 A(+/-) → 뒤 왼쪽 (극성 뒤집기)
  - 쉴드1 B(+/-) → 앞 오른쪽 (정방향)
  - 쉴드2 B(+/-) → 뒤 오른쪽 (정방향)
- **엔코더**: VCC=5V, GND, A→D2, B→D4
- **I2C**: A4, A5, GND → Arduino #1
- **사용 핀**: D2(INT0 엔코더), D3,D8,D9,D11,D12,D13(모터쉴드), D4(엔코더 B), A4/A5(I2C)

### 전원
- 두 아두이노 로직: USB (또는 별도 5V)
- 모터: 7~12V 배터리 → 위쪽 모터쉴드 Vin 터미널
- **모든 GND는 한 점에 공통 연결 필수**

## CAN 프로토콜

DBC 정본은 `../ACC-CANDB/acc_db.dbc` + `../ACC-CANDB/README.md` 참조. 본 표는 그 발췌이며 차이가 있을 경우 DBC 가 우선. CAN 버스 500 kbit/s.

### Arduino #1 의 CAN 트래픽

| 방향 | 상수 (config.h) | ID | DLC | 주기 | 설명 |
|---|---|---|---|---|---|
| RX | `CAN_ID_MTR_CMD` | `0x210` | 6 | 10 ms | ECU → MTR 4륜 PWM 명령 (E2E P01) |
| TX | `CAN_ID_MTR_SPD_FB` | `0x310` | 8 | 10 ms | 엔코더 속도 피드백 (AVG + 4륜 broadcast) |
| TX | `CAN_ID_MTR_HEARTBEAT` | `0x320` | 2 | 10 ms | HB_MTR + ERR_MTR |
| RX | `CAN_ID_ECU_HEARTBEAT` | `0x480` | 2 | 10 ms | ECU 생존 감시 (3주기=30ms 미수신 → FAULT) |

> DBC v3.x 이후 ID 변경 (Project.dbc 충돌 + NM 예약범위 회피):
> - `MTR_SPD_FB`    `0x300` → `0x310`
> - `MTR_HEARTBEAT` `0x310` → `0x320`
> - `ECU_HEARTBEAT` `0x410` → `0x480`
>
> 기존 코드/문서가 옛 ID 를 참조하면 stale.

**페이로드 `0x210 MTR_CMD` (RX, 6B):**
```
Byte 0: SET_PWM_LF  (int8, -128~127) Left-Front  PWM
Byte 1: SET_PWM_RF  (int8, -128~127) Right-Front PWM
Byte 2: SET_PWM_LR  (int8, -128~127) Left-Rear   PWM
Byte 3: SET_PWM_RR  (int8, -128~127) Right-Rear  PWM
Byte 4: MTR_RC      (4bit)  E2E P01 Rolling Counter (low nibble = 0 padding)
Byte 5: MTR_CRC     (uint8) E2E P01 CRC-8/AUTOSAR (poly 0x2F, init 0xFF, no XOR-out)
```

> **E2E 검증 구현됨** (`acc_can_node/can_handler.cpp` `crc8_autosar()` + Rolling Counter 연속성 체크). E2E 실패(CRC 또는 RC 순서 위반) 시 명령 무효 처리 → `ERR_E2E` 플래그 set + 30 ms 타임아웃 로직(SWR018) 에 의해 모터 정지. ECU 측도 동일 알고리즘으로 송신해야 호환됨.

> **하드웨어 차원 어댑터**: 모터 쉴드 2채널 (L pair / R pair) 만 운용하므로 `acc_can_node/can_handler.cpp` 가 4 PWM 을 `L = avg(LF, LR)`, `R = avg(RF, RR)` 로 축소해 I2C 전달. 의도된 운영 모드 — 4륜 독립 제어로 확장 시 어댑터만 교체 (DBC/요구사항 불변).

**페이로드 `0x310 MTR_SPD_FB` (TX, 8B):**
```
Byte 0-1: GET_SPD_AVG  (int16, factor 0.02 cm/s) 대표 속도 (수신: ECU + SENSOR)
Byte 2-3 + Byte 4 (low nibble): GET_SPD_LF  (int12, factor 0.3 cm/s) 4륜 (수신: ECU)
Byte 4 (high) + Byte 5: GET_SPD_RF  (int12, factor 0.3 cm/s)
Byte 6 + Byte 7 (low):  GET_SPD_LR  (int12, factor 0.3 cm/s)
Byte 7 (high) + ...:    GET_SPD_RR  (int12, factor 0.3 cm/s)
```

> 정확한 비트 packing 은 DBC 의 `BO_ 768 MTR_SPD_FB` 정의 참조. 엔코더 1개 (단일 축) 운용이라 Arduino 는 단일 측정값을 AVG 와 4륜 모두에 동일하게 broadcast (각 시그널 factor 차이만 적용). 다중 엔코더 확장 시 MTR 쪽에서 실제 평균 연산.

> 대표 속도 `GET_SPD_AVG` 의 용도: ECU `MotorControl` SWC 의 inner-loop 속도 PI (SWR034) 입력 + RPi HMI 표시 (SYS019). 4륜 시그널 `LF/RF/LR/RR` 는 ECU 수신만, 현재 미사용 (4륜 개별 제어 확장 / 로깅 예약).

**페이로드 `0x320 MTR_HEARTBEAT` (TX, 2B):**
```
Byte 0: HB_MTR    (uint8) Heartbeat 카운터 (매 송신 +1, wrap)
Byte 1: ERR_MTR   (uint8) 0=Normal, ≠0=Error code
```

**페이로드 `0x480 ECU_HEARTBEAT` (RX, 2B, ASIL-B):**
```
Byte 0: HB_ECU    (uint8) ECU 생존 카운터
Byte 1: ERR_ECU   (uint8) 0=Normal, ≠0=Error code
```

수신 측 (Arduino #1) 은 30 ms (3주기) 동안 `HB_ECU` 카운터가 갱신되지 않으면 ECU 통신 단절로 판정 → I2C `cmd_l = cmd_r = 0` 즉시 전송 + 자체 HB 의 `ERR_MTR` 에 타임아웃 비트 set (SAF018 ASIL-B 요건).

### `ERR_MTR` 비트

| 비트 | 값 | 상수 | 의미 |
|---|---|---|---|
| 0 | `0x01` | `ERR_CAN` | CAN 통신 초기화/하드웨어 오류 |
| 1 | `0x02` | `ERR_TIMEOUT` | `MTR_CMD` 30ms 미수신 (SAF010 ASIL-B) |
| 2 | `0x04` | `ERR_E2E` | `MTR_CMD` E2E P01 RC/CRC 검증 실패 |
| 3 | `0x08` | `ERR_I2C` | Arduino #2 (`acc_motor_node`) I2C 응답 실패 |
| 4 | `0x10` | `ERR_ECU_HB` | `ECU_HEARTBEAT` 30ms 미수신 (SAF018 ASIL-B) |

## I2C 프로토콜 (내부)

Arduino #1 ↔ Arduino #2 통신. 슬레이브 주소 `0x10`, 100kHz.

### Ma