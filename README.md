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
| RX | `CAN_ID_MTR_CMD` | `0x210` | 6 | 10 ms | ECU → MTR 4륜 PWM 명령 (E2E P01 자리 예약) |
| TX | `CAN_ID_MTR_SPD_FB` | `0x300` | 8 | 10 ms | 엔코더 속도 피드백 (AVG + 4륜 broadcast) |
| TX | `CAN_ID_MTR_HEARTBEAT` | `0x310` | 2 | 10 ms | HB_MTR + ERR_MTR |
| RX | `CAN_ID_ECU_HEARTBEAT` | `0x410` | 2 | 10 ms | ECU 생존 감시 (3주기=30ms 미수신 → FAULT) |

> 구 `CAN_ID_MOTOR_*` 별칭 (0x300/0x400/0x410) 은 모두 제거됨. 기존 코드/문서가 이 ID 를 참조하면 stale.

**페이로드 `0x210 MTR_CMD` (RX, 6B):**
```
Byte 0: SET_PWM_LF  (int8, -128~127) Left-Front  PWM
Byte 1: SET_PWM_RF  (int8, -128~127) Right-Front PWM
Byte 2: SET_PWM_LR  (int8, -128~127) Left-Rear   PWM
Byte 3: SET_PWM_RR  (int8, -128~127) Right-Rear  PWM
Byte 4: MTR_RC      (4bit)  E2E P01 Rolling Counter (low nibble = 0 padding)
Byte 5: MTR_CRC     (uint8) E2E P01 CRC-8 (poly 0x2F, init 0xFF, no XOR-out)
```

> **E2E (RC/CRC) 현재 미구현**: DBC 시그널 자리는 예약되어 있으나 양측(ECU + Arduino) 모두 송신 0 / 수신 검증 skip. SAF010 ASIL-B 본격 적용은 후속 작업이며, 현재 safe-state 는 30 ms 타임아웃(SWR018) 단일 의존.

> **하드웨어 차원 어댑터**: 모터 쉴드 2채널 (L pair / R pair) 만 운용하므로 `acc_can_node/can_handler.cpp` 가 4 PWM 을 `L = avg(LF, LR)`, `R = avg(RF, RR)` 로 축소해 I2C 전달. 의도된 운영 모드 — 4륜 독립 제어로 확장 시 어댑터만 교체 (DBC/요구사항 불변).

**페이로드 `0x300 MTR_SPD_FB` (TX, 8B):**
```
Byte 0-1: GET_SPD_AVG  (int16, factor 0.02 cm/s) 대표 속도 (수신: ECU + SENSOR)
Byte 2-3 + Byte 4 (low nibble): GET_SPD_LF  (int12, factor 0.3 cm/s) 4륜 (수신: ECU)
Byte 4 (high) + Byte 5: GET_SPD_RF  (int12, factor 0.3 cm/s)
Byte 6 + Byte 7 (low):  GET_SPD_LR  (int12, factor 0.3 cm/s)
Byte 7 (high) + ...:    GET_SPD_RR  (int12, factor 0.3 cm/s)
```

> 정확한 비트 packing 은 DBC 의 `BO_ 768 MTR_SPD_FB` 정의 참조. 엔코더 1개 (단일 축) 운용이라 Arduino 는 단일 측정값을 AVG 와 4륜 모두에 동일하게 broadcast (각 시그널 factor 차이만 적용). 다중 엔코더 확장 시 MTR 쪽에서 실제 평균 연산.

> 대표 속도 `GET_SPD_AVG` 의 용도: ECU `MotorControl` SWC 의 inner-loop 속도 PI (SWR034) 입력 + RPi HMI 표시 (SYS019). 4륜 시그널 `LF/RF/LR/RR` 는 ECU 수신만, 현재 미사용 (4륜 개별 제어 확장 / 로깅 예약).

**페이로드 `0x310 MTR_HEARTBEAT` (TX, 2B):**
```
Byte 0: HB_MTR    (uint8) Heartbeat 카운터 (매 송신 +1, wrap)
Byte 1: ERR_MTR   (uint8) 0=Normal, ≠0=Error code
```

**페이로드 `0x410 ECU_HEARTBEAT` (RX, 2B, ASIL-B):**
```
Byte 0: HB_ECU    (uint8) ECU 생존 카운터
Byte 1: ERR_ECU   (uint8) 0=Normal, ≠0=Error code
```

수신 측 (Arduino #1) 은 30 ms (3주기) 동안 `HB_ECU` 카운터가 갱신되지 않으면 ECU 통신 단절로 판정 → I2C `cmd_l = cmd_r = 0` 즉시 전송 + 자체 HB 의 `ERR_MTR` 에 타임아웃 비트 set (SAF018 ASIL-B 요건).

### `ERR_MTR` 비트

| 비트 | 상수 | 의미 |
|---|---|---|
| 0 | `ERR_CAN` | CAN 통신 오류 (송신 실패 등) |
| 1 | `ERR_TIMEOUT` | `MTR_CMD` 또는 `ECU_HEARTBEAT` 30ms 미수신 |
| 2 | `ERR_E2E` | (TODO) `MTR_CMD` E2E P01 RC/CRC 검증 실패 — 현재 미구현, 자리만 예약 |
| 3 | `ERR_I2C` | Arduino #2 (`acc_motor_node`) I2C 응답 실패 |

## I2C 프로토콜 (내부)

Arduino #1 ↔ Arduino #2 통신. 슬레이브 주소 `0x10`, 100kHz.

### Master → Slave (쓰기, 3바이트)
```
Byte 0: cmd_l (int8)
Byte 1: cmd_r (int8)
Byte 2: ctrl  (bit0=enable, bit2=brake)
```

### Slave → Master (읽기, 3바이트)
```
Byte 0: spd LSB  (int16 little-endian)
Byte 1: spd MSB
Byte 2: err_flags
```

## 필요한 라이브러리 (Arduino IDE)

### Arduino #1 (acc_can_node)
- **DFRobot_MCP2515** — CAN BUS Shield 드라이버 (Library Manager에서 설치)
- **Wire** — I2C (내장)
- **SPI** — (내장)

### Arduino #2 (acc_motor_node)
- **Wire** — I2C (내장)

## 빌드 & 업로드

1. Arduino IDE에서 `acc_can_node/acc_can_node.ino` 열기
2. 보드: Arduino Uno 선택
3. 해당 Arduino #1 포트 선택 → 업로드
4. 새 창에서 `acc_motor_node/acc_motor_node.ino` 열기
5. Arduino #2 포트 선택 → 업로드

## 테스트 절차

### 단계 1: Arduino #2 단독 모터 테스트
- `acc_motor_node.ino` 맨 아래 loop() 직전에 임시로 삽입:
  ```cpp
  // 테스트: 3초 후 전진 시작
  if (millis() > 3000 && g_cmd_l == 0) {
      g_cmd_l = 80;  g_cmd_r = 80;  g_ctrl = 0x01;
      g_t_last_cmd = millis();
  }
  ```
- 모터 4개가 차체 기준 앞으로 구르는지 확인
- 반대로 도는 모터가 있으면 해당 모터 전선 2가닥 바꿔 꽂기
- 완료 후 위 코드 제거

### 단계 2: Arduino #1 단독 CAN 테스트
- PCAN-View에서 `0x210 MTR_CMD` 에 `50 50 50 50 00 00` 송신
  - 4륜 모두 80 PWM (LF/RF/LR/RR = 0x50). MTR_RC/MTR_CRC 자리는 0 (현재 미검증).
  - 어댑터 동작 확인: `cmd_l = avg(LF, LR) = 80`, `cmd_r = avg(RF, RR) = 80`.
- 또한 `0x410 ECU_HEARTBEAT` 를 10ms 주기로 송신해 줘야 Arduino #1 이 ECU 생존을 인식한다 (없으면 30ms 후 타임아웃 → 모터 정지).
- 시리얼 모니터에 CAN 수신 로그 확인
- I2C 에러(`ERR_I2C` 비트 set)가 떠도 OK (Arduino #2 아직 연결 안 했으면)

### 단계 3: I2C 통합 테스트
- A4-A4, A5-A5, GND-GND 연결
- PCAN 에서 `0x210 MTR_CMD` + `0x410 ECU_HEARTBEAT` 동시 송신
- Arduino #2의 모터 움직이는지 확인
- PCAN 수신창에서 `0x300 MTR_SPD_FB` 피드백, `0x310 MTR_HEARTBEAT` 들어오는지 확인

### 단계 4: ECU 연결 + 실제 주행

## 엔코더 PPR 측정

`acc_motor_node/encoder.cpp` 에서 `DEBUG_ENCODER` 를 `1` 로 변경 후 업로드.
시리얼 모니터(115200) 열고 엔코더가 달린 모터를 **정확히 1바퀴** 돌린 뒤
`total` 출력값을 `config.h` 의 `ENCODER_PPR` 에 대입. 끝나면 `DEBUG_ENCODER` 0으로 복구.

## 주의사항

- **공통 GND 필수**: Arduino #1 GND / #2 GND / 배터리 GND / ECU GND / 엔코더 GND 전부 연결
- **모터 전원 분리**: 모터 배터리는 절대 아두이노 USB 포트 쓰지 말 것
- **CAN 종단 저항**: 버스 양 끝단에 120Ω. CAN BUS Shield J1 점퍼 ON 상태 확인
- **모터 쉴드 스택 시 전류 센싱(A0/A1)은 부정확**하므로 사용하지 않음
