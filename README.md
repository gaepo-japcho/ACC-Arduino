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

### 수신 (ECU → Arduino #1)
| ID | DLC | 설명 |
|---|---|---|
| `0x300` | 6 | 모터 명령 |

**페이로드 (0x300):**
```
Byte 0: cmd_l      (int8, -127~127) 왼쪽 쌍 PWM 명령
Byte 1: cmd_r      (int8, -127~127) 오른쪽 쌍 PWM 명령
Byte 2: reserved
Byte 3: reserved
Byte 4: ctrl       (bit0=enable, bit2=brake)
Byte 5: checksum   (XOR of byte 0~4)
```

### 송신 (Arduino #1 → ECU)
| ID | DLC | 주기 | 설명 |
|---|---|---|---|
| `0x400` | 4 | 10ms | 엔코더 속도 피드백 |
| `0x410` | 2 | 50ms | Heartbeat + 에러 플래그 |

**페이로드 (0x400 피드백):**
```
Byte 0: spd MSB    (int16 big-endian, 단위 0.1 cm/s)
Byte 1: spd LSB
Byte 2: reserved
Byte 3: reserved
```

**페이로드 (0x410 Heartbeat):**
```
Byte 0: 0xAA (생존 마커)
Byte 1: 에러 플래그
```

### 에러 플래그 비트
| 비트 | 상수 | 의미 |
|---|---|---|
| 0 | `ERR_CAN` | CAN 통신 오류 |
| 1 | `ERR_TIMEOUT` | 명령 타임아웃 (30ms) |
| 2 | `ERR_CHECKSUM` | CAN 체크섬 오류 |
| 3 | `ERR_I2C` | 모터 노드 I2C 오류 |

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
- PCAN-View에서 `0x300`에 `50 50 00 00 01 00` (왼쪽 80, 오른쪽 80, enable, 체크섬)
  - 체크섬 = 0x50 ^ 0x50 ^ 0 ^ 0 ^ 0x01 = 0x01
- 시리얼 모니터에 CAN 수신 로그 확인
- I2C 에러(err=2)가 떠도 OK (Arduino #2 아직 연결 안 했으면)

### 단계 3: I2C 통합 테스트
- A4-A4, A5-A5, GND-GND 연결
- PCAN에서 `0x300` 프레임 전송
- Arduino #2의 모터 움직이는지 확인
- PCAN 수신창에서 `0x400` 피드백, `0x410` 하트비트 들어오는지 확인

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
