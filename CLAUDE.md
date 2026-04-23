# CLAUDE.md — ACC-Arduino

이 저장소는 ACC 스케일카의 **모터 서브시스템 펌웨어**다. 핀 충돌을 피하려 Arduino Uno **2대를 분리**해 역할을 나눴다. 메인 ECU(MPC5606B) 와 CAN 으로 통신하는 게이트웨이 보드 + 모터 4개와 엔코더를 직접 제어하는 드라이버 보드 구조. 독립 git repo. 상위 맥락은 `/mnt/c/acc/CLAUDE.md`, CAN 스키마 정본은 `/mnt/c/acc/ACC-CANDB/acc_db.dbc` 를 참조한다.

## 디렉토리 구조 (현 구조)

```
ACC-Arduino/
├── README.md                       # 하드웨어·배선·빌드·테스트 절차 (본 문서의 사용자용 요약판)
├── acc_can_node/                   # Arduino #1 — CAN ↔ I2C 게이트웨이
│   ├── acc_can_node.ino
│   ├── config.h                    # ★ CAN ID, 주기, I2C 설정
│   ├── can_handler.{h,cpp}         # DFRobot_MCP2515 래퍼
│   └── i2c_handler.{h,cpp}         # Wire 마스터
└── acc_motor_node/                 # Arduino #2 — I2C 슬레이브 + 모터 4개
    ├── acc_motor_node.ino
    ├── config.h                    # 핀 맵, 엔코더 상수 (CAN 없음)
    ├── motor_driver.{h,cpp}        # Arduino Motor Shield R3 ×2 스택 제어
    ├── encoder.{h,cpp}             # 단일 엔코더 (INT0 기반)
    └── i2c_handler.{h,cpp}         # Wire 슬레이브
```

> **참고**: 이전에 `acc_motor_front/` / `acc_motor_rear/` 로 앞뒤 Arduino 분리 설계(`CAN_ID 0x300/0x301` 등) 였으나 `f4dd45c refactor: 아두이노 시스템 구조 변경` (2026 초) 에서 **CAN 게이트웨이 + 모터 드라이버 분리**로 바뀌었다. 구 ID 나 앞/뒤 분리를 참조하는 문서/코드가 남아 있으면 stale.

## 두 보드의 역할

```
┌──────┐   CAN    ┌───────────────────┐   I2C   ┌─────────────────────┐
│ ECU  │══500k═══▶│ Arduino #1         │═100k═══▶│ Arduino #2          │
│MPC5606B◄═══════ │ (acc_can_node)     │◀════════│ (acc_motor_node)    │
└──────┘          │ + DFRobot CAN Shld │         │ + Motor Shield R3×2 │
                  └───────────────────┘         │ + 엔코더 1개 (1-axis)│
                                                 └─────────┬───────────┘
                                                           ▼
                                              4 DC 모터 (L pair / R pair)
```

| 보드 | 역할 | CAN | I2C 역할 | 모터 | 엔코더 |
|---|---|---|---|---|---|
| `acc_can_node/` | 게이트웨이 | RX/TX (ECU↔) | **Master** | — | — |
| `acc_motor_node/` | 드라이버 | 없음 | **Slave** (`0x10`) | 4개 (L/R 2-channel pair) | 1개 (D2 INT0 + D4) |

**원칙**: `config.h` 2개는 역할이 다르니 공통화하지 말 것. `can_handler` 는 #1 에만, `motor_driver`/`encoder` 는 #2 에만 존재. `i2c_handler` 는 양쪽에 각각 있으나 **Master 버전 vs Slave 버전으로 서로 다른 구현**이다.

## CAN 프레임 (DBC 정본: `../ACC-CANDB/acc_db.dbc`)

Arduino #1 이 수신·송신하는 메시지. 상세 시그널 layout 은 DBC + `ACC-CANDB/README.md` 참조.

| 방향 | 상수 (config.h) | ID | DLC | 주기 | 용도 | 근거 |
|---|---|---|---|---|---|---|
| RX | `CAN_ID_MTR_CMD` | **0x210** | 4 | 10 ms | ECU → MTR PWM 명령 | SYS016 |
| TX | `CAN_ID_MTR_SPD_FB` | **0x300** | 8 | 10 ms | 4륜 속도 피드백 | SYS017 |
| TX | `CAN_ID_MTR_HEARTBEAT` | **0x310** | 2 | 10 ms | HB_MTR + ERR_MTR | SYS025 |
| RX | `CAN_ID_ECU_HEARTBEAT` | **0x410** | 2 | 10 ms | ECU 생존 감시 (3주기=30ms 미수신 → FAULT) | SAF018 (ASIL-B) |

구 상수명(`CAN_ID_MOTOR_CMD/FB/HB`) 은 config.h 에 **#define 별칭으로만 유지** — 레거시 호출부가 재컴파일 시 자동 이전. 신규 코드는 `MTR_*` 형식 쓸 것.

## ⚠️ DBC↔하드웨어 차원 불일치 (미해결, 어댑터 필요)

현재 CAN frame 포맷과 하드웨어 구조 사이에 **구조적 gap** 이 있어 `can_handler.cpp` 의 payload 파싱이 아직 DBC 정본과 정합 되지 않았다. 작업 전 반드시 읽을 것.

| 항목 | DBC 정본 | 하드웨어 실제 | 어댑터 필요 지점 |
|---|---|---|---|
| `MTR_CMD` RX | DLC 4, `SET_PWM_LF/RF/LR/RR` 4×int8 | 모터 2 채널 (L pair / R pair) | `can_handler.cpp::can_get_cmd` — DBC 4 PWM 수신 → `L=avg(LF,LR)`, `R=avg(RF,RR)` 로 축소 후 I2C 전달 |
| `MTR_SPD_FB` TX | DLC 8, `GET_SPD_LF/RF/LR/RR` 4×int16 @ 0.02 cm/s | 엔코더 1개 (단일 축 속도) | `can_handler.cpp::can_tx_feedback` — 동일 값을 4채널 broadcast, factor 0.02 cm/s 준수 |
| ctrl/checksum | DBC 에 없음 (제거됨) | 현 `can_handler.cpp` 는 `ctrl_byte(B4)` + `XOR checksum(B5)` 를 기대 (구 DLC 6 포맷) | **DLC 4 포맷으로 재작성 필요** — enable/brake 는 ECU 의 AccStateMachine 에서 PWM=0 으로 표현됨, checksum 은 DBC E2E 프레임워크가 따로 다룸 (현재 MTR_CMD 에는 미적용) |

→ **TODO**: `can_handler.cpp` 를 DBC 정본 (DLC 4, L/R 어댑터 매핑) 으로 재작성. 팀 결정에 따라 DBC 를 2채널(`SET_PWM_L/R`) 로 축소하는 대안도 가능 — 요구사항 SYS016 개정이 동반된다.

## ASIL-B 세이프티 타이머 (30 ms command timeout)

`acc_can_node/config.h`:

```c
#define TIMEOUT_CMD_MS      30   // MTR_CMD 미수신 → 모터 정지 (SAF ASIL-B, 3주기)
#define TIMEOUT_ECU_HB_MS   30   // ECU_HEARTBEAT 미수신 → FAULT  (SAF018, 3주기)
```

둘 다 **3 × 10 ms 주기** 기반. 30 ms 넘게 CAN 명령 프레임이 없거나 ECU HB 가 끊기면 게이트웨이는 즉시:

- I2C 로 `cmd_l = cmd_r = 0` 전송 → 모터 정지
- `ERR_TIMEOUT` 플래그 올리고 다음 HB 송신에 포함

실제 정지 동작은 `acc_motor_node.ino::loop()` 의 "명령 타임아웃 체크" 블록이 수행 (I2C 쪽도 200 ms 타임아웃 별도 보유 — 게이트웨이가 죽은 경우 대비). **절대 타임아웃을 느슨하게 늘리거나 체크를 건너뛰지 말 것** — SAF018/SAF 계열 ASIL-B 요구사항이다.

## Arduino Uno 핀 맵

### Arduino #1 (acc_can_node) — DFRobot CAN BUS Shield 사용

Shield 가 SPI 핀 점유. 실제로 이 노드에서 자유 핀은 거의 없음.

| 핀 | 용도 |
|---|---|
| D2 | CAN INT (현재 폴링 기반이라 미사용. Shield 그대로 둠) |
| D10 | CAN SPI CS (`CAN_CS_PIN`) |
| D11 / D12 / D13 | SPI MOSI / MISO / SCK |
| A4 / A5 | I2C SDA / SCL (Wire Master → Arduino #2) |

- CAN 수신은 `loop()` 에서 `_can.checkReceive()` 폴링. ISR + SPI 조합은 경합 위험이 커서 의도적으로 폴링 방식.
- 크리스털: **16 MHz** (DFRobot 쉴드). 라이브러리 내부 하드코딩이라 `config.h` 에 MCP_8MHZ/16MHZ 같은 매크로는 없음.

### Arduino #2 (acc_motor_node) — Motor Shield R3 ×2 스택

```
D2  — ENC_A   (엔코더 A, INT0 기반 하드웨어 인터럽트)
D3  — PWM_A   (채널 A 속도, 왼쪽 pair)
D4  — ENC_B   (엔코더 B, 방향 판별용 digital read)
D8  — BRK_B   (채널 B 브레이크, 오른쪽 pair)
D9  — BRK_A   (채널 A 브레이크)
D11 — PWM_B   (채널 B 속도, 오른쪽 pair)
D12 — DIR_A   (채널 A 방향)
D13 — DIR_B   (채널 B 방향)
A4/A5 — I2C SDA/SCL (Wire Slave @ 0x10)
```

- **왼쪽 pair (LF+LR) = 모터쉴드 채널 A**, **오른쪽 pair (RF+RR) = 채널 B**.
- 왼쪽 모터는 배선 극성 반대 (차체 전진 기준) — `motor_driver` 에서 직접 DIR 반전이 아니라 배선 레벨에서 뒤집음.
- 2 쉴드 스택 시 전류 센싱 (A0/A1) 은 부정확 → **사용 안 함**.
- 엔코더는 1축만. `PIN_ENC_A=D2 (INT0)`, `PIN_ENC_B=D4 (polling)`. ENC_B 를 ISR 에서 못 받는 건 D3 (INT1) 이 PWM 에 점유되어서.

```
ENCODER_PPR   = 20   // 실측 후 수정 필요
WHEEL_CIRC_MM = 204  // π × 65mm (SO18ED9TC35S-15 + 65mm 바퀴)
```

## 빌드 & 업로드

Arduino IDE 전용. PlatformIO 설정 없음.

| 보드 | 스케치 | 필요 라이브러리 |
|---|---|---|
| Arduino #1 | `acc_can_node/acc_can_node.ino` | **DFRobot_MCP2515** (Library Manager), Wire, SPI (둘 다 내장) |
| Arduino #2 | `acc_motor_node/acc_motor_node.ino` | Wire (내장) |

보드 선택: 둘 다 Arduino Uno. 업로드 순서는 무관하지만 #2 (모터) → #1 (CAN) 순서로 업로드 후 테스트 돌리는 게 안전 (모터가 먼저 정지 상태로 대기).

## 편집 시 주의

- **공통 파일 아님** — 과거 "front/rear sibling sketch" 패턴은 더 이상 존재하지 않는다. 각 보드는 별개 코드베이스. 로직을 복제하지 말고 I2C 로 책임 분리 유지.
- **30 ms command timeout** / ECU HB 모니터링 / I2C 200 ms 타임아웃은 다층 방어. safety 명목 외로 건드리지 말 것. 근거: `../ACC-docs/reqs/STK/SYS/SAF.sdoc` (SAF018 외).
- **CAN ID / 주기는 DBC 정본** — `config.h` 상수 바꾸기 전에 `../ACC-CANDB/acc_db.dbc` 와 `../ACC-docs/reqs/STK/SYS.sdoc` (SYS016/017/025, SAF018) 양쪽 확인.
- **DLC 4 vs 6 포맷** — 위 "DBC↔하드웨어 차원 불일치" 표의 adapter 작업 완료 전에는 `can_handler.cpp` 가 ECU 로부터 DBC 정본 프레임을 받으면 `len >= 5` 검증에서 통과하지만 체크섬 검증이 깨지거나 ctrl_byte 가 랜덤해질 수 있음. 실기 테스트 전 먼저 고칠 것.
- **DFRobot_MCP2515** vs **mcp_can** — 혼동 주의. 이 프로젝트는 DFRobot 쉴드라 `DFRobot_MCP2515.h` 사용. Library Manager 검색 시 "MCP_CAN" 하나만 나오면 안 됨.
- **한국어 주석 유지** — 기존 코드·헤더가 전부 한국어. 번역하지 말 것.

## 요구사항 트레이스 (현황)

소스 곳곳에 `SWR###` / `SAF###` / `HWR###` 태그를 아직 심지 않았다. 최소한 다음 지점엔 주석으로 부착할 것:

- `acc_can_node/config.h::TIMEOUT_CMD_MS` / `TIMEOUT_ECU_HB_MS` → SAF018 ASIL-B
- `acc_can_node/can_handler.cpp` 프레임 포맷 → SYS016 / SYS017 / SYS025
- `acc_motor_node.ino::motor_stop_all()` safe-state 진입 → SAF 계열

원문은 `../ACC-docs/reqs/STK/SYS.sdoc` 및 `SYS/SAF.sdoc`.
