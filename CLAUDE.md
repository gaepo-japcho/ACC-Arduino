# CLAUDE.md — ACC_ARDUINO

이 저장소는 ACC 스케일카의 **모터 노드 펌웨어**다. 전륜(front) / 후륜(rear) 두 개의 Arduino Uno 가 각각 1대씩 DC 모터 2개를 구동하며, MCP2515 를 통해 메인 ECU(MPC5606B) 와 CAN 통신한다. 독립 git repo. 상위 맥락은 `/mnt/c/acc/CLAUDE.md` 참조.

## 디렉토리 구조

```
ACC_ARDUINO/
├── README.md                       # 제목만 있는 스텁
└── acc_motor_node/
    ├── acc_motor_front/            # 전륜 스케치
    │   ├── acc_motor_front.ino
    │   ├── config.h                # ★ BOARD_FRONT 정의
    │   ├── can_handler.{h,cpp}
    │   ├── motor_driver.{h,cpp}
    │   └── encoder.{h,cpp}
    └── acc_motor_rear/             # 후륜 스케치 (front 와 파일 구조 동일)
        ├── acc_motor_rear.ino
        ├── config.h                # ★ BOARD_REAR 정의
        └── ...
```

두 스케치는 **자매 스케치(sibling sketch)** — 구조가 동일하고 내용도 거의 같다. 차이는 `config.h` 의 `#define BOARD_FRONT` vs `#define BOARD_REAR` 매크로 하나뿐이며, 그 매크로가 CAN ID 와 로그 문자열을 갈라낸다.

## 공유 로직 vs 보드별 분기

| 파일 | 공유 여부 | 주의 |
|---|---|---|
| `can_handler.{h,cpp}` | **완전 공유** | 버그 수정 시 양쪽 모두 반영 |
| `motor_driver.{h,cpp}` | **완전 공유** | 핀 상수는 `config.h` 에서 가져옴 |
| `encoder.{h,cpp}` | **완전 공유** | PPR, 둘레는 `config.h` 에서 가져옴 |
| `config.h` | **보드별로 다름** | `BOARD_FRONT` / `BOARD_REAR` 매크로, CAN ID 분기, 로그 라벨 |
| `*.ino` | **거의 동일** | 로그 문자열("FRONT"/"REAR") 외 로직 동일 |

**원칙**: 공유 파일을 고치면 front/rear 양쪽에 반드시 같은 내용으로 복사한다. 한쪽만 고치면 두 보드 간 동작이 silently 어긋난다.

## CAN 프레임 레이아웃

`config.h:41-49` 에서 ID 가 결정된다:

| 방향 | 메시지 | Front | Rear | 주기 | 페이로드 |
|---|---|---|---|---|---|
| RX (ECU→노드) | `CAN_ID_MOTOR_CMD` | 0x300 | 0x301 | 10 ms | B0=L_cmd(int8), B1=R_cmd(int8), B4=ctrl flags, B5=XOR checksum |
| TX (노드→ECU) | `CAN_ID_FEEDBACK` | 0x400 | 0x401 | 10 ms | int16 big-endian L/R 속도 (0.1 cm/s) |
| TX (노드→ECU) | `CAN_ID_HEARTBEAT` | 0x410 | 0x411 | 50 ms | B0=0xAA, B1=err flags |

- 500 kbps, MCP2515 크리스탈 **8 MHz** (`config.h:21-24`).
- ISR 기반 RX 필터가 `CAN_ID_MOTOR_CMD` 하나만 받도록 세팅되어 있다 (`can_handler.cpp`).
- RX 체크섬은 B0~B4 XOR 이며 실패 시 해당 프레임 drop + `ERR_CHECKSUM` 플래그 set.

## ASIL-B 세이프티 타이머 (30 ms 타임아웃)

**위치**: `acc_motor_front.ino:74-81` (rear 도 동일 위치).

```cpp
if ((now - g_t_last_cmd) > TIMEOUT_CMD_MS) {   // TIMEOUT_CMD_MS = 30  (config.h:101)
    if (g_cmd_l != 0 || g_cmd_r != 0) {
        g_cmd_l = g_cmd_r = 0;
        motor_stop_all();
    }
    g_err_flags |= ERR_TIMEOUT;
}
```

- 30 ms 넘게 CMD 프레임이 없으면 즉시 모터 정지 + `ERR_TIMEOUT` 올림.
- 이건 **ISO 26262 ASIL-B 요구사항**이므로 절대로 "간단하게" 제거하거나 타임아웃을 느슨하게 늘리지 말 것. `../ACC-docs/reqs/STK/SYS/SAF.sdoc` 에 대응 요구사항이 있다.
- `motor_stop_all()` 은 EN PWM=0 + 방향 핀 모두 LOW (브레이크).

## Arduino Uno 핀 맵 제약

Uno 는 PWM 가용 핀이 **3, 5, 6, 9, 10, 11** 6개뿐이고, 이 프로젝트에서 대부분 소진된다. `config.h:53-92` 에 ASCII 표로 상세 기재:

```
D2  — MCP2515 INT      (PWM 아님, 외부 인터럽트 INT0)
D3  — ENC_L interrupt  (PWM 가능하지만 INT1 로 사용)
D4  — L Motor IN1      (digital)
D5  — L Motor EN       (PWM)                  ← 모터 속도
D6  — R Motor EN       (PWM)                  ← 모터 속도
D7  — L Motor IN2      (digital)
D8  — R Motor IN1      (digital)
D9  — R Motor IN2      (PWM 가능하지만 digital 로 사용)
D10 — MCP2515 CS       (SPI)
D11 — MCP2515 MOSI     (SPI)
D12 — MCP2515 MISO     (SPI)
D13 — MCP2515 SCK      (SPI)
A0  — ENC_R polling    ★ INT 핀이 부족해 소프트웨어 폴링
```

- **ENC_R 은 폴링 기반** — 외부 인터럽트 핀이 D2(CAN INT) / D3(ENC_L INT) 에 모두 잡혀서 어쩔 수 없다. 고속 회전 시 샘플 누락 가능성 있음.
- 모터 드라이버는 L298N (IN1/IN2 로 방향, EN PWM 으로 속도). `motor_driver.cpp` 의 `cmd << 1` 로 int8 범위 −127…+127 를 PWM 0…254 로 스케일.
- **핀을 추가로 쓰고 싶으면 위 표부터 보고 시작할 것.** 자유 핀은 사실상 A1~A5 뿐이며 PWM 은 아니다.

## 엔코더 상수 (측정 필요)

`config.h:27-34`:
- `ENCODER_PPR = 20` ← 실측 후 수정 필요 (주석에 명시)
- `WHEEL_CIRC_MM = 204` ← 1/10 스케일 가정, 실측 후 수정 필요
- 속도 계산: `count × WHEEL_CIRC_MM × 1000 / (ENCODER_PPR × 2)` → 0.1 cm/s

## 빌드

- **Arduino IDE** 로 `.ino` 를 열어 빌드/업로드. PlatformIO 설정은 없음.
- 외부 라이브러리: **MCP_CAN** (Arduino Library Manager 에서 설치). 이것 하나뿐.
- 보드 선택: Arduino Uno.

## 요구사항 트레이스 (현황)

- 현재 소스에 `SWR###` / `SAF###` / `HWR###` 태그가 **들어가 있지 않다**. 루트 CLAUDE.md 의 트레이스 원칙상 추가하는 것이 좋다 — 특히 세이프티 타임아웃은 SAF 대응, CAN 포맷은 SWR 대응. 원문은 `../ACC-docs/reqs/STK/SYS/{SWR,SAF,HWR}.sdoc`.

## 편집 시 주의

- 공유 파일 수정은 front + rear 둘 다에 반영 (grep 으로 누락 확인).
- 30 ms 타임아웃 / 체크섬 검증 / 하트비트 주기를 safety 명목 외로 건드리지 말 것.
- MCP2515 크리스탈은 보드에 따라 8 MHz 또는 16 MHz — 실물 확인 후 `config.h` 맞출 것.
- 한국어 주석은 그대로 유지.
