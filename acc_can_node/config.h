#pragma once

/*
 * config.h — Arduino #1 (CAN 게이트웨이) 설정
 *
 * ┌──────────────────────────────────────────────────────┐
 * │ 이 보드의 역할                                        │
 * │  ECU ↔ CAN(500kbps) ↔ [이 보드] ↔ I2C ↔ 모터 노드     │
 * │                                                       │
 * │ 하드웨어                                              │
 * │  아두이노 우노 + DFRobot CAN BUS Shield               │
 * │  (CAN BUS Shield의 MCP2515 크리스털은 16MHz)         │
 * └──────────────────────────────────────────────────────┘
 */

/* =========================================================
   CAN 설정 — DFRobot CAN BUS Shield
   ========================================================= */
#define CAN_CS_PIN       10         // CAN BUS Shield SPI CS
#define CAN_SPEED        CAN_500KBPS
/*
 * DFRobot_MCP2515 라이브러리는 내부에서 16MHz로 자동 설정함
 * (mcp_can 처럼 MCP_8MHZ/MCP_16MHZ 파라미터 필요 없음)
 *
 * 수신 방식: 폴링 (loop에서 checkReceive)
 *   CAN INT 핀(D2)은 현재 사용하지 않음 → 쉴드에서 그대로 두면 됨
 *   폴링이 ISR+SPI 조합보다 경합 위험이 없고 DFRobot 표준 패턴
 */

/* =========================================================
   CAN ID 구성  (ACC-CANDB/acc_db.dbc 정본)
   =========================================================

   ECU → 이 보드  MTR_CMD (0x210)  주기 10ms  DLC 6  (E2E P01, SAF010 ASIL-B)
     Byte 0: SET_PWM_LF (int8, -128~127)   앞왼쪽
     Byte 1: SET_PWM_RF (int8, -128~127)   앞오른쪽
     Byte 2: SET_PWM_LR (int8, -128~127)   뒤왼쪽
     Byte 3: SET_PWM_RR (int8, -128~127)   뒤오른쪽
     Byte 4 (low nibble): MTR_RC  (Rolling Counter 4bit, 0~15 순환)
     Byte 5:              MTR_CRC (CRC-8 over Byte 0..4, poly 0x07)
     NOTE:
       - E2E 검증 실패(CRC 또는 RC 순서 위반) 시 명령 무효 처리 → 30ms 타임
         아웃 로직(SAF010/SWR018)에 의해 자동 모터 정지로 진입.
       - 이 보드의 모터 노드는 L/R 2-channel 구조 → can_handler 가
         L_pair = avg(LF, LR), R_pair = avg(RF, RR) 로 adapter 매핑.

   이 보드 → ECU,SENSOR  MTR_SPD_FB (0x300)  주기 10ms  DLC 8  (signal-level multicast)
     Byte 0-1: GET_SPD_AVG (int16 LE, factor 0.02 cm/s, ±650 cm/s) — ECU+SENSOR
     Byte 2~7: GET_SPD_LF/RF/LR/RR (int12 × 4개, factor 0.3 cm/s, ±614 cm/s) — ECU only
         비트 레이아웃 (Intel LE):
           LF  : bit 16-27 → Byte 2 + Byte 3 [3:0]
           RF  : bit 28-39 → Byte 3 [7:4] + Byte 4
           LR  : bit 40-51 → Byte 5 + Byte 6 [3:0]
           RR  : bit 52-63 → Byte 6 [7:4] + Byte 7
     NOTE:
       - AVG 는 HMI 고정밀 표시용 (factor 0.02), 4륜은 PID 제어 입력 (factor 0.3).
       - 실제 엔코더는 1개뿐 → AVG 와 4륜 모두 같은 속도값을 scale 만 달리 broadcast.
         다중 엔코더 확장 시 이 보드에서 실제 평균 연산 수행.

   이 보드 → ECU  MTR_HEARTBEAT (0x310)  주기 10ms  DLC 2  (SYS025)
     Byte 0: HB_MTR (uint8, 0~255 순환)
     Byte 1: ERR_MTR (uint8, 0=정상, 비트 플래그 — ERR_* 상수 참조)

   ECU → 이 보드  ECU_HEARTBEAT (0x410)  주기 10ms  DLC 2  (SAF018, ASIL-B)
     Byte 0: HB_ECU, Byte 1: ERR_ECU
     NOTE: 3주기(30ms) 연속 미수신 시 ECU 고장 판단 → 모터 정지.
 */
#define CAN_ID_MTR_CMD       0x210   // 수신 ID: ECU→MTR PWM 명령 (DLC 6, E2E P01)
#define CAN_ID_MTR_SPD_FB    0x300   // 송신 ID: AVG + 4륜 속도 피드백 (DLC 8)
#define CAN_ID_MTR_HEARTBEAT 0x310   // 송신 ID: 모터 노드 HB+ERR
#define CAN_ID_ECU_HEARTBEAT 0x410   // 수신 ID: ECU HB 감시 (SAF018)

/* =========================================================
   I2C 설정 — 모터 노드와 통신
   ========================================================= */
#define MOTOR_SLAVE_ADDR     0x10    // 모터 노드(Arduino #2) I2C 주소
#define I2C_CLOCK            100000L // 100kHz (안전한 기본 속도)

/* =========================================================
   타이밍 (단위: ms)
   ========================================================= */
#define PERIOD_FEEDBACK_MS     10    // 엔코더 피드백 CAN 송신 주기 (MTR_SPD_FB, SYS017)
#define PERIOD_HEARTBEAT_MS    10    // 하트비트 CAN 송신 주기 (MTR_HEARTBEAT, SYS025)
#define TIMEOUT_CMD_MS         30    // MTR_CMD 미수신 타임아웃 → 모터 정지 (SAF ASIL-B, 3주기)
#define TIMEOUT_ECU_HB_MS      30    // ECU_HEARTBEAT 미수신 타임아웃 → FAULT (SAF018, 3주기)

/* =========================================================
   에러 플래그 비트 정의 (ERR_MTR 시그널에 인코딩)
   ========================================================= */
#define ERR_CAN        0x01    // bit0: CAN 통신 초기화/하드웨어 오류
#define ERR_TIMEOUT    0x02    // bit1: MTR_CMD 30ms 타임아웃 (SAF010)
#define ERR_E2E        0x04    // bit2: MTR_CMD E2E 검증 실패 (CRC/RC)
#define ERR_I2C        0x08    // bit3: 모터 노드 I2C 통신 오류
#define ERR_ECU_HB     0x10    // bit4: ECU_HEARTBEAT 30ms 타임아웃 (SAF018)
