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

   ECU → 이 보드  MTR_CMD (0x210)  주기 10ms  DLC 4
     Byte 0: SET_PWM_LF (int8, -128~127)   앞왼쪽
     Byte 1: SET_PWM_RF (int8, -128~127)   앞오른쪽
     Byte 2: SET_PWM_LR (int8, -128~127)   뒤왼쪽
     Byte 3: SET_PWM_RR (int8, -128~127)   뒤오른쪽
     NOTE: 이 보드의 모터 노드는 L/R 2-channel 구조 → can_handler 가
           L_pair = avg(LF, LR), R_pair = avg(RF, RR) 로 adapter 매핑.

   이 보드 → ECU  MTR_SPD_FB (0x300)  주기 10ms  DLC 8
     Byte 0-1: GET_SPD_LF (int16 LE, factor 0.02 cm/s, ±650 cm/s)
     Byte 2-3: GET_SPD_RF
     Byte 4-5: GET_SPD_LR
     Byte 6-7: GET_SPD_RR
     NOTE: 실제 엔코더는 1개뿐 → 4채널에 같은 값 broadcast.
           (DBC는 향후 4엔코더 확장 대비. TODO: 요구사항 논의)

   이 보드 → ECU  MTR_HEARTBEAT (0x310)  주기 10ms  DLC 2  (SYS025)
     Byte 0: HB_MTR (uint8, 0~255 순환)
     Byte 1: ERR_MTR (uint8, 0=정상, 비트 플래그)

   ECU → 이 보드  ECU_HEARTBEAT (0x410)  주기 10ms  DLC 2  (SAF018, ASIL-B)
     Byte 0: HB_ECU
     Byte 1: ERR_ECU
     NOTE: 3주기(30ms) 연속 미수신 시 ECU 고장 판단 → 모터 정지.
 */
#define CAN_ID_MTR_CMD       0x210   // 수신 ID: ECU→MTR PWM 명령
#define CAN_ID_MTR_SPD_FB    0x300   // 송신 ID: 4륜 속도 피드백
#define CAN_ID_MTR_HEARTBEAT 0x310   // 송신 ID: 모터 노드 HB+ERR
#define CAN_ID_ECU_HEARTBEAT 0x410   // 수신 ID: ECU HB 감시 (SAF018)

/* 하위 호환 별칭 (can_handler.cpp 에서 기존 이름 참조 시) */
#define CAN_ID_MOTOR_CMD     CAN_ID_MTR_CMD
#define CAN_ID_MOTOR_FB      CAN_ID_MTR_SPD_FB
#define CAN_ID_MOTOR_HB      CAN_ID_MTR_HEARTBEAT

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
   에러 플래그 비트 정의
   ========================================================= */
#define ERR_CAN        0x01    // bit0: CAN 통신 오류
#define ERR_TIMEOUT    0x02    // bit1: ECU 명령 타임아웃
#define ERR_CHECKSUM   0x04    // bit2: CAN 체크섬 오류
#define ERR_I2C        0x08    // bit3: 모터 노드 I2C 오류
