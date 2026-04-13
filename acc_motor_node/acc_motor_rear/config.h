#pragma once

/*
 * config.h — 뒤 아두이노 (REAR) 설정
 *
 * ┌─────────────────────────────────────────────┐
 * │ 앞 아두이노 담당: LF(왼쪽 앞), RF(오른쪽 앞) │
 * │ 뒤 아두이노 담당: LR(왼쪽 뒤), RR(오른쪽 뒤) │
 * └─────────────────────────────────────────────┘
 */

/* =========================================================
   이 보드가 앞(FRONT)인지 뒤(REAR)인지 선택
   ========================================================= */
// #define BOARD_FRONT     // 앞 아두이노
#define BOARD_REAR   // 뒤 아두이노

/* =========================================================
   CAN 설정
   ========================================================= */
#define CAN_CS_PIN      10      // MCP2515 칩 선택 핀 (SPI)
#define CAN_INT_PIN     2       // MCP2515 인터럽트 핀 (INT0)
#define CAN_SPEED       CAN_500KBPS
#define CAN_CLOCK       MCP_8MHZ    // 구매한 모듈 크리스탈: 8MHz

/*
 * CAN ID 구성
 *
 * ECU → 아두이노 (명령):
 *   0x300: 앞 아두이노로 LF, RF 모터 명령
 *   0x301: 뒤 아두이노로 LR, RR 모터 명령
 *
 * 아두이노 → ECU (피드백):
 *   0x400: 앞 아두이노가 LF, RF 속도 피드백 전송
 *   0x401: 뒤 아두이노가 LR, RR 속도 피드백 전송
 *
 * Heartbeat:
 *   0x410: 앞 아두이노 생존 신호
 *   0x411: 뒤 아두이노 생존 신호
 */
#ifdef BOARD_FRONT
  #define CAN_ID_MOTOR_CMD    0x300   // 이 보드가 수신할 명령 ID
  #define CAN_ID_MOTOR_FB     0x400   // 이 보드가 송신할 피드백 ID
  #define CAN_ID_MOTOR_HB     0x410   // 이 보드의 Heartbeat ID
#else
  #define CAN_ID_MOTOR_CMD    0x301
  #define CAN_ID_MOTOR_FB     0x401
  #define CAN_ID_MOTOR_HB     0x411
#endif

/* =========================================================
   핀 배치
   아두이노 우노 PWM 가능 핀: 3, 5, 6, 9, 10, 11
     - 10: MCP2515 CS → 사용 불가
     - 11: MCP2515 MOSI → 사용 불가
     - 2:  MCP2515 INT → 사용 불가
     - 3:  엔코더 INT1 → 사용 불가
   사용 가능 PWM: 5, 6, 9 (모터 2개 EN에 딱 맞음)

   ┌─────────────────────────────────────────────────────┐
   │ 핀  │ 용도                                          │
   ├─────────────────────────────────────────────────────┤
   │  2  │ MCP2515 INT (CAN 인터럽트)                    │
   │  3  │ 엔코더 L_ENC_A (하드웨어 인터럽트 INT1)       │
   │  4  │ L모터 IN1 (방향)                              │
   │  5  │ L모터 EN (PWM 속도)                           │
   │  6  │ R모터 EN (PWM 속도)                           │
   │  7  │ L모터 IN2 (방향)                              │
   │  8  │ R모터 IN1 (방향)                              │
   │  9  │ R모터 IN2 (방향)  ← IN2를 9번에 배치          │
   │ 10  │ MCP2515 CS (SPI)                             │
   │ 11  │ MCP2515 MOSI (SPI)                           │
   │ 12  │ MCP2515 MISO (SPI)                           │
   │ 13  │ MCP2515 SCK (SPI)                            │
   │ A0  │ 엔코더 R_ENC_A (소프트웨어 폴링)              │
   └─────────────────────────────────────────────────────┘
   ========================================================= */

/* 왼쪽 모터 (L) — 앞이면 LF, 뒤면 LR */
#define PIN_L_EN    5       // PWM — 속도 제어
#define PIN_L_IN1   4       // 방향 제어
#define PIN_L_IN2   7       // 방향 제어

/* 오른쪽 모터 (R) — 앞이면 RF, 뒤면 RR */
#define PIN_R_EN    6       // PWM — 속도 제어
#define PIN_R_IN1   8       // 방향 제어
#define PIN_R_IN2   9       // 방향 제어 (방향 핀이라 PWM 불필요)

/* 엔코더 핀 */
#define PIN_ENC_L       3   // 왼쪽 엔코더 — 하드웨어 인터럽트 INT1
#define PIN_ENC_R       A0  // 오른쪽 엔코더 — 소프트웨어 폴링 (핀 부족)
#define ENC_L_INT_NUM   1   // attachInterrupt(1, ...) = 핀 3

/* =========================================================
   타이밍 (단위: ms)
   ========================================================= */
#define PERIOD_CTRL_MS          10  // 모터 제어 주기
#define PERIOD_FEEDBACK_MS      10  // CAN 피드백 송신 주기
#define PERIOD_HEARTBEAT_MS     50  // 생존 신호 송신 주기
#define PERIOD_ENC_CALC_MS      10  // 엔코더 속도 계산 주기
#define TIMEOUT_CMD_MS          30  // ECU 명령 타임아웃 (30ms 초과 시 정지)

/* =========================================================
   에러 플래그 비트 정의
   ========================================================= */
#define ERR_CAN         0x01    // bit0: CAN 통신 오류
#define ERR_TIMEOUT     0x02    // bit1: 명령 타임아웃
#define ERR_CHECKSUM    0x04    // bit2: 체크섬 오류
