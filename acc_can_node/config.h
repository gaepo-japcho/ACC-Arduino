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
   CAN ID 구성
   ========================================================= */
/*
 * ECU → 이 보드 (모터 명령)
 *   DLC 6
 *   Byte 0: 왼쪽 쌍 PWM 명령 (int8, -127~127)
 *   Byte 1: 오른쪽 쌍 PWM 명령 (int8, -127~127)
 *   Byte 2, 3: reserved
 *   Byte 4: Control (bit0=enable, bit2=brake)
 *   Byte 5: Checksum (XOR of byte 0~4)
 *
 * 이 보드 → ECU (속도 피드백)
 *   DLC 4
 *   Byte 0, 1: 엔코더 속도 (int16 big-endian, 0.1 cm/s)
 *   Byte 2, 3: reserved
 *
 * 이 보드 → ECU (Heartbeat)
 *   DLC 2
 *   Byte 0: 0xAA (생존 마커)
 *   Byte 1: 에러 플래그
 */
#define CAN_ID_MOTOR_CMD     0x300   // 수신 ID
#define CAN_ID_MOTOR_FB      0x400   // 송신 (피드백) ID
#define CAN_ID_MOTOR_HB      0x410   // 송신 (하트비트) ID

/* =========================================================
   I2C 설정 — 모터 노드와 통신
   ========================================================= */
#define MOTOR_SLAVE_ADDR     0x10    // 모터 노드(Arduino #2) I2C 주소
#define I2C_CLOCK            100000L // 100kHz (안전한 기본 속도)

/* =========================================================
   타이밍 (단위: ms)
   ========================================================= */
#define PERIOD_FEEDBACK_MS     10    // 엔코더 피드백 CAN 송신 주기
#define PERIOD_HEARTBEAT_MS    50    // 하트비트 CAN 송신 주기
#define TIMEOUT_CMD_MS         30    // ECU 명령 타임아웃 (30ms 초과 → 모터 정지)

/* =========================================================
   에러 플래그 비트 정의
   ========================================================= */
#define ERR_CAN        0x01    // bit0: CAN 통신 오류
#define ERR_TIMEOUT    0x02    // bit1: ECU 명령 타임아웃
#define ERR_CHECKSUM   0x04    // bit2: CAN 체크섬 오류
#define ERR_I2C        0x08    // bit3: 모터 노드 I2C 오류
