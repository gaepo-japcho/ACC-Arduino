#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * can_handler.h — CAN 통신 인터페이스 (DFRobot_MCP2515 기반)
 *
 * DBC 정본: ACC-CANDB/acc_db.dbc
 *
 * 수신 방식: 폴링 (can_poll() 을 loop() 에서 주기적으로 호출).
 *   - ISR + SPI 방식은 송신 SPI 와 경합 위험이 있어 폴링으로 운영.
 *   - 500 kbps 기준 프레임 간격(~250µs) 대비 loop iteration 이 훨씬 빠르므로
 *     정상 트래픽에서 프레임 누락 없음.
 *
 * 메시지 요약:
 *
 *   RX  MTR_CMD       (0x210, DLC 6, 10ms) — E2E P01 (SAF010 ASIL-B)
 *       Byte 0..3: SET_PWM_LF/RF/LR/RR (int8)
 *       Byte 4 [3:0]: MTR_RC  (Rolling Counter, 0~15 순환)
 *       Byte 5      : MTR_CRC (CRC-8 over Byte 0..4, poly 0x07 CCITT)
 *     → 4채널 → L/R pair 어댑터: L = avg(LF, LR), R = avg(RF, RR)
 *
 *   RX  ECU_HEARTBEAT (0x410, DLC 2, 10ms) — SAF018 ASIL-B
 *       Byte 0: HB_ECU, Byte 1: ERR_ECU
 *
 *   TX  MTR_SPD_FB    (0x300, DLC 8, 10ms) — signal-level multicast
 *       Byte 0-1: GET_SPD_AVG (int16, factor 0.02 cm/s)
 *       Byte 2-7: GET_SPD_LF/RF/LR/RR (int12 × 4, factor 0.3 cm/s, bit-packed)
 *     엔코더 1개 → AVG 와 4륜 값 모두 같은 측정치를 scale 변환만 달리 인코딩.
 *
 *   TX  MTR_HEARTBEAT (0x310, DLC 2, 10ms)
 *       Byte 0: HB_MTR  (순환 카운터, can_handler 내부 자동 증가)
 *       Byte 1: ERR_MTR (ERR_* 비트 플래그)
 */

/* CAN 초기화. 실패 시 false. */
bool can_init(void);

/*
 * RX 큐 드레인 (한 loop() iteration 마다 1회 호출).
 * 수신 프레임을 ID 로 분기해 내부 상태를 갱신:
 *   - MTR_CMD      : E2E 검증 + 4→2 어댑터 → _cmd_l/_cmd_r + _has_new_cmd
 *   - ECU_HEARTBEAT: _hb_ecu / _err_ecu    + _has_new_hb
 *   그 외 ID 는 무시.
 */
void can_poll(void);

/*
 * 최근 수신한 유효 MTR_CMD 를 조회.
 *   cmd_l, cmd_r : L/R pair PWM (-128~127). E2E 통과한 값만 반영.
 *   returns      : true=신규 명령, false=없음. true 반환 후 내부 플래그 클리어.
 */
bool can_get_cmd(int8_t *cmd_l, int8_t *cmd_r);

/*
 * 최근 수신한 ECU_HEARTBEAT 조회.
 *   hb, err : 수신 시 갱신값
 *   returns : true=신규 수신, false=없음. true 반환 후 내부 플래그 클리어.
 */
bool can_get_ecu_hb(uint8_t *hb, uint8_t *err);

/*
 * MTR_SPD_FB 송신 (DLC 8).
 *   spd_raw_02 : int16, factor 0.02 cm/s raw 값 (GET_SPD_AVG 시그널에 그대로).
 *                4륜 int12 값은 내부에서 spd_raw_02 / 15 로 스케일 후 bit-pack.
 */
void can_tx_feedback(int16_t spd_raw_02);

/*
 * MTR_HEARTBEAT 송신 (DLC 2). HB_MTR 카운터는 내부에서 자동 증가.
 *   err_flags : ERR_MTR 바이트 (config.h ERR_* 플래그 OR)
 */
void can_tx_heartbeat(uint8_t err_flags);

/* CAN 하드웨어 상태 조회 */
bool can_is_ok(void);

/*
 * E2E 검증 실패 플래그 소비.
 *   can_poll() 중 CRC 또는 RC 순서 불일치를 만나면 내부 플래그가 set 됨.
 *   이 함수를 호출하면 플래그 상태를 반환하고 동시에 클리어.
 *   returns: true=최근 E2E 오류 발생
 */
bool can_consume_e2e_err(void);
