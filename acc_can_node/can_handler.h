#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * can_handler.h — CAN 통신 인터페이스 (DFRobot_MCP2515 기반)
 *
 * 수신 방식: 폴링 (can_get_cmd()가 loop에서 호출될 때마다 체크)
 *   - ISR + SPI 방식은 송신 SPI와 경합 위험이 있어 폴링으로 변경
 *   - DFRobot_MCP2515 라이브러리의 표준 사용 패턴과 일치
 *   - 500kbps 기준 프레임 간격(~250µs) 대비 loop iteration이 훨씬 빠르므로
 *     정상 트래픽에서 프레임 누락 없음
 *
 * 메시지 포맷:
 *   RX (ECU → 이 보드): 0x300, DLC=6
 *     Byte 0: 왼쪽 쌍 cmd (int8)
 *     Byte 1: 오른쪽 쌍 cmd (int8)
 *     Byte 2~3: reserved
 *     Byte 4: 제어 (bit0=enable, bit2=brake)
 *     Byte 5: checksum (XOR byte0~4)
 *
 *   TX 피드백 (0x400, DLC=4): 속도(int16 BE) + reserved 2B
 *   TX 하트비트 (0x410, DLC=2): 0xAA + 에러 플래그
 */

/* CAN 초기화 */
bool can_init(void);

/*
 * CAN 수신 폴링 + 새 명령 반환
 *   cmd_l, cmd_r: -127~127 (enable=0 또는 brake=1 시 0)
 *   ctrl: bit0=enable, bit2=brake
 *   returns: true=새 유효 명령, false=없음
 */
bool can_get_cmd(int8_t *cmd_l, int8_t *cmd_r, uint8_t *ctrl);

/* 속도 피드백 송신 */
void can_tx_feedback(int16_t spd);

/* 하트비트 송신 */
void can_tx_heartbeat(uint8_t err_flags);

/* CAN 상태 확인 */
bool can_is_ok(void);

/*
 * 체크섬 오류 플래그 소비
 *   can_get_cmd() 수행 중 체크섬 불일치를 만나면 내부 플래그가 세워짐
 *   이 함수를 호출하면 플래그 상태를 반환하고 동시에 클리어
 *   returns: true=최근 체크섬 오류 있었음
 */
bool can_consume_checksum_err(void);
