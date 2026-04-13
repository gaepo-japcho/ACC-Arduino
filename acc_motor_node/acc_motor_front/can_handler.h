#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * can_handler.h — CAN 통신 인터페이스
 *
 * 수신 방식: CAN 인터럽트 (INT 핀 LOW → ISR 호출)
 *   이전 방식(폴링): loop()마다 digitalRead(INT핀) 확인
 *   인터럽트 방식:   메시지 도착 즉시 ISR이 자동 호출 → 더 빠르고 CPU 효율적
 *
 * CAN DB (이 보드 기준):
 *   RX: 0x300(앞) 또는 0x301(뒤) — ECU가 보내는 모터 명령
 *   TX: 0x400(앞) 또는 0x401(뒤) — 이 보드가 보내는 속도 피드백
 *   HB: 0x410(앞) 또는 0x411(뒤) — 생존 신호
 */

/* CAN 초기화 — setup()에서 호출 */
bool can_init(void);

/*
 * CAN 인터럽트 핸들러 — setup()에서 attachInterrupt로 등록
 * 직접 호출하지 말 것 (ISR 자동 호출)
 *
 * ISR 제약:
 *   - 최대한 짧게 작성
 *   - delay(), Serial.print() 금지
 *   - 공유 변수는 volatile 선언 필수
 */
void isr_can_rx(void);

/*
 * 수신된 명령을 꺼내옴
 * loop()에서 매 주기 호출 — 새 명령이 없으면 이전 값 유지
 *
 * cmd_l: 왼쪽 모터 명령 출력 (-127~127)
 * cmd_r: 오른쪽 모터 명령 출력 (-127~127)
 * returns: true = 새 명령이 도착했음, false = 새 명령 없음
 */
bool can_get_cmd(int8_t *cmd_l, int8_t *cmd_r);

/* 속도 피드백 송신 (10ms 주기) */
void can_tx_feedback(int16_t spd_l, int16_t spd_r);

/* Heartbeat 송신 (50ms 주기) */
void can_tx_heartbeat(uint8_t err_flags);

/* CAN 연결 상태 확인 */
bool can_is_ok(void);
