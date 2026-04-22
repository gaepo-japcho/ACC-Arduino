#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * motor_driver.h — 아두이노 모터 쉴드 R3 제어 (2장 스택)
 *
 * 쉴드 2장을 스택하면 핀(D3, D8, D9, D11, D12, D13)이 공유됨 →
 * 쉴드1-A와 쉴드2-A가 동일 신호, 쉴드1-B와 쉴드2-B가 동일 신호.
 * 결과적으로 "왼쪽 쌍(2개)" / "오른쪽 쌍(2개)" 두 채널만 독립 제어.
 *
 * 배선 규칙:
 *   쉴드1 채널 A → 앞 왼쪽 모터    (극성 뒤집기)
 *   쉴드2 채널 A → 뒤 왼쪽 모터    (극성 뒤집기)
 *   쉴드1 채널 B → 앞 오른쪽 모터  (정방향)
 *   쉴드2 채널 B → 뒤 오른쪽 모터  (정방향)
 *
 * 명령 값 (cmd):
 *   -127 ~ -1  : 후진 (절대값이 클수록 빠름)
 *        0     : 정지
 *   +1 ~ +127  : 전진
 */

/* 초기화 — setup()에서 한 번 호출 */
void motor_init(void);

/*
 * 두 채널(왼/오른쪽 쌍)을 한꺼번에 설정
 *   cmd_l: 왼쪽 쌍 명령 (-127~127)
 *   cmd_r: 오른쪽 쌍 명령 (-127~127)
 *   ctrl : bit0=enable, bit2=brake
 *          enable=0 또는 brake=1 이면 정지
 */
void motor_set_both(int8_t cmd_l, int8_t cmd_r, uint8_t ctrl);

/* 즉시 전체 정지 — 타임아웃/에러 시 호출 */
void motor_stop_all(void);
