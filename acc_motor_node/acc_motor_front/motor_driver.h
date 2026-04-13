#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * motor_driver.h — 모터 제어 인터페이스
 *
 * L298N H-Bridge 드라이버 기준:
 *   EN핀(PWM): 속도 결정 — 0~255 (0=정지, 255=최대)
 *   IN1, IN2:  방향 결정
 *     IN1=HIGH, IN2=LOW  → 전진
 *     IN1=LOW,  IN2=HIGH → 후진
 *     IN1=LOW,  IN2=LOW  → 정지(브레이크)
 *
 * cmd 값 규칙 (CAN DB 기준):
 *   -127 ~ -1  : 후진 (절대값이 클수록 빠름)
 *       0      : 정지
 *   +1 ~ +127  : 전진 (클수록 빠름)
 */

/* 초기화 — setup()에서 한 번 호출 */
void motor_init(void);

/*
 * 두 모터를 한꺼번에 설정
 * cmd_l: 왼쪽 모터 명령 (-127~127)
 * cmd_r: 오른쪽 모터 명령 (-127~127)
 */
void motor_set_both(int8_t cmd_l, int8_t cmd_r);

/* 즉시 전체 정지 — 타임아웃/FAULT 시 호출 */
void motor_stop_all(void);
