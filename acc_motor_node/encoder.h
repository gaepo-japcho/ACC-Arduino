#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * encoder.h — 엔코더 속도 계산 (단일 엔코더)
 *
 * 직진/정지/후진만 하는 차량이므로 엔코더 1개로 전체 차속 대표.
 * D2(INT0) 하드웨어 인터럽트 사용.
 *
 * 속도 공식:
 *   spd(0.1 cm/s) = count × WHEEL_CIRC_MM × 100 / ENCODER_PPR
 *
 *   ENCODER_PPR 은 CHANGE 트리거 기준 1회전당 펄스 수 (config.h 측정법 참조).
 *   따라서 식의 분모는 PPR 그대로 — ×2 보정 불필요.
 *   ×100 = 10ms 주기 → 1초 환산. (mm/s 와 0.1 cm/s 는 수치가 동일하므로 단위 변환 추가 계수 없음.)
 *
 * 부호:
 *   cmd(명령) >= 0 → 속도 양수 (전진)
 *   cmd      <  0 → 속도 음수 (후진)
 */

/* 핀 초기화 + 인터럽트 등록 — setup()에서 호출 */
void encoder_init(void);

/*
 * 속도 계산 — loop()에서 10ms마다 호출
 *   cmd: 현재 모터 명령 (방향 부호 판단용)
 */
void encoder_calc(int8_t cmd);

/* 현재 속도 반환 (단위: 0.1 cm/s, 예: 510 → 51.0 cm/s) */
int16_t encoder_get_spd(void);

/* 누적 펄스 수 (디버그/오도메트리용) */
uint32_t encoder_get_count(void);
