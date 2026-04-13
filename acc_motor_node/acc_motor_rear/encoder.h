#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * encoder.h — 엔코더 속도 계산
 *
 * 나중에 실제 PPR, 바퀴 직경 측정 후 아래 상수만 수정하면 됩니다.
 * 이 파일 자체는 건드릴 필요 없어요.
 */

/* =========================================================
   실측 후 수정할 상수 2개
   ========================================================= */

/*
 * PPR (Pulses Per Revolution): 모터 1회전당 엔코더 펄스 수
 *
 * 측정 방법:
 *   1. 아래 encoder.cpp의 DEBUG_ENCODER를 1로 설정
 *   2. 모터를 정확히 1바퀴 돌린다
 *   3. 시리얼 모니터에 출력된 count 값 = PPR
 *      (CHANGE 트리거 사용 중이므로 실제 스펙 PPR × 2 값이 나옴)
 *   4. 그 값을 ENCODER_PPR에 넣는다
 */
#define ENCODER_PPR     20      // ← 실측 후 수정

/*
 * 바퀴 둘레 (단위: mm)
 * 계산: π × 직경(mm)
 * 예) 직경 65mm → 3.14159 × 65 ≈ 204
 */
#define WHEEL_CIRC_MM   204     // ← 실측 후 수정

/* =========================================================
   공개 함수
   ========================================================= */

/* 엔코더 핀 초기화 + 인터럽트 등록 — setup()에서 호출 */
void encoder_init(void);

/*
 * 속도 계산 — loop()에서 10ms마다 호출
 * ISR이 누적한 펄스 수 → 속도(0.1 cm/s) 계산 → 내부 저장
 *
 * cmd_l, cmd_r: 현재 모터 명령 (방향 판단용)
 */
void encoder_calc(int8_t cmd_l, int8_t cmd_r);

/*
 * 속도값 반환 (단위: 0.1 cm/s, int16)
 * 예) 510 반환 → 51.0 cm/s
 * 후진 중이면 음수
 */
int16_t encoder_get_spd_l(void);
int16_t encoder_get_spd_r(void);
