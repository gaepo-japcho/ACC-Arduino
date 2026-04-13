#include "encoder.h"

/*
 * encoder.cpp — 엔코더 속도 계산 구현
 *
 * 왼쪽(L): 하드웨어 인터럽트 INT1(핀3) CHANGE 트리거
 * 오른쪽(R): 소프트웨어 폴링 (핀 A0) — 핀 부족으로 인터럽트 불가
 *
 * 속도 공식:
 *   speed(0.1cm/s) = count × WHEEL_CIRC_MM × 1000
 *                    / (ENCODER_PPR × 2)
 *
 *   CHANGE 트리거 → 상승+하강 모두 카운트 → ÷2 보정 포함
 *   ×1000 = ×100(10ms→1s 환산) × ×10(mm/s→0.1cm/s 환산)
 */

/* 1로 바꾸면 시리얼로 카운트/속도 출력 — PPR 측정할 때 사용 */
#define DEBUG_ENCODER   0

static volatile uint16_t _count_l = 0;     /* ISR이 올림, volatile 필수 */
static uint8_t  _prev_r  = 0;              /* 오른쪽 이전 핀 상태 */
static uint16_t _count_r = 0;              /* 오른쪽 카운터 */
static int16_t  _spd_l   = 0;              /* 계산된 왼쪽 속도 */
static int16_t  _spd_r   = 0;              /* 계산된 오른쪽 속도 */

/* 왼쪽 엔코더 ISR — 짧게 유지 */
static void isr_enc_l(void) {
    _count_l++;
}

/* count와 방향 명령으로 0.1cm/s 단위 속도 계산 */
static int16_t calc_speed(uint16_t count, int8_t cmd) {
    uint32_t spd = (uint32_t)count
                   * (uint32_t)WHEEL_CIRC_MM
                   * 1000UL
                   / ((uint32_t)ENCODER_PPR * 2UL);

    if (spd > 32767UL) spd = 32767UL;
    return (cmd >= 0) ? (int16_t)spd : -(int16_t)spd;
}

void encoder_init(void) {
    pinMode(PIN_ENC_L, INPUT_PULLUP);
    attachInterrupt(ENC_L_INT_NUM, isr_enc_l, CHANGE);

    pinMode(PIN_ENC_R, INPUT_PULLUP);
    _prev_r = digitalRead(PIN_ENC_R);
}

void encoder_calc(int8_t cmd_l, int8_t cmd_r) {
    /* 왼쪽: 인터럽트 차단 후 원자적 읽기 */
    noInterrupts();
    uint16_t cnt_l = _count_l;
    _count_l = 0;
    interrupts();

    /* 오른쪽: 상태 변화 감지 */
    uint8_t cur_r = digitalRead(PIN_ENC_R);
    if (cur_r != _prev_r) {
        _count_r++;
        _prev_r = cur_r;
    }

    _spd_l = calc_speed(cnt_l, cmd_l);
    _spd_r = calc_speed(_count_r, cmd_r);
    _count_r = 0;

#if DEBUG_ENCODER
    Serial.print(F("L cnt:")); Serial.print(cnt_l);
    Serial.print(F(" spd:")); Serial.print(_spd_l);
    Serial.print(F("  R cnt:")); Serial.print(_count_r);
    Serial.print(F(" spd:")); Serial.println(_spd_r);
#endif
}

int16_t encoder_get_spd_l(void) { return _spd_l; }
int16_t encoder_get_spd_r(void) { return _spd_r; }
