#include "encoder.h"

/*
 * 1로 바꾸면 시리얼에 카운트/속도 출력 — PPR 측정에 사용
 * PPR 측정 순서:
 *   1) DEBUG_ENCODER = 1 로 변경 후 업로드
 *   2) 시리얼 모니터 열기 (115200)
 *   3) 모터를 정확히 1바퀴 수동 회전
 *   4) 누적 count 값 = PPR  →  config.h의 ENCODER_PPR 에 대입
 */
#define DEBUG_ENCODER   0

static volatile uint16_t _count       = 0;    /* 10ms 주기 펄스 수 */
static volatile uint32_t _count_total = 0;    /* 누적 펄스 수 */
static int16_t           _spd         = 0;    /* 계산된 속도 */

/* =========================================================
   ISR — 최대한 짧게
   ========================================================= */
static void isr_encoder(void) {
    _count++;
    _count_total++;
}

/* =========================================================
   공개 함수
   ========================================================= */
void encoder_init(void) {
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);

    attachInterrupt(ENC_INT_NUM, isr_encoder, CHANGE);
}

void encoder_calc(int8_t cmd) {
    /* ISR 공유 변수 원자적 읽기 */
    noInterrupts();
    uint16_t cnt = _count;
    _count = 0;
    interrupts();

    /*
     * 속도 계산 — uint32 연산으로 오버플로 방지
     *   cnt(최대 ~수백) × WHEEL_CIRC_MM(~204) × 1000 = 수천만 → uint32 안전
     */
    uint32_t spd_u = (uint32_t)cnt
                   * (uint32_t)WHEEL_CIRC_MM
                   * 1000UL
                   / ((uint32_t)ENCODER_PPR * 2UL);

    if (spd_u > 32767UL) spd_u = 32767UL;
    _spd = (cmd >= 0) ? (int16_t)spd_u : -(int16_t)spd_u;

#if DEBUG_ENCODER
    Serial.print(F("cnt: "));   Serial.print(cnt);
    Serial.print(F("  total: ")); Serial.print(encoder_get_count());
    Serial.print(F("  spd: "));   Serial.println(_spd);
#endif
}

int16_t encoder_get_spd(void) {
    return _spd;
}

uint32_t encoder_get_count(void) {
    noInterrupts();
    uint32_t c = _count_total;
    interrupts();
    return c;
}
