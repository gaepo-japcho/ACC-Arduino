#include "motor_driver.h"

/* =========================================================
   내부 함수 (이 파일에서만 사용 — 외부에 노출 안 함)
   ========================================================= */

/*
 * 단일 모터 제어
 *
 * en_pin : PWM 핀 (속도)
 * in1_pin: 방향 핀 1
 * in2_pin: 방향 핀 2
 * cmd    : -127~127
 *
 * PWM 스케일링:
 *   cmd 절대값 0~127 → PWM 0~254
 *   cmd << 1 = cmd × 2 (비트 시프트가 곱셈보다 빠름)
 *   127 × 2 = 254 (최대 255에 근접, 충분)
 */
static void motor_set_one(uint8_t en_pin, uint8_t in1_pin,
                           uint8_t in2_pin, int8_t cmd) {
    if (cmd == 0) {
        /* 정지: 속도=0, 방향 핀 모두 LOW (브레이크 모드) */
        analogWrite(en_pin, 0);
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);

    } else if (cmd > 0) {
        /* 전진 */
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        analogWrite(en_pin, (uint8_t)cmd << 1);     /* 0~127 → 0~254 */

    } else {
        /* 후진: cmd가 음수이므로 -cmd로 양수 변환 후 스케일링 */
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        analogWrite(en_pin, (uint8_t)(-cmd) << 1);  /* 0~127 → 0~254 */
    }
}

/* =========================================================
   공개 함수 구현
   ========================================================= */

void motor_init(void) {
    /* EN 핀 — PWM 출력, 초기값 0(정지) */
    pinMode(PIN_L_EN, OUTPUT);
    pinMode(PIN_R_EN, OUTPUT);
    analogWrite(PIN_L_EN, 0);
    analogWrite(PIN_R_EN, 0);

    /* 방향 핀 — 디지털 출력, 초기값 LOW */
    uint8_t dir_pins[] = {
        PIN_L_IN1, PIN_L_IN2,
        PIN_R_IN1, PIN_R_IN2
    };
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(dir_pins[i], OUTPUT);
        digitalWrite(dir_pins[i], LOW);
    }
}

void motor_set_both(int8_t cmd_l, int8_t cmd_r) {
    motor_set_one(PIN_L_EN, PIN_L_IN1, PIN_L_IN2, cmd_l);
    motor_set_one(PIN_R_EN, PIN_R_IN1, PIN_R_IN2, cmd_r);
}

void motor_stop_all(void) {
    motor_set_one(PIN_L_EN, PIN_L_IN1, PIN_L_IN2, 0);
    motor_set_one(PIN_R_EN, PIN_R_IN1, PIN_R_IN2, 0);
}
