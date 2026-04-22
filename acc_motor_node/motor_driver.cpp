#include "motor_driver.h"

/* =========================================================
   내부 함수
   =========================================================
   아두이노 모터 쉴드 R3 제어 규칙:
     Brake HIGH  = 제동 (모터 강제 정지)
     Brake LOW   = 주행 가능
     DIR  HIGH   = 정방향
     DIR  LOW    = 역방향
     PWM         = 속도 (0~255)

   PWM 스케일링:
     cmd 절대값 0~127  →  PWM 0~254
     (cmd << 1) = cmd × 2, 비트시프트가 곱셈보다 빠름
   ========================================================= */

static void motor_set_pair(uint8_t dir_pin, uint8_t pwm_pin,
                            uint8_t brk_pin, int8_t cmd) {
    if (cmd == 0) {
        /* 정지: Brake ON, PWM 0 */
        analogWrite(pwm_pin, 0);
        digitalWrite(brk_pin, HIGH);

    } else if (cmd > 0) {
        /* 전진 */
        digitalWrite(brk_pin, LOW);
        digitalWrite(dir_pin, HIGH);
        analogWrite(pwm_pin, (uint8_t)cmd << 1);

    } else {
        /* 후진 */
        digitalWrite(brk_pin, LOW);
        digitalWrite(dir_pin, LOW);
        analogWrite(pwm_pin, (uint8_t)(-cmd) << 1);
    }
}

/* =========================================================
   공개 함수
   ========================================================= */

void motor_init(void) {
    pinMode(PIN_DIR_A, OUTPUT);
    pinMode(PIN_PWM_A, OUTPUT);
    pinMode(PIN_BRK_A, OUTPUT);
    pinMode(PIN_DIR_B, OUTPUT);
    pinMode(PIN_PWM_B, OUTPUT);
    pinMode(PIN_BRK_B, OUTPUT);

    /* 기동 시 안전을 위해 모든 채널 정지 상태로 초기화 */
    analogWrite(PIN_PWM_A, 0);
    analogWrite(PIN_PWM_B, 0);
    digitalWrite(PIN_BRK_A, HIGH);
    digitalWrite(PIN_BRK_B, HIGH);
    digitalWrite(PIN_DIR_A, LOW);
    digitalWrite(PIN_DIR_B, LOW);
}

void motor_set_both(int8_t cmd_l, int8_t cmd_r, uint8_t ctrl) {
    bool enable = (ctrl & 0x01) != 0;
    bool brake  = (ctrl & 0x04) != 0;

    /* enable=0 또는 brake=1 → 강제 정지 */
    if (!enable || brake) {
        motor_stop_all();
        return;
    }

    motor_set_pair(PIN_DIR_A, PIN_PWM_A, PIN_BRK_A, cmd_l);   /* 왼쪽 쌍 */
    motor_set_pair(PIN_DIR_B, PIN_PWM_B, PIN_BRK_B, cmd_r);   /* 오른쪽 쌍 */
}

void motor_stop_all(void) {
    analogWrite(PIN_PWM_A, 0);
    analogWrite(PIN_PWM_B, 0);
    digitalWrite(PIN_BRK_A, HIGH);
    digitalWrite(PIN_BRK_B, HIGH);
}
