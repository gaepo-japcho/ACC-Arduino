/*
 * acc_motor_node.ino — Arduino #2 (모터 컨트롤러)
 *
 * 역할: CAN 노드 → I2C ↔ [이 보드] → 모터 4개 + 엔코더
 *
 * 하드웨어:
 *   아두이노 우노
 *   + 아두이노 모터 쉴드 R3 (아래)
 *   + 아두이노 모터 쉴드 R3 (위, 스택)
 *   + DC 엔코더 모터 4개
 *   + 엔코더 1개 (D2 인터럽트)
 *
 * 동작:
 *   1) I2C(A4/A5)로 CAN 노드에서 명령 수신
 *   2) 명령(cmd_l, cmd_r, ctrl) → 모터 쉴드 A/B 채널 PWM+DIR 출력
 *   3) 엔코더 펄스 주기적 속도 계산
 *   4) I2C 요청 시 속도/에러 플래그 송신
 *   5) 200ms 명령 없으면 안전 정지
 */

#include <Arduino.h>
#include "config.h"
#include "motor_driver.h"
#include "encoder.h"
#include "i2c_handler.h"

/* =========================================================
   전역 상태
   ========================================================= */
static int8_t   g_cmd_l = 0;
static int8_t   g_cmd_r = 0;
static uint8_t  g_ctrl  = 0;
static uint32_t g_t_last_cmd = 0;
static uint8_t  g_err_flags  = 0;

static uint32_t g_t_ctrl = 0;
static uint32_t g_t_enc  = 0;

/* =========================================================
   setup
   ========================================================= */
void setup() {
    Serial.begin(115200);
    Serial.println(F("[ACC] Motor Node"));

    motor_init();
    Serial.println(F("[MOTOR] ready"));

    encoder_init();
    Serial.println(F("[ENC] ready"));

    i2c_init();
    Serial.print(F("[I2C] slave @ 0x"));
    Serial.println(MY_I2C_ADDR, HEX);

    uint32_t now = millis();
    g_t_last_cmd = now;
    g_t_ctrl     = now;
    g_t_enc      = now;

    Serial.println(F("[ACC] READY"));
}

/* =========================================================
   loop
   ========================================================= */
void loop() {
    uint32_t now = millis();

    /* ─────────────────────────────────────────────
       ① 새 I2C 명령 확인
       ───────────────────────────────────────────── */
    {
        int8_t  cl, cr;
        uint8_t ctrl;
        if (i2c_get_cmd(&cl, &cr, &ctrl)) {
            g_cmd_l = cl;
            g_cmd_r = cr;
            g_ctrl  = ctrl;
            g_t_last_cmd = now;
            g_err_flags &= ~ERR_TIMEOUT;
        }
    }

    /* ─────────────────────────────────────────────
       ② 명령 타임아웃 감시 (200ms)
         - CAN 게이트웨이는 10ms 주기로 명령 전송
         - 20주기 이상 없으면 통신 이상 → 안전 정지
       ───────────────────────────────────────────── */
    if ((now - g_t_last_cmd) > TIMEOUT_CMD_MS) {
        if (g_cmd_l != 0 || g_cmd_r != 0 || (g_ctrl & 0x01)) {
            g_cmd_l = 0;
            g_cmd_r = 0;
            g_ctrl  = 0;
            motor_stop_all();
        }
        g_err_flags |= ERR_TIMEOUT;
    }

    /* ─────────────────────────────────────────────
       ③ 엔코더 속도 계산 (10ms)
         - 방향 부호는 왼쪽 쌍 명령(cmd_l)을 기준으로 결정
           (직진만 하므로 좌우 동일 부호)
       ───────────────────────────────────────────── */
    if ((now - g_t_enc) >= PERIOD_ENC_CALC_MS) {
        g_t_enc = now;
        encoder_calc(g_cmd_l);
    }

    /* ─────────────────────────────────────────────
       ④ 모터 출력 갱신 (10ms)
       ───────────────────────────────────────────── */
    if ((now - g_t_ctrl) >= PERIOD_CTRL_MS) {
        g_t_ctrl = now;
        motor_set_both(g_cmd_l, g_cmd_r, g_ctrl);
    }

    /* ─────────────────────────────────────────────
       ⑤ I2C 피드백 값 갱신 (매 loop, 매우 빠름)
         onRequest 콜백이 이 값을 읽어서 마스터에 송신
       ───────────────────────────────────────────── */
    i2c_set_feedback(encoder_get_spd(), g_err_flags);
}
