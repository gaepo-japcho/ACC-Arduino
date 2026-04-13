/*
 * acc_motor_rear.ino — 메인
 */

#include <Arduino.h>
#include "config.h"
#include "motor_driver.h"
#include "can_handler.h"
#include "encoder.h"

/* ── 전역 상태 ──────────────────────────────────────── */
static int8_t   g_cmd_l      = 0;
static int8_t   g_cmd_r      = 0;
static uint32_t g_t_last_cmd = 0;
static uint8_t  g_err_flags  = 0;

static uint32_t g_t_ctrl      = 0;
static uint32_t g_t_feedback  = 0;
static uint32_t g_t_heartbeat = 0;
static uint32_t g_t_enc       = 0;

/* =========================================================
   setup
   ========================================================= */
void setup() {
    Serial.begin(115200);

#ifdef BOARD_FRONT
    Serial.println(F("[ACC] Front Motor Node"));
#else
    Serial.println(F("[ACC] Rear Motor Node"));
#endif

    motor_init();
    encoder_init();         /* 엔코더 핀 + ISR 등록 */

    if (!can_init()) {
        Serial.println(F("[CAN] FAILED"));
        g_err_flags |= ERR_CAN;
    } else {
        Serial.println(F("[CAN] OK"));
    }

    uint32_t now = millis();
    g_t_last_cmd  = now;
    g_t_ctrl      = now;
    g_t_feedback  = now;
    g_t_heartbeat = now;
    g_t_enc       = now;

    Serial.println(F("[ACC] READY"));
}

/* =========================================================
   loop 
   ========================================================= */
void loop() {
    uint32_t now = millis();

    /* ① 새 CAN 명령 적용 */
    {
        int8_t cl, cr;
        if (can_get_cmd(&cl, &cr)) {
            g_cmd_l      = cl;
            g_cmd_r      = cr;
            g_t_last_cmd = now;
            g_err_flags &= ~ERR_TIMEOUT;
        }
    }

    /* ② 타임아웃 감시 (30ms — ASIL B) */
    if ((now - g_t_last_cmd) > TIMEOUT_CMD_MS) {
        if (g_cmd_l != 0 || g_cmd_r != 0) {
            g_cmd_l = g_cmd_r = 0;
            motor_stop_all();
        }
        g_err_flags |= ERR_TIMEOUT;
    }

    /* ③ 엔코더 속도 계산 (10ms) */
    if ((now - g_t_enc) >= PERIOD_ENC_CALC_MS) {
        g_t_enc = now;
        encoder_calc(g_cmd_l, g_cmd_r);
    }

    /* ④ 모터 제어 (10ms) */
    if ((now - g_t_ctrl) >= PERIOD_CTRL_MS) {
        g_t_ctrl = now;
        motor_set_both(g_cmd_l, g_cmd_r);
    }

    /* ⑤ 속도 피드백 송신 (10ms)
     * encoder_get_spd_l/r()는 실측 속도 반환
     * 엔코더 미연결 시 0 반환 — ECU에서 타임아웃 처리 필요
     */
    if ((now - g_t_feedback) >= PERIOD_FEEDBACK_MS) {
        g_t_feedback = now;
        if (can_is_ok()) {
            can_tx_feedback(encoder_get_spd_l(), encoder_get_spd_r());
        }
    }

    /* ⑥ Heartbeat (50ms) */
    if ((now - g_t_heartbeat) >= PERIOD_HEARTBEAT_MS) {
        g_t_heartbeat = now;
        if (can_is_ok()) {
            can_tx_heartbeat(g_err_flags);
        }
    }
}
