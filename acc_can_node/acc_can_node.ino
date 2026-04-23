/*
 * acc_can_node.ino — Arduino #1 (CAN 게이트웨이)
 *
 * 역할: ECU ↔ CAN ↔ [이 보드] ↔ I2C ↔ 모터 노드(Arduino #2)
 *
 * 하드웨어:
 *   Arduino Uno + DFRobot CAN BUS Shield (스택)
 *   A4/A5 핀을 점퍼선으로 모터 노드의 A4/A5와 연결
 *   양 보드의 GND 공통 연결 필수
 *
 * 흐름 (DBC: ACC-CANDB/acc_db.dbc):
 *   1) CAN 폴링 (MTR_CMD 0x210 + ECU_HEARTBEAT 0x410)
 *   2) MTR_CMD 수신 시 E2E P01 (Rolling Counter + CRC-8) 검증 후
 *      4채널 → L/R pair 어댑터 → I2C 로 모터 노드에 전달
 *   3) 모터 노드의 엔코더 속도 / 에러 플래그 주기적 요청 (I2C)
 *   4) MTR_SPD_FB (0x300) + MTR_HEARTBEAT (0x310) CAN 송신
 *   5) MTR_CMD 30ms 미수신 → 모터 정지 강제 (SAF010)
 *   6) ECU_HEARTBEAT 30ms 미수신 → ERR_ECU_HB 플래그 (SAF018)
 */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "can_handler.h"
#include "i2c_handler.h"

/* =========================================================
   전역 상태
   ========================================================= */
static int8_t   g_cmd_l = 0;
static int8_t   g_cmd_r = 0;
static uint32_t g_t_last_cmd    = 0;
static uint32_t g_t_last_ecu_hb = 0;
static uint8_t  g_err_flags     = 0;

static uint32_t g_t_feedback  = 0;
static uint32_t g_t_heartbeat = 0;

/* =========================================================
   setup
   ========================================================= */
void setup() {
    Serial.begin(115200);
    Serial.println(F("[ACC] CAN Node"));

    /* I2C 마스터 초기화 */
    i2c_init();
    /*
     * I2C 블로킹 방지 타임아웃 (AVR Wire 라이브러리 기능)
     *   25ms 안에 응답 없으면 TWI 모듈을 리셋하고 에러 반환
     */
    Wire.setWireTimeout(25000, true);
    Serial.println(F("[I2C] master ready (timeout=25ms)"));

    /* CAN 초기화 */
    if (!can_init()) {
        Serial.println(F("[CAN] FAILED"));
        g_err_flags |= ERR_CAN;
    } else {
        Serial.println(F("[CAN] OK"));
    }

    uint32_t now = millis();
    g_t_last_cmd    = now;
    g_t_last_ecu_hb = now;
    g_t_feedback    = now;
    g_t_heartbeat   = now;

    Serial.println(F("[ACC] READY"));
}

/* =========================================================
   loop
   ========================================================= */
void loop() {
    uint32_t now = millis();

    /* ─────────────────────────────────────────────
       ① CAN 수신 폴링 (MTR_CMD + ECU_HEARTBEAT 드레인)
       ───────────────────────────────────────────── */
    can_poll();

    /* ② MTR_CMD 수신 처리 → I2C 전달 */
    {
        int8_t cl, cr;
        if (can_get_cmd(&cl, &cr)) {
            g_cmd_l = cl;
            g_cmd_r = cr;
            g_t_last_cmd = now;
            g_err_flags &= ~ERR_TIMEOUT;

            /* ctrl 은 더 이상 CAN 에서 오지 않음 — enable=1 고정.
             * 정지는 ECU AccStateMachine 이 PWM=0 으로 표현한다. */
            if (i2c_send_cmd(cl, cr, 0x01)) {
                g_err_flags &= ~ERR_I2C;
            } else {
                g_err_flags |= ERR_I2C;
            }
        }
    }

    /* ③ ECU_HEARTBEAT 수신 처리 (SAF018 ASIL-B) */
    {
        uint8_t hb, err;
        if (can_get_ecu_hb(&hb, &err)) {
            g_t_last_ecu_hb = now;
            g_err_flags &= ~ERR_ECU_HB;
            /* ECU 측 내부 에러는 ERR_ECU 로 별도 보고됨. 게이트웨이는 통과. */
            (void)hb;
            (void)err;
        }
    }

    /* ④ E2E 오류 플래그 소비 (can_handler 내부에서 세워진 것) */
    if (can_consume_e2e_err()) {
        g_err_flags |= ERR_E2E;
        /* 개별 프레임 E2E 실패 — 하트비트 송신 후 클리어하는 패턴 */
    }

    /* ─────────────────────────────────────────────
       ⑤ MTR_CMD 타임아웃 감시 (30ms, SAF010)
         - ECU 명령이 끊기면 모터 즉시 정지
       ───────────────────────────────────────────── */
    if ((now - g_t_last_cmd) > TIMEOUT_CMD_MS) {
        if (g_cmd_l != 0 || g_cmd_r != 0) {
            g_cmd_l = 0;
            g_cmd_r = 0;
            i2c_send_cmd(0, 0, 0);      /* enable=0, 강제 정지 */
        }
        g_err_flags |= ERR_TIMEOUT;
    }

    /* ⑥ ECU_HEARTBEAT 타임아웃 감시 (30ms, SAF018 ASIL-B) */
    if ((now - g_t_last_ecu_hb) > TIMEOUT_ECU_HB_MS) {
        g_err_flags |= ERR_ECU_HB;
        /* 보조 safety: ECU HB 도 끊기면 모터 정지 확실히 강제.
         * (MTR_CMD 도 함께 끊길 가능성이 높지만 중복 안전.) */
        if (g_cmd_l != 0 || g_cmd_r != 0) {
            g_cmd_l = 0;
            g_cmd_r = 0;
            i2c_send_cmd(0, 0, 0);
        }
    }

    /* ─────────────────────────────────────────────
       ⑦ 모터 노드 피드백 요청 → MTR_SPD_FB 송신 (10ms)
         - I2C 실패해도 프레임은 계속 송신 (spd=0, ERR_I2C 플래그)
       ───────────────────────────────────────────── */
    if ((now - g_t_feedback) >= PERIOD_FEEDBACK_MS) {
        g_t_feedback = now;

        int16_t spd = 0;
        uint8_t motor_err = 0;
        if (i2c_request_feedback(&spd, &motor_err)) {
            g_err_flags &= ~ERR_I2C;
        } else {
            g_err_flags |= ERR_I2C;
            spd = 0;
        }

        if (can_is_ok()) {
            /* 단위 변환: i2c_handler 프로토콜은 0.1 cm/s 단위.
             * DBC GET_SPD_AVG 는 0.02 cm/s 단위 → ×5 배율.
             * 물리 최고속 541 cm/s → i2c raw 5410 → DBC raw 27050 (int16 범위 내). */
            int32_t spd32 = (int32_t)spd * 5;
            if (spd32 >  32767) spd32 =  32767;
            if (spd32 < -32768) spd32 = -32768;
            can_tx_feedback((int16_t)spd32);
        }
    }

    /* ─────────────────────────────────────────────
       ⑧ MTR_HEARTBEAT 송신 (10ms)
         - 송신 후 E2E 오류 플래그 클리어 — 순간 오류만 보고
       ───────────────────────────────────────────── */
    if ((now - g_t_heartbeat) >= PERIOD_HEARTBEAT_MS) {
        g_t_heartbeat = now;
        if (can_is_ok()) {
            can_tx_heartbeat(g_err_flags);
        }
        g_err_flags &= ~ERR_E2E;
    }
}
