/*
 * acc_can_node.ino — Arduino #1 (CAN 게이트웨이)
 *
 * 역할: ECU ↔ CAN ↔ [이 보드] ↔ I2C ↔ 모터 노드(Arduino #2)
 *
 * 하드웨어:
 *   아두이노 우노 + DFRobot CAN BUS Shield (스택)
 *   A4/A5 핀을 점퍼선으로 모터 노드의 A4/A5와 연결
 *   양 보드의 GND 공통 연결 필수
 *
 * 흐름:
 *   1) CAN으로 ECU 명령(0x300) 수신(폴링) → I2C로 모터 노드에 전달
 *   2) 모터 노드의 엔코더 속도 / 에러 플래그 주기적 요청
 *   3) CAN으로 피드백(0x400) + 하트비트(0x410) ECU에 송신
 *   4) ECU 명령이 30ms 이상 오지 않으면 모터 정지 명령 강제 전송
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
static uint8_t  g_ctrl  = 0;
static uint32_t g_t_last_cmd  = 0;
static uint8_t  g_err_flags   = 0;

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
     *   모터 노드가 리셋 중이거나 통신 케이블이 빠졌을 때 안전장치
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
    g_t_last_cmd  = now;
    g_t_feedback  = now;
    g_t_heartbeat = now;

    Serial.println(F("[ACC] READY"));
}

/* =========================================================
   loop
   ========================================================= */
void loop() {
    uint32_t now = millis();

    /* ─────────────────────────────────────────────
       ① CAN 수신 폴링 → I2C 전달
       ───────────────────────────────────────────── */
    {
        int8_t  cl, cr;
        uint8_t ctrl;
        if (can_get_cmd(&cl, &cr, &ctrl)) {
            g_cmd_l = cl;
            g_cmd_r = cr;
            g_ctrl  = ctrl;
            g_t_last_cmd = now;
            g_err_flags &= ~ERR_TIMEOUT;

            if (i2c_send_cmd(cl, cr, ctrl)) {
                g_err_flags &= ~ERR_I2C;
            } else {
                g_err_flags |= ERR_I2C;
            }
        }
    }

    /* ② 체크섬 오류 플래그 소비 (can_handler 내부에서 세워진 것) */
    if (can_consume_checksum_err()) {
        g_err_flags |= ERR_CHECKSUM;
        /*
         * 체크섬 오류는 개별 프레임에 대한 것이므로
         * 다음 정상 프레임이 오면 자동으로 클리어되지 않음.
         * 50ms마다 하트비트 송신 후 클리어하는 패턴이 무난.
         */
    }

    /* ─────────────────────────────────────────────
       ③ 명령 타임아웃 감시 (30ms)
         - ECU에서 명령이 끊기면 모터 즉시 정지
       ───────────────────────────────────────────── */
    if ((now - g_t_last_cmd) > TIMEOUT_CMD_MS) {
        if (g_cmd_l != 0 || g_cmd_r != 0 || (g_ctrl & 0x01)) {
            g_cmd_l = 0;
            g_cmd_r = 0;
            g_ctrl  = 0;
            i2c_send_cmd(0, 0, 0);      /* 강제 정지 명령 */
        }
        g_err_flags |= ERR_TIMEOUT;
    }

    /* ─────────────────────────────────────────────
       ④ 모터 노드 피드백 요청 → CAN 송신 (10ms)
         - I2C 실패해도 0x400 프레임은 계속 송신 (spd=0)
         - ECU 관점에서 "CAN 노드 살아있음" 확인 가능
         - 실제 속도 신뢰도는 하트비트의 ERR_I2C 비트로 판단
       ───────────────────────────────────────────── */
    if ((now - g_t_feedback) >= PERIOD_FEEDBACK_MS) {
        g_t_feedback = now;

        int16_t spd = 0;
        uint8_t motor_err = 0;
        if (i2c_request_feedback(&spd, &motor_err)) {
            g_err_flags &= ~ERR_I2C;
        } else {
            g_err_flags |= ERR_I2C;
            spd = 0;                 /* 안전 기본값 */
        }

        /* I2C 성공/실패 무관하게 피드백 프레임은 항상 송신 */
        if (can_is_ok()) {
            can_tx_feedback(spd);
        }
    }

    /* ─────────────────────────────────────────────
       ⑤ 하트비트 송신 (50ms)
         - 전송 후 체크섬 오류 플래그는 클리어해서 순간 오류만 보고
       ───────────────────────────────────────────── */
    if ((now - g_t_heartbeat) >= PERIOD_HEARTBEAT_MS) {
        g_t_heartbeat = now;
        if (can_is_ok()) {
            can_tx_heartbeat(g_err_flags);
        }
        g_err_flags &= ~ERR_CHECKSUM;   /* 한 번 보고한 뒤 클리어 */
    }
}
