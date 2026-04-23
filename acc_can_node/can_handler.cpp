#include <SPI.h>
#include "DFRobot_MCP2515.h"
#include "can_handler.h"

/* =========================================================
   내부 상태
   ========================================================= */
static DFRobot_MCP2515 _can(CAN_CS_PIN);
static bool _can_ok = false;

/* MTR_CMD E2E P01 수신 상태 */
static bool    _rc_initialized = false;
static uint8_t _rc_last  = 0;
static bool    _e2e_err  = false;

/* 최근 유효 MTR_CMD (E2E 통과 + 4→2 어댑터 적용) */
static int8_t  _cmd_l = 0;
static int8_t  _cmd_r = 0;
static bool    _has_new_cmd = false;

/* 최근 ECU_HEARTBEAT */
static uint8_t _ecu_hb  = 0;
static uint8_t _ecu_err = 0;
static bool    _has_new_hb = false;

/* MTR_HEARTBEAT 송신 카운터 */
static uint8_t _hb_mtr_counter = 0;

/* =========================================================
   CRC-8 (CCITT, poly 0x07, init 0x00)
   =========================================================
   AUTOSAR E2E Profile 01 의 CRC-8 은 엄밀히 SAE-J1850 poly 0x1D 를 쓰지만,
   본 스케일카 프로젝트에서는 교육적 단순화를 위해 CCITT poly 0x07 을 채택.
   검출 능력(단일 비트 오류 + 버스트 ≤ 8bit) 은 동일 수준이며, 송·수신 양측이
   같은 다항식만 쓰면 됨. ECU 측(AUTOSAR CanComm SWC) 도 동일 poly 사용 합의.
   ========================================================= */
static uint8_t crc8_ccitt(const uint8_t *buf, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* =========================================================
   CAN 초기화
   =========================================================
   DFRobot_MCP2515.begin() 은 성공 시 0, 실패 시 non-zero 반환.
   5회 재시도 후에도 실패하면 false 리턴.
   ========================================================= */
bool can_init(void) {
    uint8_t retry = 0;
    while (_can.begin(CAN_SPEED)) {
        if (++retry >= 5) {
            _can_ok = false;
            return false;
        }
        delay(200);
    }
    _can_ok = true;
    return true;
}

/* =========================================================
   MTR_CMD (0x210) 디스패치 — E2E P01 검증 + 4→2 어댑터
   ========================================================= */
static void dispatch_mtr_cmd(const uint8_t *buf, uint8_t len) {
    /* DLC 엄격 검사 — SAF010 은 DLC 6 을 명시 */
    if (len != 6) { _e2e_err = true; return; }

    /* CRC-8 검증 (Byte 0..4 → Byte 5) */
    uint8_t crc_calc = crc8_ccitt(buf, 5);
    if (crc_calc != buf[5]) { _e2e_err = true; return; }

    /* Rolling Counter 연속성 검증 (4bit, Byte 4 low nibble) */
    uint8_t rc = (uint8_t)(buf[4] & 0x0F);
    if (_rc_initialized) {
        uint8_t expected = (uint8_t)((_rc_last + 1) & 0x0F);
        if (rc != expected) {
            /* 순서 위반 → E2E 오류 플래그. 값은 여전히 반영(연속 실패 시
             * 30ms 타임아웃으로 safe stop 진입이 상위 안전 메커니즘). */
            _e2e_err = true;
        }
    } else {
        _rc_initialized = true;
    }
    _rc_last = rc;

    /* 4채널 PWM → L/R pair 어댑터 */
    int8_t lf = (int8_t)buf[0];
    int8_t rf = (int8_t)buf[1];
    int8_t lr = (int8_t)buf[2];
    int8_t rr = (int8_t)buf[3];
    _cmd_l = (int8_t)(((int16_t)lf + lr) / 2);
    _cmd_r = (int8_t)(((int16_t)rf + rr) / 2);
    _has_new_cmd = true;
}

/* =========================================================
   ECU_HEARTBEAT (0x410) 디스패치
   ========================================================= */
static void dispatch_ecu_hb(const uint8_t *buf, uint8_t len) {
    if (len < 2) return;
    _ecu_hb  = buf[0];
    _ecu_err = buf[1];
    _has_new_hb = true;
}

/* =========================================================
   RX 폴링 — ID 별 분기
   =========================================================
   한 loop() iteration 당 최대 4 프레임 처리 (burst 대응 + 레이턴시 제한).
   ========================================================= */
void can_poll(void) {
    for (uint8_t n = 0; n < 4; n++) {
        if (CAN_MSGAVAIL != _can.checkReceive()) {
            return;
        }

        uint8_t len = 0;
        uint8_t buf[8];
        if (_can.readMsgBuf(&len, buf) != CAN_OK) {
            return;
        }

        uint32_t rx_id = _can.getCanId();
        if (rx_id == CAN_ID_MTR_CMD) {
            dispatch_mtr_cmd(buf, len);
        } else if (rx_id == CAN_ID_ECU_HEARTBEAT) {
            dispatch_ecu_hb(buf, len);
        }
        /* 그 외 ID 는 무시 (이 노드가 구독하지 않음) */
    }
}

/* =========================================================
   수신 상태 조회 (edge-consume)
   ========================================================= */
bool can_get_cmd(int8_t *cmd_l, int8_t *cmd_r) {
    if (!_has_new_cmd) return false;
    *cmd_l = _cmd_l;
    *cmd_r = _cmd_r;
    _has_new_cmd = false;
    return true;
}

bool can_get_ecu_hb(uint8_t *hb, uint8_t *err) {
    if (!_has_new_hb) return false;
    *hb  = _ecu_hb;
    *err = _ecu_err;
    _has_new_hb = false;
    return true;
}

/* =========================================================
   MTR_SPD_FB 송신 (DLC 8)
   =========================================================
   GET_SPD_AVG (int16, factor 0.02 cm/s)  : Byte 0-1 (Intel LE)
   GET_SPD_LF/RF/LR/RR (int12, factor 0.3): Byte 2-7, bit-packed

   Scale 변환: wheel_raw × 0.3 = avg_raw × 0.02  →  wheel_raw = avg_raw / 15.

   bit layout (모든 wheel 이 같은 w12 값 — 엔코더 1개):
     Byte 2 = LF[7:0]                            = w_lo8
     Byte 3 = RF[3:0]<<4 | LF[11:8]              = ((w12&0x0F)<<4) | w_hi4
     Byte 4 = RF[11:4]                           = w_mid8
     Byte 5 = LR[7:0]                            = w_lo8
     Byte 6 = RR[3:0]<<4 | LR[11:8]              = ((w12&0x0F)<<4) | w_hi4
     Byte 7 = RR[11:4]                           = w_mid8
   ========================================================= */
void can_tx_feedback(int16_t spd_raw_02) {
    /* int12 raw 변환 + 클램프 */
    int16_t w = (int16_t)(spd_raw_02 / 15);
    if (w >  2047) w =  2047;
    if (w < -2048) w = -2048;
    uint16_t w12 = (uint16_t)w & 0x0FFF;   /* 12bit two's complement */

    uint8_t w_lo8  = (uint8_t)(w12 & 0xFF);
    uint8_t w_hi4  = (uint8_t)((w12 >> 8) & 0x0F);
    uint8_t w_mid8 = (uint8_t)((w12 >> 4) & 0xFF);
    uint8_t w_pack = (uint8_t)(((w12 & 0x0F) << 4) | w_hi4);

    uint8_t buf[8];
    /* AVG (int16 LE) */
    buf[0] = (uint8_t)(spd_raw_02 & 0xFF);
    buf[1] = (uint8_t)((spd_raw_02 >> 8) & 0xFF);
    /* 4륜 int12 × 4 */
    buf[2] = w_lo8;
    buf[3] = w_pack;
    buf[4] = w_mid8;
    buf[5] = w_lo8;
    buf[6] = w_pack;
    buf[7] = w_mid8;

    _can.sendMsgBuf(CAN_ID_MTR_SPD_FB, 0, 8, buf);
}

/* =========================================================
   MTR_HEARTBEAT 송신 (DLC 2)
   ========================================================= */
void can_tx_heartbeat(uint8_t err_flags) {
    uint8_t buf[2];
    buf[0] = _hb_mtr_counter++;    /* post-increment, 자동 wrap */
    buf[1] = err_flags;
    _can.sendMsgBuf(CAN_ID_MTR_HEARTBEAT, 0, 2, buf);
}

/* =========================================================
   상태 조회
   ========================================================= */
bool can_is_ok(void) {
    return _can_ok;
}

bool can_consume_e2e_err(void) {
    bool e = _e2e_err;
    _e2e_err = false;
    return e;
}
