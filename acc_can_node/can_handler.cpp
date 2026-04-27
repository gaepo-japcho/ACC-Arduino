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
   CRC-8/AUTOSAR (E2E Profile 01)
   =========================================================
   DBC 정본 (../ACC-CANDB/acc_db.dbc) 의 MTR_CRC 시그널과 정합:
     poly  = 0x2F   (AUTOSAR CRC-8H2F, x^8+x^5+x^3+x^2+x+1)
     init  = 0xFF
     refin / refout = false (no reflection)
     XOR-out = 0x00 (no final XOR)
   입력: MTR_CMD payload Byte 0..4 (Byte 4 의 low nibble = MTR_RC)
   ECU(AUTOSAR CanComm SWC) 측도 동일 알고리즘 사용 합의.
   ========================================================= */
static uint8_t crc8_autosar(const uint8_t *buf, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x2F) : (uint8_t)(crc << 1);
        }
    }
    /* no XOR-out */
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
    uint8_t crc_calc = crc8_autosar(buf, 5);
    if (crc_calc != buf[5]) { _e2e_err = true; return; }

    /* Rolling Counter 연속성 검증 (4bit, Byte 4 low nibble) */
    uint8_t rc = (uint8_t)(buf[4] & 0x0F);
    if (_rc_initialized) {
        uint8_t expected = (uint8_t)((_rc_last + 1) & 0x0F);
        if (rc != expected) {
            /* 순서 위반 → E2E 오류 플래그. 값은 여�