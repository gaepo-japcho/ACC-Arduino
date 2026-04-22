#include <SPI.h>
#include "DFRobot_MCP2515.h"
#include "can_handler.h"

/* =========================================================
   내부 상태
   ========================================================= */
static DFRobot_MCP2515 _can(CAN_CS_PIN);
static bool _can_ok = false;
static bool _checksum_err = false;

/* =========================================================
   CAN 초기화
   =========================================================
   DFRobot_MCP2515.begin() 은 성공 시 0, 실패 시 non-zero 반환
   (원본 DFRobot 예제 그대로의 사용 패턴)
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
   수신 폴링 — loop에서 호출
   =========================================================
   처리 순서:
     1) MCP2515에 새 메시지가 있는지 checkReceive
     2) 있으면 readMsgBuf + getCanId
     3) ID 필터링 (CAN_ID_MOTOR_CMD만)
     4) DLC, 체크섬 검증
     5) ctrl 해석 (enable/brake)
     6) 유효하면 true, 아니면 false
   ========================================================= */
bool can_get_cmd(int8_t *cmd_l, int8_t *cmd_r, uint8_t *ctrl) {
    if (CAN_MSGAVAIL != _can.checkReceive()) {
        return false;
    }

    uint8_t len = 0;
    uint8_t buf[8];
    if (_can.readMsgBuf(&len, buf) != CAN_OK) {
        return false;
    }

    uint32_t rx_id = _can.getCanId();
    if (rx_id != CAN_ID_MOTOR_CMD || len < 5) {
        return false;
    }

    /* 체크섬 검증 (DLC >= 6일 때만) */
    if (len >= 6) {
        uint8_t chk = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4];
        if (chk != buf[5]) {
            _checksum_err = true;      /* 플래그 세움, loop에서 소비 */
            return false;
        }
    }

    uint8_t ctrl_byte = buf[4];
    bool motor_en = (ctrl_byte & 0x01) != 0;
    bool brake    = (ctrl_byte & 0x04) != 0;

    if (!motor_en || brake) {
        *cmd_l = 0;
        *cmd_r = 0;
    } else {
        *cmd_l = (int8_t)buf[0];
        *cmd_r = (int8_t)buf[1];
    }
    *ctrl = ctrl_byte;

    return true;
}

/* =========================================================
   송신
   ========================================================= */
void can_tx_feedback(int16_t spd) {
    /* 빅엔디안 패킹 (네트워크 표준) */
    uint8_t buf[4];
    buf[0] = (uint8_t)(spd >> 8);
    buf[1] = (uint8_t)(spd & 0xFF);
    buf[2] = 0;
    buf[3] = 0;

    _can.sendMsgBuf(CAN_ID_MOTOR_FB, 0, 4, buf);
}

void can_tx_heartbeat(uint8_t err_flags) {
    uint8_t buf[2] = { 0xAA, err_flags };
    _can.sendMsgBuf(CAN_ID_MOTOR_HB, 0, 2, buf);
}

bool can_is_ok(void) {
    return _can_ok;
}

bool can_consume_checksum_err(void) {
    bool e = _checksum_err;
    _checksum_err = false;
    return e;
}
