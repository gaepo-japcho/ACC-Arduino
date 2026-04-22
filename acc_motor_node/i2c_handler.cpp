#include <Wire.h>
#include "i2c_handler.h"

/* =========================================================
   I2C 콜백과 메인 루프가 공유하는 상태 (volatile 필수)
   ========================================================= */
static volatile int8_t  _rx_cmd_l = 0;
static volatile int8_t  _rx_cmd_r = 0;
static volatile uint8_t _rx_ctrl  = 0;
static volatile bool    _rx_ready = false;

static volatile int16_t _fb_spd = 0;
static volatile uint8_t _fb_err = 0;

/* =========================================================
   I2C 수신 콜백 — 마스터가 write()한 바이트가 들어옴
   ISR 문맥에서 호출되므로 delay/Serial 금지
   ========================================================= */
static void onI2CReceive(int n) {
    if (n >= 3) {
        _rx_cmd_l = (int8_t)Wire.read();
        _rx_cmd_r = (int8_t)Wire.read();
        _rx_ctrl  = (uint8_t)Wire.read();
        _rx_ready = true;
    }
    /* 남은 바이트가 있으면 비워냄 */
    while (Wire.available()) Wire.read();
}

/* =========================================================
   I2C 요청 콜백 — 마스터가 requestFrom()으로 읽기 요청
   항상 3바이트 송신
   ========================================================= */
static void onI2CRequest(void) {
    /* volatile 변수를 안전하게 스냅샷 */
    int16_t spd = _fb_spd;
    uint8_t err = _fb_err;

    uint8_t buf[3];
    buf[0] = (uint8_t)(spd & 0xFF);         /* LSB */
    buf[1] = (uint8_t)((spd >> 8) & 0xFF);  /* MSB */
    buf[2] = err;

    Wire.write(buf, 3);
}

/* =========================================================
   공개 함수
   ========================================================= */
void i2c_init(void) {
    Wire.begin(MY_I2C_ADDR);        /* 슬레이브 모드 진입 */
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
}

bool i2c_get_cmd(int8_t *cmd_l, int8_t *cmd_r, uint8_t *ctrl) {
    if (!_rx_ready) return false;

    noInterrupts();
    *cmd_l = _rx_cmd_l;
    *cmd_r = _rx_cmd_r;
    *ctrl  = _rx_ctrl;
    _rx_ready = false;
    interrupts();

    return true;
}

void i2c_set_feedback(int16_t spd, uint8_t err) {
    noInterrupts();
    _fb_spd = spd;
    _fb_err = err;
    interrupts();
}
