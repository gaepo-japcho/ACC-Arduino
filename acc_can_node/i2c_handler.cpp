#include <Wire.h>
#include "i2c_handler.h"

void i2c_init(void) {
    Wire.begin();                  // I2C 마스터 시작
    Wire.setClock(I2C_CLOCK);      // 100kHz (config.h)
}

bool i2c_send_cmd(int8_t cmd_l, int8_t cmd_r, uint8_t ctrl) {
    Wire.beginTransmission(MOTOR_SLAVE_ADDR);
    Wire.write((uint8_t)cmd_l);
    Wire.write((uint8_t)cmd_r);
    Wire.write(ctrl);

    /*
     * endTransmission 리턴값:
     *   0: 성공
     *   1: 데이터 너무 김
     *   2: 주소 NACK (슬레이브 응답 없음)
     *   3: 데이터 NACK
     *   4: 기타 에러
     */
    uint8_t err = Wire.endTransmission();
    return (err == 0);
}

bool i2c_request_feedback(int16_t *spd, uint8_t *err) {
    /* 슬레이브에게 3바이트 요청 */
    uint8_t n = Wire.requestFrom((uint8_t)MOTOR_SLAVE_ADDR, (uint8_t)3);
    if (n < 3) return false;

    uint8_t b0 = Wire.read();      // spd LSB
    uint8_t b1 = Wire.read();      // spd MSB
    uint8_t b2 = Wire.read();      // err_flags

    *spd = (int16_t)((uint16_t)b0 | ((uint16_t)b1 << 8));
    *err = b2;
    return true;
}
