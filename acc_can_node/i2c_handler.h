#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * i2c_handler.h — I2C 마스터 (CAN 노드 → 모터 노드)
 *
 * Arduino 우노 I2C 핀:
 *   SDA = A4
 *   SCL = A5
 *
 * 프로토콜:
 *   Master→Slave (쓰기, 3바이트):
 *     Byte 0: cmd_l (int8)
 *     Byte 1: cmd_r (int8)
 *     Byte 2: ctrl  (bit0=enable, bit2=brake)
 *
 *   Slave→Master (읽기, 3바이트 요청):
 *     Byte 0~1: spd (int16 little-endian, 0.1 cm/s)
 *     Byte 2:   err_flags
 */

void i2c_init(void);

/* 모터 노드로 명령 송신 — returns true=성공 */
bool i2c_send_cmd(int8_t cmd_l, int8_t cmd_r, uint8_t ctrl);

/* 모터 노드에서 피드백 요청 — returns true=성공 */
bool i2c_request_feedback(int16_t *spd, uint8_t *err);
