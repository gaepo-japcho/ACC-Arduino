#pragma once

#include <Arduino.h>
#include "config.h"

/*
 * i2c_handler.h — I2C 슬레이브 (CAN 노드 ← 이 보드)
 *
 * Arduino 우노 I2C 핀:
 *   SDA = A4
 *   SCL = A5
 *   (모터 쉴드 R3는 A4/A5를 사용하지 않으므로 자유롭게 사용 가능)
 *
 * 프로토콜:
 *   Master(CAN 노드) → Slave(이 보드) 쓰기: 3바이트
 *     Byte 0: cmd_l (int8)
 *     Byte 1: cmd_r (int8)
 *     Byte 2: ctrl  (bit0=enable, bit2=brake)
 *
 *   Master ← Slave 읽기: 3바이트
 *     Byte 0~1: spd (int16 little-endian, 0.1 cm/s)
 *     Byte 2:   err_flags
 */

/* I2C 슬레이브 초기화 + 콜백 등록 */
void i2c_init(void);

/*
 * 새 명령을 꺼냄 — loop()에서 매 주기 호출
 *   returns: true=새 명령, false=없음
 */
bool i2c_get_cmd(int8_t *cmd_l, int8_t *cmd_r, uint8_t *ctrl);

/*
 * 피드백 데이터 갱신 (onRequest 콜백이 사용할 값)
 * loop()에서 주기적으로 호출해서 최신 속도/에러를 반영
 */
void i2c_set_feedback(int16_t spd, uint8_t err);
