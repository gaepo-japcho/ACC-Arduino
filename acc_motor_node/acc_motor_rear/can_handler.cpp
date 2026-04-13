#include <SPI.h>
#include <mcp_can.h>
#include "can_handler.h"

/* =========================================================
   내부 상태 변수 (이 파일 안에서만 접근)
   ========================================================= */
static MCP_CAN _can(CAN_CS_PIN);
static bool    _can_ok = false;

/*
 * ISR과 메인 코드가 공유하는 수신 버퍼
 *
 * volatile: ISR이 비동기로 값을 바꾸므로 컴파일러 최적화 방지
 * _rx_ready: 새 명령이 버퍼에 들어왔는지 표시하는 플래그
 */
static volatile int8_t _rx_cmd_l   = 0;
static volatile int8_t _rx_cmd_r   = 0;
static volatile bool   _rx_ready   = false;     /* 새 명령 도착 플래그 */
static volatile bool   _rx_stop    = false;     /* 즉시 정지 명령 플래그 */

/* =========================================================
   내부 함수
   ========================================================= */

/*
 * MCP2515 필터 설정
 *
 * 이 보드에 해당하는 CAN ID(config.h의 CAN_ID_MOTOR_CMD)만 통과시킵니다.
 * 버스에 흐르는 다른 메시지는 하드웨어 수준에서 자동으로 걸러져요.
 *
 * MCP2515 수신 버퍼 구조:
 *   RXB0 — 필터 0, 1 담당 (우선순위 높음)
 *   RXB1 — 필터 2, 3, 4, 5 담당 (우선순위 낮음)
 *
 * 마스크 0x7FF: 11비트 전부 비교 (표준 CAN 프레임 ID)
 */
static void _apply_filters(void) {
    _can.init_Mask(0, 0, 0x7FF);
    _can.init_Filt(0, 0, CAN_ID_MOTOR_CMD);
    _can.init_Filt(1, 0, CAN_ID_MOTOR_CMD);

    _can.init_Mask(1, 0, 0x7FF);
    _can.init_Filt(2, 0, CAN_ID_MOTOR_CMD);
    _can.init_Filt(3, 0, CAN_ID_MOTOR_CMD);
    _can.init_Filt(4, 0, CAN_ID_MOTOR_CMD);
    _can.init_Filt(5, 0, CAN_ID_MOTOR_CMD);
}

/* =========================================================
   CAN 인터럽트 서비스 루틴 (ISR)
   =========================================================
   언제 실행: MCP2515 INT 핀이 LOW가 될 때 (새 메시지 수신)
   주의: ISR 안에서는 delay(), Serial.print(), millis() 금지
         가능한 한 빠르게 끝내야 함 (수 마이크로초 목표)

   동작 순서:
   1. MCP2515에서 메시지 읽기 (SPI 통신, 약 10~50μs)
   2. ID 확인
   3. 파싱해서 내부 버퍼에 저장
   4. 플래그 세우고 리턴
   ========================================================= */
void isr_can_rx(void) {
    uint32_t rx_id;
    uint8_t  rx_len;
    uint8_t  rx_buf[8];

    /* MCP2515에서 메시지 읽기 */
    if (_can.readMsgBuf(&rx_id, &rx_len, rx_buf) != CAN_OK) {
        return;     /* 읽기 실패 시 무시 */
    }

    /* 이 보드가 처리할 ID인지 확인 (필터를 뚫고 온 다른 ID 방어) */
    if (rx_id != CAN_ID_MOTOR_CMD || rx_len < 4) {
        return;
    }

    /*
     * CAN DB 수신 메시지 구조 (DLC=6):
     *   Byte 0: 왼쪽 모터(L) PWM 명령  signed int8
     *   Byte 1: 오른쪽 모터(R) PWM 명령 signed int8
     *   Byte 2: (예비 — 이 보드는 모터 2개만 담당)
     *   Byte 3: (예비)
     *   Byte 4: MotorEnable(bit0) | BrakeCmd(bit2)
     *   Byte 5: 체크섬 = Byte0 ^ Byte1 ^ Byte2 ^ Byte3 ^ Byte4
     */
    uint8_t ctrl  = (rx_len >= 5) ? rx_buf[4] : 0x01;
    bool motor_en = (ctrl & 0x01) != 0;
    bool brake    = (ctrl & 0x04) != 0;

    /* 체크섬 검증 */
    if (rx_len >= 6) {
        uint8_t chk = rx_buf[0] ^ rx_buf[1] ^ rx_buf[2] ^ rx_buf[3] ^ rx_buf[4];
        if (chk != rx_buf[5]) {
            return;     /* 체크섬 불일치 → 무시 (플래그는 can_get_cmd에서 처리) */
        }
    }

    /* 정지 명령 */
    if (!motor_en || brake) {
        _rx_cmd_l = 0;
        _rx_cmd_r = 0;
        _rx_stop  = true;
    } else {
        _rx_cmd_l = (int8_t)rx_buf[0];
        _rx_cmd_r = (int8_t)rx_buf[1];
        _rx_stop  = false;
    }

    _rx_ready = true;   /* 새 명령 도착 표시 */
}

/* =========================================================
   공개 함수 구현
   ========================================================= */

bool can_init(void) {
    for (uint8_t retry = 0; retry < 3; retry++) {
        if (_can.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK) == CAN_OK) {
            _apply_filters();
            _can.setMode(MCP_NORMAL);   /* 정상 동작 모드로 전환 */

            /* CAN INT 핀에 인터럽트 등록 (INT0 = 핀2, FALLING 엣지) */
            pinMode(CAN_INT_PIN, INPUT);
            attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN),
                            isr_can_rx, FALLING);
            /*
             * FALLING: HIGH→LOW 전환 시 트리거
             * MCP2515 INT 핀은 메시지 수신 시 HIGH→LOW로 내려가므로 FALLING 사용
             */

            _can_ok = true;
            return true;
        }
        delay(100);     /* 재시도 전 대기 (초기화 중이므로 delay 허용) */
    }

    _can_ok = false;
    return false;
}

bool can_get_cmd(int8_t *cmd_l, int8_t *cmd_r) {
    if (!_rx_ready) {
        return false;   /* 새 명령 없음 */
    }

    /*
     * ISR과 공유 변수를 안전하게 읽기
     * noInterrupts/interrupts로 읽는 동안 ISR이 값을 바꾸는 것을 방지
     */
    noInterrupts();
    *cmd_l    = _rx_cmd_l;
    *cmd_r    = _rx_cmd_r;
    _rx_ready = false;  /* 플래그 클리어 (이 명령 소비 완료) */
    interrupts();

    return true;
}

void can_tx_feedback(int16_t spd_l, int16_t spd_r) {
    /*
     * int16(2바이트)을 빅엔디안으로 CAN 페이로드에 패킹
     *
     * 빅엔디안: 상위 바이트가 먼저 (네트워크 표준)
     * 예) 510(0x01FE): buf[0]=0x01, buf[1]=0xFE
     * ECU 복원: (buf[0] << 8) | buf[1] = 510 → × 0.1 = 51.0 cm/s
     *
     * DLC=4 (L 2바이트 + R 2바이트)
     */
    uint8_t buf[4];
    buf[0] = (uint8_t)(spd_l >> 8);
    buf[1] = (uint8_t)(spd_l & 0xFF);
    buf[2] = (uint8_t)(spd_r >> 8);
    buf[3] = (uint8_t)(spd_r & 0xFF);

    _can.sendMsgBuf(CAN_ID_MOTOR_FB, 0, 4, buf);
}

void can_tx_heartbeat(uint8_t err_flags) {
    /*
     * Heartbeat 메시지 (DLC=2):
     *   Byte 0: 0xAA (생존 확인용 고정값)
     *   Byte 1: 에러 플래그 (0 = 정상)
     *
     * ECU는 이 메시지를 150ms(3주기) 이상 못 받으면 FAULT 처리
     */
    uint8_t buf[2] = {0xAA, err_flags};
    _can.sendMsgBuf(CAN_ID_MOTOR_HB, 0, 2, buf);
}

bool can_is_ok(void) {
    return _can_ok;
}
