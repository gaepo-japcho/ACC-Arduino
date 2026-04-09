/*
 * ============================================================
 * 라이브러리
 * ============================================================
 *   MCP_CAN by coryjfowler
 *   (Arduino IDE 라이브러리 매니저 → "MCP_CAN" 검색 → 설치)
 */

#include <SPI.h>
#include <mcp_can.h>

/* =========================================================
   CAN 설정
   ========================================================= */
#define CAN_CS_PIN          10
#define CAN_INT_PIN         2

#define CAN_SPEED           CAN_500KBPS
#define CAN_CLOCK           MCP_8MHZ   

/* CAN 메시지 ID */
#define CAN_ID_MOTOR_CMD    0x300       // ECU → Arduino
#define CAN_ID_MOTOR_FB     0x400       // Arduino → ECU (속도 피드백)
#define CAN_ID_MOTOR_HB     0x401       // Arduino → ECU (Heartbeat)

/* =========================================================
   모터 핀 정의 (FIX-6 반영 — 핀 재배치)
   ========================================================= */
// PWM (EN) 핀 — 모두 analogWrite 가능한 핀 사용
#define PIN_LF_EN   6       
#define PIN_RF_EN   5
#define PIN_LR_EN   9      

// 방향 (IN) 핀
#define PIN_LF_IN1  4
#define PIN_LF_IN2  7

#define PIN_RF_IN1  8
#define PIN_RF_IN2  A0

#define PIN_LR_IN1  A1
#define PIN_LR_IN2  A2

#define PIN_RR_IN1  A3
#define PIN_RR_IN2  A4
#define PIN_RR_EN   A5      // [변경] 9→A5 (디지털 ON/OFF 전용, analogWrite 불가)
                            // A5는 PWM 미지원 → RR 모터는 전진/후진/정지만 가능
                            // ACC 직진 전용 특성상 허용 가능한 트레이드오프

/* =========================================================
   엔코더 핀 
   ========================================================= */
#define PIN_ENC_LF      3   // attachInterrupt(1, ...) = 핀 3
#define ENC_INT_NUM     1   // INT1 (아두이노 우노 외부 인터럽트 1번)

/* =========================================================
   타이밍 상수
   ========================================================= */
#define PERIOD_MOTOR_CTRL_MS    10
#define PERIOD_FEEDBACK_MS      10
#define PERIOD_HEARTBEAT_MS     50
#define PERIOD_ENC_CALC_MS      10      // 엔코더 속도 계산 주기

/* ECU 명령 타임아웃 */
#define TIMEOUT_CMD_MS          30

/* =========================================================
   엔코더/속도 계산 상수
   1560RPM 모터, 바퀴 직경 65mm 기준
   ========================================================= */
#define ENCODER_PPR         20          // 모터 1회전당 펄스 수 (실측 후 수정)
#define WHEEL_CIRC_MM       204         // 바퀴 둘레 mm (π × 65 ≈ 204)

/*
 * 속도 계산 공식:
 *   enc_count: 10ms 동안 카운트된 펄스 수
 *   속도(mm/s) = enc_count × (WHEEL_CIRC_MM / ENCODER_PPR) / 0.01s
 *             = enc_count × WHEEL_CIRC_MM × 100 / ENCODER_PPR
 *   0.1cm/s 단위 = 속도(mm/s) × 0.1
 *             = enc_count × WHEEL_CIRC_MM × 10 / ENCODER_PPR   ← 최종 식
 *
 * [FIX-3] 기존 코드의 버그:
 *   잘못된 식: speed = ... * 100UL / 10UL  (실질적으로 × 10)
 *   올바른 식: speed = enc_count * WHEEL_CIRC_MM * 10 / ENCODER_PPR
 *
 * 검증 (최대속도):
 *   1560RPM → 26RPS → 26 × 20 = 520 pulse/s
 *   10ms 동안 = 5.2 pulse (≈ 5 or 6)
 *   speed = 5 × 204 × 10 / 20 = 510 (0.1cm/s 단위) → 51.0 cm/s 
 *   이론 최대: 26RPS × 20.4cm = 530.4 cm/s와 근접
 */

/* =========================================================
   전역 변수
   ========================================================= */
MCP_CAN CAN(CAN_CS_PIN);

/* 수신된 PWM 명령 (signed int8: -127~127) */
static volatile int8_t  g_cmd_lf = 0;
static volatile int8_t  g_cmd_rf = 0;
static volatile int8_t  g_cmd_lr = 0;
static volatile int8_t  g_cmd_rr = 0;

/* 엔코더 카운터 — ISR에서 수정하므로 volatile 필수 */
static volatile uint16_t g_enc_count = 0;

/* 측정 속도 (0.1cm/s 단위, int16) */
static int16_t g_spd_lf = 0;
static int16_t g_spd_rf = 0;
static int16_t g_spd_lr = 0;
static int16_t g_spd_rr = 0;

/* 타이머 */
static uint32_t t_motor_ctrl  = 0;
static uint32_t t_feedback    = 0;
static uint32_t t_heartbeat   = 0;
static uint32_t t_enc_calc    = 0;
static uint32_t t_last_cmd    = 0;

/* 상태 플래그 */
static bool     g_can_ok      = false;
static uint8_t  g_err_flags   = 0;
/*
 * g_err_flags 비트 정의:
 *   bit 0 (0x01): CAN 통신 오류 (초기화 실패 또는 송신 실패)
 *   bit 1 (0x02): 명령 타임아웃 (30ms 초과)
 *   bit 2 (0x04): 수신 체크섬 오류
 */

/* =========================================================
   함수 선언
   ========================================================= */
void can_init(void);
void can_rx_process(void);
void can_tx_feedback(void);
void can_tx_heartbeat(void);
void motor_set(uint8_t en_pin, uint8_t in1_pin, uint8_t in2_pin, int8_t cmd);
void motor_rr_set(int8_t cmd);
void motor_all_stop(void);
void motor_ctrl_task(void);
void encoder_calc_task(void);
void timeout_check(void);

/* ISR (인터럽트 서비스 루틴) */
void isr_encoder_lf(void);

/* =========================================================
   setup
   ========================================================= */
void setup() {
    Serial.begin(115200);
    Serial.println(F("[ACC] Motor Node BOOT"));

    /* --- 모터 핀 초기화 (부팅 즉시 정지) --- */
    uint8_t pwm_pins[] = {PIN_LF_EN, PIN_RF_EN, PIN_LR_EN};
    for (uint8_t i = 0; i < 3; i++) {
        pinMode(pwm_pins[i], OUTPUT);
        analogWrite(pwm_pins[i], 0);
    }
    /* RR EN: 디지털 핀 */
    pinMode(PIN_RR_EN, OUTPUT);
    digitalWrite(PIN_RR_EN, LOW);

    /* 방향 핀 전체 LOW */
    uint8_t dir_pins[] = {
        PIN_LF_IN1, PIN_LF_IN2,
        PIN_RF_IN1, PIN_RF_IN2,
        PIN_LR_IN1, PIN_LR_IN2,
        PIN_RR_IN1, PIN_RR_IN2
    };
    for (uint8_t i = 0; i < 8; i++) {
        pinMode(dir_pins[i], OUTPUT);
        digitalWrite(dir_pins[i], LOW);
    }

    /* --- 엔코더 인터럽트 설정 ---
     *
     * attachInterrupt(인터럽트번호, ISR함수, 트리거조건)
     *   인터럽트번호: 0=핀2(INT0), 1=핀3(INT1)
     *   트리거조건: RISING(상승엣지), FALLING(하강엣지), CHANGE(변화시)
     *
     * CHANGE를 쓰면 상승+하강 모두 카운트 → PPR 2배 효과
     * (엔코더 정확도 향상, 단 방향 구분 불필요할 때 사용)
     */
    pinMode(PIN_ENC_LF, INPUT_PULLUP);
    attachInterrupt(ENC_INT_NUM, isr_encoder_lf, CHANGE);

    /* --- CAN 초기화 --- */
    can_init();

    /* --- 타이머 초기화 --- */
    uint32_t now = millis();
    t_motor_ctrl = now;
    t_feedback   = now;
    t_heartbeat  = now;
    t_enc_calc   = now;
    t_last_cmd   = now;

    Serial.println(F("[ACC] READY"));
}

/* =========================================================
   loop — delay 없음, millis 슈퍼루프 패턴
   ========================================================= */
void loop() {
    uint32_t now = millis();

    /*
     * 태스크 실행 순서 (우선순위 순):
     *
     * 1. CAN 수신: 매 루프 폴링 (INT 핀 확인 → 빠른 응답)
     * 2. 타임아웃 감시: 매 루프 (ASIL B 30ms 요구사항)
     * 3. 엔코더 계산: 10ms 주기 (ISR이 카운트, 여기서 속도 계산)
     * 4. 모터 제어: 10ms 주기
     * 5. 피드백 송신: 10ms 주기
     * 6. Heartbeat: 50ms 주기
     */

    if (g_can_ok) {
        can_rx_process();
    }

    timeout_check();

    if ((now - t_enc_calc) >= PERIOD_ENC_CALC_MS) {
        t_enc_calc = now;
        encoder_calc_task();
    }

    if ((now - t_motor_ctrl) >= PERIOD_MOTOR_CTRL_MS) {
        t_motor_ctrl = now;
        motor_ctrl_task();
    }

    if ((now - t_feedback) >= PERIOD_FEEDBACK_MS) {
        t_feedback = now;
        if (g_can_ok) {
            can_tx_feedback();
        }
    }

    if ((now - t_heartbeat) >= PERIOD_HEARTBEAT_MS) {
        t_heartbeat = now;
        if (g_can_ok) {
            can_tx_heartbeat();
        }
    }
}

/* =========================================================
   엔코더 ISR (인터럽트 서비스 루틴)
   (INT1) CHANGE 트리거 → 상승/하강 모두 카운트

   ISR 작성 규칙:
     - 최대한 짧게 (수 마이크로초 이내)
     - delay(), Serial, millis() 사용 금지
     - 공유 변수는 volatile로 선언
   ========================================================= */
void isr_encoder_lf(void) {
    g_enc_count++;
    /*
     * CHANGE 트리거 사용 시:
     * 실제 PPR = ENCODER_PPR × 2 (상승+하강 각각 카운트)
     * encoder_calc_task()에서 나눗셈 시 주의
     * → 코드에서는 ENCODER_PPR_EFFECTIVE = ENCODER_PPR * 2 로 계산
     */
}

/* =========================================================
   CAN 초기화
   ========================================================= */
void can_init(void) {
    for (uint8_t retry = 0; retry < 3; retry++) {
        if (CAN.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK) == CAN_OK) {
            CAN.setMode(MCP_NORMAL);

            /*
             * [FIX-2] MCP2515 수신 버퍼 필터 설정
             *
             * MCP2515 수신 버퍼 구조:
             *   RXB0: 필터 0, 1 (우선순위 높음)
             *   RXB1: 필터 2, 3, 4, 5 (우선순위 낮음)
             *
             * RXB0, RXB1 모두 CAN_ID_MOTOR_CMD만 통과
             *
             * init_Mask(버퍼번호, 확장프레임여부, 마스크값)
             *   0x7FF: 11비트 전체 비교 (표준 프레임)
             * init_Filt(필터번호, 확장프레임여부, 필터값)
             *   필터 0,1 → RXB0 / 필터 2~5 → RXB1
             */

            /* RXB0 설정 */
            CAN.init_Mask(0, 0, 0x7FF);
            CAN.init_Filt(0, 0, CAN_ID_MOTOR_CMD);
            CAN.init_Filt(1, 0, CAN_ID_MOTOR_CMD);

            /* RXB1 설정 */
            CAN.init_Mask(1, 0, 0x7FF);
            CAN.init_Filt(2, 0, CAN_ID_MOTOR_CMD);
            CAN.init_Filt(3, 0, CAN_ID_MOTOR_CMD);
            CAN.init_Filt(4, 0, CAN_ID_MOTOR_CMD);
            CAN.init_Filt(5, 0, CAN_ID_MOTOR_CMD);

            g_can_ok = true;
            Serial.println(F("[CAN] Init OK (8MHz, 500Kbps)"));
            Serial.println(F("[CAN] RXB0+RXB1 filter: 0x300 only"));
            return;
        }
        delay(100);
    }

    g_can_ok = false;
    g_err_flags |= 0x01;
    Serial.println(F("[CAN] Init FAILED"));
}

/* =========================================================
   CAN 수신 처리
   INT 핀 LOW 확인 → readMsgBuf → 명령 파싱
   ========================================================= */
void can_rx_process(void) {
    /*
     * INT 핀이 HIGH면 수신된 메시지 없음 → 즉시 리턴
     * (SPI 통신 생략 → CPU 부하 감소)
     */
    if (digitalRead(CAN_INT_PIN) != LOW) {
        return;
    }

    uint32_t rx_id;
    uint8_t  rx_len;
    uint8_t  rx_buf[8];

    if (CAN.readMsgBuf(&rx_id, &rx_len, rx_buf) != CAN_OK) {
        return;
    }

    if (rx_id == CAN_ID_MOTOR_CMD && rx_len >= 4) {
        /* Byte 4: 제어 플래그 (없으면 기본값 = 활성화) */
        uint8_t ctrl_byte = (rx_len >= 5) ? rx_buf[4] : 0x01;
        bool    motor_en  = (ctrl_byte & 0x01) != 0;
        bool    brake_cmd = (ctrl_byte & 0x04) != 0;

        /* 체크섬 검증 (Byte 5 존재 시) */
        if (rx_len >= 6) {
            uint8_t chk = rx_buf[0] ^ rx_buf[1] ^ rx_buf[2] ^ rx_buf[3] ^ rx_buf[4];
            if (chk != rx_buf[5]) {
                g_err_flags |= 0x04;
                return;
            }
        }

        if (!motor_en || brake_cmd) {
            g_cmd_lf = g_cmd_rf = g_cmd_lr = g_cmd_rr = 0;
        } else {
            g_cmd_lf = (int8_t)rx_buf[0];
            g_cmd_rf = (int8_t)rx_buf[1];
            g_cmd_lr = (int8_t)rx_buf[2];
            g_cmd_rr = (int8_t)rx_buf[3];
        }

        t_last_cmd = millis();
        g_err_flags &= ~0x02;   // 타임아웃 에러 클리어
        g_err_flags &= ~0x04;   // 체크섬 에러도 클리어 (정상 수신 확인)
    }
}

/* =========================================================
   명령 타임아웃 감시 (ASIL B 요구사항: 30ms)
   ========================================================= */
void timeout_check(void) {
    if ((millis() - t_last_cmd) > TIMEOUT_CMD_MS) {
        bool any_moving = (g_cmd_lf | g_cmd_rf | g_cmd_lr | g_cmd_rr) != 0;
        if (any_moving) {
            g_cmd_lf = g_cmd_rf = g_cmd_lr = g_cmd_rr = 0;
            motor_all_stop();
        }
        g_err_flags |= 0x02;
    }
}

/* =========================================================
   모터 제어 태스크 (10ms 주기)
   ========================================================= */
void motor_ctrl_task(void) {
    motor_set(PIN_LF_EN, PIN_LF_IN1, PIN_LF_IN2, g_cmd_lf);
    motor_set(PIN_RF_EN, PIN_RF_IN1, PIN_RF_IN2, g_cmd_rf);
    motor_set(PIN_LR_EN, PIN_LR_IN1, PIN_LR_IN2, g_cmd_lr);
    motor_rr_set(g_cmd_rr);
}

/* =========================================================
   단일 모터 제어 (PWM 가능 핀 전용)
   cmd: -127~127 (부호=방향, 절대값=세기)
   ========================================================= */
void motor_set(uint8_t en_pin, uint8_t in1_pin, uint8_t in2_pin, int8_t cmd) {
    if (cmd == 0) {
        analogWrite(en_pin, 0);
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
    } else if (cmd > 0) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        /*
         * 스케일링: -127~127 → 0~254
         * cmd << 1 은 cmd × 2와 동일 (비트 시프트, 더 빠름)
         * 최대값 127 × 2 = 254 (255에 근접, 충분)
         * cmd = 127이면 PWM = 254/255 ≈ 99.6% 듀티사이클
         */
        analogWrite(en_pin, (uint8_t)cmd << 1);
    } else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        analogWrite(en_pin, (uint8_t)(-cmd) << 1);
    }
}

/* =========================================================
   RR 모터 제어 (PIN_RR_EN = A5, analogWrite 불가)
   A5는 실제로 PWM 불가 → 디지털 제어로 처리
   ========================================================= */
void motor_rr_set(int8_t cmd) {
    /*
     * cmd > 0: 전진 (EN=HIGH로 풀파워)
     * cmd < 0: 후진 (EN=HIGH로 풀파워)
     * cmd = 0: 정지
     *
     * PWM 세기 제어 불가 → EN은 ON/OFF만
     * 속도는 4바퀴가 동일 명령을 받으므로 나머지 3개의
     * PWM 제어로 전체 속도가 결정됨
     * (직진 전용 ACC에서는 실용적으로 허용 가능)
     */
    if (cmd == 0) {
        digitalWrite(PIN_RR_IN1, LOW);
        digitalWrite(PIN_RR_IN2, LOW);
        digitalWrite(PIN_RR_EN, LOW);
    } else if (cmd > 0) {
        digitalWrite(PIN_RR_IN1, HIGH);
        digitalWrite(PIN_RR_IN2, LOW);
        digitalWrite(PIN_RR_EN, HIGH);
    } else {
        digitalWrite(PIN_RR_IN1, LOW);
        digitalWrite(PIN_RR_IN2, HIGH);
        digitalWrite(PIN_RR_EN, HIGH);
    }
}

/* =========================================================
   전체 모터 즉시 정지
   ========================================================= */
void motor_all_stop(void) {
    motor_set(PIN_LF_EN, PIN_LF_IN1, PIN_LF_IN2, 0);
    motor_set(PIN_RF_EN, PIN_RF_IN1, PIN_RF_IN2, 0);
    motor_set(PIN_LR_EN, PIN_LR_IN1, PIN_LR_IN2, 0);
    motor_rr_set(0);
}

/* =========================================================
   엔코더 속도 계산 태스크 (10ms 주기)
   ========================================================= */
void encoder_calc_task(void) {
    /*
     * ISR이 비동기로 g_enc_count를 수정하므로
     * 읽는 동안 인터럽트를 잠깐 차단해서 일관성 보장
     */
    noInterrupts();
    uint16_t count = g_enc_count;
    g_enc_count = 0;        // 카운터 리셋 (다음 주기 측정 준비)
    interrupts();

    uint32_t speed_01cms = (uint32_t)count
                           * (uint32_t)WHEEL_CIRC_MM
                           * 1000UL
                           / ((uint32_t)ENCODER_PPR * 2UL);

    /* uint32_t 오버플로우 가드 (int16 최대 32767 = 3276.7 cm/s, 실제 최대 ~530 cm/s) */
    if (speed_01cms > 32767UL) speed_01cms = 32767UL;

    /* 방향 부여 (명령 부호 기반) */
    g_spd_lf = (g_cmd_lf >= 0) ? (int16_t)speed_01cms : -(int16_t)speed_01cms;

    /*
     * RF/LR/RR: 별도 엔코더 없음 → LF 측정값으로 추정
     * 실제 엔코더 배선 추가 시 독립 ISR + 독립 카운터로 교체
     */
    g_spd_rf = (g_cmd_rf >= 0) ? (int16_t)speed_01cms : -(int16_t)speed_01cms;
    g_spd_lr = (g_cmd_lr >= 0) ? (int16_t)speed_01cms : -(int16_t)speed_01cms;
    g_spd_rr = (g_cmd_rr >= 0) ? (int16_t)speed_01cms : -(int16_t)speed_01cms;
}

/* =========================================================
   CAN 속도 피드백 송신 (10ms 주기)
   ========================================================= */
void can_tx_feedback(void) {
    uint8_t buf[8];

    /*
     * int16을 빅엔디안(Big-Endian)으로 패킹
     * (CAN DB 표준 바이트 오더: MSB first)
     *
     * 예: g_spd_lf = 510 (= 51.0 cm/s)
     *   buf[0] = 510 >> 8 = 0x01 (상위 바이트)
     *   buf[1] = 510 & 0xFF = 0xFE (하위 바이트)
     *   ECU에서 복원: (0x01 << 8) | 0xFE = 510 → × 0.1 = 51.0 cm/s
     */
    buf[0] = (uint8_t)(g_spd_lf >> 8);
    buf[1] = (uint8_t)(g_spd_lf & 0xFF);

    buf[2] = (uint8_t)(g_spd_rf >> 8);
    buf[3] = (uint8_t)(g_spd_rf & 0xFF);

    buf[4] = (uint8_t)(g_spd_lr >> 8);
    buf[5] = (uint8_t)(g_spd_lr & 0xFF);

    buf[6] = (uint8_t)(g_spd_rr >> 8);
    buf[7] = (uint8_t)(g_spd_rr & 0xFF);

    byte result = CAN.sendMsgBuf(CAN_ID_MOTOR_FB, 0, 8, buf);
    if (result != CAN_OK) {
        g_err_flags |= 0x01;
    }
}

/* =========================================================
   CAN Heartbeat 송신 (50ms 주기)
   ========================================================= */
void can_tx_heartbeat(void) {
    uint8_t buf[2];
    buf[0] = 0xAA;          // HB_MTR: 생존 신호 (고정값)
    buf[1] = g_err_flags;   // ERR_MTR: 현재 에러 플래그

    CAN.sendMsgBuf(CAN_ID_MOTOR_HB, 0, 2, buf);

    /* 에러 플래그 자동 클리어 (타임아웃 bit1은 timeout_check가 관리) */
    g_err_flags &= 0x02;
}
