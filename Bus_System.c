/*
  JMOD-128 (ATmega128) Bus Door Controller + Bus Bell
  - LDR on PF0 / ADC0 : 밝기 확인
  - LCD 표시: 상단 "CLOSE"/"OPEN", 하단 막대기 5개로 상태 표시
  - 문 동작: 버튼 -> OPEN -> 완전열림 -> 10s 대기 -> 자동 닫힘
  - 닫는 중 초음파 감지 -> 다시 OPEN -> 10s 대기
  - 부저: OPEN / 재OPEN 동안에만 울림
  - 모터 속도: 소프트 PWM 사용, 오픈/클로즈 각각 속도 최적화
  - A Motor: 기존 문 제어 (원래 속도 유지)
  - B Motor: 버스 바퀴용, CLOSE(run) 상태에서 최대 속도로 동작 (연속 ON)
  - Bus Bell: B_button 누르면 B_LED 켜지고 B_buzzer 2초 울림
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

/* ----------------- PIN DEFINES ----------------- */
#define LCD_PORT    PORTC
#define LCD_DDR     DDRC
#define LCD_PINMASK 0xF0

#define LCD_RS_PORT PORTD
#define LCD_RS_DDR  DDRD
#define LCD_RS_BIT  PD2

#define LCD_RW_PORT PORTD
#define LCD_RW_DDR  DDRD
#define LCD_RW_BIT  PD3

#define LCD_E_PORT  PORTD
#define LCD_E_DDR   DDRD
#define LCD_E_BIT   PD4

/* --------- A (기존) 버튼/부저 --------- */
#define A_BUTTON_PORT PORTE
#define A_BUTTON_PIN  PINE
#define A_BUTTON_DDR  DDRE
#define A_BUTTON_BIT  PE4

#define A_BUZZER_PORT PORTB
#define A_BUZZER_DDR  DDRB
#define A_BUZZER_BIT  PB3

/* --------- B (새로 추가) 버튼/부저/LED --------- */
#define B_BUTTON_PORT PORTF
#define B_BUTTON_PIN  PINF
#define B_BUTTON_DDR  DDRF
#define B_BUTTON_BIT  PF1

#define B_LED_PORT    PORTB
#define B_LED_DDR     DDRB
#define B_LED_BIT     PB0

#define B_BUZZER_PORT PORTB
#define B_BUZZER_DDR  DDRB
#define B_BUZZER_BIT  PB1

#define TRIG_PORT   PORTE
#define TRIG_DDR    DDRE
#define TRIG_BIT    PE6
#define ECHO_PINP   PINE
#define ECHO_BIT    PE7

/* ----------------- A MOTOR (PB4~7) ----------------- */
#define A_MOTOR_PORT PORTB
#define A_MOTOR_DDR  DDRB
#define A_AIN1_BIT   PB7
#define A_AIN2_BIT   PB6
#define A_PWMA_BIT   PB5
#define A_STBY_BIT   PB4

/* ----------------- B MOTOR (PA4~7) ----------------- */
#define B_MOTOR_PORT PORTA
#define B_MOTOR_DDR  DDRA
#define B_AIN1_BIT   PA7
#define B_AIN2_BIT   PA6
#define B_PWMA_BIT   PA5
#define B_STBY_BIT   PA4

/* ----------------- CONFIG ----------------- */
#define BRIGHT_THRESHOLD    600
#define BAR_MAX             5
#define BAR_STEP_MS         300
#define OPEN_WAIT_MS        10000
#define US_DETECT_CM_TH     50
#define US_TIMEOUT_US       30000
#define MOTOR_PWM_PERIOD_MS 40

#define MOTOR_DUTY_OPEN     55   // 문 모터 듀티 (열기)
#define MOTOR_DUTY_CLOSE    57   // 문 모터 듀티 (닫기)

/* ----------------- HELPERS ----------------- */
static inline void set_bit(volatile uint8_t *port, uint8_t b) { *port |= (1 << b); }
static inline void clr_bit(volatile uint8_t *port, uint8_t b) { *port &= ~(1 << b); }

static void delay_ms_var(uint32_t ms) { while (ms--) _delay_ms(1); }
static void delay_us_var(uint32_t us) { while (us--) _delay_us(1); }

/* ----------------- LCD ----------------- */
static void lcd_pulse(void) {
    set_bit(&LCD_E_PORT, LCD_E_BIT);
    delay_us_var(1);
    clr_bit(&LCD_E_PORT, LCD_E_BIT);
    delay_us_var(50);
}

static void lcd_write4(uint8_t nibble) {
    uint8_t cur = LCD_PORT & ~LCD_PINMASK;
    LCD_PORT = cur | ((nibble << 4) & LCD_PINMASK);
    lcd_pulse();
}

static void lcd_cmd(uint8_t cmd) {
    clr_bit(&LCD_RS_PORT, LCD_RS_BIT);
    lcd_write4(cmd >> 4);
    lcd_write4(cmd & 0x0F);
    delay_ms_var(2);
}

static void lcd_putc(char c) {
    set_bit(&LCD_RS_PORT, LCD_RS_BIT);
    lcd_write4(c >> 4);
    lcd_write4(c & 0x0F);
    delay_us_var(50);
}

static void lcd_puts(const char *s) { while (*s) lcd_putc(*s++); }

static void lcd_goto(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    lcd_cmd(0x80 | addr);
}

static void lcd_init(void) {
    LCD_DDR |= LCD_PINMASK;
    LCD_RS_DDR |= (1 << LCD_RS_BIT);
    LCD_RW_DDR |= (1 << LCD_RW_BIT);
    LCD_E_DDR  |= (1 << LCD_E_BIT);
    clr_bit(&LCD_RW_PORT, LCD_RW_BIT);
    clr_bit(&LCD_E_PORT, LCD_E_BIT);
    delay_ms_var(50);

    clr_bit(&LCD_RS_PORT, LCD_RS_BIT);
    lcd_write4(0x03); delay_ms_var(5);
    lcd_write4(0x03); delay_us_var(150);
    lcd_write4(0x02); delay_ms_var(1);

    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01); delay_ms_var(2);
}

static void lcd_draw_bars(uint8_t bars) {
    lcd_goto(0, 1);
    for (int i = 0; i < bars; i++) lcd_putc('|');
    for (int i = bars; i < BAR_MAX; i++) lcd_putc(' ');
}

static void lcd_update_state(const char *str, uint8_t bars) {
    lcd_cmd(0x01); delay_ms_var(1);
    lcd_goto(0, 0);
    lcd_puts(str);
    lcd_draw_bars(bars);
}

/* ----------------- ADC ----------------- */
static void adc_init(void) {
    DDRF &= ~(1 << PF0);      // LDR 입력 (ADC0)
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

static uint16_t adc_read(uint8_t ch) {
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

/* ----------------- Ultrasonic ----------------- */
static void us_trig_pulse(void) {
    TRIG_DDR  |= (1 << TRIG_BIT);
    TRIG_PORT &= ~(1 << TRIG_BIT);
    delay_us_var(2);
    TRIG_PORT |= (1 << TRIG_BIT);
    delay_us_var(10);
    TRIG_PORT &= ~(1 << TRIG_BIT);
}

static uint32_t us_measure_us(void) {
    us_trig_pulse(); 

    uint32_t cnt = 0;
    while (!(ECHO_PINP & (1 << ECHO_BIT))) {
        delay_us_var(1);
        if (++cnt > US_TIMEOUT_US) return 0;
    }

    uint32_t t = 0;
    while (ECHO_PINP & (1 << ECHO_BIT)) {
        delay_us_var(1);
        if (++t > US_TIMEOUT_US) return 0;
    }

    return t;
}

static uint16_t us_cm_from_us(uint32_t us) { 
    return (us == 0) ? 0xFFFF : (uint16_t)(us / 58); 
}

/* ----------------- A Motor (문) ----------------- */
static void a_motor_hw_init(void) {
    A_MOTOR_DDR |= (1 << A_AIN1_BIT) | (1 << A_AIN2_BIT) | (1 << A_PWMA_BIT) | (1 << A_STBY_BIT);
    A_MOTOR_PORT &= ~((1 << A_AIN1_BIT) | (1 << A_AIN2_BIT) | (1 << A_PWMA_BIT));
    A_MOTOR_PORT |= (1 << A_STBY_BIT);
}

static void a_motor_stop(void) { 
    A_MOTOR_PORT &= ~((1 << A_AIN1_BIT) | (1 << A_AIN2_BIT) | (1 << A_PWMA_BIT)); 
}

static void a_motor_drive_ms(bool dir_open, uint16_t duration, uint8_t duty) {
    if (duty == 0) { 
        a_motor_stop(); 
        delay_ms_var(duration); 
        return; 
    }
    if (duty > 100) duty = 100;

    if (dir_open) { 
        A_MOTOR_PORT |=  (1 << A_AIN1_BIT); 
        A_MOTOR_PORT &= ~(1 << A_AIN2_BIT); 
    } else { 
        A_MOTOR_PORT &= ~(1 << A_AIN1_BIT); 
        A_MOTOR_PORT |=  (1 << A_AIN2_BIT); 
    }

    A_MOTOR_PORT |= (1 << A_STBY_BIT);

    uint16_t period = MOTOR_PWM_PERIOD_MS;
    uint16_t on_ms  = (period * duty) / 100;
    if (on_ms == 0) on_ms = 1;
    uint16_t off_ms = period - on_ms;
    uint16_t elapsed = 0;

    while (elapsed < duration) {
        A_MOTOR_PORT |= (1 << A_PWMA_BIT);
        if (elapsed + on_ms > duration) on_ms = duration - elapsed;
        delay_ms_var(on_ms); 
        elapsed += on_ms;

        A_MOTOR_PORT &= ~(1 << A_PWMA_BIT);
        if (elapsed + off_ms > duration) off_ms = duration - elapsed;
        if (off_ms > 0) delay_ms_var(off_ms); 
        elapsed += off_ms;
    }

    A_MOTOR_PORT &= ~(1 << A_PWMA_BIT);
}

/* ----------------- B Motor (바퀴) ----------------- */
static void b_motor_hw_init(void) {
    B_MOTOR_DDR |= (1 << B_AIN1_BIT) | (1 << B_AIN2_BIT) | (1 << B_PWMA_BIT) | (1 << B_STBY_BIT);
    B_MOTOR_PORT &= ~((1 << B_AIN1_BIT) | (1 << B_AIN2_BIT) | (1 << B_PWMA_BIT));
    B_MOTOR_PORT |= (1 << B_STBY_BIT);
}

static void b_motor_stop(void) {
    B_MOTOR_PORT &= ~((1 << B_AIN1_BIT) | (1 << B_AIN2_BIT) | (1 << B_PWMA_BIT));
}

/* ★ 바퀴 모터: 한 번 켜면 계속 최대속도로 회전 */
static void b_motor_run(void) {
    B_MOTOR_PORT |=  (1 << B_AIN1_BIT);
    B_MOTOR_PORT &= ~(1 << B_AIN2_BIT);
    B_MOTOR_PORT |=  (1 << B_STBY_BIT);
    B_MOTOR_PORT |=  (1 << B_PWMA_BIT);   // continuous ON
}

/* ----------------- A Buzzer ----------------- */
static void a_buzzer_init(void) { 
    A_BUZZER_DDR |= (1 << A_BUZZER_BIT); 
    A_BUZZER_PORT &= ~(1 << A_BUZZER_BIT); 
}

static void a_buzzer_on(void) { 
    for (int i = 0; i < 50; i++) { 
        A_BUZZER_PORT |= (1 << A_BUZZER_BIT); 
        delay_ms_var(2); 
        A_BUZZER_PORT &= ~(1 << A_BUZZER_BIT); 
        delay_ms_var(2); 
    } 
}

static void a_buzzer_off(void) { 
    A_BUZZER_PORT &= ~(1 << A_BUZZER_BIT); 
}

/* ----------------- A Button ----------------- */
static void a_button_init(void) { 
    A_BUTTON_DDR &= ~(1 << A_BUTTON_BIT); 
    A_BUTTON_PORT |= (1 << A_BUTTON_BIT); 
}

static bool a_button_pressed(void) { 
    return !(A_BUTTON_PIN & (1 << A_BUTTON_BIT)); 
}

/* ----------------- B (Bus Bell) ----------------- */
static void b_button_init(void) {
    B_BUTTON_DDR &= ~(1 << B_BUTTON_BIT);
    B_BUTTON_PORT |= (1 << B_BUTTON_BIT);
}

static bool b_button_pressed(void) {
    return !(B_BUTTON_PIN & (1 << B_BUTTON_BIT));
}

static void b_led_init(void) {
    B_LED_DDR |= (1 << B_LED_BIT);
    B_LED_PORT &= ~(1 << B_LED_BIT);
}

static void b_led_on(void)  { B_LED_PORT |= (1 << B_LED_BIT); }
static void b_led_off(void) { B_LED_PORT &= ~(1 << B_LED_BIT); }

static void b_buzzer_init(void) {
    B_BUZZER_DDR |= (1 << B_BUZZER_BIT);
    B_BUZZER_PORT &= ~(1 << B_BUZZER_BIT);
}

static void b_buzzer_on_2s(void) {
    for (int i = 0; i < 50; i++) {      // 50 * (5+5)ms ≈ 0.5초 → 2초 만들려면 200번 정도로 늘려도 됨
        B_BUZZER_PORT |= (1 << B_BUZZER_BIT);
        _delay_ms(5);
        B_BUZZER_PORT &= ~(1 << B_BUZZER_BIT);
        _delay_ms(5);
    }
}

/* ----------------- Button/LED/Buzzer check ----------------- */
static void check_bus_bell(void) {
    if (b_button_pressed()) {
        b_motor_run();      // 바퀴는 계속 구름
        b_led_on();
        b_buzzer_on_2s();
        // 부저 끝나도 바퀴는 계속 ON (b_motor_stop() 안 부름)
    }
}

/* ----------------- State Machine ----------------- */
typedef enum { S_CLOSED, S_OPENING, S_OPEN_WAIT, S_CLOSING } state_t;

/* ----------------- MAIN ----------------- */
int main(void) {
    lcd_init();
    adc_init();
    a_motor_hw_init();
    b_motor_hw_init();
    a_buzzer_init();
    a_button_init();

    b_button_init();
    b_led_init();
    b_buzzer_init();

    TRIG_DDR  |= (1 << TRIG_BIT);
    TRIG_PORT &= ~(1 << TRIG_BIT);

    state_t state = S_CLOSED;
    uint8_t bars  = BAR_MAX;
    uint16_t light_adc;

    lcd_update_state("CLOSE (run)", bars); // 초기 상태

    while (1) {
        light_adc = adc_read(0);
        bool bright = (light_adc > BRIGHT_THRESHOLD);

        // 하차벨 처리 (바퀴는 멈추지 않음)
        check_bus_bell();

        // 조도에 따라 문 제어
        if (!bright) {
            a_motor_stop();
            a_buzzer_off();
            state = S_CLOSED;
            bars  = BAR_MAX;
            lcd_update_state("CLOSE (run)", bars);

            // 주행 중: 바퀴 모터 최대 속도 유지
            b_motor_run();
            delay_ms_var(250);
            continue;
        }

        switch (state) {
        case S_CLOSED:
            a_buzzer_off();
			b_motor_stop();
            lcd_update_state("CLOSE (stop)", bars);
            if (a_button_pressed()) {
                delay_ms_var(30);
                if (a_button_pressed()) state = S_OPENING;
            }
            delay_ms_var(100);
            break;

        case S_OPENING:
            b_led_off();
            lcd_update_state("OPEN (stop)", bars);
            b_motor_stop();          // 문 열릴 때는 바퀴 정지 (필요 없으면 지워도 됨)
            while (bars > 0) {
                a_buzzer_on();
                a_motor_drive_ms(true, BAR_STEP_MS, MOTOR_DUTY_OPEN);
                bars--;
                lcd_update_state("OPEN (stop)", bars);
                delay_ms_var(500);
            }
            a_motor_stop();
            a_buzzer_off();
            state = S_OPEN_WAIT;
            break;

        case S_OPEN_WAIT:
            a_buzzer_off();
            a_motor_stop();
            b_motor_stop();
            lcd_update_state("OPEN (stop)", bars);
            for (int sec = OPEN_WAIT_MS / 1000; sec > 0; sec--) {
                lcd_goto(12, 0);
                lcd_putc('0' + (sec / 10));
                lcd_putc('0' + (sec % 10));
                delay_ms_var(1000);
            }
            state = S_CLOSING;
            break;

        case S_CLOSING:
            a_buzzer_off();
            lcd_update_state("CLOSE (stop)", bars);
            b_motor_stop();
            while (bars < BAR_MAX) {
                bool detected = false;
                a_motor_drive_ms(false, BAR_STEP_MS, MOTOR_DUTY_CLOSE);

                uint32_t us = us_measure_us();
                uint16_t cm = us_cm_from_us(us);
                if (cm != 0xFFFF && cm > 0 && cm <= US_DETECT_CM_TH) {
                    detected = true;
                }

                a_motor_stop();

                if (detected) {
                    while (bars > 0) {
                        a_buzzer_on();
                        a_motor_drive_ms(true, BAR_STEP_MS, MOTOR_DUTY_OPEN);
                        bars--;
                        lcd_update_state("OPEN (stop)", bars);
                        delay_ms_var(500);
                    }
                    a_buzzer_off();
                    state = S_OPEN_WAIT;
                    break;
                } else {
                    bars++;
                    lcd_update_state("CLOSE (stop)", bars);
                    delay_ms_var(500);
                }
            }
            if (bars >= BAR_MAX && state == S_CLOSING) {
                a_motor_stop();
                b_motor_stop();
                a_buzzer_off();
                state = S_CLOSED;
                lcd_update_state("CLOSE (run)", bars);
                delay_ms_var(200);
            }
            break;
        }
    }
    return 0;
}
