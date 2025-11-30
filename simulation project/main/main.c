/*
 * main.c
 * ADC free-running (ADC5) -> hits discretos no LDR
 * Cada hit tira 1 vida, aplica STUN (espera parado, sem aceitar comandos)
 * Reset INT0 com debounce, PWM Timer1, UART não-blocking
 * Laser em PC3
 * Henry - Projeto 2025 (arrumado)
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* ---------------------
   Configurações / thresholds
   --------------------- */
#define LDR_ADC_CH       5       // ADC5 -> A5 / PC5
#define LDR_THRESHOLD    300     // limite de “dano” (ajuste experimentando)
#define LDR_HYST         40      // histerese pro LDR
#define STUN_TIME_MS     3000    // tempo de espera após tomar dano (3s)

/* ---------------------
   UART
   --------------------- */
#define BAUD 9600
#define MYUBRR ((F_CPU / (16UL * BAUD)) - 1)

/* ---------------------
   Masks / Pinos
   --------------------- */
/* PORTB pins */
#define MASK_ENA   (1 << PB1)  // D9  - OC1A - ENA (PWM)
#define MASK_ENB   (1 << PB2)  // D10 - OC1B - ENB (PWM)
#define MASK_IN1   (1 << PB0)  // D8  - IN1

/* PORTD pins */
#define MASK_IN2   (1 << PD7)  // D7  - IN2
#define MASK_IN3   (1 << PD6)  // D6  - IN3
#define MASK_IN4   (1 << PD5)  // D5  - IN4
#define MASK_INT0  (1 << PD2)  // PD2 = INT0 (botão reset)

/* PORTC pins - LEDs + Laser */
#define MASK_LED1  (1 << PC0)  // LED1 (vida 1)
#define MASK_LED2  (1 << PC1)  // LED2 (vida 2)
#define MASK_LED3  (1 << PC2)  // LED3 (vida 3)
#define MASK_LASER (1 << PC3)  // Laser (saída para base do transistor)

/* ---------------------
   Macros auxiliares
   --------------------- */
#define SET_BITS(reg, mask)    ((reg) |= (mask))
#define CLEAR_BITS(reg, mask)  ((reg) &= ~(mask))

/* ---------------------
   Estados do jogo
   --------------------- */
typedef enum {
    STATE_NORMAL = 0,   // pode andar/atirar
    STATE_STUN,         // tomou dano: parado, ignora comandos
    STATE_DEAD          // 0 vidas: parado até reset (INT0)
} game_state_t;

/* ---------------------
   Prototypes
   --------------------- */
void usart_init(unsigned int ubrr);
int usart_available(void);
unsigned char usart_receive_nonblocking(void);
void usart_transmit(char c);
void usart_print_uint(uint16_t v);

void gpio_init(void);
void pwm_init(void);
void adc_init_freerun(uint8_t ch);
void set_motorA_dir(uint8_t dir);
void set_motorB_dir(uint8_t dir);
void set_motorA_speed(uint8_t duty);
void set_motorB_speed(uint8_t duty);
void motor_control(char cmd);

void laser_on(void);
void laser_off(void);

void update_leds_from_lives(uint8_t l);
void reset_lives(void);

void adc_init_freerun(uint8_t ch);

/* ---------------------
   Globals
   --------------------- */
volatile uint8_t  lives       = 3;      // 0..3
volatile uint16_t adc_value   = 0;      // última leitura do ADC (0..1023)
volatile uint8_t  ldr_dark    = 0;      // 0 = claro, 1 = já detectou escuro
volatile game_state_t game_state = STATE_NORMAL;

uint32_t tick_ms        = 0;           // contador de "ms" do main
uint32_t stun_end_time  = 0;           // quando acaba o stun (em ms)

/* ---------------------
   ISR: INT0 with debounce busy-wait
   --------------------- */
ISR(INT0_vect)
{
    // pequeno debounce: garante que o botão ficou pressionado
    for (volatile uint16_t i = 0; i < 3000; ++i) {
        if (PIND & MASK_INT0) {
            // liberou — bounce, sai sem reset
            return;
        }
    }

    // reset imediato do jogo
    reset_lives();

    // Desarma INT0 por um curto período para evitar retriggers
    EIMSK &= ~(1 << INT0); // desabilita INT0
    for (volatile uint32_t j = 0; j < 20000UL; ++j) {
        if (PIND & MASK_INT0) break; // se solto, sai antes
    }
    EIMSK |= (1 << INT0);  // reabilita INT0
}

/* ---------------------
   ISR: ADC free-running - grava valor
   --------------------- */
ISR(ADC_vect)
{
    adc_value = ADC; // leitura 10-bit salva em variável global
}

/* ---------------------
   MAIN
   --------------------- */
int main(void)
{
    cli();

    gpio_init();
    pwm_init();
    usart_init(MYUBRR);
    adc_init_freerun(LDR_ADC_CH);

    /* Configura INT0: ISC01=1, ISC00=0 -> falling edge (1,0) */
    EICRA &= ~(1<<ISC00);
    EICRA |=  (1<<ISC01);
    /* Habilita INT0 */
    EIMSK |= (1 << INT0);

    sei();

    /* estado inicial */
    lives      = 3;
    game_state = STATE_NORMAL;
    set_motorA_speed(0);
    set_motorB_speed(0);
    laser_off();
    update_leds_from_lives(lives);

    const uint16_t loop_delay = 50;            // ms
    const uint16_t serial_print_interval = 500; // ms
    uint32_t last_print = 0;

    for (;;)
    {
        /* 1) DETECÇÃO DE DANO PELO LDR (sempre roda) */
        if (game_state == STATE_NORMAL) {
            // hit quando cruzar de claro -> escuro
            if (!ldr_dark && adc_value < LDR_THRESHOLD) {
                ldr_dark = 1;   // marcamos que entrou em "escuro"

                if (lives > 0) {
                    lives--;
                    update_leds_from_lives(lives);

                    // para tudo imediatamente
                    set_motorA_speed(0);
                    set_motorB_speed(0);
                    set_motorA_dir(0);
                    set_motorB_dir(0);
                    laser_off();

                    if (lives > 0) {
                        // entra em STUN: não anda, não atira, ignora comandos
                        game_state   = STATE_STUN;
                        stun_end_time = tick_ms + STUN_TIME_MS;
                    } else {
                        // 0 vidas = morto
                        game_state = STATE_DEAD;
                    }
                }
            }
            // libera nova detecção só quando ficar bem claro de novo
            else if (ldr_dark && adc_value > (LDR_THRESHOLD + LDR_HYST)) {
                ldr_dark = 0;
            }
        } else {
            // em STUN ou DEAD, só “reseta” o ldr_dark quando estiver claro de novo
            if (ldr_dark && adc_value > (LDR_THRESHOLD + LDR_HYST)) {
                ldr_dark = 0;
            }
        }

        /* 2) GERÊNCIA DO ESTADO STUN (espera parado) */
        if (game_state == STATE_STUN) {
            // garante que tudo permaneça parado e laser off
            set_motorA_speed(0);
            set_motorB_speed(0);
            set_motorA_dir(0);
            set_motorB_dir(0);
            laser_off();

            // verifica se passou o tempo
            if ((int32_t)(tick_ms - stun_end_time) >= 0) {
                if (lives > 0) {
                    game_state = STATE_NORMAL;
                } else {
                    game_state = STATE_DEAD;
                }
            }
        }

        /* 3) UART: só aceita comandos se estiver em STATE_NORMAL */
        if (usart_available()) {
            char c = usart_receive_nonblocking();

            if (game_state == STATE_NORMAL) {
                // Comandos de movimento + laser
                motor_control(c);
            } else {
                // STATE_STUN ou STATE_DEAD -> ignora TUDO, inclusive laser
            }
        }

        /* 4) Debug serial: imprime ADC a cada intervalo configurado */
        if ((tick_ms - last_print) >= serial_print_interval) {
            last_print = tick_ms;
            usart_print_uint(adc_value);
        }

        _delay_ms(loop_delay);
        tick_ms += loop_delay;
    }

    return 0;
}

/* ---------------------
   UART helpers
   --------------------- */
void usart_init(unsigned int ubrr)
{
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);               // RX/TX enable
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);             // 8-bit, 1 stop
}

int usart_available(void)
{
    return (UCSR0A & (1 << RXC0)) ? 1 : 0;
}

unsigned char usart_receive_nonblocking(void)
{
    if (UCSR0A & (1 << RXC0)) return UDR0;
    return 0;
}

void usart_transmit(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}

void usart_print_uint(uint16_t v)
{
    char buf[6];
    itoa(v, buf, 10);
    for (char *p = buf; *p; ++p) usart_transmit(*p);
    usart_transmit('\r');
    usart_transmit('\n');
}

/* ---------------------
   GPIO / PWM / motors / laser
   --------------------- */
void gpio_init(void)
{
    /* DDRs: ENA (PB1), ENB (PB2), IN1 (PB0) */
    SET_BITS(DDRB, MASK_ENA | MASK_ENB | MASK_IN1);

    /* DDRD: IN2 (PD7), IN3 (PD6), IN4 (PD5) */
    SET_BITS(DDRD, MASK_IN2 | MASK_IN3 | MASK_IN4);

    /* DDRC: LEDs (PC0, PC1, PC2) + LASER (PC3) */
    SET_BITS(DDRC, MASK_LED1 | MASK_LED2 | MASK_LED3 | MASK_LASER);

    /* PD2 (INT0) como entrada com pull-up (botão ligado a GND) */
    CLEAR_BITS(DDRD, MASK_INT0);   // entrada
    SET_BITS(PORTD, MASK_INT0);    // pull-up ativo

    /* Inicialmente limpar saídas (IN low, PWM desligados, laser off) */
    CLEAR_BITS(PORTB, MASK_ENA | MASK_ENB | MASK_IN1);
    CLEAR_BITS(PORTD, MASK_IN2 | MASK_IN3 | MASK_IN4);
    CLEAR_BITS(PORTC, MASK_LASER);

    /* LEDs ligados por padrão (vidas cheias) */
    SET_BITS(PORTC, MASK_LED1 | MASK_LED2 | MASK_LED3);
}

void pwm_init(void)
{
    /* Timer1 - Fast PWM 8-bit (modo 5), non-inverting OC1A/OC1B, prescaler 64 */
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    OCR1A = 0;
    OCR1B = 0;
}

void set_motorA_dir(uint8_t dir)
{
    switch (dir) {
        case 1:
            SET_BITS(PORTB, MASK_IN1);
            CLEAR_BITS(PORTD, MASK_IN2);
            break;
        case 2:
            CLEAR_BITS(PORTB, MASK_IN1);
            SET_BITS(PORTD, MASK_IN2);
            break;
        default:
            CLEAR_BITS(PORTB, MASK_IN1);
            CLEAR_BITS(PORTD, MASK_IN2);
            break;
    }
}

void set_motorB_dir(uint8_t dir)
{
    switch (dir) {
        case 1:
            SET_BITS(PORTD, MASK_IN3);
            CLEAR_BITS(PORTD, MASK_IN4);
            break;
        case 2:
            CLEAR_BITS(PORTD, MASK_IN3);
            SET_BITS(PORTD, MASK_IN4);
            break;
        default:
            CLEAR_BITS(PORTD, MASK_IN3);
            CLEAR_BITS(PORTD, MASK_IN4);
            break;
    }
}

void set_motorA_speed(uint8_t duty) { OCR1A = duty; }
void set_motorB_speed(uint8_t duty) { OCR1B = duty; }

void laser_on(void)
{
    SET_BITS(PORTC, MASK_LASER);
}

void laser_off(void)
{
    CLEAR_BITS(PORTC, MASK_LASER);
}

/* ---------------------
   Motor + Laser via UART commands
   --------------------- */
void motor_control(char cmd)
{
    switch (cmd) {
        case 'F':
            set_motorA_dir(1); set_motorB_dir(1);
            set_motorA_speed(200); set_motorB_speed(200);
            break;
        case 'B':
            set_motorA_dir(2); set_motorB_dir(2);
            set_motorA_speed(200); set_motorB_speed(200);
            break;
        case 'L':
            set_motorA_dir(2); set_motorB_dir(1);
            set_motorA_speed(200); set_motorB_speed(200);
            break;
        case 'R':
            set_motorA_dir(1); set_motorB_dir(2);
            set_motorA_speed(200); set_motorB_speed(200);
            break;
        case 'S':
            set_motorA_speed(0); set_motorB_speed(0);
            set_motorA_dir(0); set_motorB_dir(0);
            break;
        case 'Q': // ligar laser
            laser_on();
            break;
        case 'W': // desligar laser
            laser_off();
            break;
        default:
            break;
    }
}

/* ---------------------
   LEDs e reset
   --------------------- */
void update_leds_from_lives(uint8_t l)
{
    /* Primeiro limpa todos */
    CLEAR_BITS(PORTC, MASK_LED1 | MASK_LED2 | MASK_LED3);
    /* Acende conforme número de vidas (1..3) */
    if (l >= 1) SET_BITS(PORTC, MASK_LED1);
    if (l >= 2) SET_BITS(PORTC, MASK_LED2);
    if (l >= 3) SET_BITS(PORTC, MASK_LED3);
}

/* Reseta vidas para 3 e re-acende LEDs; 
*  para motores e laser.
*/
void reset_lives(void)
{
    lives      = 3;
    game_state = STATE_NORMAL;
    ldr_dark   = 0;
    stun_end_time = 0;

    update_leds_from_lives(lives);

    set_motorA_speed(0);
    set_motorB_speed(0);
    set_motorA_dir(0);
    set_motorB_dir(0);
    laser_off();
}

void adc_init_freerun(uint8_t ch)
{
    ADMUX = (1 << REFS0) | (ch & 0x0F);
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE)
           | (1 << ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    ADCSRB = 0;
}

