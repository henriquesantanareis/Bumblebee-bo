/**
 * @file main.c
 * @brief LDR -> hits que tiram vidas, STUN com Timer, laser em PC3, reset via UART 'R'
 *
 * Projeto: sistema embarcado para ATmega328P (Arduino Uno)
 * - Leitura ADC em modo free-running (ADC5 / PC5)
 * - Cada "hit" detectado no LDR reduz 1 vida e aplica STUN (fica parado, ignora comandos)
 * - Laser controlado em PC3 via transistor
 * - PWM para motores com Timer1 (OC1A/OC1B)
 * - Comunicação UART não-blocking (Serial0) — aceita comandos de um ESP32
 *
 * Autores: Henrique, Fernando & Daniel
 *
 * Observações:
 * - Ajuste LDR_THRESHOLD e LDR_HYST conforme ensaio prático.
 * - Reset de vidas pode ser via INT0 (botão) ou via UART (comando 'R').
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/** @name Configurações / thresholds
 *  Thresholds do LDR e tempo de stun
 *  @{ */
#define LDR_ADC_CH       5       /**< ADC5 -> A5 / PC5 */
#define LDR_THRESHOLD    300     /**< limite de “dano” (ajuste experimental) */
#define LDR_HYST         40      /**< histerese do LDR para evitar bouncing/ruído */
#define STUN_TIME_MS     3000    /**< tempo de espera após dano (ms) */
/** @} */

/** @name UART
 *  Configuração heurística do UART (Serial0) para comunicação com o ESP32
 *  @{ */
#define BAUD 9600
#define MYUBRR ((F_CPU / (16UL * BAUD)) - 1)
/** @} */

/** @name Masks / Pinos
 *  Máscaras para manipular os registradores DDRx / PORTx diretamente
 *  @{ */
/* PORTB pins */
#define MASK_ENA   (1 << PB1)  /**< D9  - OC1A - ENA (PWM) */
#define MASK_ENB   (1 << PB2)  /**< D10 - OC1B - ENB (PWM) */
#define MASK_IN1   (1 << PB0)  /**< D8  - IN1 */

/* PORTD pins */
#define MASK_IN2   (1 << PD7)  /**< D7  - IN2 */
#define MASK_IN3   (1 << PD6)  /**< D6  - IN3 */
#define MASK_IN4   (1 << PD5)  /**< D5  - IN4 */
#define MASK_INT0  (1 << PD2)  /**< PD2 = INT0 (botão reset) */

/* PORTC pins - LEDs + Laser */
#define MASK_LED1  (1 << PC0)  /**< LED1 (vida 1) */
#define MASK_LED2  (1 << PC1)  /**< LED2 (vida 2) */
#define MASK_LED3  (1 << PC2)  /**< LED3 (vida 3) */
#define MASK_LASER (1 << PC3)  /**< Laser (saída para base do transistor) */
/** @} */

/** @name Macros auxiliares
 *  Macros para facilitar operações sobre registradores
 *  @{ */
#define SET_BITS(reg, mask)    ((reg) |= (mask))  /**< seta bits em reg */
#define CLEAR_BITS(reg, mask)  ((reg) &= ~(mask)) /**< limpa bits em reg */
/** @} */

/**
 * @brief Estados possíveis do jogo/sistema
 */
typedef enum {
    STATE_NORMAL = 0,   /**< pode andar/atirar */
    STATE_STUN,         /**< tomou dano: parado, ignora comandos */
    STATE_DEAD          /**< 0 vidas: parado até reset (INT0 ou comando UART) */
} game_state_t;

/* ---------------------
   Prototypes (funções públicas / utilitárias)
   --------------------- */

/**
 * @brief Inicializa USART0 com UBRR fornecido.
 * @param ubrr valor pre-calculado para UBRR.
 */
void usart_init(unsigned int ubrr);

/**
 * @brief Verifica se há dado disponível na UART (não bloqueante).
 * @return 1 se há dado, 0 caso contrário.
 */
int usart_available(void);

/**
 * @brief Recebe 1 byte da UART de forma não-bloqueante.
 * @return byte recebido ou 0 se nada disponível.
 */
unsigned char usart_receive_nonblocking(void);

/**
 * @brief Envia 1 caractere pela UART (bloqueante até buffer livre).
 * @param c caractere a transmitir.
 */
void usart_transmit(char c);

/**
 * @brief Imprime um inteiro sem sinal (até 5 dígitos) seguido de CRLF.
 * @param v valor a imprimir.
 */
void usart_print_uint(uint16_t v);

/**
 * @brief Configura GPIOs iniciais: motores, LEDs, laser, pull-up botão.
 */
void gpio_init(void);

/**
 * @brief Inicializa Timer1 para PWM (Fast PWM 8-bit) e zera OCR1A/B.
 */
void pwm_init(void);

/**
 * @brief Define direção do motor A (1 = forward, 2 = reverse, 0 = stop).
 * @param dir direção desejada.
 */
void set_motorA_dir(uint8_t dir);

/**
 * @brief Define direção do motor B (1 = forward, 2 = reverse, 0 = stop).
 * @param dir direção desejada.
 */
void set_motorB_dir(uint8_t dir);

/**
 * @brief Define duty cycle para motor A (0..255).
 * @param duty valor de duty (8-bit).
 */
void set_motorA_speed(uint8_t duty);

/**
 * @brief Define duty cycle para motor B (0..255).
 * @param duty valor de duty (8-bit).
 */
void set_motorB_speed(uint8_t duty);

/**
 * @brief Interpreta comando recebido via UART e controla motores/laser.
 * @param cmd caractere do comando.
 */
void motor_control(char cmd);

/**
 * @brief Liga o laser (ativa saída PC3).
 */
void laser_on(void);

/**
 * @brief Desliga o laser (desativa saída PC3).
 */
void laser_off(void);

/**
 * @brief Atualiza LEDs de vidas com base no valor lido.
 * @param l número de vidas (0..3).
 */
void update_leds_from_lives(uint8_t l);

/**
 * @brief Reseta vidas para 3, apaga motores/laser e re-acende LEDs.
 */
void reset_lives(void);

/**
 * @brief Inicializa ADC em modo free-running para o canal ch.
 * @param ch canal ADC (0..7 para ATmega328P).
 */
void adc_init_freerun(uint8_t ch);

/* ---------------------
   Globals
   --------------------- */

/** número de vidas atuais (0..3) */
volatile uint8_t  lives       = 3;

/** última leitura do ADC (0..1023) — atualizada pela ISR ADC */
volatile uint16_t adc_value   = 0;

/** flag de histerese do LDR: 0 = claro, 1 = escuro (para detectar bordas) */
volatile uint8_t  ldr_dark    = 0;

/** estado atual do jogo */
volatile game_state_t game_state = STATE_NORMAL;

/** contador virtual de ms mantido pelo laço principal */
uint32_t tick_ms        = 0;

/** instante (tick_ms) em que o stun termina */
uint32_t stun_end_time  = 0;

/* ---------------------
   ISRs
   --------------------- */

/**
 * @brief INT0 interrupt service routine — botão físico de reset com debounce.
 *
 * O ISR faz um debounce simples via busy-wait curto: confirma que PD2 permaneceu
 * pressionado por um pequeno período antes de efetuar reset.
 */
ISR(INT0_vect)
{
    for (volatile uint16_t i = 0; i < 3000; ++i) {
        if (PIND & MASK_INT0) {
            /* liberou — provável bounce/falso, sai sem reset */
            return;
        }
    }

    /* Se aqui, botão está estável (pressionado) — executa reset imediato */
    reset_lives();

    /* Desarma INT0 por um curto período para evitar retriggers por bounce */
    EIMSK &= ~(1 << INT0); // desabilita INT0
    for (volatile uint32_t j = 0; j < 20000UL; ++j) {
        if (PIND & MASK_INT0) break;
    }
    EIMSK |= (1 << INT0);  // reabilita INT0
}

/**
 * @brief ADC conversion complete ISR (free-running).
 *
 * Grava a leitura 10-bit do ADC na variável global adc_value.
 * A lógica de detecção de hit (borda claro->escuro) é feita no loop principal
 * para evitar processamento pesado dentro de ISR.
 */
ISR(ADC_vect)
{
    adc_value = ADC;
}

/* ---------------------
   MAIN
   --------------------- */

/**
 * @brief Função principal - inicializa periféricos e executa loop principal.
 *
 * Loop:
 *  - checa ADC e detecta borda claro->escuro para reduzir vidas
 *  - aplica STUN (parar e ignorar comandos) quando tomar dano
 *  - aceita comandos UART apenas em STATE_NORMAL
 *  - imprime ADC via UART periodicamente para debug
 */
int main(void)
{
    cli(); /* desabilita interrupções globais durante configuração */

    gpio_init();
    pwm_init();
    usart_init(MYUBRR);
    adc_init_freerun(LDR_ADC_CH);

    /* Configura INT0: ISC01=1, ISC00=0 -> falling edge (1,0) */
    EICRA &= ~(1<<ISC00);
    EICRA |=  (1<<ISC01);
    /* Habilita INT0 */
    EIMSK |= (1 << INT0);

    sei(); /* habilita interrupções globais */

    /* estado inicial */
    lives      = 3;
    game_state = STATE_NORMAL;
    set_motorA_speed(0);
    set_motorB_speed(0);
    laser_off();
    update_leds_from_lives(lives);

    const uint16_t loop_delay = 50;            /* ms */
    const uint16_t serial_print_interval = 500; /* ms */
    uint32_t last_print = 0;

    for (;;)
    {
        /* 1) DETECÇÃO DE DANO PELO LDR (sempre roda) */
        if (game_state == STATE_NORMAL) {
            /* hit quando cruzar de claro -> escuro */
            if (!ldr_dark && adc_value < LDR_THRESHOLD) {
                ldr_dark = 1;   /* entrou em "escuro" */

                if (lives > 0) {
                    lives--;
                    update_leds_from_lives(lives);

                    /* para tudo imediatamente */
                    set_motorA_speed(0);
                    set_motorB_speed(0);
                    set_motorA_dir(0);
                    set_motorB_dir(0);
                    laser_off();

                    if (lives > 0) {
                        /* entra em STUN: não anda, não atira, ignora comandos */
                        game_state   = STATE_STUN;
                        stun_end_time = tick_ms + STUN_TIME_MS;
                    } else {
                        /* 0 vidas = morto */
                        game_state = STATE_DEAD;
                    }
                }
            }
            /* libera nova detecção só quando ficar bem claro de novo */
            else if (ldr_dark && adc_value > (LDR_THRESHOLD + LDR_HYST)) {
                ldr_dark = 0;
            }
        } else {
            /* em STUN ou DEAD, só “reseta” o ldr_dark quando estiver claro de novo */
            if (ldr_dark && adc_value > (LDR_THRESHOLD + LDR_HYST)) {
                ldr_dark = 0;
            }
        }

        /* 2) GERÊNCIA DO ESTADO STUN (espera parado) */
        if (game_state == STATE_STUN) {
            /* garante que tudo permaneça parado e laser off */
            set_motorA_speed(0);
            set_motorB_speed(0);
            set_motorA_dir(0);
            set_motorB_dir(0);
            laser_off();

            /* verifica se passou o tempo */
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
                /* Comandos de movimento + laser */
                motor_control(c);
            } else {
                /* STATE_STUN ou STATE_DEAD -> ignora TUDO, inclusive laser */
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
   Implementações auxiliares / UART / GPIO / PWM / ADC
   --------------------- */

/**
 * @brief Inicializa a UART0 (Serial) com UBRR configurado.
 * @param ubrr valor para UBRR
 */
void usart_init(unsigned int ubrr)
{
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); /* RX/TX enable */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8-bit, 1 stop */
}

/**
 * @brief Verifica se há dado recebido (não-bloqueante).
 * @return 1 se há dado, 0 caso contrário.
 */
int usart_available(void)
{
    return (UCSR0A & (1 << RXC0)) ? 1 : 0;
}

/**
 * @brief Recebe um byte não-bloqueante da UART.
 * @return byte recebido; 0 se nada disponível.
 */
unsigned char usart_receive_nonblocking(void)
{
    if (UCSR0A & (1 << RXC0)) return UDR0;
    return 0;
}

/**
 * @brief Transmite um caractere pela UART (bloqueante até espaço em buffer).
 * @param c caractere a ser enviado.
 */
void usart_transmit(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}

/**
 * @brief Imprime um inteiro (0..1023) via UART com CRLF.
 * @param v valor a imprimir.
 */
void usart_print_uint(uint16_t v)
{
    char buf[6];
    itoa(v, buf, 10);
    for (char *p = buf; *p; ++p) usart_transmit(*p);
    usart_transmit('\r');
    usart_transmit('\n');
}

/**
 * @brief Inicializa GPIOs: pinos de motor, LEDs e laser.
 */
void gpio_init(void)
{
    /* DDRs: ENA (PB1), ENB (PB2), IN1 (PB0) */
    SET_BITS(DDRB, MASK_ENA | MASK_ENB | MASK_IN1);

    /* DDRD: IN2 (PD7), IN3 (PD6), IN4 (PD5) */
    SET_BITS(DDRD, MASK_IN2 | MASK_IN3 | MASK_IN4);

    /* DDRC: LEDs (PC0, PC1, PC2) + LASER (PC3) */
    SET_BITS(DDRC, MASK_LED1 | MASK_LED2 | MASK_LED3 | MASK_LASER);

    /* PD2 (INT0) como entrada com pull-up (botão ligado a GND) */
    CLEAR_BITS(DDRD, MASK_INT0);   /* entrada */
    SET_BITS(PORTD, MASK_INT0);    /* pull-up ativo */

    /* Inicialmente limpar saídas (IN low, PWM desligados, laser off) */
    CLEAR_BITS(PORTB, MASK_ENA | MASK_ENB | MASK_IN1);
    CLEAR_BITS(PORTD, MASK_IN2 | MASK_IN3 | MASK_IN4);
    CLEAR_BITS(PORTC, MASK_LASER);

    /* LEDs ligados por padrão (vidas cheias) */
    SET_BITS(PORTC, MASK_LED1 | MASK_LED2 | MASK_LED3);
}

/**
 * @brief Inicializa Timer1 em Fast PWM 8-bit para controlar motores.
 */
void pwm_init(void)
{
    /* Timer1 - Fast PWM 8-bit (modo 5), non-inverting OC1A/OC1B, prescaler 64 */
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    OCR1A = 0;
    OCR1B = 0;
}

/**
 * @brief Define a direção do motor A.
 */
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

/**
 * @brief Define a direção do motor B.
 */
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

/**
 * @brief Define duty do motor A (OCR1A).
 */
void set_motorA_speed(uint8_t duty) { OCR1A = duty; }

/**
 * @brief Define duty do motor B (OCR1B).
 */
void set_motorB_speed(uint8_t duty) { OCR1B = duty; }

/**
 * @brief Liga o laser (ativa PC3).
 */
void laser_on(void)
{
    SET_BITS(PORTC, MASK_LASER);
}

/**
 * @brief Desliga o laser (desativa PC3).
 */
void laser_off(void)
{
    CLEAR_BITS(PORTC, MASK_LASER);
}

/**
 * @brief Interpreta comandos recebidos via UART e controla motores/laser.
 *
 * Comandos de exemplo:
 *  - 'F' : frente
 *  - 'B' : ré
 *  - 'L' : esquerda
 *  - 'R' : direita
 *  - 'S' : stop
 *  - 'Q' : laser on
 *  - 'W' : laser off
 */
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
        case 'Q': /* ligar laser */
            laser_on();
            break;
        case 'W': /* desligar laser */
            laser_off();
            break;
        default:
            break;
    }
}

/**
 * @brief Atualiza LEDs que indicam número de vidas.
 * @param l número de vidas (0..3).
 */
void update_leds_from_lives(uint8_t l)
{
    CLEAR_BITS(PORTC, MASK_LED1 | MASK_LED2 | MASK_LED3);
    if (l >= 1) SET_BITS(PORTC, MASK_LED1);
    if (l >= 2) SET_BITS(PORTC, MASK_LED2);
    if (l >= 3) SET_BITS(PORTC, MASK_LED3);
}

/**
 * @brief Reseta o sistema para estado inicial (3 vidas, motores off, laser off).
 *
 * Esta função pode ser chamada tanto pelo INT0 (botão) quanto via UART (comando 'R').
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

/**
 * @brief Inicializa ADC em free-running para o canal ch.
 * @param ch número do canal ADC (0..7).
 */
void adc_init_freerun(uint8_t ch)
{
    ADMUX = (1 << REFS0) | (ch & 0x0F); /* AVcc ref, canal */
    ADCSRA = (1 << ADEN)  /* ADC enable */
           | (1 << ADSC)  /* start conversion */
           | (1 << ADATE) /* auto trigger enable (free-run) */
           | (1 << ADIE)  /* ADC interrupt enable */
           | (1 << ADPS2) | (1<<ADPS1) | (1<<ADPS0); /* prescaler 128 */
    ADCSRB = 0; /* free running mode */
}
