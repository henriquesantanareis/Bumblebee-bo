# Bumblebee-bo 
## Sistema Embarcado com ATmega328P + ESP32 + LDR + Laser + L293D

Projeto acadêmico desenvolvido para a disciplina de Programação de Hardware.  
Consiste em um sistema embarcado com:

- Controle remoto via **ESP32** utilizando o aplicativo **Dabble**  
- Processamento principal e controle de motores via **ATmega328P**
- Hit-detection usando **LDR** + **laser**
- Sistema de **vidas**, **STUN** e **game over**
- Controle de **motores DC** usando **ponte H L293D**
- Comunicação serial UART entre ESP32 ⇆ ATmega328P  
- Implementação baseada **100% em registradores**, sem bibliotecas Arduino no AVR

---

## 📌 **Funcionalidades do Sistema**

### 🟢 ATmega328P (microcontrolador principal)
- Lê continuamente o LDR em modo **ADC Free-Running**
- Detecta “hit” quando a luz do laser reduz o valor do ADC
- Cada hit remove **1 vida**
- Quando um hit ocorre:
  - Motores param imediatamente
  - Laser desliga automaticamente
  - Entra em estado **STUN** por 3 segundos (Timer0)
  - Durante o STUN, **não aceita comandos do ESP32**
- Quando as vidas chegam a **0**, entra em **STATE_DEAD**
- LEDs indicam o número de vidas
- Reset é feito enviando comando `'R'` via UART

### 🔵 ESP32 (controle remoto)
- Conecta ao aplicativo **Dabble**
- Usa o módulo **GamePad**
- Envia comandos pela UART2 para o ATmega328P:
  | Botão | Ação | Código enviado |
  |-------|------|----------------|
  | Up | Frente | `'F'` |
  | Down | Ré | `'B'` |
  | Left | Esquerda | `'L'` |
  | Right | Direita | `'R'` |
  | Square | Liga laser | `'Q'` |
  | Circle | Desliga laser | `'W'` |
  | Cross | Stop | `'S'` |
  | Start | Reset vidas | `'R'` |

---

## 🛠️ **Hardware Utilizado**

### ATmega328P (bare metal)
- Alimentação 5V
- Oscilador 16 MHz externo
- Pinos usados:
  | Função | Pino | Porta |
  |--------|------|-------|
  | LDR | ADC5 | PC5 |
  | Laser (transistor) | PC3 | PORTC |
  | LED vidas | PC0–PC2 | PORTC |
  | Motores ENA/ENB | PB1/PB2 | PWM |
  | Motor direção | PB0, PD5, PD6, PD7 |
  | UART RX/TX | PD0/PD1 |

### ESP32 DevKit V1
- UART2 utilizada:
  - RX2 = GPIO16
  - TX2 = GPIO17

### Ponte H L293D
- Motor A e Motor B
- Alimentação separada para motores (recomendado)

### LDR + Resistores
- Divisor resistivo para entrada do ADC

### Laser + Transistor (BC548 / BC548B)
- PC3 → resistor base 1k → transistor → laser → GND

---

## 🔌 **Diagrama de Conexões (Resumo)**

```txt
         +------------------+            +----------------+
         |    ESP32 DevKit |            |  ATmega328P    |
         |                  |            |                |
   TX2 --+ GPIO17           +----------->+ RX0 (PD0)      |
   RX2 <-+ GPIO16           <-----------+ TX0 (PD1)      |
         |                  |            |                |
         |    Dabble BLE    |            |                |
         +------------------+            +----------------+

                    Motores (ponte H L293D)
       IN1 = PB0, IN2 = PD7, IN3 = PD6, IN4 = PD5
       ENA = PB1 (PWM),  ENB = PB2 (PWM)

                 Laser
       PC3 -> R(1k) -> Base do transistor -> Laser

                 LDR
       LDR → divisor resistivo → PC5 (ADC5)

                 LEDs vidas
       PC0, PC1, PC2 → LEDs (3 vidas)

📁 ProjetoBumbleBee/
│
├── src/
│   ├── main.c              → Código completo do ATmega328P
│   └── esp32_control.ino   → Código do ESP32 com Dabble
│
├── docs/
│   ├── Doxyfile            → Configuração do Doxygen
│   ├── html/               → Site gerado pelo Doxygen
│   └── diagrams/           → Diagramas e esquemas
│
├── kicad/
│   ├── POM.kicad_pro
│   ├── POE.kicad_pro
│   └── POP.kicad_pro
│
└── README.md   ← (este arquivo)
