🚗 ESP32 Gamepad Controller for ATmega328P (via Dabble Bluetooth)
Controle remoto do robô / carro usando ESP32 DevKit V1 + App Dabble
Autor: Henry
📌 Descrição do Projeto

Este firmware para ESP32 DevKit V1 permite controlar um robô baseado em ATmega328P usando o Bluetooth BLE via aplicativo Dabble (módulo GamePad).

O ESP32 recebe os botões do GamePad, interpreta seus eventos e envia comandos simples, via Serial2 (UART), diretamente para o ATmega328P, onde o firmware principal controla motores, laser e lógica de vidas.

O código implementa:

✔ Controle de movimento (Frente, Trás, Esquerda, Direita)
✔ Controle do laser (ON/OFF)
✔ Parar motores
✔ Reset remoto das vidas (R)
✔ Botão adicional (X)
✔ Edge detection (comando enviado apenas ao pressionar)
✔ Comunicação UART estável via Serial2 (TX2/RX2)
✔ Comunicação BLE totalmente gerenciada pela biblioteca DabbleESP32

🧩 Arquitetura Geral

📱 App Dabble (Gamepad)
⬇
🔵 ESP32 DevKit V1 (BLE)
⬇ (Serial2 TX/RX)
🔶 ATmega328P
⬇
⚙️ Motores / Laser / LEDs / LDR

📡 Mapa de Botões → Comandos enviados
Botão Dabble	Comando enviado	Função no ATmega328P
Up	F	Mover para frente
Down	B	Ré
Left	L	Girar à esquerda
Right	R	Girar à direita
Square	Q	Laser ON
Circle	W	Laser OFF
Cross	S	STOP (parar motores)
Triangle	T	Comando extra livre
Start	R	RESET total (vidas)
Select	X	Comando alternativo
🔌 Conexões do Hardware (ESP32 ⇆ ATmega328P)
ESP32	ATmega328P	Descrição
TX2 (GPIO 17)	RX (PD0)	Envia comandos ao AVR
RX2 (GPIO 16)	TX (PD1)	Recebe debug (opcional)
3V3	VCC	Alimentação (se compatível)
GND	GND	Referência comum


📦 Dependências

Instale a biblioteca:

DabbleESP32 1.5.1 ou superior


E use o core:

ESP32 Arduino Core 2.0.x

🧪 Como Usar

Instale o app Dabble no celular

Ative o GamePad

Faça upload deste código no ESP32

Ligue o ATmega328P com seu firmware correspondente

Abra o GamePad → Conectar → Procure por "Gigantes"

Controles já funcionarão imediatamente