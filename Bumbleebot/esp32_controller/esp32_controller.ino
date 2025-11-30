// ESP32 DevKit V1 + Dabble -> envia comandos ao ATmega328P via Serial2 (TX2/RX2)
// Requer: biblioteca DabbleESP32 instalada
// Ajuste TX2_PIN / RX2_PIN se desejar outros pinos (defaults comuns: TX2=17, RX2=16)

#include <DabbleESP32.h>

#define BAUD_MCU 9600          // baud para comunicar com o ATmega328P
#define BT_NAME "Gigantes"     // nome BLE mostrado pelo Dabble

// pinos Serial2 (ajustáveis)
const int RX2_PIN = 16; // RX2 (conecta no TX do AVR)
const int TX2_PIN = 17; // TX2 (conecta no RX do AVR)

// controle de debounce/edge detection dos botões do GamePad
struct BtnState {
  bool up=false;
  bool down=false;
  bool left=false;
  bool right=false;
  bool square=false;
  bool circle=false;
  bool cross=false;
  bool triangle=false;
  bool start=false;
  bool select=false;
} prevState;

void sendToMCU(char c) {
  // envia apenas ASCII simples seguido de newline opcional (a MCU aguenta só char também)
  Serial2.write((uint8_t)c);
  // opcional: também imprimir no Serial USB para debug
  Serial.print("-> MCU: ");
  Serial.println(c);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  // inicializa Dabble (BLE)
  Dabble.begin(BT_NAME);

  // Serial2 para comunicar com o AVR (UART hardware)
  Serial2.begin(BAUD_MCU, SERIAL_8N1, RX2_PIN, TX2_PIN);

  Serial.println("Dabble + Serial2 ready.");
}

void loop() {
  // precisa chamar sempre para Dabble processar inputs
  Dabble.processInput();

  // Ler status atual dos botões
  bool up = GamePad.isUpPressed();
  bool down = GamePad.isDownPressed();
  bool left = GamePad.isLeftPressed();
  bool right = GamePad.isRightPressed();
  bool square = GamePad.isSquarePressed();
  bool circle = GamePad.isCirclePressed();
  bool cross = GamePad.isCrossPressed();
  bool triangle = GamePad.isTrianglePressed();
  bool start = GamePad.isStartPressed();
  bool select = GamePad.isSelectPressed();

  // EDGE DETECTION: somente ao pressionar (transição 0->1) enviamos um comando.
  // Você pode mudar para também enviar ao soltar se quiser.
  if (up && !prevState.up) {
    sendToMCU('F'); // frente
  }
  if (down && !prevState.down) {
    sendToMCU('B'); // ré
  }
  if (left && !prevState.left) {
    sendToMCU('L'); // esquerda
  }
  if (right && !prevState.right) {
    sendToMCU('R'); // direita (ATENÇÃO: envia 'R')
  }
  if (square && !prevState.square) {
    sendToMCU('Q'); // laser ON
  }
  if (circle && !prevState.circle) {
    sendToMCU('W'); // laser OFF
  }
  if (cross && !prevState.cross) {
    sendToMCU('S'); // stop
  }
  if (triangle && !prevState.triangle) {
    sendToMCU('T'); // ação extra (pode mapear)
  }

  // Start envia 'R' (reset vidas) por pedido seu
  if (start && !prevState.start) {
    sendToMCU('R'); // RESET
  }

  // Select envia 'X' (reset alternativo / livre)
  if (select && !prevState.select) {
    sendToMCU('X');
  }

  // atualiza estados anteriores
  prevState.up = up;
  prevState.down = down;
  prevState.left = left;
  prevState.right = right;
  prevState.square = square;
  prevState.circle = circle;
  prevState.cross = cross;
  prevState.triangle = triangle;
  prevState.start = start;
  prevState.select = select;

  // curtos delays não bloqueantes (Dabble precisa de chamadas continuas; delay pequeno ok)
  delay(10);
}