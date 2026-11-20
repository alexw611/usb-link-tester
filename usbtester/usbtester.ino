/*
  ESP32 USB-Strecken-Tester  (Gegenstelle zu usblinktester.py)

  Board:  ESP32 Dev Module (ESP32-WROOM), Arduino-ESP32 Core >= 2.0.14 oder 3.x
  Tools -> Core Debug Level: "None"   (sonst stoeren Log-Ausgaben den Test)
  Serial Monitor in der Arduino IDE muss GESCHLOSSEN sein!

  Ablauf:
   - Start mit 115200 Baud, wartet auf Kommandos vom PC (Textzeilen):
       P        -> antwortet "PONG"
       B:<baud> -> wechselt die Baudrate (antwortet vorher "OK B")
       G:<mode> -> Test starten (1 = ESP sendet Vollgas, 2 = PC sendet Vollgas, 3 = beides)
       X        -> Test stoppen
       K        -> Keepalive
   - Datenframes (256 Byte, beide Richtungen):
       AA 55 | seq u32 | gut u32 | verloren u32 | 240 Byte Payload | CRC16-CCITT
     "gut"/"verloren" = Zaehler des ESP fuer Frames, die er vom PC empfangen hat.
   - Kommt 3 s lang nichts Gueltiges vom PC, faellt der ESP auf 115200 Baud zurueck.
*/

#include <Arduino.h>

constexpr uint32_t BAUD_BASE          = 115200;
constexpr size_t   FRAME_LEN          = 256;
constexpr size_t   PAYLOAD_LEN        = 240;
constexpr uint8_t  SYNC0              = 0xAA;
constexpr uint8_t  SYNC1              = 0x55;
constexpr uint32_t REVERT_MS          = 3000;
constexpr uint32_t STATUS_INTERVAL_MS = 100;   // Statusframe, wenn ESP nicht "Vollgas" sendet

uint32_t curBaud   = BAUD_BASE;
uint8_t  mode      = 0;
bool     streaming = false;

uint32_t txSeq = 0;
uint32_t rxGood = 0, rxLost = 0, rxExpected = 0;
bool     rxFirst = true;

uint32_t lastValidMs  = 0;
uint32_t lastStatusMs = 0;

uint8_t  rxBuf[FRAME_LEN];
size_t   rxLen = 0;
uint8_t  txBuf[FRAME_LEN];
char     lineBuf[32];
uint8_t  lineLen = 0;
uint16_t crcTab[256];

void initCrc() {
  for (uint16_t i = 0; i < 256; i++) {
    uint16_t c = i << 8;
    for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    crcTab[i] = c;
  }
}

uint16_t crc16(const uint8_t *p, size_t len) {
  uint16_t c = 0xFFFF;
  while (len--) c = (uint16_t)((c << 8) ^ crcTab[((c >> 8) ^ *p++) & 0xFF]);
  return c;
}

void startSerial(uint32_t baud) {
  Serial.end();
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(baud);
  curBaud = baud;
}

void sendFrame() {
  txBuf[0] = SYNC0;
  txBuf[1] = SYNC1;
  uint32_t seq = txSeq++;
  memcpy(&txBuf[2],  &seq,    4);
  memcpy(&txBuf[6],  &rxGood, 4);
  memcpy(&txBuf[10], &rxLost, 4);
  uint32_t s = seq * 7;
  for (size_t i = 0; i < PAYLOAD_LEN; i++) txBuf[14 + i] = (uint8_t)(s + i * 13);
  uint16_t crc = crc16(&txBuf[2], FRAME_LEN - 4);
  txBuf[254] = crc & 0xFF;
  txBuf[255] = crc >> 8;
  Serial.write(txBuf, FRAME_LEN);
}

void handleLine() {
  lineBuf[lineLen] = 0;
  if (strcmp(lineBuf, "P") == 0) {
    Serial.print("PONG\n");
    lastValidMs = millis();
  } else if (strcmp(lineBuf, "X") == 0) {
    streaming = false;
    Serial.print("OK X\n");
    lastValidMs = millis();
  } else if (strcmp(lineBuf, "K") == 0) {
    lastValidMs = millis();
  } else if (strncmp(lineBuf, "B:", 2) == 0) {
    uint32_t b = strtoul(lineBuf + 2, nullptr, 10);
    if (b >= 9600 && b <= 5000000) {
      streaming = false;
      Serial.print("OK B\n");
      Serial.flush();
      delay(30);
      startSerial(b);
      lastValidMs = millis();
    }
  } else if (strncmp(lineBuf, "G:", 2) == 0) {
    mode = atoi(lineBuf + 2);
    rxGood = rxLost = 0;
    rxFirst = true;
    txSeq = 0;
    Serial.print("OK G\n");
    streaming = true;
    lastStatusMs = millis();
    lastValidMs = millis();
  }
}

void textByte(uint8_t b) {
  if (b == '\n') {
    handleLine();
    lineLen = 0;
  } else if (b >= 32 && b < 127) {
    if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = (char)b;
  }
}

void shift1() {
  rxLen--;
  memmove(rxBuf, rxBuf + 1, rxLen);
}

void processFrame() {
  uint32_t seq;
  memcpy(&seq, &rxBuf[2], 4);
  lastValidMs = millis();
  if (rxFirst) {
    rxFirst = false;
    rxExpected = seq;
  }
  if (seq >= rxExpected) {
    rxLost += seq - rxExpected;
    rxExpected = seq + 1;
    rxGood++;
  }
}

void feedByte(uint8_t b) {
  rxBuf[rxLen++] = b;
  while (rxLen > 0) {
    if (rxBuf[0] != SYNC0) { textByte(rxBuf[0]); shift1(); continue; }
    if (rxLen >= 2 && rxBuf[1] != SYNC1) { shift1(); continue; }
    if (rxLen < FRAME_LEN) break;
    uint16_t crcRx = rxBuf[254] | (rxBuf[255] << 8);
    if (crc16(&rxBuf[2], FRAME_LEN - 4) == crcRx) {
      processFrame();
      rxLen = 0;
    } else {
      shift1();   // Resync: ein Byte verwerfen und weitersuchen
    }
  }
}

void setup() {
  initCrc();
  startSerial(BAUD_BASE);
  lastValidMs = millis();
}

void loop() {
  uint8_t tmp[256];
  int n = Serial.available();
  if (n > 0) {
    n = Serial.readBytes(tmp, n > 256 ? 256 : n);
    for (int i = 0; i < n; i++) feedByte(tmp[i]);
  }

  uint32_t now = millis();

  if (curBaud != BAUD_BASE && now - lastValidMs > REVERT_MS) {
    streaming = false;
    startSerial(BAUD_BASE);
    lastValidMs = now;
  }

  if (streaming) {
    if (mode & 1) {
      for (int i = 0; i < 8 && Serial.availableForWrite() >= (int)FRAME_LEN; i++) sendFrame();
    } else if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
      lastStatusMs = now;
      if (Serial.availableForWrite() >= (int)FRAME_LEN) sendFrame();
    }
  }
}