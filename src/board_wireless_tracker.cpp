#include <Arduino.h>
#include "device_config.h"
#include "board.h"
#if defined(DEVICE_HELTEC_WIRELESS_TRACKER)
#include <SPI.h>
#if HAS_TFT
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
// Externe Objekte, in main.cpp definiert
extern SPIClass SPI_TFT;
extern Adafruit_ST7735 tft;
#endif

void boardEarlyInit() {
  pinMode(LED_BUILTIN, OUTPUT);
  // GNSS Power & Mode Pins für UC6580 früh setzen, damit beim späteren UART-Start schon NMEA läuft
#ifdef VGNSS_CTRL
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH); // einschalten
#endif
#ifdef GNSS_BOOT_MODE
  pinMode(GNSS_BOOT_MODE, OUTPUT);
  digitalWrite(GNSS_BOOT_MODE, LOW); // Normaler Boot (anpassen falls anders nötig)
#endif
#ifdef GNSS_D_SEL
  pinMode(GNSS_D_SEL, OUTPUT);
  digitalWrite(GNSS_D_SEL, HIGH); // HIGH = NMEA (laut üblichen Belegungen, ggf. invertieren)
#endif
#ifdef GNSS_RST
  pinMode(GNSS_RST, OUTPUT);
  digitalWrite(GNSS_RST, HIGH); // RST inactive
#endif
  delay(30); // kurze Stabilisierung
}

void boardBeforeRadioBegin(SX1262 &) {
  // Could add reset pulses if needed later
}

void boardAfterRadioBegin() {}

void boardDisplayPreUIInit() {
#if HAS_TFT
  pinMode(ST7735_VTFT_CTRL_Pin, OUTPUT); digitalWrite(ST7735_VTFT_CTRL_Pin, HIGH);
  pinMode(ST7735_LED_K_Pin, OUTPUT); digitalWrite(ST7735_LED_K_Pin, HIGH);
  SPI_TFT.begin(ST7735_SCLK_Pin, -1, ST7735_MOSI_Pin, ST7735_CS_Pin);
  tft.initR(ST7735_MODEL);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(true);
  tft.setTextSize(1);
#endif
}

long boardGpsInit(HardwareSerial &serial) {
  serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("[GPS] WT RX=%d TX=%d baud=%lu (fixed)\n", GPS_RX_PIN, GPS_TX_PIN, (unsigned long)GPS_BAUD);
  // Kurzer Probe-Read
  uint32_t t0 = millis(); size_t bytes=0; size_t nonZero=0;
  while (millis() - t0 < 500) {
    while (serial.available()) { uint8_t c = serial.read(); bytes++; if (c>0 && c<128) nonZero++; if (bytes>200) break; }
    if (bytes>200) break; delay(5);
  }
  Serial.printf("[GPS] initial bytes=%u printable=%u\n", (unsigned)bytes, (unsigned)nonZero);
  if (bytes == 0) {
    Serial.println("[GPS] no bytes -> send wake/config hint");
    const char *cfg = "$CFGSYS,h35155*68\r\n";
    serial.write((const uint8_t*)cfg, strlen(cfg));
    delay(120);
  }
  return (long)GPS_BAUD;
}

#endif
