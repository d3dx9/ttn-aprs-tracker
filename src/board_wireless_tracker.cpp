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

#endif
