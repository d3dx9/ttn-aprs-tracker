#include <Arduino.h>
#include "device_config.h"
#include "logging.h"

#if HAS_TFT
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
extern SPIClass SPI_TFT; // defined in main.cpp or elsewhere
// We rely on main.cpp creating the tft object; we just extern it here.
extern Adafruit_ST7735 tft;
static bool s_uiReady = false;
static bool s_logoShown = false;
void uiInit() {
  s_uiReady = true; // main.cpp initialisiert bereits
}
void uiPrintLines(const char* l1, const char* l2, const char* l3, const char* l4) {
  if (!s_uiReady) return;
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,0);
  tft.setTextColor(ST77XX_WHITE);
  if (l1 && *l1) tft.println(l1);
  if (l2 && *l2) tft.println(l2);
  if (l3 && *l3) tft.println(l3);
  if (l4 && *l4) tft.println(l4);
}
void uiShowBootLogo() {
  if (!s_uiReady || s_logoShown) return;
  s_logoShown = true;
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(10,10);
  tft.setTextSize(2);
  tft.println("APRS");
  tft.setTextSize(1);
  tft.setCursor(10,40);
  tft.println("Tracker Boot");
  delay(600);
}
#elif HAS_OLED
#include <Wire.h>
#include <SSD1306Wire.h>
#ifndef OLED_SDA
#define OLED_SDA 17
#endif
#ifndef OLED_SCL
#define OLED_SCL 18
#endif
static SSD1306Wire display(0x3C, OLED_SDA, OLED_SCL);

static bool s_uiReady = false;
static bool s_oledOk = false;
static bool s_logoShown = false;

static bool oledPing(uint8_t addr = 0x3C) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}



static void oledShowLogo() {
  if(!(s_uiReady && s_oledOk) || s_logoShown) return;
  s_logoShown = true;
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64,8,"APRS");
  display.setFont(ArialMT_Plain_10);
  display.drawString(64,34,"Tracker Boot");
  display.display();
  delay(600);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}
void uiInit() {
  // Kurzer Ping bevor wir initialisieren (verhindert Hänger falls Bus/Device fehlt)
  uint8_t addr = 0x3C;
  if(!oledPing(addr)) {
    delay(50); // retry after delay
    if(!oledPing(addr)) {
      // try alternate 0x3D
  if(oledPing(0x3D)) { addr = 0x3D; logInfo("[OLED] using alt addr 0x3D"); }
    }
  }
  if(!oledPing(addr)) {
  logInfo("[OLED] no ACK (0x3C/0x3D) -> skip");
    s_uiReady = false; s_oledOk = false; return;
  }
  if(addr != 0x3C) {
    // Recreate display object is complex; note only 0x3C supported by current instance
  logInfo("[OLED] Detected 0x3D but driver fixed at 0x3C -> needs code change");
  } else {
  logInfo("[OLED] addr 0x3C detected -> init...");
  }
  display.init();
  display.displayOn();
  display.clear();
  display.display();
  s_uiReady = true; s_oledOk = true;
  logInfo("[OLED] init done");
  oledShowLogo();
}
void uiPrintLines(const char* l1, const char* l2, const char* l3, const char* l4) {
  if(!(s_uiReady && s_oledOk)) {
    // Fallback Serial
    if (l1 && *l1) Serial.println(l1);
    if (l2 && *l2) Serial.println(l2);
    if (l3 && *l3) Serial.println(l3);
    if (l4 && *l4) Serial.println(l4);
    return;
  }
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  if (l1 && *l1) display.drawString(0,0,l1);
  if (l2 && *l2) display.drawString(0,12,l2);
  if (l3 && *l3) display.drawString(0,24,l3);
  if (l4 && *l4) display.drawString(0,36,l4);
  display.display();
}
void uiShowBootLogo() { oledShowLogo(); }
#else
void uiInit() {}
void uiPrintLines(const char* l1, const char* l2, const char* l3, const char* l4) {
  // Fallback: Serial
  if (l1 && *l1) Serial.println(l1);
  if (l2 && *l2) Serial.println(l2);
  if (l3 && *l3) Serial.println(l3);
  if (l4 && *l4) Serial.println(l4);
}
void uiShowBootLogo() {}
#endif
