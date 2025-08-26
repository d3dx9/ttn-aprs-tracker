#include <Arduino.h>
#include "device_config.h"
#include "board.h"
#if defined(DEVICE_HELTEC_LORA32_V3)
#include <Wire.h>
#include <RadioLib.h>

// VEXT helper (LOW=on, high-Z=off)
static void heltecVext(bool on) {
  if (on) { pinMode(VEXT_PIN, OUTPUT); digitalWrite(VEXT_PIN, LOW); }
  else { pinMode(VEXT_PIN, INPUT); }
}

void boardEarlyInit() {
  heltecVext(true);
  delay(80);
  // Reset OLED
  #ifdef RST_OLED
    pinMode(RST_OLED, OUTPUT);
    digitalWrite(RST_OLED, HIGH); delay(2);
    digitalWrite(RST_OLED, LOW);  delay(8);
    digitalWrite(RST_OLED, HIGH); delay(8);
  #else
    pinMode(21, OUTPUT); digitalWrite(21, HIGH);
  #endif
  // Start I2C (SDA=17,SCL=18) – no manual pinMode afterwards!
  Wire.begin(17,18);
  Wire.setClock(400000);
  // Quick scan once for debug
  uint8_t found=0; for(uint8_t a=0x3C; a<=0x3D; ++a){ Wire.beginTransmission(a); if(Wire.endTransmission()==0){found=a; break;} }
  if(found) Serial.printf("[HELTEC_V3] OLED addr 0x%02X detected\n", found); else Serial.println("[HELTEC_V3] OLED not detected (yet)");
  // LED blink pattern
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0;i<3;i++){ digitalWrite(LED_BUILTIN,HIGH); delay(60); digitalWrite(LED_BUILTIN,LOW); delay(60);}  
}

void boardBeforeRadioBegin(SX1262 &radio) {
  // Ensure radio reset pulse
  #ifdef LORA_RST
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, HIGH); delay(2);
    digitalWrite(LORA_RST, LOW);  delay(6);
    digitalWrite(LORA_RST, HIGH); delay(8);
  #endif
  // Chip select idle high
  #ifdef LORA_CS
    pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  #endif
}

void boardAfterRadioBegin() {
  Serial.println("[HELTEC_V3] Radio init done");
}

void boardDisplayPreUIInit() {
  // OLED braucht hier nichts; uiInit erledigt Init
}

#endif // DEVICE_HELTEC_LORA32_V3
