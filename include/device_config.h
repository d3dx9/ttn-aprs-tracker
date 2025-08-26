#pragma once
// Zentrale Geräte-spezifische Konfiguration.
// Waehle das Ziel über Build-Flag:
//  -DDEVICE_HELTEC_WIRELESS_TRACKER  (aktuelles Board mit ST7735 TFT)
//  -DDEVICE_HELTEC_LORA32_V3         (Heltec WiFi LoRa 32 V3 mit OLED)


#if defined(DEVICE_HELTEC_WIRELESS_TRACKER)
  #define DEVICE_NAME "heltec_wireless_tracker"
  #define GPS_BAUD 115200
  #define KEEP_GNSS_ON 1
  #define HAS_TFT 1
  #define HAS_OLED 0
  #ifndef LORA_CS
    #define LORA_CS 8
    #endif
    #ifndef LORA_SCK
    #define LORA_SCK 9
    #endif
    #ifndef LORA_MOSI
    #define LORA_MOSI 10
    #endif
    #ifndef LORA_MISO
    #define LORA_MISO 11
    #endif
    #ifndef LORA_RST
    #define LORA_RST 12
    #endif
    #ifndef LORA_BUSY
    #define LORA_BUSY 13
    #endif
    #ifndef LORA_DIO1
    #define LORA_DIO1 14
    #endif
  // TFT Pins (falls nicht schon gesetzt)
  #ifndef ST7735_CS_Pin
  #define ST7735_CS_Pin 38
  #endif
  #ifndef ST7735_REST_Pin
  #define ST7735_REST_Pin 39
  #endif
  #ifndef ST7735_DC_Pin
  #define ST7735_DC_Pin 40
  #endif
  #ifndef ST7735_SCLK_Pin
  #define ST7735_SCLK_Pin 41
  #endif
  #ifndef ST7735_MOSI_Pin
  #define ST7735_MOSI_Pin 42
  #endif
  #ifndef ST7735_LED_K_Pin
  #define ST7735_LED_K_Pin 21
  #endif
  #ifndef ST7735_VTFT_CTRL_Pin
  #define ST7735_VTFT_CTRL_Pin 3
  #endif
  #ifndef ST7735_MODEL
  #define ST7735_MODEL INITR_MINI160x80_PLUGIN
  #endif
  // GPS Default Pins
  // Referenz: GNSS_TX=33, GNSS_RX=34 -> Modul TX (33) -> MCU RX; Modul RX (34) -> MCU TX
  #ifndef GPS_RX_PIN
  #define GPS_RX_PIN 33
  #endif
  #ifndef GPS_TX_PIN
  #define GPS_TX_PIN 34
  #endif
  #ifndef VGNSS_CTRL
  #define VGNSS_CTRL 3      // Vorgabe aus Wunschbelegung
  #endif
  // UC6580 (UN6580) Zusatz-Pins laut Referenz-Header
  #ifndef GNSS_RST
  #define GNSS_RST 35
  #endif
  #ifndef GNSS_BOOT_MODE
  #define GNSS_BOOT_MODE 47
  #endif
  #ifndef GNSS_D_SEL
  #define GNSS_D_SEL 48
  #endif
#elif defined(DEVICE_HELTEC_LORA32_V3)
  #define DEVICE_NAME "heltec_wifi_lora_32_V3"
  // Viele BN-220/NEO-Modules laufen default bei 9600 Baud
  #define GPS_BAUD 9600
  #define HAS_TFT 0
  #define HAS_OLED 1
  // Onboard LED (Heltec WiFi LoRa 32 V3) – falls anders bitte anpassen
  #ifndef LED_BUILTIN
  #define LED_BUILTIN 35
  #endif
  // External power control (VEXT) – LOW = ON auf Heltec Boards
  #ifndef VEXT_PIN
  #define VEXT_PIN 36
  #endif
  // GPS Pins fuer BN-220 Beispiel
  #ifndef GPS_RX_PIN
  #define GPS_RX_PIN 20
  #endif
  #ifndef GPS_TX_PIN
  #define GPS_TX_PIN 19
  #endif
  // LoRa Pins laut Heltec WiFi LoRa 32 V3 (SX1262)
  #ifndef LORA_SCK
  #define LORA_SCK 9
  #endif
  #ifndef LORA_MISO
  #define LORA_MISO 11
  #endif
  #ifndef LORA_MOSI
  #define LORA_MOSI 10
  #endif
  #ifndef LORA_CS
  #define LORA_CS 8
  #endif
  #ifndef LORA_RST
  #define LORA_RST 12
  #endif
  #ifndef LORA_BUSY
  #define LORA_BUSY 13
  #endif
  #ifndef LORA_DIO1
  #define LORA_DIO1 14
  #endif
#else
  #pragma message("Kein DEVICE_* Build-Flag gesetzt – Standard: heltec_wireless_tracker")
  #define DEVICE_NAME "default"
  #define HAS_TFT 1
  #define HAS_OLED 0
#endif

// Gemeinsamer Helper: irgendein Display vorhanden
#ifndef HAS_DISPLAY
#define HAS_DISPLAY (HAS_TFT || HAS_OLED)
#endif

