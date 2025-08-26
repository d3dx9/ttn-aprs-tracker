// SPDX-License-Identifier: MIT
// Minimal variant pin mapping for Heltec Wireless Tracker (ESP32-S3)
// NOTE: Verify pins against latest Heltec schematic; adjust TODO items.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifndef ARDUINO_BOARD
#define ARDUINO_BOARD "Wireless_Track"
#endif

#define BOARD_HAS_PSRAM

// UART0 (USB serial) – default console
#define PIN_SERIAL0_RX 44
#define PIN_SERIAL0_TX 43

// GPS (dedicated UART1)  (TODO: confirm pins)
#define PIN_GPS_RX 33
#define PIN_GPS_TX 34

// I2C (OLED + sensors / fuel gauge)
#define PIN_WIRE_SDA 17
#define PIN_WIRE_SCL 18
#define SDA PIN_WIRE_SDA
#define SCL PIN_WIRE_SCL

// SPI for SX1262 LoRa
#define PIN_SPI_MOSI 10
#define PIN_SPI_MISO 11
#define PIN_SPI_SCK  9
// Verwende KEINE Macros für MOSI/MISO/SCK um Konflikte mit Library-Parameter-Namen zu vermeiden
static const uint8_t MOSI = PIN_SPI_MOSI; // standard alias
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// SX1262 control pins (Heltec Wireless Tracker)
#define PIN_LORA_CS    8   // NSS
#define PIN_LORA_RST   12  // RST
#define PIN_LORA_BUSY  13  // BUSY (SX1262)
#define PIN_LORA_DIO1  14  // DIO1 (IRQ)
// Older Semtech API compatibility (RadioLib may look for these) – map if needed
#define LORA_CS   PIN_LORA_CS
#define LORA_RST  PIN_LORA_RST
#define LORA_BUSY PIN_LORA_BUSY
#define LORA_DIO1 PIN_LORA_DIO1

// Optional RF switch (TX/RX enable) – TODO: fill real pins if exposed
//#define PIN_LORA_RXEN  XXX
//#define PIN_LORA_TXEN  XXX

// OLED reset (if present)
#define OLED_SDA PIN_WIRE_SDA
#define OLED_SCL PIN_WIRE_SCL
#define OLED_RST 21

// Buttons
#define PIN_BUTTON1 0      // BOOT button
#define PIN_BUTTON2 46     // User button (if available)

// LED
#define PIN_LED      35
#define LED_BUILTIN  PIN_LED

// Battery / Power control (placeholders – confirm ADC channel & enable pin)
#define PIN_BAT_ADC     1   // TODO: actual VBAT sense GPIO
#define PIN_VEXT_ENABLE 20  // TODO: confirm

// USB (native) – usually handled by core; listed for reference
#define PIN_USB_DM 19
#define PIN_USB_DP 20

// Do not define NUM_DIGITAL_PINS or NUM_ANALOG_INPUTS here; core supplies them.

// Simple analog aliases (example subset – adjust as required)
#define A0 1
#define A1 2
#define A2 3
#define A3 4
#define A4 5
#define A5 6

// Primary SPI SS alias expected by some libs
#define SS  PIN_LORA_CS

#ifdef __cplusplus
extern "C" void initVariant(); // Optional variant init hook
#endif
