#pragma once
#include <stdint.h>
class SX1262; // forward from RadioLib

// Board abstraction to keep main.cpp cleaner.
// Provide empty implementations for unsupported functions on other boards.

// Early hardware bring-up before Radio init (power rails, I2C, display reset, debug blink)
void boardEarlyInit();
void boardBeforeRadioBegin(SX1262 &radio);
void boardAfterRadioBegin();
// Optionale Display-Initialisierung vor uiInit (Board-spezifisch)
void boardDisplayPreUIInit();
// Board-spezifische GPS/UART Initialisierung + evtl. Auto-Baud Erkennung.
// Rueckgabe: tatsächlich gesetzte Baudrate.
long boardGpsInit(class HardwareSerial &serial);

#if !defined(DEVICE_HELTEC_LORA32_V3) && !defined(DEVICE_HELTEC_WIRELESS_TRACKER)
// Provide inline no-op defaults if no known board macro set
inline void boardEarlyInit() {}
inline void boardBeforeRadioBegin(SX1262 &) {}
inline void boardAfterRadioBegin() {}
inline void boardDisplayPreUIInit() {}
inline long boardGpsInit(class HardwareSerial &serial) { (void)serial; serial.begin(9600); return 9600; }
#endif
