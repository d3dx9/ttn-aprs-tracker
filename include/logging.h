#pragma once
#include <Arduino.h>

// Vereinheitlichte Logging-Helfer: logInfo / logError / logDebug
// Primär immer auf Serial. Zweiter Port optional: define LOG_SECONDARY_PORT (z.B. Serial1) und LOG_ENABLE_SECONDARY 1.

#ifndef LOG_ENABLE_SECONDARY
#define LOG_ENABLE_SECONDARY 0
#endif

#if LOG_ENABLE_SECONDARY && defined(LOG_SECONDARY_PORT)
  #define LOG_WRITE_SECONDARY(x) do { LOG_SECONDARY_PORT.println(x); } while(0)
  #define LOG_WRITE_SECONDARY_PFX(pfx,msg) do { LOG_SECONDARY_PORT.print(pfx); LOG_SECONDARY_PORT.println(msg); } while(0)
#else
  #define LOG_WRITE_SECONDARY(x) do {} while(0)
  #define LOG_WRITE_SECONDARY_PFX(pfx,msg) do {} while(0)
#endif

inline void logBothRaw(const __FlashStringHelper* f) { Serial.println(f); LOG_WRITE_SECONDARY(f); }
inline void logBoth(const char* msg) { Serial.println(msg); LOG_WRITE_SECONDARY(msg); }
inline void logInfo(const char* msg) { logBoth(msg); }
inline void logError(const char* msg) { Serial.print(F("[ERR] ")); Serial.println(msg); LOG_WRITE_SECONDARY_PFX(F("[ERR] "), msg); }
inline void logDebug(const char* msg) { Serial.print(F("[DBG] ")); Serial.println(msg); LOG_WRITE_SECONDARY_PFX(F("[DBG] "), msg); }

// STAGE Makro zentral – LED-Blink zur visuellen Sequenzanzeige
#ifndef LOG_STAGE_LED_PIN
#define LOG_STAGE_LED_PIN LED_BUILTIN
#endif

#if defined(DEVICE_HELTEC_LORA32_V3)
  // V3: kein (oder instabiles) USB CDC -> nutze Standard Serial (UART0)
  #define LOG_STAGE_PORT Serial
#elif defined(DEVICE_HELTEC_WIRELESS_TRACKER)
  // Wireless Tracker: ebenfalls Serial (USB CDC oder UART je nach Core-Einstellung)
  #define LOG_STAGE_PORT Serial
#else
  #define LOG_STAGE_PORT Serial
#endif

#define LOG_STAGE(n, txt) do { \
  LOG_STAGE_PORT.print(F("[STAGE]")); LOG_STAGE_PORT.print(n); LOG_STAGE_PORT.print(F(" ")); LOG_STAGE_PORT.println(F(txt)); \
  for (int _b=0; _b<(n); ++_b) { digitalWrite(LOG_STAGE_LED_PIN, HIGH); delay(120); digitalWrite(LOG_STAGE_LED_PIN, LOW); delay(180);} \
} while(0)

