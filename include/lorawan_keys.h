#pragma once
// LoRaWAN OTAA Credentials (per device set via DEVICE_* build flag)
// Trenne Keys vom Code. Optional: ersetze diese Datei lokal durch eine
// nicht eingecheckte Variante (gitignore), falls echte Produktions-Keys.

#include <stdint.h>

extern const uint64_t JOIN_EUI;   // aka AppEUI (LSB in TTN UI, hier als 64-bit Wert eingeben)
extern const uint64_t DEV_EUI;    // Device EUI
extern const uint8_t  APP_KEY[16];
extern const uint8_t  NWK_KEY[16];
