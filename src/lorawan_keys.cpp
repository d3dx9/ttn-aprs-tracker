#include "device_config.h"
#include "lorawan_keys.h"

// Default placeholder Keys – austauschen durch reale.
// Hinweis: TTN zeigt AppEUI/JoinEUI und DevEUI meist als MSB-first Hex an.
// Hier verwenden wir denselben Hexwert als 64-bit Literal (ULL). Prüfe Endianness.

// Beispiel-Keys fuer zweites Device (ersetzen durch echte Werte)
const uint64_t JOIN_EUI = 0x5497BB7E4505D4E8ULL;
const uint64_t DEV_EUI  = 0x5C0DB189A8CC2B7A;
// 7E83616A9625D9A0D1F9BAF2AB8F955A
const uint8_t  APP_KEY[16] = {
0x7E, 0x83, 0x61, 0x6A, 0x96, 0x25, 0xD9, 0xA0, 0xD1, 0xF9, 0xBA, 0xF2, 0xAB, 0x8F, 0x95, 0x5A
};
const uint8_t  NWK_KEY[16] = {
0x7E, 0x83, 0x61, 0x6A, 0x96, 0x25, 0xD9, 0xA0, 0xD1, 0xF9, 0xBA, 0xF2, 0xAB, 0x8F, 0x95, 0x5A
};
