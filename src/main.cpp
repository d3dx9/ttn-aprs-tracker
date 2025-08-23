#include <TinyGPSPlus.h>
// BN-220 GPS on UART1: RX=20, TX=19
#ifndef GPS_RX_PIN
#define GPS_RX_PIN 20
#endif
#ifndef GPS_TX_PIN
#define GPS_TX_PIN 19
#endif
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
RTC_DATA_ATTR int32_t last_lat_i32 = 0;
RTC_DATA_ATTR int32_t last_lon_i32 = 0;
RTC_DATA_ATTR uint16_t last_alt_u16 = 0;
RTC_DATA_ATTR bool last_pos_valid = false;

#define MINIMUM_DELAY 20 

#include <Arduino.h>
#include <stdint.h>
#include <cmath>
#include <heltec_unofficial.h>
#include <LoRaWAN_ESP32.h>
#include <RadioLib.h>  // RadioLib bands (e.g. EU868, US915)
#include <cstring>
#include <cstdlib>
#include <SSD1306Wire.h>


// LoRaWAN OTAA credentials (fill with your actual values)
// Note: Many networks still use LoRaWAN 1.0.x – in that case set NWK_KEY equal to APP_KEY.
// Region/Band: set to your deployment (default EU868 below).
static const uint64_t JOIN_EUI = 0x041613BF4582160AULL;   // a.k.a. AppEUI
static const uint64_t DEV_EUI  = 0x8EEB7EFAB1CA76BFULL;

// C1DD1C023068DCDE2B46F137F19BE0BD
static const uint8_t  APP_KEY[16] = { 
  // 21513FC22D064439077E151EFE86BC19
  0x21, 0x51, 0x3F, 0xC2, 0x2D, 0x06, 0x44, 0x39,
  0x07, 0x7E, 0x15, 0x1E, 0xFE, 0x86, 0xBC, 0x19
};
static const uint8_t  NWK_KEY[16] = { 
  // For LoRaWAN 1.0.x, set this identical to APP_KEY
  0x21, 0x51, 0x3F, 0xC2, 0x2D, 0x06, 0x44, 0x39,
  0x07, 0x7E, 0x15, 0x1E, 0xFE, 0x86, 0xBC, 0x19
};

LoRaWANNode* node;
static LoRaWANNode* makeNode() {
  // Choose band here – adjust for your region
  // EU868 by default; for US915, use &US915 and set correct subband (1..8)
  static LoRaWANNode n(&radio, &EU868, 0 /* subBand */);
  return &n;
}

RTC_DATA_ATTR uint8_t count = 0;

// OLED (Heltec V3 onboard SSD1306 via I2C)
#ifndef HELTEC_OLED_SDA
#define HELTEC_OLED_SDA 17
#endif
#ifndef HELTEC_OLED_SCL
#define HELTEC_OLED_SCL 18
#endif


static void oledShow(const char* l1, const char* l2 = "", const char* l3 = "") {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  if (l1 && *l1) display.drawString(0, 0, l1);
  if (l2 && *l2) display.drawString(0, 12, l2);
  if (l3 && *l3) display.drawString(0, 24, l3);
  display.display();
}




static void be_store_i32(uint8_t* buf, int32_t v) {
  buf[0] = (uint8_t)((v >> 24) & 0xFF);
  buf[1] = (uint8_t)((v >> 16) & 0xFF);
  buf[2] = (uint8_t)((v >> 8) & 0xFF);
  buf[3] = (uint8_t)(v & 0xFF);
}

static void be_store_u16(uint8_t* buf, uint16_t v) {
  buf[0] = (uint8_t)((v >> 8) & 0xFF);
  buf[1] = (uint8_t)(v & 0xFF);
}

void goToSleep() {
  Serial.println("Going to deep sleep now");
  persist.saveSession(node);
  uint32_t interval = node->timeUntilUplink();
  uint32_t delayMs = max(interval, (uint32_t)MINIMUM_DELAY * 1000);
  Serial.printf("Next TX in %i s\n", delayMs/1000);
  delay(100);
  heltec_deep_sleep(delayMs/1000);
}
 
void setup() {
  heltec_setup();
  // Init OLED
  display.init();
  display.clear();
  display.display();
  oledShow("TTN APRS Tracker", "Booting...", "");
  // Initialize BN-220 GPS UART
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GPSSerial.setRxBufferSize(1024);
  Serial.printf("BN-220 GPS UART started on RX=%d TX=%d at 9600 baud\n", GPS_RX_PIN, GPS_TX_PIN);

  // Obtain directly after deep sleep
  // May or may not reflect room temperature, sort of. 
  float temp = heltec_temperature();
  Serial.printf("Temperature: %.1f °C\n", temp);

  // initialize radio
  Serial.println("Radio init");
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("Radio did not initialize. We'll try again later.");
  oledShow("Radio init FAIL", "Sleep & retry", "");
    goToSleep();
  }

  // Create node for selected band and try to restore session (deep sleep)
  node = makeNode();
  bool restored = persist.loadSession(node);
  Serial.printf("loadSession: restored=%d, isActivated=%d\n", restored ? 1 : 0, node->isActivated() ? 1 : 0);
  if (!restored || !node->isActivated()) {
    // No active session: configure OTAA credentials and join
    node->setADR(false);
    state = node->beginOTAA(JOIN_EUI, DEV_EUI, NWK_KEY, APP_KEY);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("beginOTAA failed: %d\n", state);
  oledShow("Join setup FAIL", "Sleep & retry", "");
      goToSleep();
    }
    // Try a few times with a reasonable join DR (e.g., SF9/DR3 in EU868)
    int16_t joinRes = -1;
    // DR index depends on band; for EU868, DR3 ~ SF9/125kHz
    const uint8_t JOIN_DR = 3;
    for (int i = 0; i < 3; ++i) {
      joinRes = node->activateOTAA(JOIN_DR);
      if (joinRes == RADIOLIB_LORAWAN_NEW_SESSION || joinRes == RADIOLIB_LORAWAN_SESSION_RESTORED) {
        Serial.printf("activateOTAA success (%s)\n", joinRes == RADIOLIB_LORAWAN_NEW_SESSION ? "new session" : "restored session");
        oledShow("LoRaWAN: Joined", "EU868 OTAA", "");
        break;
      }
      Serial.printf("activateOTAA failed: %d (try %d/3)\n", joinRes, i+1);
      oledShow("Joining...", "retrying", "");
      delay(2000);
    }
    if (!(joinRes == RADIOLIB_LORAWAN_NEW_SESSION || joinRes == RADIOLIB_ERR_NONE)) {
      Serial.printf("activateOTAA failed: %d, will retry later.\n", joinRes);
      oledShow("Join FAIL", "Sleep & retry", "");
      goToSleep();
    }
  // Save session immediately to persist nonces and session state
  persist.saveSession(node);
  } else {
    Serial.println("Session restored; skipping join.");
  oledShow("LoRaWAN: Restored", "EU868", "");
  }

  // If we're still here, it means we joined, and we can send something

  // Manages uplink intervals to the TTN Fair Use Policy
  node->setDutyCycle(true, 1250);

  // Try to get a GPS fix from BN-220 (longer timeout on cold start)
  double lat = 0, lon = 0, alt_m = 0;
  uint32_t tStart = millis();
  bool gotFix = false;
  const uint32_t gpsTimeoutMs = (last_pos_valid ? 45000u : 600000u); // 45s warm, 600s cold
  uint32_t lastStatus = 0;
  size_t bytesSeen = 0;
  // Peek at occasional NMEA to confirm valid sentences are coming in
  char nmeaBuf[128];
  size_t nmeaPos = 0;
  uint32_t lastNmeaPrint = 0;
  uint32_t lastOledUpdate = 0;
  int gsvInView = -1;
  Serial.printf("Waiting for GPS (%s start), timeout %lus...\n", last_pos_valid ? "warm" : "cold", (unsigned)(gpsTimeoutMs/1000));
  while (millis() - tStart < gpsTimeoutMs) {
    while (GPSSerial.available()) {
      bytesSeen++;
      char c = (char)GPSSerial.read();
      gps.encode(c);
      if (c == '\n' || nmeaPos >= sizeof(nmeaBuf) - 1) {
        nmeaBuf[nmeaPos] = 0;
        if (nmeaPos > 6) {
          // Track satellites-in-view from GSV
          if (nmeaBuf[0] == '$' && strstr(nmeaBuf, "GSV")) {
            int commas = 0;
            const char* s = nmeaBuf;
            const char* start = nullptr;
            while (*s) {
              if (*s == ',') {
                commas++;
                if (commas == 3) { start = s + 1; break; }
              }
              ++s;
            }
            if (start) {
              gsvInView = atoi(start);
            }
          }
          // Print a sample GGA/RMC occasionally
          if (millis() - lastNmeaPrint > 5000) {
            if (nmeaBuf[0] == '$' && (strstr(nmeaBuf, "GGA") || strstr(nmeaBuf, "RMC"))) {
              Serial.printf("NMEA: %s\n", nmeaBuf);
              lastNmeaPrint = millis();
            }
          }
        }
        nmeaPos = 0;
      } else if (c != '\r') {
        nmeaBuf[nmeaPos++] = c;
      }
    }
    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      alt_m = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
      gotFix = true;
      break;
    }
    if (millis() - lastStatus >= 1000) {
      lastStatus = millis();
      Serial.printf("Waiting GPS... sats=%d hdop=%.1f inview=%d bytes=%u time=%02d:%02d:%02d\n",
        (int)gps.satellites.value(), gps.hdop.hdop(), gsvInView, (unsigned)bytesSeen,
        gps.time.isValid() ? gps.time.hour() : -1,
        gps.time.isValid() ? gps.time.minute() : -1,
        gps.time.isValid() ? gps.time.second() : -1);
      if (millis() - lastOledUpdate > 1000) {
        char l1[24], l2[24], l3[32];
        snprintf(l1, sizeof(l1), "GPS %s", last_pos_valid ? "warm" : "cold");
        snprintf(l2, sizeof(l2), "sats:%d in:%d", (int)gps.satellites.value(), gsvInView);
        int altDisp = -1;
        if (gps.altitude.isValid()) altDisp = (int)round(gps.altitude.meters());
        else if (last_pos_valid) altDisp = (int)last_alt_u16;
        if (altDisp >= 0) snprintf(l3, sizeof(l3), "Alt:%dm HDOP:%.1f", altDisp, gps.hdop.hdop());
        else snprintf(l3, sizeof(l3), "Alt:--  HDOP:%.1f", gps.hdop.hdop());
        oledShow(l1, l2, l3);
        lastOledUpdate = millis();
      }
    }
    delay(5);
  }

  int32_t lat_i32, lon_i32;
  uint16_t alt_u16;
  if (gotFix) {
    lat_i32 = (int32_t)round(lat * 1e5);
    lon_i32 = (int32_t)round(lon * 1e5);
    alt_u16 = (alt_m < 0) ? 0 : (uint16_t)round(alt_m);
    last_lat_i32 = lat_i32;
    last_lon_i32 = lon_i32;
    last_alt_u16 = alt_u16;
    last_pos_valid = true;
    Serial.printf("BN-220 GPS fix: lat=%.7f lon=%.7f alt=%.1f\n", lat, lon, alt_m);
  char l1[24], l2[24];
  snprintf(l1, sizeof(l1), "FIX %.5f", lat);
  snprintf(l2, sizeof(l2), "%.5f %um", lon, (unsigned)alt_u16);
  oledShow("GPS FIX", l1, l2);
  } else if (last_pos_valid) {
    lat_i32 = last_lat_i32;
    lon_i32 = last_lon_i32;
    alt_u16 = last_alt_u16;
  Serial.printf("No fresh GPS fix; using last known position. Alt=%um\n", (unsigned)alt_u16);
  char l2[24];
  snprintf(l2, sizeof(l2), "Alt:%um", (unsigned)alt_u16);
  oledShow("GPS no fix", "Using last pos", l2);
  } else {
    // Fallback: simulate position if no fix ever
    static RTC_DATA_ATTR double sim_lat = 47.6296324;
    static RTC_DATA_ATTR double sim_lon = 8.3302733;
    static RTC_DATA_ATTR double sim_alt = 340.0;
    sim_lat += 0.00010 * ((count % 3) - 1);
    sim_lon += 0.00010 * (((count+1) % 3) - 1);
    sim_alt += ((count % 2) ? 5 : -5);
    lat_i32 = (int32_t)round(sim_lat * 1e5);
    lon_i32 = (int32_t)round(sim_lon * 1e5);
    alt_u16 = (sim_alt < 0) ? 0 : (uint16_t)round(sim_alt);
  Serial.printf("Simulated position: lat=%.7f lon=%.7f alt=%.1f\n", sim_lat, sim_lon, sim_alt);
  char l2[24];
  snprintf(l2, sizeof(l2), "Alt:%um", (unsigned)alt_u16);
  oledShow("GPS no fix", "Simulated pos", l2);
  }
  Serial.printf("Bytes: %02X %02X %02X %02X  %02X %02X %02X %02X  %02X %02X\n",
    (lat_i32 >> 24) & 0xFF, (lat_i32 >> 16) & 0xFF, (lat_i32 >> 8) & 0xFF, lat_i32 & 0xFF,
    (lon_i32 >> 24) & 0xFF, (lon_i32 >> 16) & 0xFF, (lon_i32 >> 8) & 0xFF, lon_i32 & 0xFF,
    (alt_u16 >> 8) & 0xFF, alt_u16 & 0xFF);

  uint8_t uplinkData[10];
  be_store_i32(&uplinkData[0], lat_i32);
  be_store_i32(&uplinkData[4], lon_i32);
  be_store_u16(&uplinkData[8], alt_u16);

  uint8_t downlinkData[256];
  size_t lenDown = sizeof(downlinkData);

  state = node->sendReceive(uplinkData, sizeof(uplinkData), 1, downlinkData, &lenDown);

  if(state == RADIOLIB_ERR_NONE) {
    Serial.println("Message sent, no downlink received.");
  oledShow("Uplink sent", "No downlink", "");
  } else if (state > 0) {
    Serial.println("Message sent, downlink received.");
  oledShow("Uplink sent", "Downlink rx", "");
  } else {
    Serial.printf("sendReceive returned error %d, we'll try again later.\n", state);
  oledShow("Uplink error", "Sleep & retry", "");
  }

  goToSleep();    // Does not return, program starts over next round

}

void loop() {}
