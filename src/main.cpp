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

// Last sent position (to compare movement)
RTC_DATA_ATTR int32_t last_sent_lat_i32 = 0;
RTC_DATA_ATTR int32_t last_sent_lon_i32 = 0;
RTC_DATA_ATTR uint16_t last_sent_alt_u16 = 0;
RTC_DATA_ATTR bool last_sent_valid = false;

#define MINIMUM_DELAY 20 
// Wake every 2 minutes (adjust to 60 for faster updates if needed)
#define WAKE_INTERVAL_SECONDS 90
// Min distance movement required for a new uplink (meters)
#define POSITION_CHANGE_THRESHOLD_M 15.0
// GPS acquisition time limits to avoid 10 min blocking (was 600000 ms for cold start)
#define GPS_TIMEOUT_WARM_MS 30000UL   // 30s when we had a recent fix
#define GPS_TIMEOUT_COLD_MS 60000UL   // 60s on cold start (instead of 600s) so we wake again sooner

#include <Arduino.h>
#include <stdint.h>
#include <cmath>
#include <heltec_unofficial.h>
#include <LoRaWAN_ESP32.h>
#include <RadioLib.h>  // RadioLib bands (e.g. EU868, US915)
#include <cstring>
#include <cstdlib>
#include <SSD1306Wire.h>

// Faster GPS timeouts for higher reporting cadence
#ifndef GPS_TIMEOUT_WARM_MS
#define GPS_TIMEOUT_WARM_MS 20000UL
#endif
#ifndef GPS_TIMEOUT_COLD_MS
#define GPS_TIMEOUT_COLD_MS 40000UL
#endif


// LoRaWAN OTAA credentials (fill with your actual values)
// Note: Many networks still use LoRaWAN 1.0.x – in that case set NWK_KEY equal to APP_KEY.
// Region/Band: set to your deployment (default EU868 below).
static const uint64_t JOIN_EUI = 0x6EA284F73C6F34DAULL;   // a.k.a. AppEUI
static const uint64_t DEV_EUI  = 0xA47057B5E0D0C582ULL;

// C1DD1C023068DCDE2B46F137F19BE0BD
static const uint8_t  APP_KEY[16] = { 
  // 33E407A1425F053D2D83173A0D814863
  0x33, 0xE4, 0x07, 0xA1, 0x42, 0x5F, 0x05, 0x3D,
  0x2D, 0x83, 0x17, 0x3A, 0x0D, 0x81, 0x48, 0x63
};
static const uint8_t  NWK_KEY[16] = { 
  // For LoRaWAN 1.0.x, set this identical to APP_KEY
  0x33, 0xE4, 0x07, 0xA1, 0x42, 0x5F, 0x05, 0x3D,
  0x2D, 0x83, 0x17, 0x3A, 0x0D, 0x81, 0x48, 0x63
};

LoRaWANNode* node;
static LoRaWANNode* makeNode() {
  // Choose band here – adjust for your region
  // EU868 by default; for US915, use &US915 and set correct subband (1..8)
  static LoRaWANNode n(&radio, &EU868, 0 /* subBand */);
  return &n;
}

RTC_DATA_ATTR uint8_t count = 0;
// Keep track of last uplink (epoch seconds) to enforce a max silent period
RTC_DATA_ATTR uint32_t last_uplink_epoch = 0;

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

// Haversine distance between two 1e5 scaled lat/lon pairs
static double distanceMeters(int32_t lat1_i32, int32_t lon1_i32, int32_t lat2_i32, int32_t lon2_i32) {
  if (lat1_i32 == lat2_i32 && lon1_i32 == lon2_i32) return 0.0;
  const double kDegToRad = 0.017453292519943295; // PI/180
  double lat1 = lat1_i32 / 1e5;
  double lon1 = lon1_i32 / 1e5;
  double lat2 = lat2_i32 / 1e5;
  double lon2 = lon2_i32 / 1e5;
  double dlat = (lat2 - lat1) * kDegToRad;
  double dlon = (lon2 - lon1) * kDegToRad;
  double a = sin(dlat/2)*sin(dlat/2) + cos(lat1*kDegToRad)*cos(lat2*kDegToRad)*sin(dlon/2)*sin(dlon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return 6371000.0 * c;
}

static void goToSleepSeconds(uint32_t seconds) {
  Serial.println("Going to deep sleep now");
  persist.saveSession(node);
  uint32_t dcMs = node ? node->timeUntilUplink() : 0;
  uint32_t finalSec = max((uint32_t)MINIMUM_DELAY, max(seconds, dcMs/1000));
  Serial.printf("Next wake in %u s (dutySuggest=%u ms)\n", (unsigned)finalSec, (unsigned)dcMs);
  delay(100);
  heltec_deep_sleep(finalSec);
}

void goToSleep() { goToSleepSeconds(WAKE_INTERVAL_SECONDS); }
 
void setup() {
  heltec_setup();
  heltec_ve(true);
  // Init OLED
  display.init();
  display.clear();
  display.display();
  oledShow("TTN APRS Tracker", "Booting...", "");
  // Initialize BN-220 GPS UART
  // Set RX buffer size BEFORE begin() (esp32 HardwareSerial limitation)
  GPSSerial.setRxBufferSize(1024);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("BN-220 GPS UART started on RX=%d TX=%d at 9600 baud (rxbuf=1024)\n", GPS_RX_PIN, GPS_TX_PIN);

  // Obtain directly after deep sleep
  // May or may not reflect room temperature, sort of. 
  float temp = heltec_temperature();
  Serial.printf("Temperature: %.1f °C\n", temp);

  // initialize radio
  Serial.println("Radio init");
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("Radio init failed, retrying...");
    delay(200);
    state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
      Serial.println("Radio failed again, sleeping.");
      oledShow("Radio FAIL", "Sleep & retry", "");
      goToSleep();
    } else {
      oledShow("Radio OK 2nd", "", "");
    }
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
    // Multi-DR join attempts: start mid (DR3 ~ SF9) then go slower for reach, then faster
    const uint8_t joinDRs[] = {3, 2, 1, 0}; // EU868: DR3=SF9 .. DR0=SF12
    int16_t joinRes = -1;
    bool joined = false;
    for (size_t d = 0; d < sizeof(joinDRs); ++d) {
      uint8_t dr = joinDRs[d];
      for (int attempt = 1; attempt <= 2; ++attempt) { // two tries per DR
        Serial.printf("OTAA join attempt DR%d try %d/2...\n", dr, attempt);
        oledShow("Joining", (String("DR") + dr).c_str(), (String("try ") + attempt).c_str());
        joinRes = node->activateOTAA(dr);
        if (joinRes == RADIOLIB_LORAWAN_NEW_SESSION || joinRes == RADIOLIB_LORAWAN_SESSION_RESTORED) {
          Serial.printf("activateOTAA success (%s) on DR%d\n", joinRes == RADIOLIB_LORAWAN_NEW_SESSION ? "new session" : "restored session", dr);
          oledShow("LoRaWAN: Joined", (String("DR") + dr).c_str(), "EU868");
          joined = true;
          break;
        }
        Serial.printf("activateOTAA failed: %d on DR%d (attempt %d)\n", joinRes, dr, attempt);
        delay(1800); // small pause between attempts
      }
      if (joined) break;
    }
    if (!joined) {
      Serial.printf("All OTAA attempts failed, last error=%d. Suggest: check keys/endian, remove device in TTN to reset DevNonce list. Sleeping.\n", joinRes);
      oledShow("Join FAIL", "Check keys", String(joinRes).c_str());
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
  const uint32_t gpsTimeoutMs = (last_pos_valid ? GPS_TIMEOUT_WARM_MS : GPS_TIMEOUT_COLD_MS);
  Serial.printf("GPS timeout this wake: %lus (%s)\n", (unsigned)(gpsTimeoutMs/1000), last_pos_valid?"warm":"cold");
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
    if (lat_i32 == 0 && lon_i32 == 0 && last_pos_valid) { // discard implausible 0/0
      Serial.println("Discarding (0,0) fix; using last position");
      lat_i32 = last_lat_i32;
      lon_i32 = last_lon_i32;
      alt_u16 = last_alt_u16;
      gotFix = false; // treat as not a new fix
    }
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

  // Decide whether to send based on movement threshold
  bool sendNow = false;
  double moveDist = 0.0;
  if (!last_sent_valid) {
    sendNow = true; // first ever
    Serial.println("No previous sent pos -> will send");
  } else {
    moveDist = distanceMeters(last_sent_lat_i32, last_sent_lon_i32, lat_i32, lon_i32);
  Serial.printf("Distance since last uplink: %.2fm (threshold %.2fm)\n", moveDist, (double)POSITION_CHANGE_THRESHOLD_M);
    if (moveDist >= POSITION_CHANGE_THRESHOLD_M) {
      sendNow = true;
      Serial.printf("Moved %.1fm >= %.1fm -> send\n", moveDist, POSITION_CHANGE_THRESHOLD_M);
    } else {
      Serial.printf("Moved only %.1fm (<%.1fm) -> skip uplink\n", moveDist, POSITION_CHANGE_THRESHOLD_M);
    }
  }

  if (sendNow) {
    uint8_t uplinkData[10];
    be_store_i32(&uplinkData[0], lat_i32);
    be_store_i32(&uplinkData[4], lon_i32);
    be_store_u16(&uplinkData[8], alt_u16);
    uint8_t downlinkData[256];
    size_t lenDown = sizeof(downlinkData);
    state = node->sendReceive(uplinkData, sizeof(uplinkData), 1, downlinkData, &lenDown);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("Uplink OK (no DL)");
      oledShow("Uplink OK", "No DL", "");
      last_sent_lat_i32 = lat_i32;
      last_sent_lon_i32 = lon_i32;
      last_sent_alt_u16 = alt_u16;
      last_sent_valid = true;
    } else if (state > 0) {
      Serial.println("Uplink OK (DL)");
      oledShow("Uplink OK", "DL recv", "");
      last_sent_lat_i32 = lat_i32;
      last_sent_lon_i32 = lon_i32;
      last_sent_alt_u16 = alt_u16;
      last_sent_valid = true;
    } else {
      Serial.printf("Uplink error %d\n", state);
      oledShow("Uplink ERR", String(state).c_str(), "");
    }
  } else {
    // Enforce a periodic heartbeat (e.g. every 10 min) even without movement
    uint32_t nowEpoch = millis()/1000; // coarse (resets after deep sleep but acceptable for spacing)
    const uint32_t HEARTBEAT_SEC = 600; // 10 minutes
    if (!last_sent_valid || (nowEpoch - last_uplink_epoch) >= HEARTBEAT_SEC) {
      Serial.println("Heartbeat uplink (no movement but max interval reached)");
      sendNow = true; // fall through to send path by re-running logic
    } else {
      char l1[24], l2[24], l3[24];
      snprintf(l1, sizeof(l1), "No move <%.0fm", POSITION_CHANGE_THRESHOLD_M);
      snprintf(l2, sizeof(l2), "d=%.1fm", moveDist);
      snprintf(l3, sizeof(l3), "Sleep %us", WAKE_INTERVAL_SECONDS);
      oledShow(l1, l2, l3);
    }
  }


  goToSleep();

}

void loop() {}
