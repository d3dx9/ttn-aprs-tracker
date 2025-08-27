// Heltec-freier Tracker mit LoRaWAN, GPS, Bewegungs-/Heartbeat-Logik und Deep-Sleep
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <LoRaWAN_ESP32.h>
#include "device_config.h"
#include "lorawan_keys.h"
#include "ui.h"
#include "board.h"
#include "logging.h"

#define LOG_LORAWAN_KEYS 1
#define CHECK_PROVISIONING_MISMATCH 1
#define AUTO_REPROVISION_ON_MISMATCH 1

#if HAS_TFT
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#endif
#if HAS_OLED
#include <Wire.h>
#endif
#ifdef ARDUINO_ARCH_ESP32
#include "driver/rtc_io.h"
#include <nvs_flash.h>
#endif


#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// Optional: wie lange auf geöffneten Serial Monitor warten (ms). 0 = sofort weiter.
#ifndef SERIAL_WAIT_MS
#define SERIAL_WAIT_MS 0
#endif

// Debug-Hold: Wenn dieser Pin beim Boot LOW ist, bleibt das Gerät wach und schläft nicht ein.
#ifndef DEBUG_HOLD_PIN
#define DEBUG_HOLD_PIN 0   // GPIO0 (Boot-Taste auf vielen ESP32 Dev Boards)
#endif
static bool g_debugHold = false;


// TFT Pin/Modell-Definitionen kommen jetzt ausschließlich aus device_config.h (nur Wireless Tracker)

// TFT Objekt (nur instanziert falls HAS_TFT)
#if HAS_TFT
SPIClass SPI_TFT(FSPI);
Adafruit_ST7735 tft = Adafruit_ST7735(ST7735_CS_Pin, ST7735_DC_Pin, ST7735_REST_Pin);
#endif

// Gemeinsames Logging auf USB (Serial) und UART0 (Serial0) falls verfügbar
// Logging jetzt über logging.h
#define STAGE(n,txt) LOG_STAGE(n,txt)


// Gewünschte Sendeleistung (dBm). Hardware (SX1262) kann bis ~22 dBm.
// ACHTUNG: Regulatorische Limits (z.B. EU868 meist 16 dBm EIRP) einhalten! Antennengewinn abziehen.
#ifndef LORAWAN_TX_POWER_DBM
#define LORAWAN_TX_POWER_DBM 22
#endif

// Wake-Interval und Bewegungs-/Heartbeat-Parameter
#define WAKE_INTERVAL_SECONDS 90
#define POSITION_CHANGE_THRESHOLD_M 15.0
#define HEARTBEAT_SECONDS 600UL
#define MINIMUM_DELAY_SEC 20UL

// GPS Timeouts (warm/cold)
#define GPS_TIMEOUT_WARM_MS 300000UL
#define GPS_TIMEOUT_COLD_MS 600000UL
// Neue schnellere Fast-Fix Limits (a): wir warten nur so lange auf frischen Fix,
// danach benutzen wir ggf. den letzten gespeicherten oder simulierten.
#ifndef GPS_FAST_WARM_MS
#define GPS_FAST_WARM_MS 15000UL   // 15s bei warmem Start
#endif
#ifndef GPS_FAST_COLD_MS
#define GPS_FAST_COLD_MS 45000UL   // 45s bei kaltem Start
#endif

// LoRaWAN Provisioning Daten jetzt ausgelagert in lorawan_keys.* (per Device)

// ------------------- Zustandsvariablen (RTC) --------------------
RTC_DATA_ATTR int32_t last_lat_i32 = 0;
RTC_DATA_ATTR int32_t last_lon_i32 = 0;
RTC_DATA_ATTR uint16_t last_alt_u16 = 0;
RTC_DATA_ATTR bool last_pos_valid = false;
RTC_DATA_ATTR int32_t last_sent_lat_i32 = 0;
RTC_DATA_ATTR int32_t last_sent_lon_i32 = 0;
RTC_DATA_ATTR uint16_t last_sent_alt_u16 = 0;
RTC_DATA_ATTR bool last_sent_valid = false;
RTC_DATA_ATTR uint32_t last_uplink_epoch = 0; // sek seit Boot (coarse)
RTC_DATA_ATTR uint32_t wake_counter = 0;
// Merken ob Boot-Logo schon einmal gezeigt wurde (DeepSleep-überdauernd via RTC RAM)
RTC_DATA_ATTR bool boot_logo_shown = false;

// Optional: festen LoRaWAN DataRate (Spreading Factor) erzwingen.
// EU868 Zuordnung: DR0=SF12, DR1=SF11, DR2=SF10, DR3=SF9, DR4=SF8, DR5=SF7 (alle 125kHz)
// Standard hier: DR3 (SF9) – anpassen per Build-Flag: -DFIXED_LORAWAN_DR=5 z.B. für SF7
#ifndef FIXED_LORAWAN_DR
#define FIXED_LORAWAN_DR 3
#endif
// Schalter ob wir den festen DR erzwingen wollen.
#ifndef FORCE_FIXED_DR
#define FORCE_FIXED_DR 1
#endif

// ------------------- Objekte --------------------
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
// Ein gemeinsames Radio-Objekt (SX1262) – Board-spezifische Pin-Konfiguration via device_config.h
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI, SPISettings());
LoRaWANNode* node = nullptr;

// Hilfsfunktionen
static void be_store_i32(uint8_t* buf, int32_t v) {
  buf[0] = (uint8_t)(v >> 24); buf[1] = (uint8_t)(v >> 16); buf[2] = (uint8_t)(v >> 8); buf[3] = (uint8_t)v;
}
static void be_store_u16(uint8_t* buf, uint16_t v) {
  buf[0] = (uint8_t)(v >> 8); buf[1] = (uint8_t)v; }

static double distanceMeters(int32_t lat1_i32, int32_t lon1_i32, int32_t lat2_i32, int32_t lon2_i32) {
  if (lat1_i32 == lat2_i32 && lon1_i32 == lon2_i32) return 0.0;
  const double kDegToRad = 0.017453292519943295; // PI/180
  double lat1 = lat1_i32 / 1e5; double lon1 = lon1_i32 / 1e5; double lat2 = lat2_i32 / 1e5; double lon2 = lon2_i32 / 1e5;
  double dlat = (lat2 - lat1) * kDegToRad; double dlon = (lon2 - lon1) * kDegToRad;
  double a = sin(dlat/2)*sin(dlat/2) + cos(lat1*kDegToRad)*cos(lat2*kDegToRad)*sin(dlon/2)*sin(dlon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return 6371000.0 * c;
}

static void deepSleepSeconds(uint32_t seconds) {
  if (seconds < MINIMUM_DELAY_SEC) seconds = MINIMUM_DELAY_SEC;
  Serial.printf("Going to deep sleep for %u s\n", (unsigned)seconds);
  persist.saveSession(node);
  // GNSS NICHT ausschalten wenn KEEP_GNSS_ON gesetzt
#if defined KEEP_GNSS_ON
#if KEEP_GNSS_ON
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  Serial.println("GNSS kept ON during deep sleep");
#ifdef ARDUINO_ARCH_ESP32
  // Versuche den Pegel über Deep Sleep zu halten (RTC IO Hold)
  gpio_deep_sleep_hold_en();
  rtc_gpio_hold_en((gpio_num_t)VGNSS_CTRL);
#endif
#else
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, LOW);
  Serial.println("GNSS Power OFF");
#endif
#endif
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  Serial.flush();
  esp_deep_sleep_start();
}


// Ersetzt waitForSerial: optionales kurzes Warten damit erste Logs sichtbar werden.
static void initSerial() {
  Serial.begin(115200);
#if SERIAL_WAIT_MS > 0
  uint32_t start = millis();
  while (!Serial && (millis() - start < SERIAL_WAIT_MS)) { delay(10); }
  delay(50);
#endif
  // USB CDC auf ESP32S3 braucht oft etwas Zeit zum Enumerieren
  delay(200);
  Serial.println("[BOOT] Serial init complete");
}

void setup() {
  initSerial();
  // NVS init (for persistence flash storage)
#ifdef ARDUINO_ARCH_ESP32
  esp_err_t nvsr = nvs_flash_init();
  if (nvsr != ESP_OK) {
    Serial.printf("[NVS] init err=%d -> erase & re-init\n", (int)nvsr);
    if (nvsr == ESP_ERR_NVS_NO_FREE_PAGES || nvsr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      nvs_flash_erase();
      nvsr = nvs_flash_init();
    }
  }
  Serial.printf("[NVS] status=%d\n", (int)nvsr);
#endif
  Serial.printf("[RTC] wake_counter=%lu boot_logo_shown=%d last_sent_valid=%d\n", (unsigned long)wake_counter+1, boot_logo_shown?1:0, last_sent_valid?1:0);
  logInfo("[BOOT] Setup start");
  // Board-spezifische frühe Initialisierung (Power Rails, I2C, Reset etc.)
  boardEarlyInit();
  pinMode(DEBUG_HOLD_PIN, INPUT_PULLUP);
  g_debugHold = (digitalRead(DEBUG_HOLD_PIN) == LOW);
  if (g_debugHold) {
    Serial.println("DEBUG HOLD aktiv: kein Deep-Sleep (Pin LOW)");
  }
  pinMode(LED_BUILTIN, OUTPUT);
  // Frühes Blink zur Lebenszeichen-Ausgabe falls Serial nicht sichtbar ist
  for (int i=0;i<3;i++){ digitalWrite(LED_BUILTIN, HIGH); delay(80); digitalWrite(LED_BUILTIN, LOW); delay(80);} 
  digitalWrite(LED_BUILTIN, HIGH);
  wake_counter++;
  Serial.printf("\n==== Wake #%lu ====/\n", (unsigned long)wake_counter);

#if HAS_DISPLAY
  STAGE(4,"before_uiInit");
  boardDisplayPreUIInit();
  uiInit();
  if (!boot_logo_shown) { // Logo nur beim allerersten Boot zeigen
    uiShowBootLogo();
    boot_logo_shown = true;
  }
  STAGE(5,"after_uiInit");
#endif
  {
    String w = String("Wake #") + wake_counter;
    uiPrintLines("Tracker Boot", w.c_str());
  }
#if defined(KEEP_GNSS_ON)
  // GNSS Power
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  Serial.println("GNSS Power ON");
#endif
  // SPI + Radio
  STAGE(6,"before_SPI");
  logBoth("[DBG] Pre SPI.begin"); Serial.flush();
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  logBoth("[DBG] SPI.begin done"); Serial.flush();
  boardBeforeRadioBegin(radio);
  STAGE(2,"before_radio_begin");
  int16_t r = radio.begin(868.0); // EU868
  if (r != RADIOLIB_ERR_NONE) {
    Serial.printf("Radio init failed (%d) -> retry\n", r);
    delay(200);
    r = radio.begin(868.0);
    if (r != RADIOLIB_ERR_NONE) {
      Serial.println("Radio fail - sleep");
      deepSleepSeconds(WAKE_INTERVAL_SECONDS);
    }
  }
  logBoth("[DBG] radio.begin done");
  boardAfterRadioBegin();
  STAGE(3,"after_radio_begin");
  // Sendeleistung setzen (so früh wie möglich nach begin). Achtung: Network ADR/LinkADRReq kann später anpassen.
  int setPwr = LORAWAN_TX_POWER_DBM; if (setPwr > 22) setPwr = 22; if (setPwr < -9) setPwr = -9; // SX1262 bounds
  int16_t pwrRes = radio.setOutputPower(setPwr);
  if (pwrRes == RADIOLIB_ERR_NONE) {
    Serial.printf("TX Power set to %d dBm (req=%d)\n", setPwr, LORAWAN_TX_POWER_DBM);
  } else {
    Serial.printf("TX Power set failed (%d)\n", pwrRes);
  }
  Serial.println("Radio OK");
  uiPrintLines("Radio OK", "Session load...");

  // Provision falls nötig und manage() für automatisches Laden/Join
#ifdef FORCE_REPROVISION
  Serial.println("[LWN] FORCE_REPROVISION set -> wiping existing provisioning");
  persist.wipe();
#endif
  bool prov = persist.isProvisioned();
  if (!prov) {
    Serial.println("[LWN] Not provisioned -> provisioning from constants");
    // Speichere feste Keys einmalig in Flash
    bool pOK = persist.provision("EU868", 0, JOIN_EUI, DEV_EUI, APP_KEY, NWK_KEY);
    Serial.printf("[LWN] provision result=%d\n", pOK?1:0);
  } else {
    Serial.println("[LWN] Already provisioned");
  }

#ifdef CHECK_PROVISIONING_MISMATCH
  if (prov) {
    bool mismatch = false;
    if (persist.getJoinEUI() != JOIN_EUI) mismatch = true;
    if (persist.getDevEUI()  != DEV_EUI)  mismatch = true;
    if (memcmp(persist.getAppKey(), APP_KEY, 16) != 0) mismatch = true;
    if (memcmp(persist.getNwkKey(), NWK_KEY, 16) != 0) mismatch = true;
    if (mismatch) {
      Serial.println("[LWN] Provisioned data differs from compiled lorawan_keys.* constants");
#ifdef AUTO_REPROVISION_ON_MISMATCH
      Serial.println("[LWN] AUTO_REPROVISION_ON_MISMATCH -> wiping + reprovisioning with constants");
      persist.wipe();
      bool pOK2 = persist.provision("EU868", 0, JOIN_EUI, DEV_EUI, APP_KEY, NWK_KEY);
      Serial.printf("[LWN] reprovision result=%d\n", pOK2?1:0);
      prov = pOK2;
#else
      Serial.println("[LWN] (Define AUTO_REPROVISION_ON_MISMATCH to overwrite stored provisioning automatically.)");
#endif
    } else {
      Serial.println("[LWN] Provisioned data matches compiled constants");
    }
  }
#endif // CHECK_PROVISIONING_MISMATCH

  #ifdef LOG_LORAWAN_KEYS
    // Hilfsfunktion zum Hex-Dump mit optionaler Maskierung
    auto dumpKey = [](const uint8_t* k, const char* name) {
      char buf[3*16+1]; buf[0]='\0';
      for (int i=0;i<16;i++) {
  #ifdef LOG_LORAWAN_KEYS_FULL
        sprintf(buf + i*3, "%02X ", k[i]);
  #else
        // Maskiere mittlere Bytes (nur erste 3 und letzte 2 sichtbar)
        if (i<3 || i>=14) sprintf(buf + i*3, "%02X ", k[i]); else sprintf(buf + i*3, "** ");
  #endif
      }
      Serial.printf("[LWN] %s: %s\n", name, buf);
    };
    // Join/Dev EUI ausgeben
    auto dumpEUI = [](uint64_t eui, const char* name) {
      char line[40];
      sprintf(line, "%s: %016llX", name, (unsigned long long)eui);
      Serial.print("[LWN] "); Serial.println(line);
    };
    dumpEUI(JOIN_EUI, "JOIN_EUI");
    dumpEUI(DEV_EUI,  "DEV_EUI");
    dumpKey(APP_KEY, "APP_KEY");
    dumpKey(NWK_KEY, "NWK_KEY");
  #ifndef LOG_LORAWAN_KEYS_FULL
    Serial.println("[LWN] (Zwischen-Bytes mit ** maskiert. LOG_LORAWAN_KEYS_FULL für kompletten Dump setzen.)");
  #endif
  #endif // LOG_LORAWAN_KEYS
  node = persist.manage(&radio, false); // false = nicht automatisch joinen (wir steuern unten)
  if (!node) {
    uiPrintLines("Node alloc FAIL");
    Serial.println("[ERR] manage() returned nullptr");
    deepSleepSeconds(WAKE_INTERVAL_SECONDS);
  }
  // loadSession wurde bereits einmal von manage() aufgerufen. NICHT erneut aufrufen (bootcount!).
  Serial.printf("[LWN] post-manage activated=%d (session buffers restored=%s)\n", node->isActivated()?1:0, "unknown");
  // Falls Activation-Flag noch nicht gesetzt: ein einzelner activateOTAA() Versuch mit FIXED_LORAWAN_DR.
  // RadioLib wird dabei entweder die vorhandene Session als restored erkennen oder eine neue Join-Prozedur starten.
  if (!node->isActivated()) {
    node->setADR(false);
    Serial.printf("[LWN] activateOTAA attempt DR%d (restore-or-join)\n", (int)FIXED_LORAWAN_DR);
    uiPrintLines("Join", (String("DR")+FIXED_LORAWAN_DR).c_str());
    int16_t jr = node->activateOTAA(FIXED_LORAWAN_DR);
    Serial.printf("[LWN] activateOTAA result code=%d activated=%d\n", jr, node->isActivated()?1:0);
    if (!node->isActivated()) {
      // Kein Erfolg -> Retry mit wenigen alternativen DRs (Fallback)
      const uint8_t fallbackDRs[] = { 3,2,1,0 };
      for (uint8_t dr : fallbackDRs) {
        if (dr == FIXED_LORAWAN_DR) continue; // bereits versucht
        Serial.printf("[LWN] Fallback join DR%d\n", dr);
        uiPrintLines("Join", (String("DR")+dr).c_str());
        jr = node->activateOTAA(dr);
        Serial.printf("[LWN] result code=%d activated=%d\n", jr, node->isActivated()?1:0);
        if (node->isActivated()) break;
        delay(1400);
      }
    }
    if (!node->isActivated()) {
      Serial.println("[LWN] Join failed -> sleep");
      uiPrintLines("Join FAIL");
      deepSleepSeconds(WAKE_INTERVAL_SECONDS);
    } else {
      uiPrintLines("Join OK");
      persist.saveSession(node); // neue oder restaurierte Session sichern
    }
  }
  // Feste DR ggf. setzen
  #if FORCE_FIXED_DR
    node->setADR(false);
    int16_t drRes = node->setDatarate(FIXED_LORAWAN_DR);
    Serial.printf("[LWN] enforce DR%d res=%d\n", FIXED_LORAWAN_DR, drRes);
  #endif

  // Makro-Pflicht: DutyCycle aktivieren (FUP einhalten)
  node->setDutyCycle(true, 1250); // ca. 1% max airtime

  // GPS starten über Board-Abstraktion
  GPSSerial.setRxBufferSize(1024);
  long gpsBaud = boardGpsInit(GPSSerial);
  (void)gpsBaud;
#ifdef GNSS_RST
  pinMode(GNSS_RST, OUTPUT);
  digitalWrite(GNSS_RST, LOW); delay(5); digitalWrite(GNSS_RST, HIGH);
  Serial.println("GNSS RST Pulse");
#endif
#ifdef GNSS_PPS
  pinMode(GNSS_PPS, INPUT);
  Serial.println("GNSS PPS input configured");
#endif

  // GPS Fix suchen
  uint32_t start = millis();
  // (a) Verwende schnelle Fast-Fix Timeouts statt der langen ursprünglichen Werte
  uint32_t timeout = last_pos_valid ? GPS_FAST_WARM_MS : GPS_FAST_COLD_MS;
  bool gotFix = false;
  double lat=0, lon=0, alt=0;
  Serial.printf("GPS fast timeout %lus (%s)\n", (unsigned)timeout/1000, last_pos_valid?"warm":"cold");
  uiPrintLines("GPS wait", last_pos_valid?"warm":"cold");
  size_t nmeaCount = 0, nmeaGGA = 0, nmeaRMC = 0, nmeaGSV = 0, bytesRead = 0;
  char nmeaBuf[128]; size_t nmeaPos = 0;
  uint32_t lastNmeaPrint = 0, lastStatusPrint = 0;
  while (millis() - start < timeout) {
    while (GPSSerial.available()) {
      char c = (char)GPSSerial.read(); bytesRead++;
      gps.encode(c);
      if (c == '\n' || nmeaPos >= sizeof(nmeaBuf)-1) {
        nmeaBuf[nmeaPos] = 0;
        nmeaCount++;
        if (strstr(nmeaBuf, "GGA")) nmeaGGA++;
        if (strstr(nmeaBuf, "RMC")) nmeaRMC++;
        if (strstr(nmeaBuf, "GSV")) nmeaGSV++;
        // Print every NMEA sentence (throttle to 1/sec)
        if (millis() - lastNmeaPrint > 1000) {
          Serial.printf("NMEA: %s\n", nmeaBuf);
          lastNmeaPrint = millis();
        }
        nmeaPos = 0;
      } else if (c != '\r') {
        nmeaBuf[nmeaPos++] = c;
      }
    }
    if (gps.location.isValid()) {
      // Minimale Qualitätskriterien (locker, damit schnell gesendet werden kann)
      if (gps.satellites.value() >= 3) {
        lat = gps.location.lat(); lon = gps.location.lng();
        if (gps.altitude.isValid()) alt = gps.altitude.meters();
        gotFix = true;
        Serial.printf("GPS FIX (fast) lat=%.7f lon=%.7f alt=%.2f sats=%u hdop=%.2f\n", lat, lon, alt, (unsigned)gps.satellites.value(), gps.hdop.hdop());
        break;
      }
    }
    if (millis() - lastStatusPrint > 1000) {
      lastStatusPrint = millis();
      Serial.printf("GPS waiting: sats=%u hdop=%.2f bytes=%u NMEA=%u\n", (unsigned)gps.satellites.value(), gps.hdop.hdop(), (unsigned)bytesRead, (unsigned)nmeaCount);
      char l2[32]; snprintf(l2, sizeof(l2), "sats:%u hd:%.2f NMEA:%u", (unsigned)gps.satellites.value(), gps.hdop.hdop(), (unsigned)nmeaCount);
  uiPrintLines("GPS waiting", l2);
    }
    delay(5);
  }
  Serial.printf("GPS summary: bytes=%u NMEA=%u GGA=%u RMC=%u GSV=%u\n", (unsigned)bytesRead, (unsigned)nmeaCount, (unsigned)nmeaGGA, (unsigned)nmeaRMC, (unsigned)nmeaGSV);

  int32_t lat_i32 = 0, lon_i32 = 0; uint16_t alt_u16 = 0; bool hasPosition = false;
  if (gotFix) {
    lat_i32 = (int32_t)llround(lat * 1e5);
    lon_i32 = (int32_t)llround(lon * 1e5);
    alt_u16 = alt < 0 ? 0 : (uint16_t)llround(alt);
    if ((lat_i32==0 && lon_i32==0) && last_pos_valid) { // 0/0 Schutz
      lat_i32 = last_lat_i32; lon_i32 = last_lon_i32; alt_u16 = last_alt_u16; gotFix=false;
    }
    last_lat_i32 = lat_i32; last_lon_i32 = lon_i32; last_alt_u16 = alt_u16; last_pos_valid = true;
    Serial.printf("GPS FIX lat=%.6f lon=%.6f alt=%.1f\n", lat, lon, alt);
    char l1[24]; char l2[24]; snprintf(l1, sizeof(l1), "LAT %.5f", lat); snprintf(l2, sizeof(l2), "LON %.5f", lon);
    uiPrintLines("GPS FIX", l1, l2);
    hasPosition = true;
  } else if (last_pos_valid) {
    lat_i32 = last_lat_i32; lon_i32 = last_lon_i32; alt_u16 = last_alt_u16;
    Serial.println("No new fix -> using last position");
    uiPrintLines("GPS no new fix");
    hasPosition = true;
  } else {
    // Kein Fix und keine letzte Position -> diesmal kein Uplink
    Serial.println("No GPS fix yet (first run) -> skip uplink");
    uiPrintLines("GPS no fix", "skip uplink");
  }

  // (c) Immer senden: Standortänderung oder Heartbeat werden ignoriert.
  // Hinweis: Duty-Cycle Begrenzungen werden vom Stack gehandhabt; Send kann ggf. fehlschlagen.
  bool sendNow = hasPosition; // nur senden wenn echte oder letzte gültige Position vorhanden
  if (sendNow) {
    if (!last_sent_valid) {
      Serial.println("First uplink (forced)");
      uiPrintLines("Uplink", "first");
    } else {
      Serial.println("Force uplink (each wake)");
      uiPrintLines("Uplink", "forced");
    }
  }

  if (sendNow) {
    uint8_t payload[10];
    be_store_i32(&payload[0], lat_i32);
    be_store_i32(&payload[4], lon_i32);
  be_store_u16(&payload[8], alt_u16);
    uint8_t dlBuf[64]; size_t dlLen = sizeof(dlBuf);
  // Vor jedem Senden erneut gewünschte Leistung setzen (falls Stack oder ADR sie verändert hat)
  int setPwr2 = LORAWAN_TX_POWER_DBM; if (setPwr2 > 22) setPwr2 = 22; if (setPwr2 < -9) setPwr2 = -9;
  radio.setOutputPower(setPwr2);
  int16_t st = node->sendReceive(payload, sizeof(payload), 1, dlBuf, &dlLen);
    if (st >= 0) {
      Serial.printf("Uplink OK (state=%d, DLlen=%u)\n", st, (unsigned)dlLen);
      uiPrintLines("Uplink OK", dlLen?"DL":"no DL");
      last_sent_lat_i32 = last_lat_i32; last_sent_lon_i32 = last_lon_i32; last_sent_alt_u16 = last_alt_u16; last_sent_valid = true;
      last_uplink_epoch = millis()/1000;
      digitalWrite(LED_BUILTIN, HIGH); delay(80); digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.printf("Uplink error %d\n", st);
      {
        String s = String(st);
        uiPrintLines("Uplink ERR", s.c_str());
      }
    }
  }

  // Sleep planen (DutyCycle berücksichtigen)
  uint32_t dcMs = node ? node->timeUntilUplink() : 0;
  // (b) Fixes Wake-Intervall: Ignoriere Verlängerung durch timeUntilUplink
  uint32_t sleepSec = WAKE_INTERVAL_SECONDS;
  Serial.printf("DEBUG: timeUntilUplink=%lu ms, using fixed sleep=%u s\n", (unsigned long)dcMs, (unsigned)sleepSec);
  if (!g_debugHold) {
  Serial.printf("[RTC] Pre-sleep: wake_counter=%lu boot_logo_shown=%d node_activated=%d\n", (unsigned long)wake_counter, boot_logo_shown?1:0, node->isActivated()?1:0);
    deepSleepSeconds(sleepSec);
  } else {
    Serial.printf("Bleibe wach (Sleep wäre %u s)\n", (unsigned)sleepSec);
  uiPrintLines("Stay awake", "debug mode");
    // Einfaches Blink-Pattern zur Bestätigung
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
  }
}

void loop() {  }
