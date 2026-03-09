#include <Arduino.h>
#include "shared_data.h"
#include "sd_manager.h"
#include "rgb_controller.h"
#include "web_server.h"
#include "gnss_manager.h"
#include "obd2_manager.h"

// ---------------------------------------------------------------------------
// Definizione dell'istanza globale condivisa (dichiarata extern in shared_data.h)
// ---------------------------------------------------------------------------
VehicleData vehicleData;

// ---------------------------------------------------------------------------
// Pin RGB — modifica qui quando decidi i pin definitivi
// ---------------------------------------------------------------------------
static const int PIN_RGB_R = 4;
static const int PIN_RGB_G = 5;
static const int PIN_RGB_B = 6;

// ---------------------------------------------------------------------------
// Costanti sistema
// ---------------------------------------------------------------------------
static const unsigned long GNSS_FIX_TIMEOUT_MS = 5UL * 60UL * 1000UL;  // 5 minuti
static const unsigned long LOG_INTERVAL_MS      = 500;                   // log ogni 500ms

// ---------------------------------------------------------------------------
// Stato sistema
// ---------------------------------------------------------------------------
enum SystemState { SYS_WAITING_FIX, SYS_RUNNING };
static SystemState sysState = SYS_WAITING_FIX;

// ---------------------------------------------------------------------------
// LED di attesa fix: lampeggio rosso bloccante (non usa rgbUpdate)
// ---------------------------------------------------------------------------
static void blinkWaitingFix() {
  static unsigned long lastBlink = 0;
  static bool ledOn = false;
  if (millis() - lastBlink < 500) return;
  lastBlink = millis();
  ledOn = !ledOn;
  ledOn ? rgbSetColor(255, 0, 0) : rgbOff();
}

// ---------------------------------------------------------------------------
// Costruisce il nome file CSV dal timestamp GNSS: "/YYYYMMDD_HHMMSS.csv"
// Garantisce un file separato per ogni sessione di guida.
// ---------------------------------------------------------------------------
static void buildFilename(char* buf, size_t bufSize) {
  GnssData g = gnssGetData();
  snprintf(buf, bufSize, "/%04d%02d%02d_%02d%02d%02d.csv",
           g.year, g.month, g.day,
           g.hour, g.minute, g.second);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  // LED RGB
  rgbInit(PIN_RGB_R, PIN_RGB_G, PIN_RGB_B);
  //Serial.println("[RGB] OK");

  // GNSS
  gnssInit();
  //Serial.println("[GNSS] OK");

  // SD
  if (!sdInit()) { 
    //Serial.println("SD init fallita");
    return;
  }
  
  // WebServer + WiFi AP
  webServerInit();

  //Serial.println("[SYS] In attesa del fix GNSS...");
}

// ---------------------------------------------------------------------------
void loop() {
  // 1. Feed GNSS
  gnssUpdate();
  
  // 2. Gestisci richieste HTTP
  webServerHandle();
  
  // 3. Aggiorna dati OBD2 (simulati ora, TWAI in futuro)
  obd2Update();

  // ── STATO: ATTESA FIX ──────────────────────────────────────────────────
  if (sysState == SYS_WAITING_FIX) {
    blinkWaitingFix();

    bool fixOk      = gnssHasFix();
    bool timedOut   = (millis() > GNSS_FIX_TIMEOUT_MS);

    if (!fixOk && !timedOut) return;  // continua ad aspettare

    // Fix ottenuto o timeout scaduto
    if (fixOk) {
      //Serial.println("[GNSS] Fix ottenuto!");
    } else {
      //Serial.println("[GNSS] Timeout fix — avvio senza posizione valida");
    }

    // Apri file CSV con nome basato sul timestamp GNSS (o millis() se no fix)
    char filename[32];
    if (fixOk) {
      buildFilename(filename, sizeof(filename));
    } else {
      snprintf(filename, sizeof(filename), "/nognss_%lu.csv", millis());
    }

    if (!sdOpenFile(filename)) {
      //Serial.printf("[SD] Impossibile aprire %s\n", filename);
    } else {
      //Serial.printf("[SD] Logging su %s\n", filename);
    }

    // Ripristina LED all'algoritmo impostato dal webserver
    rgbSetMode(rgbGetMode());

    sysState = SYS_RUNNING;
    return;
  }

  // ── STATO: RUNNING ────────────────────────────────────────────────────
  // 4. Algoritmo LED
  rgbUpdate();

  // 5. Log su SD ogni LOG_INTERVAL_MS
  static unsigned long lastLog = 0;
  if (millis() - lastLog >= LOG_INTERVAL_MS) {
    lastLog = millis();
    
    char ts[20];
    gnssFormatTimestamp(ts, sizeof(ts));

    GnssData g = gnssGetData();
    
    sdWriteRow(
      ts,
      g.lat, g.lon,                           // lon, lat
      g.altMeters,
      g.satellites,
      g.hdop,
      vehicleData.speed,
      vehicleData.lonAcc, vehicleData.latAcc, // accelerometro non ancora integrato
      vehicleData.rpm,
      vehicleData.load,                       // load
      vehicleData.throttle                    //throttle
    );
  }
}