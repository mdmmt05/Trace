#include <Arduino.h>
#include <esp_task_wdt.h>
#include "shared_data.h"
#include "sd_manager.h"
#include "rgb_controller.h"
#include "web_server.h"
#include "gnss_manager.h"
#include "obd2_manager.h"
#include "imu_manager.h"

// ---------------------------------------------------------------------------
// Istanza globale condivisa (extern in shared_data.h)
// ---------------------------------------------------------------------------
VehicleData vehicleData;

// ---------------------------------------------------------------------------
// Pin RGB — modifica con i pin definitivi
// ---------------------------------------------------------------------------
static const int PIN_RGB_R = 4;
static const int PIN_RGB_G = 5;
static const int PIN_RGB_B = 6;

// ---------------------------------------------------------------------------
// Costanti sistema
// ---------------------------------------------------------------------------
static const unsigned long GNSS_FIX_TIMEOUT_MS = 5UL * 60UL * 1000UL;
static const unsigned long LOG_INTERVAL_MS      = 500;

// ---------------------------------------------------------------------------
// Stato sistema
// ---------------------------------------------------------------------------
enum SystemState { SYS_WAITING_FIX, SYS_RUNNING };
static SystemState sysState = SYS_WAITING_FIX;

// ---------------------------------------------------------------------------
// LED attesa fix: lampeggio rosso non bloccante
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
// Nome file CSV dalla data/ora GNSS
// ---------------------------------------------------------------------------
static void buildFilename(char* buf, size_t bufSize) {
  GnssData g = gnssGetData();
  snprintf(buf, bufSize, "/%04d%02d%02d_%02d%02d%02d.csv",
           g.year, g.month, g.day, g.hour, g.minute, g.second);
}

// ---------------------------------------------------------------------------
void setup() {
  // Serial0 — condivisa con obd2_manager in modalità UART sim.
  // IMPORTANTE: while(!Serial) senza timeout blocca il boot se l'USB non è
  // connessa (uso in auto senza PC). Si usa un timeout di 2 secondi.
  Serial.begin(115200);
  {
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 2000) { delay(10); }
  }
  delay(200);

  rgbInit(PIN_RGB_R, PIN_RGB_G, PIN_RGB_B);
  gnssInit();
  obd2Init();

  // imuInit() dura ~1 secondo (calibrazione giroscopio).
  // Deve essere chiamato a veicolo fermo — i Serial.println qui dentro
  // sono ok perché obd2Update() non è ancora nel loop.
  if (!imuInit()) {
    // IMU non trovata: continua senza dati IMU (campi a 0 nel CSV)
    rgbSetColor(255, 128, 0);   // LED arancione = warning IMU assente
    delay(2000);
  }
  esp_task_wdt_reset(); // feed watchdog dopo calibrazione

  if (!sdInit()) {
    // Serial.println("[SD] Init fallita");   // <-- commentato in modalità UART sim
    // SD non presente o corrotta: segnala con LED rosso lampeggiante 3×
    for (int i = 0; i < 3; i++) {
      rgbSetColor(255, 0, 0); delay(200);
      rgbOff();               delay(200);
    }
    // Il sistema continua — GNSS e OBD2 funzionano, solo il logging è disabilitato
  }

  webServerInit();

  // Serial.println("[SYS] In attesa del fix GNSS...");
}

// ---------------------------------------------------------------------------
void loop() {
  gnssUpdate();
  obd2Update();
  webServerHandle();
  imuUpdate(); // chiamato ad ogni loop - più frequente = migliore stima

  // ── ATTESA FIX ────────────────────────────────────────────────────────────
  if (sysState == SYS_WAITING_FIX) {
    blinkWaitingFix();

    bool fixOk    = gnssHasFix();
    bool timedOut = (millis() > GNSS_FIX_TIMEOUT_MS);

    if (!fixOk && !timedOut) return;

    char filename[32];
    if (fixOk) {
      buildFilename(filename, sizeof(filename));
    } else {
      snprintf(filename, sizeof(filename), "/nognss_%lu.csv", millis());
    }

    sdOpenFile(filename);
    rgbSetMode(rgbGetMode());
    sysState = SYS_RUNNING;
    return;
  }

  // ── RUNNING ───────────────────────────────────────────────────────────────
  rgbUpdate();

  static unsigned long lastLog = 0;
  unsigned long now = millis();
  if (now - lastLog < LOG_INTERVAL_MS) return;
  lastLog = now;

  char ts[20];
  gnssFormatTimestamp(ts, sizeof(ts));
  GnssData g = gnssGetData();
  ImuData   imu = imuGetData();

  // ── Flag affidabilità pendenza ────────────────────────────────────────────
  // Calcola |dv/dt| dalla velocità OBD2 tra due campioni consecutivi.
  // LOG_INTERVAL_MS = 500ms → dt = 0.5s
  // Soglia 0.5 m/s² ≈ 0.14G: sotto questa soglia la distorsione del pitch
  // per effetto inerziale è < 8°, accettabile come stima pendenza.
  // Conversione: OBD2 speed è in km/h → dividi per 3.6 per avere m/s.
  static float prevSpeedMs = 0.0f;
  float currSpeedMs   = vehicleData.speed / 3.6f;
  float accelEstMs2   = fabsf(currSpeedMs - prevSpeedMs) / (LOG_INTERVAL_MS / 1000.0f);
  prevSpeedMs         = currSpeedMs;
 
  // Soglia 0.5 m/s²: corrisponde a ~0.05G, distorsione pitch < 3°
  static const float SLOPE_RELIABLE_THRESHOLD_MS2 = 0.5f;
  uint8_t slopeReliable = (accelEstMs2 < SLOPE_RELIABLE_THRESHOLD_MS2) ? 1 : 0;

  sdWriteRow(ts,
    g.lat, g.lon, g.altMeters,
    g.satellites, g.hdop,
    vehicleData.speed,
    imu.lonAcc,  imu.latAcc,
    imu.roll,    imu.pitch,
    imu.slope,   slopeReliable,
    vehicleData.rpm,
    vehicleData.load,
    vehicleData.throttle
  );
}
