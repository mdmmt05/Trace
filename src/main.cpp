#include <Arduino.h>
#include <esp_task_wdt.h>
#include "shared_data.h"
#include "sd_manager.h"
#include "rgb_controller.h"
#include "web_server.h"
#include "gnss_manager.h"
#include "obd2_manager.h"
#include "imu_manager.h"
#include "time_sync_manager.h"

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
// Gestione comandi seriali per calibrazione IMU (commissioning)
// ---------------------------------------------------------------------------
static void handleSerialCommands() {
  static String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.length() > 0) {
        if (input == "cal_gyro") {
          Serial.println("Esecuzione calibrazione giroscopio (veicolo fermo)...");
          if (imuRunGyroCalibration()) {
            imuSaveCalibration();
            Serial.println("Calibrazione giroscopio completata e salvata.");
          } else {
            Serial.println("Errore calibrazione giroscopio.");
          }
        }
        else if (input == "cal_acc") {
          Serial.println("Esecuzione calibrazione accelerometro (veicolo fermo e in piano)...");
          if (imuRunAccelCalibration()) {
            imuSaveCalibration();
            Serial.println("Calibrazione accelerometro completata e salvata.");
          } else {
            Serial.println("Errore calibrazione accelerometro.");
          }
        }
        else if (input.startsWith("set_mounting")) {
          float roll, pitch;
          if (sscanf(input.c_str(), "set_mounting %f %f", &roll, &pitch) == 2) {
            imuSetMountingAlignment(roll, pitch);
            imuSaveCalibration();
            Serial.printf("Mounting impostato: roll=%.2f, pitch=%.2f\n", roll, pitch);
          } else {
            Serial.println("Uso: set_mounting <roll_deg> <pitch_deg>");
          }
        }
        else if (input == "save_cal") {
          if (imuSaveCalibration()) Serial.println("Calibrazione salvata in NVS.");
          else Serial.println("Errore salvataggio.");
        }
        else if (input == "reset_cal") {
          imuResetCalibration();
          Serial.println("Calibrazione resettata a default.");
        }
        else if (input == "show_cal") {
          ImuCalibrationInfo info = imuGetCalibrationInfo();
          Serial.println("--- Calibrazione IMU ---");
          Serial.printf("Valida in NVS: %s\n", info.hasValidCalibration ? "si" : "no");
          Serial.printf("Usa default: %s\n", info.usingDefaults ? "si" : "no");
          Serial.printf("Gyro bias (LSB): %.1f, %.1f, %.1f\n", info.gyroBiasX, info.gyroBiasY, info.gyroBiasZ);
          Serial.printf("Acc bias (LSB): %.1f, %.1f, %.1f\n", info.accBiasX, info.accBiasY, info.accBiasZ);
          Serial.printf("Acc scale: %.4f, %.4f, %.4f\n", info.accScaleX, info.accScaleY, info.accScaleZ);
          Serial.printf("Mounting (deg): roll=%.2f, pitch=%.2f\n", info.mountingRoll, info.mountingPitch);
          TimeSyncStatus ts = timeSyncGetStatus();
          Serial.printf("Time sync: valid=%d quality=%d offset=%lld lastSync=%llu\n",
                        ts.utcValid, ts.quality, ts.utcOffsetUs, ts.lastSyncMonoUs);
        }
        else if (input == "help") {
          Serial.println("Comandi disponibili:");
          Serial.println("  cal_gyro         - calibra giroscopio (veicolo fermo)");
          Serial.println("  cal_acc          - calibra accelerometro (veicolo fermo e in piano)");
          Serial.println("  set_mounting R P - imposta offset mounting (gradi)");
          Serial.println("  save_cal         - salva parametri correnti in NVS");
          Serial.println("  reset_cal        - resetta a default e salva");
          Serial.println("  show_cal         - mostra parametri attuali");
          Serial.println("  help             - questo messaggio");
        }
        else {
          Serial.println("Comando sconosciuto. Digitare 'help'.");
        }
      }
      input = "";
    } else if (c != '\r') {
      input += c;
      if (input.length() > 64) input = "";
    }
  }
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
  Serial.println("[SYS] Pronto. Comandi seriali disponibili (help).");
}

// ---------------------------------------------------------------------------
void loop() {
  gnssUpdate();
  obd2Update();
  webServerHandle();
  imuUpdate();
  handleSerialCommands();

  // Aggiorna la soft‑sync con l'ultimo fix GNSS valido (al massimo una volta al secondo)
  static uint64_t lastGnssSyncUs = 0;
  GnssData gnss = gnssGetData();
  if (gnss.utcValid && gnss.fixRxMonoUs != 0) {
    uint64_t nowUs = timeSyncNowUs();
    if (nowUs - lastGnssSyncUs > 1000000ULL) {   // al massimo 1Hz
      timeSyncUpdateFromGnss(&gnss, gnss.fixRxMonoUs);
      lastGnssSyncUs = nowUs;
    }
  }

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

  // Acquisizione snapshot temporale
  uint64_t logMonoUs = timeSyncNowUs();
  GnssData   g = gnssGetData();
  ImuData   imu = imuGetData();
  VehicleData vd;   // copia locale dei dati condivisi (per coerenza)
  noInterrupts();
  vd = vehicleData;
  interrupts();

  // Calcolo età campioni
  int imuAgeMs = (imu.lastSampleMonoUs != 0) ? (int)((logMonoUs - imu.lastSampleMonoUs) / 1000) : -1;
  int gnssAgeMs = (g.fixRxMonoUs != 0) ? (int)((logMonoUs - g.fixRxMonoUs) / 1000) : -1;
  int obdSpeedAgeMs = (vd.speedTimestampUs != 0) ? (int)((logMonoUs - vd.speedTimestampUs) / 1000) : -1;

  // UTC stimata e qualità sync
  int64_t utcUs = timeSyncMonoToUtcUs(logMonoUs);
  TimeSyncStatus ts = timeSyncGetStatus();

  // Timestamp stringa per compatibilità (opzionale, possiamo usare gnssFormatTimestamp)
  char tsStr[20];
  gnssFormatTimestamp(tsStr, sizeof(tsStr));

  sdWriteRow(tsStr,
    g.lat, g.lon, g.altMeters,
    g.satellites, g.hdop,
    vd.speed,
    imu.lonAcc,  imu.latAcc,
    imu.roll,    imu.pitch,
    imu.slope,   imu.slopeConfidence,
    vd.rpm,
    vd.load,
    vd.throttle,
    logMonoUs,
    utcUs,
    ts.utcValid,
    ts.quality,
    imu.lastSampleMonoUs,
    g.fixRxMonoUs,
    vd.speedTimestampUs,
    imuAgeMs,
    gnssAgeMs,
    obdSpeedAgeMs
  );
}
