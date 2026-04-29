#include "gear_estimator.h"
#include "shared_data.h"
#include "time_sync_manager.h"
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

// ---------------------------------------------------------------------------
// NVS
// ---------------------------------------------------------------------------
static const char* NVS_NAMESPACE = "gear_calib";
static const char* NVS_KEY = "calib";

// ---------------------------------------------------------------------------
// Dati di calibrazione correnti (RAM)
// ---------------------------------------------------------------------------
static GearCalibrationData s_calib = {
    .version = 1,
    .checksum = 0,
    .gears = { {0,0,0,false} },
    .valid = false
};

// ---------------------------------------------------------------------------
// Stato runtime
// ---------------------------------------------------------------------------
static int s_currentGear = 0;
static bool s_gearValid = false;
static uint64_t s_gearTimestampUs = 0;

static int s_hysteresisCounter = 0;
static int s_lastStableGear = 0;

// ---------------------------------------------------------------------------
// Stato macchina a stati per la calibrazione
// ---------------------------------------------------------------------------
static bool s_captureActive = false;
static int s_captureGear = 0;
static unsigned long s_captureStartMs = 0;
static const unsigned long CAPTURE_DURATION_MS = 2000; // 2 secondi

static float s_speedSum = 0.0f;
static float s_rpmSum = 0.0f;
static float s_lonAccSum = 0.0f;
static int   s_sampleCount = 0.0f;

static bool s_captureFinished = false;
static bool s_captureStable = false;
static float s_capturedRatio = 0.0f;
static float s_capturedSpeedAvg = 0.0f;
static float s_capturedRpmAvg = 0.0f;

// ---------------------------------------------------------------------------
// Helper checksum (XOR a 16 bit)
// ---------------------------------------------------------------------------
static uint16_t computeChecksum(const GearCalibrationData& data) {
    const uint8_t* ptr = (const uint8_t*)&data;
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(GearCalibrationData) - sizeof(data.checksum); i++) {
        sum ^= ptr[i];
    }
    return sum;
}

static bool validateChecksum(const GearCalibrationData& data) {
    uint16_t stored = data.checksum;
    GearCalibrationData tmp = data;
    tmp.checksum = 0;
    return (stored == computeChecksum(tmp));
}

static void updateChecksum(GearCalibrationData& data) {
    data.checksum = 0;
    data.checksum = computeChecksum(data);
}

// ---------------------------------------------------------------------------
// Salvataggio / caricamento NVS
// ---------------------------------------------------------------------------
static bool saveToNVS() {
    Preferences nvs;
    nvs.begin(NVS_NAMESPACE, false);
    updateChecksum(s_calib);
    size_t size = sizeof(GearCalibrationData);
    bool ok = nvs.putBytes(NVS_KEY, &s_calib, size) == size;
    nvs.end();
    return ok;
}

static bool loadFromNVS() {
    Preferences nvs;
    nvs.begin(NVS_NAMESPACE, true);
    size_t size = sizeof(GearCalibrationData);
    GearCalibrationData tmp;
    if (nvs.getBytes(NVS_KEY, &tmp, size) != size) {
        nvs.end();
        return false;
    }
    nvs.end();
    if (tmp.version == s_calib.version && validateChecksum(tmp)) {
        s_calib = tmp;
        s_calib.valid = true;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// API pubbliche di calibrazione
// ---------------------------------------------------------------------------
bool gearEstimatorLoadCalibration() {
    return loadFromNVS();
}

bool gearEstimatorSaveCalibration() {
    return saveToNVS();
}

void gearEstimatorResetCalibration() {
    // Reset di tutte le marce
    for (int i = 1; i <= GEAR_MAX_GEARS; i++) {
        s_calib.gears[i].valid = false;
        s_calib.gears[i].ratioK = 0.0f;
        s_calib.gears[i].speedRefKmh = 0.0f;
        s_calib.gears[i].rpmRef = 0.0f;
    }
    s_calib.valid = false;
    saveToNVS();
    Serial.println("[GEAR] Calibrazione resettata a default (nessuna marcia calibrata).");
}

bool gearEstimatorHasValidCalibration() {
    for (int i = 1; i <= GEAR_MAX_GEARS; i++) {
        if (s_calib.gears[i].valid) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Avvia acquisizione per una marcia
// ---------------------------------------------------------------------------
bool gearEstimatorStartCapture(int gear) {
    if (gear < 1 || gear > GEAR_MAX_GEARS) return false;
    if (s_captureActive) return false;

    s_captureActive = true;
    s_captureGear = gear;
    s_captureStartMs = millis();
    s_speedSum = 0.0f;
    s_rpmSum = 0.0f;
    s_lonAccSum = 0.0f;
    s_sampleCount = 0;
    s_captureFinished = false;
    s_captureStable = false;
    return true;
}

bool gearEstimatorIsCapturing() {
    return s_captureActive;
}

void gearEstimatorAbortCapture() {
    s_captureActive = false;
    s_captureFinished = false;
}

// ---------------------------------------------------------------------------
// Chiamata internamente da gearEstimatorUpdate per accumulare campioni
// ---------------------------------------------------------------------------
static void captureAccumulate() {
    if (!s_captureActive) return;
    unsigned long now = millis();
    if (now - s_captureStartMs >= CAPTURE_DURATION_MS) {
        // Finestra terminata: valuta stabilità
        if (s_sampleCount >= 10) { // almeno 10 campioni validi
            float speedMean = s_speedSum / s_sampleCount;
            float rpmMean = s_rpmSum / s_sampleCount;
            float lonAccMean = s_lonAccSum / s_sampleCount;

            // Calcolo varianze approssimative (per semplicità usiamo range)
            // Più robusto: controlliamo che max-min sia piccolo
            // Qui usiamo un indicatore: se la deviazione standard stimata è bassa.
            // Per semplicità implementiamo controllo su range (memorizziamo min/max)
            // Alternativa: usare solo lonAcc e delta tra primo e ultimo campione.
            // Scegliamo di richiedere che lonAcc media sia < 0.1G e che
            // la differenza tra primo e ultimo campione di speed sia < 2 km/h.
            // Conserviamo i primi valori per fare questo controllo.
            static float firstSpeed = 0, firstRpm = 0;
            static bool firstSample = true;
            if (firstSample) {
                firstSpeed = speedMean;
                firstRpm = rpmMean;
                firstSample = false;
            }
            float speedDrift = fabsf(speedMean - firstSpeed);
            float rpmDrift = fabsf(rpmMean - firstRpm);
            bool stable = (fabsf(lonAccMean) < 0.1f) && (speedDrift < 2.0f) && (rpmDrift < 200.0f);

            if (stable && speedMean > GEAR_MIN_SPEED && rpmMean > GEAR_MIN_RPM) {
                s_capturedRatio = rpmMean / speedMean;
                s_capturedSpeedAvg = speedMean;
                s_capturedRpmAvg = rpmMean;
                s_captureStable = true;
            } else {
                s_captureStable = false;
            }
        } else {
            s_captureStable = false;
        }
        s_captureFinished = true;
        s_captureActive = false; // fine finestra
        return;
    }

    // Altrimenti accumula campione corrente
    float speed = vehicleData.speed;
    int rpm = vehicleData.rpm;
    float lonAcc = vehicleData.lonAcc;

    if (speed > 0.0f && rpm > 0) {
        s_speedSum += speed;
        s_rpmSum += rpm;
        s_lonAccSum += lonAcc;
        s_sampleCount++;
    }
}

bool gearEstimatorGetCaptureResult(float &ratio, float &speedAvg, float &rpmAvg) {
    if (!s_captureFinished) return false;
    if (!s_captureStable) return false;
    ratio = s_capturedRatio;
    speedAvg = s_capturedSpeedAvg;
    rpmAvg = s_capturedRpmAvg;
    return true;
}

void gearEstimatorSetRatioForGear(int gear, float ratio) {
    if (gear < 1 || gear > GEAR_MAX_GEARS) return;
    s_calib.gears[gear].ratioK = ratio;
    s_calib.gears[gear].valid = true;
    // opzionale: salva anche i valori di riferimento (non obbligatorio)
    s_calib.gears[gear].speedRefKmh = 0;   // non usati runtime
    s_calib.gears[gear].rpmRef = 0;
    s_calib.valid = true;
}

// ---------------------------------------------------------------------------
// Runtime: stima marcia
// ---------------------------------------------------------------------------
static void estimateGear() {
    float speed = vehicleData.speed;
    int rpm = vehicleData.rpm;
    uint64_t nowUs = timeSyncNowUs();

    // Condizioni per marcia 0
    if (speed < GEAR_MIN_SPEED || rpm < GEAR_MIN_RPM) {
        if (s_currentGear != 0) {
            s_currentGear = 0;
            s_gearValid = false;
            s_gearTimestampUs = nowUs;
            s_hysteresisCounter = 0;
            s_lastStableGear = 0;
        }
        return;
    }

    float K_live = (float)rpm / speed;
    int bestGear = 0;
    float bestError = GEAR_MATCH_THRESHOLD; // soglia di accettazione

    // Cerca la marcia con errore relativo minimo
    for (int g = 1; g <= GEAR_MAX_GEARS; g++) {
        if (!s_calib.gears[g].valid) continue;
        float K_gear = s_calib.gears[g].ratioK;
        if (K_gear <= 0.0f) continue;
        float error = fabsf(K_live - K_gear) / K_gear;
        if (error < bestError) {
            bestError = error;
            bestGear = g;
        }
    }

    // Isteresi: richiedere più campioni consecutivi uguali prima di cambiare
    if (bestGear != 0 && bestGear == s_lastStableGear) {
        s_hysteresisCounter++;
    } else {
        s_hysteresisCounter = 1;
        s_lastStableGear = bestGear;
    }

    if (s_hysteresisCounter >= GEAR_HYSTERESIS_COUNT && bestGear != 0) {
        if (s_currentGear != bestGear) {
            s_currentGear = bestGear;
            s_gearValid = true;
            s_gearTimestampUs = nowUs;
        }
    } else if (bestGear == 0 && s_currentGear != 0) {
        // Se nessuna marcia valida, azzera dopo breve tempo
        s_currentGear = 0;
        s_gearValid = false;
        s_gearTimestampUs = nowUs;
    }
}

// ---------------------------------------------------------------------------
// API pubbliche principali
// ---------------------------------------------------------------------------
void gearEstimatorInit() {
    // Carica calibrazione da NVS
    if (!loadFromNVS()) {
        Serial.println("[GEAR] Nessuna calibrazione valida in NVS. Usare comandi seriali per calibrare.");
        s_calib.valid = false;
        for (int i = 1; i <= GEAR_MAX_GEARS; i++) {
            s_calib.gears[i].valid = false;
        }
    } else {
        Serial.println("[GEAR] Calibrazione caricata da NVS.");
    }
    s_currentGear = 0;
    s_gearValid = false;
    s_hysteresisCounter = 0;
    s_lastStableGear = 0;
    s_captureActive = false;
}

void gearEstimatorUpdate() {
    // Gestione acquisizione calibrazione (se attiva)
    captureAccumulate();

    // Stima marcia (non eseguita durante la cattura per non disturbare)
    if (!s_captureActive) {
        estimateGear();
    }

    // Aggiorna shared_data
    vehicleData.gearEstimated = s_currentGear;
    vehicleData.gearValid = s_gearValid;
    vehicleData.gearTimestampUs = s_gearTimestampUs;
}

int gearEstimatorGetCurrentGear() {
    return s_currentGear;
}

bool gearEstimatorIsGearValid() {
    return s_gearValid;
}

uint64_t gearEstimatorGetTimestampUs() {
    return s_gearTimestampUs;
}

GearCalibrationInfo gearEstimatorGetCalibrationInfo() {
    GearCalibrationInfo info;
    info.hasValidCalibration = s_calib.valid;
    info.usingDefaults = !s_calib.valid;
    for (int i = 1; i <= GEAR_MAX_GEARS; i++) {
        info.ratioForGear[i] = s_calib.gears[i].ratioK;
        info.validForGear[i] = s_calib.gears[i].valid;
    }
    return info;
}