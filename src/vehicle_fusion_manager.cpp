#include "vehicle_fusion_manager.h"
#include "imu_manager.h"
#include "gnss_manager.h"
#include "shared_data.h"
#include "time_sync_manager.h"
#include <math.h>
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Parametri di fusione
// ---------------------------------------------------------------------------
#define MIN_SPEED_FOR_COURSE_KMH    8.0f    // sotto questa soglia peso GNSS = 0
#define MAX_SPEED_FOR_COURSE_KMH   20.0f    // sopra questo peso = 1
#define HDOP_GOOD_THRESHOLD         1.5f    // hdop <= 1.5 → fattore 1
#define HDOP_BAD_THRESHOLD          5.0f    // hdop >= 5 → fattore 0.2

#define HEADING_COMPL_GAIN_MAX      0.08f   // guadagno massimo (quando weight=1)
#define YAW_RATE_EMA_ALPHA          0.2f    // smoothing yaw rate
#define POSITION_EMA_ALPHA          0.25f   // smoothing posizione GNSS

// ---------------------------------------------------------------------------
// Stato interno
// ---------------------------------------------------------------------------
static VehicleState s_state = {
    .heading_deg = 0.0f,
    .yawRate_dps = 0.0f,
    .speed_mps = 0.0f,
    .lat = 0.0,
    .lon = 0.0,
    .headingConfidence = 0.0f,
    .positionConfidence = 0.0f,
    .valid = false,
    .timestampUs = 0
};
static uint64_t s_lastUpdateUs = 0;

// ---------------------------------------------------------------------------
// Helper: angolo normalizzato [-180,180]
// ---------------------------------------------------------------------------
static float normalizeAngle180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// ---------------------------------------------------------------------------
// Helper: wrap 0-360
// ---------------------------------------------------------------------------
static float wrap360(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle;
}

// ---------------------------------------------------------------------------
// Calcola peso per la correzione heading da GNSS course
// ---------------------------------------------------------------------------
static float computeGnssCourseWeight(float speedKmh, float hdop) {
    // Peso basato sulla velocità
    float weight = 0.0f;
    if (speedKmh >= MAX_SPEED_FOR_COURSE_KMH) {
        weight = 1.0f;
    } else if (speedKmh > MIN_SPEED_FOR_COURSE_KMH) {
        weight = (speedKmh - MIN_SPEED_FOR_COURSE_KMH) /
        (MAX_SPEED_FOR_COURSE_KMH - MIN_SPEED_FOR_COURSE_KMH);
    }
    // Riduzione per hdop elevato
    float hdopFactor = 1.0f;
    if (hdop >= HDOP_BAD_THRESHOLD) {
        hdopFactor = 0.2f;
    } else if (hdop > HDOP_GOOD_THRESHOLD) {
        float t = (hdop - HDOP_GOOD_THRESHOLD) /
                  (HDOP_BAD_THRESHOLD - HDOP_GOOD_THRESHOLD);
        hdopFactor = 1.0f - t * 0.8f;   // da 1.0 a 0.2
    }
    weight *= hdopFactor;
    
    // Limita a [0, 1]
    if (weight < 0.0f) weight = 0.0f;
    if (weight > 1.0f) weight = 1.0f;
    return weight;
}

// ---------------------------------------------------------------------------
// Fusione heading: gyro integrato + GNSS course (complementary filter)
// ---------------------------------------------------------------------------
static float fuseHeading(float headingGyro, float headingGnss, float weight, float dt) {
    if (!isfinite(headingGnss)) return headingGyro;
    float diff = normalizeAngle180(headingGnss - headingGyro);
    float gain = HEADING_COMPL_GAIN_MAX * weight;
    float newHeading = headingGyro + gain * diff;
    return wrap360(newHeading);
}

// ---------------------------------------------------------------------------
// API pubbliche
// ---------------------------------------------------------------------------
void vehicleFusionInit() {
    s_state.valid = false;
    s_lastUpdateUs = 0;
    s_state.timestampUs = 0;
}

void vehicleFusionUpdate() {
    // Timestamp attuale (monotono)
    uint64_t nowUs = timeSyncNowUs();
    float dt = 0.0f;
    if (s_lastUpdateUs != 0) {
        dt = (nowUs - s_lastUpdateUs) * 1e-6f;
        if (dt > 0.1f) dt = 0.1f;   // limite per evitare salti dopo pause
        if (dt < 0.0f) dt = 0.0f;
    }

    // 1) Acquisizione dati sensori
    ImuData imu = imuGetData();
    GnssData gnss = gnssGetData();
    float obdSpeedKmh = vehicleData.speed; // da shared_data

    // Yaw rate del giroscopio (calibrato, deg/s)
    float rawYawRate = imu.gyrZ_cal; // già in deg/s
    // Smoothing semplice
    if (dt > 0.0f) {
        s_state.yawRate_dps = s_state.yawRate_dps * (1.0f - YAW_RATE_EMA_ALPHA) +
                              rawYawRate * YAW_RATE_EMA_ALPHA;
    } else {
        s_state.yawRate_dps = rawYawRate;
    }

    // 2) Integrazione heading base (se abbiamo un dt valido)
    float headingGyro = s_state.heading_deg;
    if (dt > 0.0f && s_state.valid) {
        headingGyro += s_state.yawRate_dps * dt;
        headingGyro = wrap360(headingGyro);
    } else if (!s_state.valid) {
        // primo avvio: se GNSS ha course valido usiamo quello
        if (gnss.valid && gnss.course_deg >= 0.0f) {
            headingGyro = wrap360(gnss.course_deg);
        }
    }

    // 3) Correzione con GNSS course
    float headingGnss = -1.0f;
    float weight = 0.0f;
    if (gnss.valid && gnss.course_deg >= 0.0f && gnss.speedKmh > 0.0f) {
        headingGnss = wrap360(gnss.course_deg);
        weight = computeGnssCourseWeight(gnss.speedKmh, gnss.hdop);
    }
    float newHeading = fuseHeading(headingGyro, headingGnss, weight, dt);
    s_state.heading_deg = newHeading;
    s_state.headingConfidence = weight;   // usiamo il peso come confidenza

    // 4) Velocità fusa (m/s)
    float speedMs = 0.0f;
    if (gnss.valid && gnss.speedKmh > 0.0f && gnss.hdop < 5.0f) {
        speedMs = gnss.speedKmh / 3.6f;
    } else if (obdSpeedKmh > 0.0f) {
        speedMs = obdSpeedKmh / 3.6f;
    }
    s_state.speed_mps = speedMs;

    // 5) Posizione filtrata (EMA su lat/lon)
    if (gnss.valid && gnss.lat != 0.0 && gnss.lon != 0.0) {
        if (!s_state.valid) {
            s_state.lat = gnss.lat;
            s_state.lon = gnss.lon;
        } else {
            double alpha = POSITION_EMA_ALPHA;
            // velocità alta → riduci smoothing (più reattivo)
            if (speedMs > 10.0f) alpha = 0.5f;
            s_state.lat = alpha * gnss.lat + (1.0 - alpha) * s_state.lat;
            s_state.lon = alpha * gnss.lon + (1.0 - alpha) * s_state.lon;
        }
        // Confidenza posizione: basata su hdop e velocità
        float posConf = 1.0f;
        if (gnss.hdop > 2.0f) posConf = 2.0f / gnss.hdop;
        if (posConf > 1.0f) posConf = 1.0f;
        if (gnss.speedKmh < 5.0f) posConf *= 0.8f;  // meno affidabile da fermo
        s_state.positionConfidence = posConf;
    } else {
        s_state.positionConfidence = 0.0f;
    }

    // 6) Flag validità
    s_state.valid = true;
    s_state.timestampUs = nowUs;
    s_lastUpdateUs = nowUs;
}

VehicleState vehicleFusionGetState() {
    return s_state;
}