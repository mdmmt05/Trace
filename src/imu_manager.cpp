// ---------------------------------------------------------------------------
// imu_manager.cpp
// Lettura ISM330DHCX via I2C, filtro di Madgwick adattivo per fusione acc+gyro e stima slope dedicata.
//
// Il filtro di Madgwick è un algoritmo AHRS (Attitude and Heading Reference
// System) che fonde accelerometro e giroscopio per stimare l'orientamento
// assoluto con drift minimo. Non richiede magnetometro (MARG → AHRS a 6DOF).
//
// Riferimento algoritmo:
//   S. Madgwick, "An efficient orientation filter for inertial and
//   inertial/magnetic sensor arrays", University of Bristol, 2010.
//
// Implementazione ottimizzata per ESP32-S3 (FPU hardware disponibile).
// ---------------------------------------------------------------------------

#include "imu_manager.h"
#include "shared_data.h"
#include "time_sync_manager.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>

// ---------------------------------------------------------------------------
// Registri ISM330DHCX
// ---------------------------------------------------------------------------
#define ISM330_WHO_AM_I        0x0F   // atteso 0x6B
#define ISM330_CTRL1_XL        0x10   // Acc ODR, FS, LPF
#define ISM330_CTRL2_G         0x11   // Gyro ODR, FS
#define ISM330_CTRL3_C         0x12   // Interfaccia, reset
#define ISM330_CTRL4_C         0x13   // FIFO enable, LPF2
#define ISM330_CTRL5_C         0x14   // Routing interrupt
#define ISM330_CTRL6_C         0x15   // Gyro LPF, user offset
#define ISM330_CTRL7_G         0x16   // Gyro high-pass
#define ISM330_CTRL8_XL        0x17   // Acc LPF2, input composite
#define ISM330_CTRL9_XL        0x18   // Acc axis enable
#define ISM330_CTRL10_C        0x19   // Gyro axis enable

#define ISM330_FIFO_CTRL1      0x07   // Watermark level
#define ISM330_FIFO_CTRL2      0x08   // FIFO mode, watermark interrupt
#define ISM330_FIFO_CTRL3      0x09   // Batch decimation
#define ISM330_FIFO_CTRL4      0x0A   // FIFO mode selection
#define ISM330_FIFO_STATUS1    0x0B   // FIFO level (low byte)
#define ISM330_FIFO_STATUS2    0x0C   // FIFO level (high byte, overrun)
#define ISM330_FIFO_DATA_OUT_L 0x3E   // FIFO output (low)
#define ISM330_FIFO_DATA_OUT_H 0x3F   // FIFO output (high)

#define ISM330_STATUS_REG      0x1E   // Status (FIFO threshold, overrun)

#define ISM330_OUTX_L_G        0x22   // Giroscopio X low (per debug)
#define ISM330_OUTX_L_A        0x28   // Accelerometro X low

// Valore WHO_AM_I atteso
#define ISM330_WHO_AM_I_VAL    0x6B

// ===========================================================================
// Configurazioni sensore – scelte per automotive (robustezza > max Fs)
// ===========================================================================
// Accelerometro: ±4G, ODR = 208 Hz, LPF2 = 50 Hz
// ±4G è sufficiente per frenate/accelerazioni tipiche (max ~1.2G su strada)
// ODR 208 Hz permette di catturare dinamiche del veicolo senza sovraccaricare FIFO
#define ACC_ODR_208HZ          (0x04 << 4)   // CTRL1_XL[7:4] = 0100
#define ACC_FS_4G              (0x02 << 2)   // CTRL1_XL[3:2] = 10 (±4G)
#define ACC_LPF2_50HZ          (0x01 << 0)   // CTRL1_XL[1:0] = 01 (filtro banda 50 Hz)
#define ACC_CONFIG             (ACC_ODR_208HZ | ACC_FS_4G | ACC_LPF2_50HZ)

// Giroscopio: ±500 °/s, ODR = 208 Hz, LPF1 = 50 Hz
// ±500 °/s: anche in curva estrema (autostrada) <200 °/s, range ampio senza clipping
#define GYR_ODR_208HZ          (0x04 << 4)   // CTRL2_G[7:4] = 0100
#define GYR_FS_500DPS          (0x00 << 2)   // CTRL2_G[3:2] = 00 (±500 °/s)
#define GYR_LPF1_50HZ          (0x01 << 0)   // CTRL2_G[1:0] = 01 (filtro 50 Hz)
#define GYR_CONFIG             (GYR_ODR_208HZ | GYR_FS_500DPS | GYR_LPF1_50HZ)

// Fattori di scala per conversioni in unità fisiche
#define ACC_SCALE_4G           16384.0f   // ±4G → 16384 LSB/G
#define GYR_SCALE_500DPS       131.0f     // ±500 °/s → 131 LSB/(°/s)

// ===========================================================================
// FIFO – modalità continua, watermark 16 campioni
// Ognuno dei 16 campioni è una coppia (acc + gyro) → 12 byte per campione
// Totale lettura burst di 192 byte ogni volta, ben dentro i limiti I2C.
// ===========================================================================
#define FIFO_WATERMARK         16
#define BYTES_PER_SAMPLE       12   // 6 byte acc + 6 byte gyro
#define FIFO_BURST_SIZE        (FIFO_WATERMARK * BYTES_PER_SAMPLE)

// ---------------------------------------------------------------------------
// Parametri filtro di Madgwick
// ---------------------------------------------------------------------------
// Beta: guadagno del gradiente (trade-off velocità convergenza / rumore)
// 0.033 = valore raccomandato da Madgwick per AHRS a 6DOF
#define MADGWICK_BASE_BETA     0.033f   // beta base (raccomandato da Madgwick)
#define ACCEL_WEIGHT_MIN       0.1f     // peso minimo dell'accelerometro in alta dinamica
#define ACCEL_WEIGHT_MAX       1.0f     // peso massimo (quasi-statico)
#define GYRO_MAG_THRESH        15.0f    // °/s – sopra questa soglia riduci peso accelerometro
#define ACC_MAG_1G_TOL         0.15f    // tolleranza per modulo accelerazione vicino a 1G
 
// ===========================================================================
// Stima slope lenta – costante di tempo 2.0 secondi
// ===========================================================================
#define SLOPE_TC_QUASI_STATIC  2.0f      // aggiornamento rapido in quasi-statico
#define SLOPE_TC_DYNAMIC       10.0f     // aggiornamento molto lento in dinamica

// Runtime gyro refinement (opzionale)
#define GYRO_REFINE_ENABLED    1
#define GYRO_REFINE_MIN_SAMPLES 500   // campioni quasi-statici consecutivi
#define GYRO_REFINE_RATE_MS    1000   // ogni secondo
#define GYRO_REFINE_ALPHA      0.01f  // IIR lento
#define GYRO_REFINE_MAX_CORR   0.5f   // max correzione accumulata (deg/s)

// ---------------------------------------------------------------------------
// Dati di calibrazione (persistenti in NVS)
// ---------------------------------------------------------------------------
struct CalibrationData {
    uint32_t version = 1;
    uint16_t checksum = 0;
    float gyroBias[3] = {0};      // LSB
    float accBias[3] = {0};       // LSB
    float accScale[3] = {1,1,1};  // fattore di scala
    float mountingRoll = 0;       // gradi
    float mountingPitch = 0;      // gradi
    bool valid = false;
};

static CalibrationData calData;
static Preferences nvs;

// ---------------------------------------------------------------------------
// Stato interno — tutto privato al modulo
// ---------------------------------------------------------------------------
 
// Quaternione di orientamento Madgwick (normalizzato, w=1 = identità)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// Quaternione di rotazione per mounting (sensore → veicolo)
static float qm0 = 1.0f, qm1 = 0.0f, qm2 = 0.0f, qm3 = 0.0f;
// Quaternione orientamento veicolo (calcolato = qm * q_sensore)
static float qv0, qv1, qv2, qv3;

// Runtime gyro bias refinement (non salvato)
static float gyroBiasRuntime[3] = {0};
static unsigned long refineLastTime = 0;
static unsigned int refineQuasiStaticCount = 0;

static bool ready = false;
static ImuData imuData;   // dati più recenti

// Variabili per la slope lenta
static float slopeFiltered = 0.0f;
static unsigned long lastSlopeUpdate = 0;

// Supporto FIFO
static uint8_t fifoBuffer[FIFO_BURST_SIZE];
static float odrHz = 208.0f;        // ODR effettivo (Hz)
static float sampleDt = 1.0f / odrHz;  // dt nominale tra campioni FIFO

// ---------------------------------------------------------------------------
// Helper checksum (semplice XOR a 16 bit)
// ---------------------------------------------------------------------------
static uint16_t computeChecksum(const CalibrationData& data) {
    const uint8_t* ptr = (const uint8_t*)&data;
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(CalibrationData) - sizeof(data.checksum); i++) {
        sum ^= ptr[i];
    }
    return sum;
}

static bool validateChecksum(const CalibrationData& data) {
    uint16_t stored = data.checksum;
    CalibrationData tmp = data;
    tmp.checksum = 0;
    return (stored == computeChecksum(tmp));
}

static void updateChecksum(CalibrationData& data) {
    data.checksum = 0;
    data.checksum = computeChecksum(data);
}

// ---------------------------------------------------------------------------
// Salvataggio / caricamento NVS
// ---------------------------------------------------------------------------
static bool saveToNVS() {
    nvs.begin("imu_calib", false);
    updateChecksum(calData);
    size_t size = sizeof(CalibrationData);
    bool ok = nvs.putBytes("calib", &calData, size) == size;
    nvs.end();
    return ok;
}

static bool loadFromNVS() {
    nvs.begin("imu_calib", true);
    size_t size = sizeof(CalibrationData);
    CalibrationData tmp;
    if (nvs.getBytes("calib", &tmp, size) != size) {
        nvs.end();
        return false;
    }
    nvs.end();
    if (tmp.version == calData.version && validateChecksum(tmp)) {
        calData = tmp;
        calData.valid = true;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Imposta quaternione di mounting da roll/pitch (yaw=0)
// Ordine: prima pitch (Y), poi roll (X)
// ---------------------------------------------------------------------------
static void updateMountingQuaternion() {
    float rollRad = calData.mountingRoll * M_PI / 180.0f;
    float pitchRad = calData.mountingPitch * M_PI / 180.0f;
    float cy = cosf(pitchRad * 0.5f);
    float sy = sinf(pitchRad * 0.5f);
    float cr = cosf(rollRad * 0.5f);
    float sr = sinf(rollRad * 0.5f);
    // Quaternione per rotazione ZYX? In realtà roll intorno X, pitch intorno Y.
    // q = q_roll * q_pitch (prima pitch poi roll)
    qm0 = cr * cy;
    qm1 = sr * cy;
    qm2 = cr * sy;
    qm3 = sr * sy;
    // Normalizza (dovrebbe già essere unitario)
    float norm = sqrtf(qm0*qm0 + qm1*qm1 + qm2*qm2 + qm3*qm3);
    if (norm > 0) {
        qm0 /= norm; qm1 /= norm; qm2 /= norm; qm3 /= norm;
    }
}

// Applica rotazione mounting al quaternione sensore -> veicolo
static void applyMountingToQuaternion() {
    // q_vehicle = q_mount * q_sensor
    qv0 = qm0*q0 - qm1*q1 - qm2*q2 - qm3*q3;
    qv1 = qm0*q1 + qm1*q0 + qm2*q3 - qm3*q2;
    qv2 = qm0*q2 - qm1*q3 + qm2*q0 + qm3*q1;
    qv3 = qm0*q3 + qm1*q2 - qm2*q1 + qm3*q0;
}

// ---------------------------------------------------------------------------
// Helpers I2C
// ---------------------------------------------------------------------------
static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    Wire.requestFrom((uint8_t)IMU_I2C_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// Legge `len` byte consecutivi a partire da `reg` in `buf`
static bool readRegs(uint8_t reg, uint8_t* buf, size_t len) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(IMU_I2C_ADDR, len);
    if (Wire.available() < len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

// ---------------------------------------------------------------------------
// Inizializzazione hardware ISM330DHCX
// ---------------------------------------------------------------------------
static bool initISM330() {
    // Reset del chip
    writeReg(ISM330_CTRL3_C, 0x01);   // SW_RESET
    delay(50);
    // Verifica WHO_AM_I
    if (readReg(ISM330_WHO_AM_I) != ISM330_WHO_AM_I_VAL) {
        Serial.printf("[IMU] WHO_AM_I errato – atteso 0x%02X\n", ISM330_WHO_AM_I_VAL);
        return false;
    }

    // Configura blocchi: acc e gyro abilitati, modalità continua
    writeReg(ISM330_CTRL1_XL, ACC_CONFIG);
    writeReg(ISM330_CTRL2_G,  GYR_CONFIG);
    writeReg(ISM330_CTRL3_C, 0x44);   // IF_INC = 1, BDU = 1, auto-increment, big-endian disabilitato
    writeReg(ISM330_CTRL4_C, 0x08);   // FIFO enable
    writeReg(ISM330_CTRL6_C, 0x00);   // gyro LPF2 disabilitato
    writeReg(ISM330_CTRL7_G, 0x00);   // gyro HPF disabilitato
    writeReg(ISM330_CTRL8_XL, 0x00);
    writeReg(ISM330_CTRL9_XL, 0x38);  // acc X,Y,Z abilitati
    writeReg(ISM330_CTRL10_C, 0x38);  // gyro X,Y,Z abilitati

    // Configura FIFO: modalità continua, watermark a FIFO_WATERMARK
    writeReg(ISM330_FIFO_CTRL1, (uint8_t)(FIFO_WATERMARK & 0xFF));
    writeReg(ISM330_FIFO_CTRL2, 0x06);   // FIFO continua (mode 6), watermark interrupt abilitato
    writeReg(ISM330_FIFO_CTRL3, 0x00);
    writeReg(ISM330_FIFO_CTRL4, 0x00);   // modalità FIFO (non Bypass)

    delay(50);
    return true;
}

// ---------------------------------------------------------------------------
// Applica calibrazione intrinseca (bias, scale) e runtime gyro correction
// ---------------------------------------------------------------------------
static void applyCalibration(int16_t rawAx, int16_t rawAy, int16_t rawAz,
                             int16_t rawGx, int16_t rawGy, int16_t rawGz,
                             float& ax, float& ay, float& az,
                             float& gx, float& gy, float& gz) {
    // Accelerometro
    ax = ((float)rawAx - calData.accBias[0]) * calData.accScale[0] / ACC_SCALE_4G;
    ay = ((float)rawAy - calData.accBias[1]) * calData.accScale[1] / ACC_SCALE_4G;
    az = ((float)rawAz - calData.accBias[2]) * calData.accScale[2] / ACC_SCALE_4G;

    // Giroscopio: bias statico + runtime
    gx = ((float)rawGx - calData.gyroBias[0] - gyroBiasRuntime[0]) / GYR_SCALE_500DPS;
    gy = ((float)rawGy - calData.gyroBias[1] - gyroBiasRuntime[1]) / GYR_SCALE_500DPS;
    gz = ((float)rawGz - calData.gyroBias[2] - gyroBiasRuntime[2]) / GYR_SCALE_500DPS;
}

// ---------------------------------------------------------------------------
// Lettura batch FIFO
// ---------------------------------------------------------------------------
static int readFifoBatch(uint8_t* buffer, size_t maxLen) {
    uint16_t level = 0;
    uint8_t status = readReg(ISM330_STATUS_REG);
    if (status & 0x01) {
        uint8_t low = readReg(ISM330_FIFO_STATUS1);
        uint8_t high = readReg(ISM330_FIFO_STATUS2) & 0x07;
        level = (high << 8) | low;
    }
    if (level == 0) return 0;
    if (level > FIFO_WATERMARK) level = FIFO_WATERMARK;
    size_t bytesToRead = level * BYTES_PER_SAMPLE;
    if (bytesToRead > maxLen) bytesToRead = maxLen;
    if (!readRegs(ISM330_FIFO_DATA_OUT_L, buffer, bytesToRead)) return 0;
    return bytesToRead;
}

// ---------------------------------------------------------------------------
// Filtro di Madgwick (con beta adattivo) — implementazione IMU (solo acc + gyro, no magnetometro)
//
// Aggiorna il quaternione globale q0,q1,q2,q3 con i nuovi campioni.
// Input:  accelerometro in G, giroscopio in °/s, dt in secondi
// Output: quaternione aggiornato (normalizzato)
//
// Algoritmo originale: Sebastian Madgwick, University of Bristol, 2010
// Questa implementazione è matematicamente equivalente alla reference C
// ma riscritta per chiarezza e senza dipendenze esterne.
// ---------------------------------------------------------------------------
static void madgwickUpdateAdaptive(float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float dt) {
    
    // Calcola fattore di confidenza per l'accelerometro
    float accelMag = sqrtf(ax*ax + ay*ay + az*az);
    float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);

    // Quanto la magnitudo accelerazione è vicina a 1G?
    float accWeight = 1.0f - fabsf(accelMag - 1.0f) / ACC_MAG_1G_TOL;
    accWeight = constrain(accWeight, 0.0f, 1.0f);

    // Riduci peso se giroscopio è molto attivo (alta dinamica angolare)
    float gyroFactor = 1.0f - constrain(gyroMag / GYRO_MAG_THRESH, 0.0f, 1.0f);
    float finalWeight = accWeight * gyroFactor;
    finalWeight = ACCEL_WEIGHT_MIN + finalWeight * (ACCEL_WEIGHT_MAX - ACCEL_WEIGHT_MIN);
    float beta = MADGWICK_BASE_BETA * finalWeight;

    // Converti giroscopio in rad/s
    const float DEG2RAD = M_PI / 180.0f;
    gx *= DEG2RAD; gy *= DEG2RAD; gz *= DEG2RAD;

    // Algoritmo di Madgwick (standard)
    float qDot0, qDot1, qDot2, qDot3;
    float s0, s1, s2, s3;
    float recipNorm;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // Derivata quaternione dal giroscopio
    qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // Correzione accelerometro (solo se il vettore non è nullo)
    float normAcc = sqrtf(ax*ax + ay*ay + az*az);
    if (normAcc > 0.0f && finalWeight > 0.01f) {
        ax /= normAcc; ay /= normAcc; az /= normAcc;

        _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
        _4q0 = 4.0f*q0; _4q1 = 4.0f*q1; _4q2 = 4.0f*q2;
        _8q1 = 8.0f*q1; _8q2 = 8.0f*q2;
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }

    // Integra
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normalizza
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// ---------------------------------------------------------------------------
// Conversione quaternione → angoli di Eulero (roll, pitch)
//
// Convenzione ZYX (yaw-pitch-roll), standard aeronautico / automotive.
// Restituisce gradi.
// ---------------------------------------------------------------------------
static void quatToRollPitch(float q0, float q1, float q2, float q3, float& roll, float& pitch) {
    // Roll (rotazione attorno X): atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1²+q2²))
    roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / M_PI);
 
    // Pitch (rotazione attorno Y): asin(2*(q0*q2 - q3*q1))
    // Clamp a [-1, 1] per evitare NaN da asinf su valori fuori range per errori numerici
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = constrain(sinp, -1.0f, 1.0f);
    pitch = asinf(sinp) * (180.0f / M_PI);
}

// ---------------------------------------------------------------------------
// Rotazione vettore accelerazione dal frame sensore al frame mondo (usando quaternione)
// e sottrazione della gravità per ottenere solo la componente dinamica.
//
// Il frame mondo ha Z+ verso l'alto, quindi la gravità è [0, 0, -1G].
// Sottraendo il vettore gravità nel frame mondo otteniamo l'accelerazione
// dinamica (quella causata dal moto del veicolo).
// ---------------------------------------------------------------------------
static void removeGravity(float ax, float ay, float az,
                          float q0, float q1, float q2, float q3,
                          float& lonAcc, float& latAcc) {
    // Ruota il vettore accelerazione dal frame sensore al frame mondo
    // usando la matrice di rotazione derivata dal quaternione.
    // Solo le componenti X e Y ci interessano (orizzontali nel frame mondo).
 
    // Componente X nel frame mondo (direzione avanti = longitudinale)
    float worldX = (1.0f - 2.0f*(q2*q2 + q3*q3)) * ax
                 + 2.0f*(q1*q2 - q0*q3)           * ay
                 + 2.0f*(q1*q3 + q0*q2)           * az;
 
    // Componente Y nel frame mondo (direzione laterale)
    float worldY = 2.0f*(q1*q2 + q0*q3)           * ax
                 + (1.0f - 2.0f*(q1*q1 + q3*q3))  * ay
                 + 2.0f*(q2*q3 - q0*q1)           * az;
 
    // Componente Z nel frame mondo (verticale — contiene la gravità -1G)
    // Non usata per lon/lat acc, serve solo per debug se necessario
 
    // worldX e worldY sono già l'accelerazione dinamica nel piano orizzontale
    // perché la rotazione ha già rimosso la componente gravitazionale
    // dal vettore accelerometro (la gravità nel frame mondo è solo Z).
 
    lonAcc =  worldX;   // avanti positivo = accelerazione
    latAcc = -worldY;   // convenzione: positivo = forza verso destra (curva sx)
}

// ---------------------------------------------------------------------------
// Rilevazione quasi‑statico (solo sensore)
// ---------------------------------------------------------------------------
static bool detectQuasiStatic(float ax, float ay, float az,
                              float gx, float gy, float gz) {
    float accMag = sqrtf(ax*ax + ay*ay + az*az);
    float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);
    return (fabsf(accMag - 1.0f) < 0.1f) && (gyroMag < 5.0f);
}

// ---------------------------------------------------------------------------
// Stima slope dedicata (filtro IIR con costante di tempo variabile)
// ---------------------------------------------------------------------------
static void updateSlopeEstimator(float currentPitch, bool quasiStatic, float dt) {
    float timeConstant = quasiStatic ? SLOPE_TC_QUASI_STATIC : SLOPE_TC_DYNAMIC;
    float alpha = dt / (dt + timeConstant);
    alpha = constrain(alpha, 0.0f, 0.1f);  // evita aggiornamenti troppo bruschi
    slopeFiltered = slopeFiltered * (1.0f - alpha) + currentPitch * alpha;
    imuData.slope = slopeFiltered;
}

// ---------------------------------------------------------------------------
// Runtime gyro bias refinement (opzionale, chiamato in imuUpdate)
// ---------------------------------------------------------------------------
static void updateGyroRuntimeRefinement(bool quasiStatic, float gx, float gy, float gz) {
    #if GYRO_REFINE_ENABLED
        unsigned long now = millis();
        if (quasiStatic) {
            refineQuasiStaticCount++;
            if (refineQuasiStaticCount >= GYRO_REFINE_MIN_SAMPLES &&
                (now - refineLastTime) >= GYRO_REFINE_RATE_MS) {
                refineLastTime = now;
                // Correzione lenta: media dei valori correnti (già con bias statico rimosso)
                // Vogliamo che il gyro tenda a zero in condizioni quasi-statiche.
                // gx, gy, gz sono già in deg/s.
                gyroBiasRuntime[0] = gyroBiasRuntime[0] * (1.0f - GYRO_REFINE_ALPHA) + gx * GYRO_REFINE_ALPHA;
                gyroBiasRuntime[1] = gyroBiasRuntime[1] * (1.0f - GYRO_REFINE_ALPHA) + gy * GYRO_REFINE_ALPHA;
                gyroBiasRuntime[2] = gyroBiasRuntime[2] * (1.0f - GYRO_REFINE_ALPHA) + gz * GYRO_REFINE_ALPHA;
                // Limita correzione massima
                for (int i=0; i<3; i++) {
                    gyroBiasRuntime[i] = constrain(gyroBiasRuntime[i], -GYRO_REFINE_MAX_CORR, GYRO_REFINE_MAX_CORR);
                }
            }
        } else {
            refineQuasiStaticCount = 0;
        }
    #else
        (void)quasiStatic; (void)gx; (void)gy; (void)gz;
    #endif
}

// ===========================================================================
// API PUBBLICA
// =========================================================================== 
bool imuInit() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);

    if (!initISM330()) {
        ready = false;
        return false;
    }

    // Carica calibrazione da NVS (se presente)
    if (!loadFromNVS()) {
        Serial.println("[IMU] Nessuna calibrazione valida in NVS, uso default.");
        calData.valid = false;
        // Default: bias zero, scale 1, mounting zero
        memset(calData.gyroBias, 0, sizeof(calData.gyroBias));
        memset(calData.accBias, 0, sizeof(calData.accBias));
        for (int i=0; i<3; i++) calData.accScale[i] = 1.0f;
        calData.mountingRoll = 0;
        calData.mountingPitch = 0;
    }
    updateMountingQuaternion();

    // Reset stato filtro
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    slopeFiltered = 0.0f;
    lastSlopeUpdate = millis();
    memset(gyroBiasRuntime, 0, sizeof(gyroBiasRuntime));
    refineQuasiStaticCount = 0;

    ready = true;
    imuData.valid = true;
    Serial.println("[IMU] ISM330DHCX inizializzato – FIFO + Madgwick adattivo");
    return true;
}

bool imuIsReady() { return ready; }
 
void imuUpdate() {
    if (!ready) return;

    // Legge batch dalla FIFO
    int bytesRead = readFifoBatch(fifoBuffer, FIFO_BURST_SIZE);
    if (bytesRead < BYTES_PER_SAMPLE) return;  // nessun campione

    int samplesInBatch = bytesRead / BYTES_PER_SAMPLE;
    if (samplesInBatch == 0) return;

    // Acquisizione timestamp monotono di inizio lettura
    uint64_t readStartUs = timeSyncNowUs();
    uint32_t dtUs = (uint32_t)(sampleDt * 1e6f);   // sampleDt = 1/208 ≈ 0.0048077 s → 4808 us

    // -----------------------------------------------------------------------
    // Ricostruzione timestamp dei campioni nel batch
    // Assumiamo che l'ultimo campione del batch sia stato acquisito
    // approssimativamente all'istante readStartUs (inizio lettura).
    // Il primo campione è più vecchio di (samplesInBatch-1)*dtUs.
    // -----------------------------------------------------------------------
    uint64_t firstSampleMonoUs = readStartUs - (samplesInBatch - 1) * dtUs;
    uint64_t lastSampleMonoUs = readStartUs;   // approssimazione

    // Elabora tutti i campioni del batch
    for (int i = 0; i < samplesInBatch; i++) {
        uint8_t* ptr = &fifoBuffer[i * BYTES_PER_SAMPLE];
        // Estrae raw (acc: 6 byte, gyro: 6 byte – little-endian)
        int16_t rawAx = (int16_t)((ptr[1] << 8) | ptr[0]);
        int16_t rawAy = (int16_t)((ptr[3] << 8) | ptr[2]);
        int16_t rawAz = (int16_t)((ptr[5] << 8) | ptr[4]);
        int16_t rawGx = (int16_t)((ptr[7] << 8) | ptr[6]);
        int16_t rawGy = (int16_t)((ptr[9] << 8) | ptr[8]);
        int16_t rawGz = (int16_t)((ptr[11] << 8) | ptr[10]);

        float ax, ay, az, gx, gy, gz;
        applyCalibration(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz,
                         ax, ay, az, gx, gy, gz);

        // Salva raw calibrati per debug
        imuData.accX_cal = ax;
        imuData.accY_cal = ay;
        imuData.accZ_cal = az;
        imuData.gyrX_cal = gx;
        imuData.gyrY_cal = gy;
        imuData.gyrZ_cal = gz;

        // Aggiorna filtro Madgwick con dt nominale
        madgwickUpdateAdaptive(ax, ay, az, gx, gy, gz, sampleDt);

        // Applica mounting rotation per ottenere q_vehicle
        applyMountingToQuaternion();

        // Calcola roll/pitch
        float roll, pitch;
        quatToRollPitch(qv0, qv1, qv2, qv3, roll, pitch);
        imuData.roll = roll;
        imuData.pitch = pitch;

        // Rimuovi gravità → accelerazioni dinamiche
        float lonAcc, latAcc;
        removeGravity(ax, ay, az, qv0, qv1, qv2, qv3, lonAcc, latAcc);
        imuData.lonAcc = lonAcc;
        imuData.latAcc = latAcc;

        // Rileva quasi-statico
        bool qs = detectQuasiStatic(ax, ay, az, gx, gy, gz);
        imuData.quasiStatic = qs;

        // Runtime gyro refinement (opzionale)
        updateGyroRuntimeRefinement(qs, gx, gy, gz);

        // Stima slope lenta (usa pitch corrente)
        unsigned long now = micros();
        float dtSlope = (now - lastSlopeUpdate) * 1e-6f;
        if (dtSlope > 0.05f) {  // aggiorna slope al massimo 20 Hz
            updateSlopeEstimator(pitch, qs, dtSlope);
            lastSlopeUpdate = now;
        }

        // Confidence della slope: alta in quasi-statico, bassa in dinamica
        float confidence = 0.0f;
        if (qs) confidence = 0.9f;
        else {
            float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);
            confidence = 1.0f - constrain(gyroMag / 30.0f, 0.0f, 0.8f);
        }
        imuData.slopeConfidence = confidence;
    }

    // Dopo aver processato tutto il batch, salva i metadati temporali
    imuData.batchReadMonoUs = readStartUs;
    imuData.batchSamples = samplesInBatch;
    imuData.sampleDtUs = dtUs;
    imuData.firstSampleMonoUs = firstSampleMonoUs;
    // Il campione più recente (ultimo del batch) ha timestamp = readStartUs
    imuData.lastSampleMonoUs = readStartUs;
    
    // Copia anche nella struttura condivisa (per altri moduli)
    vehicleData.lonAcc = imuData.lonAcc;
    vehicleData.latAcc = imuData.latAcc;
    vehicleData.roll   = imuData.roll;
    vehicleData.pitch  = imuData.pitch;
    vehicleData.slope  = imuData.slope;
    // opzionale: vehicleData potrebbe essere esteso con slopeConfidence e quasiStatic
}

ImuData imuGetData() {
    return imuData;   // copia per valore — thread-safe su ESP32 single-core loop
}

// ---------------------------------------------------------------------------
// API di calibrazione
// ---------------------------------------------------------------------------
bool imuLoadCalibration() {
    return loadFromNVS();
}

bool imuSaveCalibration() {
    updateMountingQuaternion();  // ricalcola quaternione prima di salvare
    return saveToNVS();
}

bool imuHasValidCalibration() {
    return calData.valid;
}

void imuResetCalibration() {
    memset(calData.gyroBias, 0, sizeof(calData.gyroBias));
    memset(calData.accBias, 0, sizeof(calData.accBias));
    for (int i=0; i<3; i++) calData.accScale[i] = 1.0f;
    calData.mountingRoll = 0;
    calData.mountingPitch = 0;
    calData.valid = true; // dopo reset viene considerata valida (default)
    saveToNVS();
    Serial.println("[IMU] Calibrazione resettata a default e salvata in NVS.");
}

bool imuRunGyroCalibration(unsigned int samples) {
    if (!ready) return false;
    Serial.printf("[IMU] Calibrazione giroscopio: acquisizione %d campioni...\n", samples);
    double sumX = 0, sumY = 0, sumZ = 0;
    for (unsigned int i = 0; i < samples; i++) {
        uint8_t buf[6];
        if (!readRegs(ISM330_OUTX_L_G, buf, 6)) {
            i--;
            delay(2);
            continue;
        }
        int16_t rx = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t ry = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t rz = (int16_t)((buf[5] << 8) | buf[4]);
        sumX += rx; sumY += ry; sumZ += rz;
        delay(2);
        if (i % 100 == 0) Serial.print(".");
    }
    calData.gyroBias[0] = (float)(sumX / samples);
    calData.gyroBias[1] = (float)(sumY / samples);
    calData.gyroBias[2] = (float)(sumZ / samples);
    Serial.printf("\n[IMU] Gyro bias: %.1f, %.1f, %.1f LSB\n",
        calData.gyroBias[0], calData.gyroBias[1], calData.gyroBias[2]);
    calData.valid = true;
    return true;
}

bool imuRunAccelCalibration(unsigned int samples) {
    if (!ready) return false;
    Serial.printf("[IMU] Calibrazione accelerometro (veicolo fermo e in piano).\n");
    Serial.printf("Acquisizione %d campioni...\n", samples);
    double sumX = 0, sumY = 0, sumZ = 0;
    for (unsigned int i = 0; i < samples; i++) {
        uint8_t buf[6];
        if (!readRegs(ISM330_OUTX_L_A, buf, 6)) {
            i--;
            delay(2);
            continue;
        }
        int16_t rx = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t ry = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t rz = (int16_t)((buf[5] << 8) | buf[4]);
        sumX += rx; sumY += ry; sumZ += rz;
        delay(2);
        if (i % 100 == 0) Serial.print(".");
    }
    float meanX = sumX / samples;
    float meanY = sumY / samples;
    float meanZ = sumZ / samples;
    // Atteso: 0, 0, +ACC_SCALE_4G (Z su)
    calData.accBias[0] = meanX - 0.0f;
    calData.accBias[1] = meanY - 0.0f;
    calData.accBias[2] = meanZ - ACC_SCALE_4G;
    // Calcola scala basata sul modulo
    float rawMag = sqrtf(meanX*meanX + meanY*meanY + meanZ*meanZ);
    float scaleCorrection = ACC_SCALE_4G / rawMag;
    for (int i=0; i<3; i++) calData.accScale[i] = scaleCorrection;
    Serial.printf("\n[IMU] Acc bias: %.1f, %.1f, %.1f LSB  scale: %.3f\n",
                  calData.accBias[0], calData.accBias[1], calData.accBias[2], scaleCorrection);
    calData.valid = true;
    return true;
}

void imuSetMountingAlignment(float roll_deg, float pitch_deg) {
    calData.mountingRoll = roll_deg;
    calData.mountingPitch = pitch_deg;
    updateMountingQuaternion();
    Serial.printf("[IMU] Mounting alignment impostato: roll=%.1f°, pitch=%.1f°\n",
        roll_deg, pitch_deg);
}

ImuCalibrationInfo imuGetCalibrationInfo() {
    ImuCalibrationInfo info;
    info.hasValidCalibration = calData.valid;
    info.usingDefaults = !calData.valid;  // se non valida, sta usando default
    info.gyroBiasX = calData.gyroBias[0];
    info.gyroBiasY = calData.gyroBias[1];
    info.gyroBiasZ = calData.gyroBias[2];
    info.accBiasX = calData.accBias[0];
    info.accBiasY = calData.accBias[1];
    info.accBiasZ = calData.accBias[2];
    info.accScaleX = calData.accScale[0];
    info.accScaleY = calData.accScale[1];
    info.accScaleZ = calData.accScale[2];
    info.mountingRoll = calData.mountingRoll;
    info.mountingPitch = calData.mountingPitch;
    return info;
}