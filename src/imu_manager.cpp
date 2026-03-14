// ---------------------------------------------------------------------------
// imu_manager.cpp
// Lettura MPU6500 via I2C + filtro di Madgwick per fusione acc+gyro.
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
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Registri MPU6500
// ---------------------------------------------------------------------------
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_CONFIG2   0x1D
#define MPU_REG_INT_ENABLE      0x38
#define MPU_REG_ACCEL_XOUT_H    0x3B   // 6 byte accel (X H/L, Y H/L, Z H/L)
#define MPU_REG_TEMP_OUT_H      0x41
#define MPU_REG_GYRO_XOUT_H     0x43   // 6 byte gyro  (X H/L, Y H/L, Z H/L)
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_WHO_AM_I        0x75
 
#define MPU_WHO_AM_I_VAL        0x70   // MPU6500 risponde 0x70

// Scala accelerometro: ±4G → sensibilità 8192 LSB/G
// (±2G = 16384, ±4G = 8192, ±8G = 4096, ±16G = 2048)
// ±4G è il range ottimale per un'auto: cattura frenate brusche senza saturare
#define ACCEL_RANGE_4G          0x08   // bit[4:3] = 01
#define ACCEL_SCALE             8192.0f

// Scala giroscopio: ±500 °/s → sensibilità 65.5 LSB/(°/s)
// Sufficiente per auto: anche in curva veloce raramente si superano 100°/s
#define GYRO_RANGE_500DPS       0x08   // bit[4:3] = 01
#define GYRO_SCALE              65.5f

// ---------------------------------------------------------------------------
// Parametri filtro di Madgwick
// ---------------------------------------------------------------------------
// Beta: guadagno del gradiente (trade-off velocità convergenza / rumore)
// 0.033 = valore raccomandato da Madgwick per AHRS a 6DOF
// Aumentare (es 0.1) se l'orientamento risponde lentamente agli input
// Diminuire (es 0.01) se l'output è troppo rumoroso
#define MADGWICK_BETA           0.033f
 
// Frequenza di campionamento target (Hz) — usata per l'integrazione del gyro
// Il loop reale misura il dt effettivo; questo è solo il valore iniziale
#define SAMPLE_FREQ_HZ          200.0f
 
// ---------------------------------------------------------------------------
// Stato interno — tutto privato al modulo
// ---------------------------------------------------------------------------
 
// Quaternione di orientamento Madgwick (normalizzato, w=1 = identità)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
 
// Offset giroscopio (calibrazione statica all'avvio)
static float _gyroOffX = 0.0f, _gyroOffY = 0.0f, _gyroOffZ = 0.0f;
 
// Timing
static unsigned long _lastUpdateUs = 0;
 
// Dati correnti (aggiornati da imuUpdate, letti da imuGetData)
static ImuData _data;
 
static bool _ready = false;

// ---------------------------------------------------------------------------
// Helpers I2C
// ---------------------------------------------------------------------------
static void _writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t _readReg(uint8_t reg) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)IMU_I2C_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// Legge `len` byte consecutivi a partire da `reg` in `buf`
static bool _readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)IMU_I2C_ADDR, len);
    if (Wire.available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

// ---------------------------------------------------------------------------
// Lettura raw accelerometro e giroscopio
// Formato: 3 registri H/L da 16 bit big-endian, complemento a 2
// ---------------------------------------------------------------------------
static bool _readRawSensors(float& ax, float& ay, float& az,
                             float& gx, float& gy, float& gz) {
    uint8_t buf[14];  // ACCEL_OUT(6) + TEMP(2) + GYRO_OUT(6)
    if (!_readRegs(MPU_REG_ACCEL_XOUT_H, buf, 14)) return false;
 
    int16_t rawAX = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t rawAY = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t rawAZ = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6..7] = temperatura — ignorata
    int16_t rawGX = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t rawGY = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t rawGZ = (int16_t)((buf[12] << 8) | buf[13]);
 
    // Converti in unità fisiche
    ax = (float)rawAX / ACCEL_SCALE;   // G
    ay = (float)rawAY / ACCEL_SCALE;   // G
    az = (float)rawAZ / ACCEL_SCALE;   // G
 
    gx = (float)rawGX / GYRO_SCALE;    // °/s
    gy = (float)rawGY / GYRO_SCALE;    // °/s
    gz = (float)rawGZ / GYRO_SCALE;    // °/s
 
    return true;
}

// ---------------------------------------------------------------------------
// Filtro di Madgwick — implementazione IMU (solo acc + gyro, no magnetometro)
//
// Aggiorna il quaternione globale q0,q1,q2,q3 con i nuovi campioni.
// Input:  accelerometro in G, giroscopio in °/s, dt in secondi
// Output: quaternione aggiornato (normalizzato)
//
// Algoritmo originale: Sebastian Madgwick, University of Bristol, 2010
// Questa implementazione è matematicamente equivalente alla reference C
// ma riscritta per chiarezza e senza dipendenze esterne.
// ---------------------------------------------------------------------------
static void _madgwickUpdate(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             float dt) {
    // Converti giroscopio in rad/s
    const float DEG2RAD = M_PI / 180.0f;
    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;
 
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;
 
    // Derivata del quaternione dall'integrazione del giroscopio
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
 
    // Applica correzione gradiente solo se l'accelerometro non è nullo
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (accNorm > 0.0f) {
        // Normalizza accelerometro
        recipNorm = 1.0f / accNorm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
 
        // Pre-calcola termini quadratici (riduce moltiplicazioni)
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1;
        q2q2 = q2 * q2; q3q3 = q3 * q3;
 
        // Funzione gradiente (obiettivo: allineare il vettore z con la gravità)
        // Derivata dell'errore di orientamento rispetto al quaternione
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
             - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
             - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
 
        // Normalizza il gradiente
        recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm;
        s2 *= recipNorm; s3 *= recipNorm;
 
        // Applica la correzione alla derivata del quaternione
        qDot0 -= MADGWICK_BETA * s0;
        qDot1 -= MADGWICK_BETA * s1;
        qDot2 -= MADGWICK_BETA * s2;
        qDot3 -= MADGWICK_BETA * s3;
    }
 
    // Integra: q_new = q_old + qDot * dt
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
 
    // Rinormalizza il quaternione (mantiene |q| = 1)
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
}

// ---------------------------------------------------------------------------
// Conversione quaternione → angoli di Eulero (roll, pitch)
//
// Convenzione ZYX (yaw-pitch-roll), standard aeronautico / automotive.
// Restituisce gradi.
// ---------------------------------------------------------------------------
static void _quaternionToEuler(float& roll, float& pitch) {
    // Roll (rotazione attorno X): atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1²+q2²))
    roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / M_PI);
 
    // Pitch (rotazione attorno Y): asin(2*(q0*q2 - q3*q1))
    // Clamp a [-1, 1] per evitare NaN da asinf su valori fuori range per errori numerici
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = (sinp >  1.0f) ?  1.0f : sinp;
    sinp = (sinp < -1.0f) ? -1.0f : sinp;
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
static void _removeGravity(float ax, float ay, float az,
                            float& lonOut, float& latOut) {
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
 
    lonOut =  worldX;   // avanti positivo = accelerazione
    latOut = -worldY;   // convenzione: positivo = forza verso destra (curva sx)
}

// ---------------------------------------------------------------------------
// Calibrazione offset giroscopio
// Media N campioni a sensore fermo → offset da sottrarre ad ogni lettura
// ---------------------------------------------------------------------------
static void _calibrateGyro(int numSamples = 500) {
    double sumX = 0, sumY = 0, sumZ = 0;
    float ax, ay, az, gx, gy, gz;
 
    Serial.print("[IMU] Calibrazione giroscopio");
    for (int i = 0; i < numSamples; i++) {
        if (_readRawSensors(ax, ay, az, gx, gy, gz)) {
            sumX += gx; sumY += gy; sumZ += gz;
        }
        if (i % 50 == 0) Serial.print(".");
        delay(2);
    }
    Serial.println(" OK");
 
    _gyroOffX = (float)(sumX / numSamples);
    _gyroOffY = (float)(sumY / numSamples);
    _gyroOffZ = (float)(sumZ / numSamples);
 
    Serial.printf("[IMU] Offset gyro: %.3f  %.3f  %.3f °/s\n",
                  _gyroOffX, _gyroOffY, _gyroOffZ);
}

// ===========================================================================
// API PUBBLICA
// ===========================================================================
 
bool imuInit() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);   // 400kHz fast mode — MPU6500 supporta fino a 400kHz
 
    // Verifica WHO_AM_I
    uint8_t whoami = _readReg(MPU_REG_WHO_AM_I);
    if (whoami != MPU_WHO_AM_I_VAL) {
        Serial.printf("[IMU] WHO_AM_I atteso 0x70, ricevuto 0x%02X — sensore non trovato\n", whoami);
        return false;
    }
 
    // Wake up (di default il chip è in sleep dopo power-on)
    _writeReg(MPU_REG_PWR_MGMT_1, 0x00);
    delay(100);
 
    // Seleziona clock interno PLL (più stabile del clock RC interno)
    _writeReg(MPU_REG_PWR_MGMT_1, 0x01);
    delay(10);
 
    // Sample rate divider: SMPLRT_DIV = 0 → sample rate = Gyro Output Rate / 1
    // Con DLPF attivo: Gyro Output Rate = 1kHz → sample rate = 1kHz
    // Noi leggiamo a ~200Hz nel loop, il chip oversampling è ok
    _writeReg(MPU_REG_SMPLRT_DIV, 0x00);
 
    // DLPF (Digital Low Pass Filter) = modalità 3 → fc = 41Hz acc / 42Hz gyro
    // Filtra le vibrazioni meccaniche alte (motore, strada) — complementa la piastra anti-vib
    // Modalità disponibili: 0=260Hz, 1=184Hz, 2=94Hz, 3=41Hz, 4=20Hz, 5=10Hz, 6=5Hz
    _writeReg(MPU_REG_CONFIG, 0x03);
 
    // Accelerometro ±4G
    _writeReg(MPU_REG_ACCEL_CONFIG, ACCEL_RANGE_4G);
 
    // Accelerometro DLPF: fc = 41Hz (registro separato su MPU6500)
    _writeReg(MPU_REG_ACCEL_CONFIG2, 0x03);
 
    // Giroscopio ±500 °/s
    _writeReg(MPU_REG_GYRO_CONFIG, GYRO_RANGE_500DPS);
 
    // Disabilita interrupt hardware (non li usiamo — polling nel loop)
    _writeReg(MPU_REG_INT_ENABLE, 0x00);
 
    delay(50);  // Aspetta stabilizzazione DLPF
 
    // Calibrazione offset giroscopio (sensore DEVE essere fermo durante l'avvio)
    _calibrateGyro(500);
 
    // Inizializza quaternione all'identità
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    _lastUpdateUs = micros();
 
    _data.valid = true;
    _ready = true;
    Serial.println("[IMU] MPU6500 inizializzato — filtro Madgwick attivo");
    return true;
}

bool imuIsReady() { return _ready; }
 
void imuUpdate() {
    if (!_ready) return;
 
    float ax, ay, az, gx, gy, gz;
    if (!_readRawSensors(ax, ay, az, gx, gy, gz)) return;
 
    // Applica offset giroscopio
    gx -= _gyroOffX;
    gy -= _gyroOffY;
    gz -= _gyroOffZ;
 
    // dt reale in secondi (evita drift da timing irregolare del loop)
    unsigned long nowUs = micros();
    float dt = (float)(nowUs - _lastUpdateUs) * 1e-6f;
    _lastUpdateUs = nowUs;
 
    // Clamp dt: se il loop si è bloccato per >100ms ignora quel campione
    // (evita salti di quaternione per interruzioni lunghe tipo SD write)
    if (dt <= 0.0f || dt > 0.1f) return;
 
    // Aggiorna filtro di Madgwick
    _madgwickUpdate(ax, ay, az, gx, gy, gz, dt);
 
    // Calcola angoli di Eulero
    float roll, pitch;
    _quaternionToEuler(roll, pitch);
 
    // Rimuovi gravità e ottieni accelerazioni dinamiche
    float lonAcc, latAcc;
    _removeGravity(ax, ay, az, lonAcc, latAcc);
 
    // Pendenza longitudinale: il pitch a velocità costante riflette
    // l'inclinazione del terreno. In accelerazione/frenata è distorto
    // dalla forza inerziale, ma è comunque utile come dato di contesto.
    // Usiamo direttamente il pitch del filtro (già compensato dal gyro).
    float slope = pitch;
 
    // Aggiorna struttura dati pubblica
    _data.lonAcc  = lonAcc;
    _data.latAcc  = latAcc;
    _data.roll    = roll;
    _data.pitch   = pitch;
    _data.slope   = slope;
    _data.rawAccX = ax;
    _data.rawAccY = ay;
    _data.rawAccZ = az;
    _data.valid   = true;
 
    // Aggiorna anche shared_data per rgb_controller e web_server
    vehicleData.lonAcc = lonAcc;
    vehicleData.latAcc = latAcc;
}

ImuData imuGetData() {
    return _data;   // copia per valore — thread-safe su ESP32 single-core loop
}