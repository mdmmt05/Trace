#pragma once
 
// ---------------------------------------------------------------------------
// imu_manager.h
// Gestione ISM330DHCX (accelerometro + giroscopio 6-DOF) via I2C.
//
// Caratteristiche:
//   - Lettura FIFO con timestamp basato su ODR fisso (nessun jitter del loop)
//   - Calibrazione persistente in NVS (Preferences) - separata tra:
//      - Intrinseca: gyro bias, acc bias, acc scale
//      - Mounting: roll e pitch del sensore rispetto al veicolo
//   - Filtro di Madgwick 6DOF con beta adattivo (peso accelerometro ridotto in condizioni dinamiche)
//   - Rilevazione quasi-statica (modulo aggelerazione circa pari a 1g e gyro basso)
//   - Stima slope dedicata (aggiornata lentamente in quasi-statica)
//   - Output di confidence per la slope (0...1)
//
// API di commissioning:
//   imuRunGyroCalibration()   – calibrazione giroscopio (veicolo fermo)
//   imuRunAccelCalibration()  – calibrazione accelerometro (veicolo fermo e in piano)
//   imuSetMountingAlignment() – imposta offset di montaggio (gradi)
//   imuSaveCalibration()      – salva in NVS
//   imuResetCalibration()     – ripristina default e salva
//   imuLoadCalibration()      – ricarica da NVS
//   imuHasValidCalibration()  – true se NVS contiene dati validi
//   imuGetCalibrationInfo()   – restituisce struct diagnostica
//
// Convenzione assi — modulo montato PARALLELO AL PAVIMENTO, X VERSO IL MUSO:
//   X+  → avanti  (accelerazione longitudinale positiva = accelerazione)
//   Y+  → sinistra (accelerazione laterale positiva = curva a destra)
//   Z+  → alto
//
// ---------------------------------------------------------------------------

// Pin I2C - modifica se usi pin diversi
#ifndef IMU_SDA_PIN
#define IMU_SDA_PIN 8
#endif
#ifndef IMU_SCL_PIN
#define IMU_SCL_PIN 9
#endif

// Indirizzo I2C MPU6500 (SA0 = GND -> 0x6A, SA0 = VCC → 0x6B)
#ifndef IMU_I2C_ADDR
#define IMU_I2C_ADDR 0x6A
#endif

// ---------------------------------------------------------------------------
// Struttura dati IMU — letta da imuGetData()
// ---------------------------------------------------------------------------
struct ImuData {
    // Accelerazioni dinamiche (gravità rimossa, filtrate) - in G
    float lonAcc = 0.0f;     // longitudinale: positivo = accelerazione in avanti
    float latAcc = 0.0f;     // laterale:      positivo = forza verso destra (curva sx)
 
    // Orientamento assoluto stimato dal filtro di Madgwick — in gradi
    float roll     = 0.0f;   // rollio:    positivo = lato destro in basso
    float pitch    = 0.0f;   // beccheggio: positivo = muso in alto
    
    // Pendenza stimata (lenta, affidabile) e relativa confidenza
    float slope           = 0.0f;   // pendenza longitudinale (in gradi sessagesimali)
    float slopeConfidence = 0.0f;   // 0 = inaffidabile, 1 = molto affidabile
 
    // Flag quasi-statico (veicolo fermo o velocità costante su piano)
    bool quasiStatic = false;

    // Dati raw calibrati (utili per debug)
    float accX_cal = 0.0f, accY_cal = 0.0f, accZ_cal = 0.0f;
    float gyrX_cal = 0.0f, gyrY_cal = 0.0f, gyrZ_cal = 0.0f;
 
    bool  valid    = false;  // false finché il sensore non è inizializzato
};

// ---------------------------------------------------------------------------
// API pubblica
// ---------------------------------------------------------------------------

// Inizializza I2C e ISM330DHCX. Esegue calibrazione statica (1 secondo).
// Restituisce true se il sensore risponde correttamente.
bool imuInit();

// Legge nuovi dati dalla FIFO, aggiorna il filtro di Madgwick.
// Chiamare nel loop() - la frequenza di lettura non è critica grazie alla FIFO.
void imuUpdate();

// Restituisce una copia thread-safe dei dati IMU correnti.
ImuData imuGetData();

// Restituisce true se il sensore è stato inizializzato con successo.
bool imuIsReady();