#pragma once
 
// ---------------------------------------------------------------------------
// imu_manager.h
// Gestione MPU6500 (accelerometro + giroscopio 6-DOF) via I2C.
//
// Produce via fusione sensori (filtro di Madgwick):
//   - Accelerazione longitudinale / laterale in G (gravità sottratta)
//   - Rollio (roll)  e beccheggio (pitch) in gradi
//   - Stima pendenza longitudinale in gradi (affidabile a veicolo fermo
//     o a velocità costante; degradata in accelerazione/frenata)
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

// Indirizzo I2C MPU6500 (AD0 = GND -> 0x68, AD0 = VCC → 0x69)
#ifndef IMU_I2C_ADDR
#define IMU_I2C_ADDR 0x68
#endif

// ---------------------------------------------------------------------------
// Struttura dati IMU — letta da imuGetData()
// ---------------------------------------------------------------------------
struct ImuData {
    // Accelerazioni dinamiche (gravità rimossa, filtrate) - in G
    float lonAcc = 0.0f; // longitudinale: positivo = accelerazione in avanti
    float latAcc   = 0.0f;   // laterale:      positivo = forza verso destra (curva sx)
 
    // Orientamento assoluto stimato dal filtro di Madgwick — in gradi
    float roll     = 0.0f;   // rollio:    positivo = lato destro in basso
    float pitch    = 0.0f;   // beccheggio: positivo = muso in alto
    float slope    = 0.0f;   // pendenza longitudinale stimata (≈ pitch a vel. costante)
 
    // Accelerazioni raw (con gravità) — utili per debug
    float rawAccX  = 0.0f;
    float rawAccY  = 0.0f;
    float rawAccZ  = 0.0f;
 
    bool  valid    = false;  // false finché il sensore non è inizializzato
};

// ---------------------------------------------------------------------------
// API pubblica
// ---------------------------------------------------------------------------

// Inizializza I2C e MPU6500. Esegue calibrazione offset giroscopio (1 secondo).
// Restituisce true se il sensore risponde correttamente.
bool imuInit();

// Legge nuovi dati dal MPU6500, aggiorna il filtro di Madgwick.
// Chiamare nel loop() il più frequentemente possibile (idealmente ogni 2-5ms).
void imuUpdate();

// Restituisce una copia thread-safe dei dati IMU correnti.
ImuData imuGetData();

// Restituisce true se il sensore è stato inizializzato con successo.
bool imuIsReady();