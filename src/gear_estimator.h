#pragma once

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// gear_estimator.h
// Stima della marcia inserita basata su rapporto RPM/velocità.
// Calibrazione persistente in NVS, comandi seriali dedicati.
// ---------------------------------------------------------------------------

#define GEAR_MAX_GEARS        5     // marce 1..5, marcia 0 = folle, indeterminata
#define GEAR_MIN_SPEED        5.0f  // km/h - sotto questa velocità -> marcia 0
#define GEAR_MIN_RPM          500   // giri/min - sotto -> marcia 0
#define GEAR_MATCH_THRESHOLD  0.15f // errore relativo massimo per accettare una marcia
#define GEAR_HYSTERESIS_COUNT 3     // campioni consecutivi prima di cambiare marcia

// ---------------------------------------------------------------------------
// Strutture di calibrazione (persistenti)
// ---------------------------------------------------------------------------
typedef struct {
    float speedRefKmh; // velocità media al momento della calibrazione (solo debug)
    float rpmRef;      // RPM medio al momento della calibrazione
    float ratioK;      // RPM/speed (km/h) - parametro caratteristico
    bool valid;
} GearCalibration;

typedef struct {
    uint32_t version;  // per futuri aggiornamenti
    uint16_t checksum;
    GearCalibration gears[GEAR_MAX_GEARS + 1]; // indice 0 non usato, 1..5
    bool valid;
} GearCalibrationData;

// ---------------------------------------------------------------------------
// Informazioni diagnostiche per i comandi seriali
// ---------------------------------------------------------------------------
typedef struct {
    bool hasValidCalibration;
    bool usingDefaults;
    float ratioForGear[GEAR_MAX_GEARS + 1]; // 1..5
    bool validForGear[GEAR_MAX_GEARS + 1];
    float rpmRefForGear[GEAR_MAX_GEARS + 1];   // 1..5
    float speedRefForGear[GEAR_MAX_GEARS + 1];
} GearCalibrationInfo;

// ---------------------------------------------------------------------------
// API pubbliche
// ---------------------------------------------------------------------------
void gearEstimatorInit();                                                         // chiamare in setup()
void gearEstimatorUpdate();                                                       // chiamare nel loop() (tipicamente 20-50Hz)

int  gearEstimatorGetCurrentGear();                                               // 0 = nessuna marcia, 1..5 = marcia
bool gearEstimatorIsGearValid();                                                  // true se la stima è affidabile
uint64_t gearEstimatorGetTimestampUs();                                           // timestamp monotono dell'ultimo aggiornamento

// Calibrazione
bool gearEstimatorLoadCalibration();                                              // carica da NVS (chiamato da init)
bool gearEstimatorSaveCalibration();                                              // salva in NVS
void gearEstimatorResetCalibration();                                             // resetta a default (nessuna marcia calibrata)
bool gearEstimatorHasValidCalibration();                                          // true se esiste almeno una marcia calibrata

// Comandi per la procedura di calibrazione (usati da handleSerialCommands)
bool gearEstimatorStartCapture(int gear);                                         // avvia finestra di acquisizione
bool gearEstimatorIsCapturing();                                                  // true se una finestra è attiva
void gearEstimatorAbortCapture();                                                 // interrompe senza salvare
bool gearEstimatorGetCaptureResult(float &ratio, float &speedAvg, float &rpmAvg); // true se acquisizione completata con successo
int  gearEstimatorGetCapturingGear();                                             // marcia per cui è stata avviata l'acquisizione
bool gearEstimatorSaveCapturedGear();                                             // salva in NVS l'ultima acquisizione stabile (usa la marcia registrata)

GearCalibrationInfo gearEstimatorGetCalibrationInfo();