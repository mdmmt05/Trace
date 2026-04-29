#pragma once

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// vehicle_fusion_manager.h
//
// Strato di fusione sensori per veicolo:
//   - heading stimato (0-360°) da gyro + GNSS course
//   - yaw rate filtrato
//   - velocità fusa (GNSS + OBD)
//   - posizione filtrata (low‑pass su lat/lon)
//   - confidenze
//
// Dipende da: imu_manager, gnss_manager, shared_data, time_sync_manager
// ---------------------------------------------------------------------------

typedef struct {
    float heading_deg;           // 0-360°, 0 = Nord
    float yawRate_dps;           // velocità angolare Z (deg/s)
    float speed_mps;             // velocità fusa (m/s)
    double lat;                  // latitudine filtrata (gradi decimali)
    double lon;                  // longitudine filtrata

    float headingConfidence;     // 0..1
    float positionConfidence;    // 0..1

    bool valid;                  // almeno un aggiornamento valido

    uint64_t timestampUs;        // tempo monotono dell'ultimo aggiornamento
} VehicleState;

// Inizializza lo stato del filtro (chiamare una volta in setup)
void vehicleFusionInit();

// Aggiorna la stima usando i dati più recenti dei sensori.
// Chiamare nel loop (tipicamente 20-100Hz).
void vehicleFusionUpdate();

// Restituisce una copia dello stato corrente (thread-safe su core singolo)
VehicleState vehicleFusionGetState();