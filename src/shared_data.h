#pragma once

// ---------------------------------------------------------------------------
// shared_data.h
// Struttura dati condivisa tra tutti i moduli.
//
// Produttori:
//   - OBD2 (UART simulata ora, TWAI in futuro) → VehicleData
//   - imu_manager                              → lonAcc, latAcc, roll, pitch, slope
//   - GNSS manager                             → GnssData (via gnssGetData())
//
// Consumatori: rgb_controller, web_server, sd_manager
// ---------------------------------------------------------------------------

struct VehicleData {
    // OBD2
    volatile int   rpm      = 0;
    volatile float speed    = 0.0f;   // km/h
    volatile int   load     = 0;      // %
    volatile float throttle = 0.0f;   // %
    volatile float coolant  = 0.0f;   // °C
 
    // IMU (imu_manager)
    volatile float lonAcc   = 0.0f;   // G  — accelerazione longitudinale (avanti+)
    volatile float latAcc   = 0.0f;   // G  — accelerazione laterale (destra+)
    volatile float roll     = 0.0f;   // °  — rollio   (lato destro in basso +)
    volatile float pitch    = 0.0f;   // °  — beccheggio (muso in alto +)
    volatile float slope    = 0.0f;   // °  — pendenza longitudinale stimata
};

extern VehicleData vehicleData;
