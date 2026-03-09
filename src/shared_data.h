#pragma once

// ---------------------------------------------------------------------------
// shared_data.h
// Struttura dati condivisa tra tutti i moduli.
//
// Produttori:
//   - OBD2 (UART simulata ora, TWAI in futuro) → VehicleData
//   - GNSS manager                              → GnssData (via gnssGetData())
//
// Consumatori: rgb_controller, web_server, sd_manager
// ---------------------------------------------------------------------------

struct VehicleData {
    volatile int   rpm      = 0;
    volatile float speed    = 0.0f;   // km/h  (da OBD2)
    volatile int   load     = 0;      // %
    volatile float throttle = 0.0f;   // %
    volatile float lonAcc   = 0.0f;   // G    — accelerometro, futuro
    volatile float latAcc   = 0.0f;   // G    — accelerometro, futuro
    volatile float coolant  = 0.0f;
};

extern VehicleData vehicleData;
