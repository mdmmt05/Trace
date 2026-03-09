#pragma once

// ---------------------------------------------------------------------------
// gnss_manager.h
// Gestione modulo GNSS ATGM336H via HardwareSerial + TinyGPSPlus.
//
// Flusso:
//   1. gnssInit()       — avvia UART e TinyGPS++
//   2. gnssUpdate()     — va chiamato nel loop(), alimenta il parser
//   3. gnssHasFix()     — true quando lat/lon/data/ora sono validi
//   4. gnssGetData()    — restituisce snapshot dei dati correnti
// ---------------------------------------------------------------------------

#include <Arduino.h>

// Dati GNSS snapshot (tutti i campi che servono al resto del sistema)
struct GnssData {
    bool valid = false;

    float lat = 0.0f;
    float lon = 0.0f;
    float altMeters = 0.0f;
    float speedKmh = 0.0f;

    uint8_t satellites = 0;
    float hdop = 99.9f;

    // Data/ora UTC dal GNSS
    uint16_t year       = 0;
    uint8_t  month      = 0;
    uint8_t  day        = 0;
    uint8_t  hour       = 0;
    uint8_t  minute     = 0;
    uint8_t  second     = 0;
};

// Inizializza UART GNSS. Chiamare nel setup().
void gnssInit();

// Alimenta il parser TinyGPS++. Chiamare ogni loop(), senza delay.
void gnssUpdate();

// Restituisce true se lat/lon e data/ora sono validi e aggiornati.
bool gnssHasFix();

// Snapshot dei dati correnti.
GnssData gnssGetData();

// Formatta data/ora UTC come stringa ISO: "2025-03-07 14:23:01"
// buf deve essere almeno 20 caratteri.
void gnssFormatTimestamp(char* buf, size_t bufSize);
