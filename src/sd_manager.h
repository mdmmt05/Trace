#pragma once // evita inclusioni doppie

#include <SD.h>

// Inizializza SPI e monta la SD - restituisce true se ok
bool sdInit();

// Apre o crea il file CSV con header
bool sdOpenFile(const char* path);

// Scrive una riga di dati
void sdWriteRow(
    const char* timestamp,
    float lat,  float lon,      // gradi decimali
    float alt,                  // metri (GNSS)
    uint8_t satellites,         // n. satelliti
    float hdop,                 // precisione GNSS
    float speedObd,             // km/h (OBD2)
    float lonAcc, float latAcc, // G (IMU)
    float roll,   float pitch,  // ° (IMU)
    float slope,                // ° pendenza stimata (IMU)
    uint8_t slopeReliable,      // 1 = velocità stabile → slope affidabile, 0 = distorta
    int rpm,
    int load,                   // %
    float throttle              // %
);

// Flush manuale (normalmente automatico ogni N righe)
void sdFlush();

// Chiude il file correttamente
void sdClose();