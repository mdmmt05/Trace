#pragma once // evita inclusioni doppie

#include <SD.h>

// Inizializza SPI e monta la SD - restituisce true se ok
bool sdInit();

// Apre o crea il file CSV con header
bool sdOpenFile(const char* path);

// Scrive una riga di dati
void sdWriteRow(
    const char* timestamp,
    float lat, float lon,
    float alt,
    uint8_t satellites,
    float hdop,
    float speedObd,
    float lonAcc, float latAcc,
    int rpm,
    int load,
    float throttle
);

// Flush manuale (normalmente automatico ogni N righe)
void sdFlush();

// Chiude il file correttamente
void sdClose();