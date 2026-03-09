#pragma once

// ---------------------------------------------------------------------------
// rgb_controller.h
// Gestione LED RGB non indirizzabili (common anode) su ESP32-S3.
// Supporta più modalità/algoritmi selezionabili a runtime.
// I pin vengono passati all'init — nulla è hardcodato.
// ---------------------------------------------------------------------------

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Modalità disponibili
// ---------------------------------------------------------------------------
enum RgbMode {
    RGB_STATIC = 0, // Colore fisso
    RGB_FADING,       // Transizione ciclica tra colori
    RGB_BREATHING,    // Fade in-out su colore fisso
    RGB_RPM_COLOR,    // Gradiente freddo→caldo in base agli RPM
    RGB_RPM_WARNING,  // Lampeggio rosso al superamento soglia RPM
    RGB_MODE_COUNT    // Sentinella — tieni sempre per ultima
};

// Nomi leggibili (utili per webserver/seriale)
extern const char* const RgbModeNames[RGB_MODE_COUNT];

// ---------------------------------------------------------------------------
// Parametri configurabili a runtime via webserver
// ---------------------------------------------------------------------------
struct RgbParams {
    // Colore base (usato da STATIC, BREATHING, RPM_WARNING)
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 255;

    // Velocità generica (0-100): fading speed, breathing speed
    uint8_t speed = 50;
    
    // Luminosità globale (0 - 100 %)
    uint8_t brightness = 100;

    // Soglia RPM per RGB_RPM_WARNING
    int rpmThreshold = 6000;

    // RPM massimi di riferimento per RGB_RPM_COLOR
    int rpmMax = 8000;
};

// ---------------------------------------------------------------------------
// API pubblica
// ---------------------------------------------------------------------------

// Inizializza i pin LEDC (PWM) dell'ESP32-S3.
// Chiamare una sola volta nel setup().
// pinR, pinG, pinB: pin fisici a cui sono collegati i gate dei MOSFET/transistor
void rgbInit(int pinR, int pinG, int pinB);

// Aggiorna il loop degli algoritmi - chiamare nel loop() principale senza delay().
void rgbUpdate();

// Imposta la modalità attiva.
void rgbSetMode(RgbMode mode);
RgbMode rgbGetMode();

// Imposta/leggi i parametri.
void rgbSetParams(const RgbParams& params);
RgbParams rgbGetParams();

// Imposta colore diretto (forza temporaneamente RGB_STATIC).
void rgbSetColor(uint8_t r, uint8_t g, uint8_t b);

// Spegni i LED.
void rgbOff();