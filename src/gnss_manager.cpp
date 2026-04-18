// ---------------------------------------------------------------------------
// gnss_manager.cpp
// ---------------------------------------------------------------------------

#include "gnss_manager.h"
#include "time_sync_manager.h"
#include <TinyGPSPlus.h>

// ---------------------------------------------------------------------------
// Pin e baud — stessi del codice di test già validato
// ---------------------------------------------------------------------------
#ifndef GNSS_TX
#define GNSS_TX 42
#endif
#ifndef GNSS_RX
#define GNSS_RX 41
#endif
#ifndef GNSS_BAUD
#define GNSS_BAUD 9600
#endif

static HardwareSerial gnssSerial(1);
static TinyGPSPlus gps;

// ---------------------------------------------------------------------------
// Stato interno persistente per il timestamp reale del fix
// ---------------------------------------------------------------------------
static GnssData s_lastValidData;          // ultimo snapshot valido
static uint64_t s_lastValidFixMonoUs = 0; // timestamp di ricezione dell'ultimo fix valido
static bool s_haveValidSnapshot = false;  // se almeno un fix valido è stato ricevuto

// ---------------------------------------------------------------------------
// Helpers interni
// ---------------------------------------------------------------------------
// Flag per capire se il fix è stato aggiornato (usando i metodi isUpdated di TinyGPS++)
static bool isFixUpdated() {
    // Consideriamo un fix "aggiornato" se almeno uno tra posizione, data o ora è stato aggiornato
    // (in pratica quando arriva una nuova stringa NMEA che modifica uno di questi campi)
    return gps.location.isUpdated() || gps.date.isUpdated() || gps.time.isUpdated();
}

// Fix valido = posizione + data + ora tutti validi
static bool _isFixValid() {
    return gps.location.isValid()
        && gps.date.isValid()
        && gps.time.isValid();
}

// Confronta due GnssData per vedere se sono cambiati (lat/lon/data/ora)
static bool isGnssDataDifferent(const GnssData& a, const GnssData& b) {
    if (a.lat != b.lat || a.lon != b.lon) return true;
    if (a.year != b.year || a.month != b.month || a.day != b.day) return true;
    if (a.hour != b.hour || a.minute != b.minute || a.second != b.second) return true;
    return false;
}

// Aggiorna lo snapshot interno se abbiamo un nuovo fix valido
static void updateInternalSnapshot() {
    if (!_isFixValid()) return;

    // Se non abbiamo ancora uno snapshot valido, lo creaiamo subito
    if (!s_haveValidSnapshot) {
        s_lastValidData.valid = true;
        s_lastValidData.utcValid = true;
        // copia campi
        if (gps.location.isValid()) {
            s_lastValidData.lat = (float)gps.location.lat();
            s_lastValidData.lon = (float)gps.location.lng();
        }
        if (gps.altitude.isValid())    s_lastValidData.altMeters  = (float)gps.altitude.meters();
        if (gps.speed.isValid())       s_lastValidData.speedKmh   = (float)gps.speed.kmph();
        if (gps.satellites.isValid())  s_lastValidData.satellites = (uint8_t)gps.satellites.value();
        if (gps.hdop.isValid())        s_lastValidData.hdop       = (float)gps.hdop.hdop();
        if (gps.date.isValid()) {
            s_lastValidData.year  = gps.date.year();
            s_lastValidData.month = gps.date.month();
            s_lastValidData.day   = gps.date.day();
        }
        if (gps.time.isValid()) {
            s_lastValidData.hour   = gps.time.hour();
            s_lastValidData.minute = gps.time.minute();
            s_lastValidData.second = gps.time.second();
        }
        s_lastValidFixMonoUs = timeSyncNowUs();
        s_haveValidSnapshot = true;
        return;
    }

    // Se abbiamo già uno snapshot, controlliamo se il fix è cambiato (usando i flag di update)
    bool updated = isFixUpdated();
    if (!updated) return;

    // Opzionale: ulteriore controllo differenza valori per evitare falsi update
    GnssData temp;
    temp.valid = true;
    temp.utcValid = true;
    if (gps.location.isValid()) {
        temp.lat = (float)gps.location.lat();
        temp.lon = (float)gps.location.lng();
    }
    if (gps.altitude.isValid())    temp.altMeters  = (float)gps.altitude.meters();
    if (gps.speed.isValid())       temp.speedKmh   = (float)gps.speed.kmph();
    if (gps.satellites.isValid())  temp.satellites = (uint8_t)gps.satellites.value();
    if (gps.hdop.isValid())        temp.hdop       = (float)gps.hdop.hdop();
    if (gps.date.isValid()) {
        temp.year  = gps.date.year();
        temp.month = gps.date.month();
        temp.day   = gps.date.day();
    }
    if (gps.time.isValid()) {
        temp.hour   = gps.time.hour();
        temp.minute = gps.time.minute();
        temp.second = gps.time.second();
    }

    if (isGnssDataDifferent(s_lastValidData, temp)) {
        // Nuovo fix valido e diverso dal precedente
        s_lastValidData = temp;
        s_lastValidFixMonoUs = timeSyncNowUs();
    }
}

// ===========================================================================
// API pubblica
// ===========================================================================

void gnssInit(){
    gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
    Serial.printf("[GNSS] UART avviata: RX=%d TX=%d baud=%d\n", GNSS_RX, GNSS_TX, GNSS_BAUD);
    s_haveValidSnapshot = false;
    s_lastValidFixMonoUs = 0;
}

void gnssUpdate(){
    // Alimenta il parser con tutti i byte disponibili
    while (gnssSerial.available()) {
        gps.encode(gnssSerial.read());
    }
    // Dopo aver processato i nuovi byte, verifica se il fix è stato aggiornato
    updateInternalSnapshot();
}

bool gnssHasFix() {
    return s_haveValidSnapshot && s_lastValidData.valid;
}

GnssData gnssGetData() {
    if (!s_haveValidSnapshot) {
        GnssData empty;
        empty.valid = false;
        empty.fixRxMonoUs = 0;
        return empty;
    }
    GnssData out = s_lastValidData;
    out.fixRxMonoUs = s_lastValidFixMonoUs; // timestamp reale di ricezione
    return out;
}

void gnssFormatTimestamp(char* buf, size_t bufSize) {
    GnssData d = gnssGetData();
    if (d.valid) {
        snprintf(buf, bufSize, "%04d-%02d-%02d %02d:%02d:%02d",
            d.year, d.month, d.day,
            d.hour, d.minute, d.second);
    } else {
        snprintf(buf, bufSize, "--------------------");
    }
}