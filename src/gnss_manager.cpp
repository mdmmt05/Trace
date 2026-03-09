// ---------------------------------------------------------------------------
// gnss_manager.cpp
// ---------------------------------------------------------------------------

#include "gnss_manager.h"
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
// Helpers interni
// ---------------------------------------------------------------------------

// Fix valido = posizione + data + ora tutti validi
static bool _isFixValid() {
    return gps.location.isValid()
        && gps.date.isValid()
        && gps.time.isValid();
}

// ===========================================================================
// API pubblica
// ===========================================================================

void gnssInit(){
    gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
    Serial.printf("[GNSS] UART avviata: RX=%d TX=%d baud=%d\n", GNSS_RX, GNSS_TX, GNSS_BAUD);
}

void gnssUpdate(){
    while (gnssSerial.available()) {
        gps.encode(gnssSerial.read());
    }
}

bool gnssHasFix() {
    return _isFixValid();
}

GnssData gnssGetData() {
    GnssData d;
    d.valid = _isFixValid();

    if (gps.location.isValid()) {
        d.lat = (float)gps.location.lat();
        d.lon = (float)gps.location.lng();
    }
    if (gps.altitude.isValid())    d.altMeters  = (float)gps.altitude.meters();
    if (gps.speed.isValid())       d.speedKmh   = (float)gps.speed.kmph();
    if (gps.satellites.isValid())  d.satellites = (uint8_t)gps.satellites.value();
    if (gps.hdop.isValid())        d.hdop       = (float)gps.hdop.hdop();

    if (gps.date.isValid()) {
        d.year  = gps.date.year();
        d.month = gps.date.month();
        d.day   = gps.date.day();
    }
    if (gps.time.isValid()) {
        d.hour   = gps.time.hour();
        d.minute = gps.time.minute();
        d.second = gps.time.second();
    }

    return d;
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