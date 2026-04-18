#include "time_sync_manager.h"
#include "gnss_manager.h"
#include <esp_timer.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Parametri del filtro offset
// ---------------------------------------------------------------------------
static constexpr float OFFSET_ALPHA = 0.05f;          // EMA lenta (20 campioni per costante di tempo ~1sec)
static constexpr uint64_t SYNC_TIMEOUT_US = 60000000ULL; // 60 secondi senza sync -> qualità 0
static constexpr uint64_t QUALITY_DECAY_START_US = 5000000ULL; // dopo 5s inizia a decadere

// Stato interno
static int64_t s_utcOffsetUs = 0;
static bool s_utcValid = false;
static uint8_t s_quality = 0;
static uint64_t s_lastSyncMonoUs = 0;
static uint64_t s_lastGnssFixMonoUs = 0;
static bool s_syncedOnce = false;

// Helper: converte data/ora GNSS in microsecondi Unix (solo secondi, frazione 0)
static int64_t gnssToUnixUs(const GnssData& g) {
    if (!g.utcValid) return 0;
    // struct tm (year è offset 1900, month 0-11)
    struct tm t = {0};
    t.tm_year = g.year - 1900;
    t.tm_mon = g.month - 1;
    t.tm_mday = g.day;
    t.tm_hour = g.hour;
    t.tm_min  = g.minute;
    t.tm_sec  = g.second;
    time_t secs = mktime(&t);
    if (secs == (time_t)-1) return 0;
    return (int64_t)secs * 1000000LL;
}

// ---------------------------------------------------------------------------
// API pubbliche
// ---------------------------------------------------------------------------
uint64_t timeSyncNowUs() {
    return (uint64_t)esp_timer_get_time();
}

uint64_t timeSyncNowMs() {
    return timeSyncNowUs() / 1000ULL;
}

void timeSyncUpdateFromGnss(const GnssData* g, uint64_t rxMonoUs) {
    if (!g || !g->utcValid) return;

    int64_t utcUs = gnssToUnixUs(*g);
    if (utcUs == 0) return;

    int64_t rawOffset = utcUs - (int64_t)rxMonoUs;

    // Primo aggiornamento?
    if (!s_syncedOnce) {
        s_utcOffsetUs = rawOffset;
        s_syncedOnce = true;
    } else {
        // EMA
        s_utcOffsetUs = (int64_t)(OFFSET_ALPHA * rawOffset + (1.0f - OFFSET_ALPHA) * s_utcOffsetUs);
    }

    s_utcValid = true;
    s_lastSyncMonoUs = rxMonoUs;
    s_lastGnssFixMonoUs = rxMonoUs;
    s_quality = 100; // sync appena aggiornata
}

void timeSyncInvalidateUtc() {
    s_utcValid = false;
    s_quality = 0;
    s_syncedOnce = false;
}

TimeSyncStatus timeSyncGetStatus() {
    TimeSyncStatus status;
    status.utcValid = s_utcValid;
    status.utcOffsetUs = s_utcOffsetUs;
    status.lastSyncMonoUs = s_lastSyncMonoUs;
    status.lastGnssFixMonoUs = s_lastGnssFixMonoUs;
    status.syncedOnce = s_syncedOnce;

    // Calcola qualità in base al tempo dall'ultima sync
    if (!s_utcValid || !s_syncedOnce) {
        status.quality = 0;
    } else {
        uint64_t nowUs = timeSyncNowUs();
        uint64_t ageUs = (nowUs > s_lastSyncMonoUs) ? (nowUs - s_lastSyncMonoUs) : 0;
        if (ageUs < QUALITY_DECAY_START_US) {
            status.quality = 100;
        } else if (ageUs >= SYNC_TIMEOUT_US) {
            status.quality = 0;
        } else {
            // decadenza lineare da 100 a 0 nell'intervallo [5s, 60s]
            uint64_t decayRangeUs = SYNC_TIMEOUT_US - QUALITY_DECAY_START_US;
            uint64_t excessUs = ageUs - QUALITY_DECAY_START_US;
            status.quality = (uint8_t)(100 - (excessUs * 100) / decayRangeUs);
        }
    }
    status.quality = (status.quality > 100) ? 100 : status.quality;
    return status;
}

int64_t timeSyncMonoToUtcUs(uint64_t monoUs) {
    if (!s_utcValid) return 0;
    return (int64_t)monoUs + s_utcOffsetUs;
}