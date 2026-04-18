#pragma once

#include <Arduino.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// API per il tempo monotono ad alta risoluzione (64 bit, microsecondi)
// ---------------------------------------------------------------------------
uint64_t timeSyncNowUs();
uint64_t timeSyncNowMs();

// ---------------------------------------------------------------------------
// Soft sync UTC basata su GNSS (senza PPS)
// ---------------------------------------------------------------------------
struct TimeSyncStatus {
    bool utcValid;          // true se l'offset UTC è affidabile
    uint8_t quality;        // 0..100 (100 = sync appena aggiornata)
    int64_t utcOffsetUs;    // offset stimato: UTC = mono + offset
    uint64_t lastSyncMonoUs; // ultimo aggiornamento dell'offset (tempo monotono)
    uint64_t lastGnssFixMonoUs; // ultimo fix GNSS usato per la sync
    bool syncedOnce;        // almeno una sync valida ricevuta
};

// Aggiorna l'offset UTC usando un fix GNSS valido.
// g: dati GNSS (con data/ora valide)
// rxMonoUs: tempo monotono al momento della ricezione del fix
void timeSyncUpdateFromGnss(const struct GnssData* g, uint64_t rxMonoUs);

// Invalida forzatamente la sincronizzazione (es. dopo reset calibrazione)
void timeSyncInvalidateUtc();

// Restituisce lo stato corrente della sincronizzazione
TimeSyncStatus timeSyncGetStatus();

// Converte un tempo monotono in microsecondi in UTC (microsecondi da epoch).
// Restituisce 0 se !utcValid (in tal caso controllare il flag).
int64_t timeSyncMonoToUtcUs(uint64_t monoUs);

#ifdef __cplusplus
}
#endif