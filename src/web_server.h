#pragma once

// ---------------------------------------------------------------------------
// web_server.h
// WebServer in modalità Access Point per controllo LED RGB via browser.
// Espone una UI mobile-first e un'API REST JSON.
//
// Endpoints:
//   GET  /            → Pagina HTML completa
//   GET  /api/status  → Stato attuale (mode, r, g, b, speed, brightness, ...)
//   POST /api/color   → {"r":255,"g":0,"b":128}
//   POST /api/mode    → {"mode":"FADING"}
//   POST /api/params  → {"speed":50,"brightness":100,"rpmThreshold":4000,"rpmMax":7000}
// ---------------------------------------------------------------------------

// Inizializza Access Point e webserver. Chiamare nel setup().
void webServerInit();

// Gestisce le richieste in arrivo. Chiamare nel loop() senza delay.
void webServerHandle();
