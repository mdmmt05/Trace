// ---------------------------------------------------------------------------
// web_server.cpp
// ---------------------------------------------------------------------------

#include "web_server.h"
#include "rgb_controller.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "sd_manager.h"

// Riferimento alle variabili condivise per lo stato di registrazione
extern volatile bool recordingActive;
extern volatile bool fileOpen;

// ---------------------------------------------------------------------------
// Credenziali Access Point
// ---------------------------------------------------------------------------
static const char* AP_SSID = "Trace";
static const char* AP_PASSWORD = "trace-lighting";
static const IPAddress AP_IP(192, 168, 4, 1);

static WebServer server(80);

// ---------------------------------------------------------------------------
// Helper: verifica se un nome file è sicuro (solo root, senza path traversal)
// ---------------------------------------------------------------------------
bool isSafeCsvFilename(const String& name) {
    if (name.length() == 0 || name.length() > 64) return false;
    // Vieta percorsi assoluti o traversal
    if (name.indexOf('/') >= 0) return false;
    if (name.indexOf('\\') >= 0) return false;
    if (name.indexOf("..") >= 0) return false;
    // Deve finire con .csv o .CSV
    String lower = name;
    lower.toLowerCase();
    if (!lower.endsWith(".csv")) return false;
    // Permetti solo caratteri sicuri (alphanumerico, punto, trattino, underscore)
    for (unsigned int i = 0; i < name.length(); i++) {
        char c = name[i];
        if (isAlphaNumeric(c)) continue;
        if (c == '.' || c == '-' || c == '_') continue;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// HTML / CSS / JS della UI — tutto embedded come raw string
// Stile: premium minimal automotive, dark mode, mobile-first reale
// Design: pulito, elegante, uso serale/notturno, micro-animazioni leggere
// Nessuna dipendenza esterna (no CDN, no font remoti, no librerie JS)
// UI focalizzata esclusivamente sul controllo LED RGB (no telemetria)
// ---------------------------------------------------------------------------
static const char UI_HTML[] PROGMEM = R"rawhtml(
  <!DOCTYPE html>
  <html lang="it">
  <head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>TRACE LIGHTING</title>
  <style>
  :root {
    --bg:       #0a0a0c;
    --bg2:      #111115;
    --bg3:      #18181e;
    --bg4:      #1f1f28;
    --border:   rgba(255,255,255,0.07);
    --border2:  rgba(255,255,255,0.12);
    --accent:   #c8aa7a;
    --accent2:  #e8cfa0;
    --accent-d: #8a7050;
    --text:     #e8e4dc;
    --text2:    #9a9490;
    --text3:    #504e4a;
    --ok:       #7ac8a0;
    --err:      #c87a7a;
    --r: 255; --g: 255; --b: 255;
  }
  
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  
  html { scroll-behavior: smooth; }
  
  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Trebuchet MS', 'Gill Sans', Optima, Candara, sans-serif;
    font-size: 15px;
    min-height: 100vh;
    -webkit-tap-highlight-color: transparent;
    overflow-x: hidden;
  }
  
  /* subtle grain */
  body::after {
    content: '';
    position: fixed;
    inset: 0;
    background-image: url("data:image/svg+xml,%3Csvg viewBox='0 0 200 200' xmlns='http://www.w3.org/2000/svg'%3E%3Cfilter id='n'%3E%3CfeTurbulence type='fractalNoise' baseFrequency='0.9' numOctaves='4' stitchTiles='stitch'/%3E%3C/filter%3E%3Crect width='100%25' height='100%25' filter='url(%23n)' opacity='0.035'/%3E%3C/svg%3E");
    pointer-events: none;
    z-index: 9998;
    opacity: 0.4;
  }
  
  /* ── HEADER ── */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 18px 20px 16px;
    border-bottom: 1px solid var(--border);
    position: sticky;
    top: 0;
    z-index: 200;
    background: rgba(10,10,12,0.92);
    backdrop-filter: blur(12px);
    -webkit-backdrop-filter: blur(12px);
  }
  
  .logo-group { display: flex; flex-direction: column; gap: 1px; }
  
  .logo {
    font-size: 0.78rem;
    font-weight: 600;
    letter-spacing: 0.32em;
    color: var(--accent);
    text-transform: uppercase;
  }
  
  .logo-sub {
    font-size: 0.58rem;
    letter-spacing: 0.18em;
    color: var(--text3);
    text-transform: uppercase;
  }
  
  .conn-indicator {
    display: flex;
    align-items: center;
    gap: 7px;
  }
  
  .conn-dot {
    width: 6px; height: 6px;
    border-radius: 50%;
    background: var(--ok);
    box-shadow: 0 0 6px var(--ok);
    transition: background 0.4s, box-shadow 0.4s;
  }
  .conn-dot.error { background: var(--err); box-shadow: 0 0 6px var(--err); }
  
  .conn-label {
    font-size: 0.58rem;
    letter-spacing: 0.14em;
    color: var(--text3);
    text-transform: uppercase;
  }
  
  /* ── MAIN ── */
  main {
    max-width: 440px;
    margin: 0 auto;
    padding: 24px 16px 12px;
    display: flex;
    flex-direction: column;
    gap: 14px;
  }
  
  /* ── HERO / COLOR STATE ── */
  .hero {
    position: relative;
    border-radius: 20px;
    overflow: hidden;
    min-height: 170px;
    display: flex;
    flex-direction: column;
    justify-content: flex-end;
    padding: 20px;
    border: 1px solid var(--border);
    transition: box-shadow 0.6s ease;
  }
  
  .hero-bg {
    position: absolute;
    inset: 0;
    background: rgb(var(--r), var(--g), var(--b));
    transition: background 0.3s ease;
    opacity: 0.22;
  }
  
  .hero-glow {
    position: absolute;
    inset: 0;
    background: radial-gradient(ellipse at 50% 110%,
      rgba(var(--r), var(--g), var(--b), 0.5) 0%,
      transparent 70%);
    transition: background 0.3s ease;
    pointer-events: none;
  }
  
  .hero-inner {
    position: relative;
    display: flex;
    align-items: flex-end;
    justify-content: space-between;
  }
  
  .hero-label {
    font-size: 0.6rem;
    letter-spacing: 0.2em;
    color: var(--text2);
    text-transform: uppercase;
    margin-bottom: 6px;
  }
  
  .hero-hex {
    font-size: 2rem;
    font-weight: 300;
    letter-spacing: 0.06em;
    color: var(--text);
    font-family: 'Courier New', 'Lucida Console', monospace;
    line-height: 1;
  }
  
  .hero-swatch {
    width: 52px; height: 52px;
    border-radius: 50%;
    background: rgb(var(--r), var(--g), var(--b));
    border: 2px solid rgba(255,255,255,0.15);
    transition: background 0.3s ease;
    box-shadow: 0 0 20px rgba(var(--r), var(--g), var(--b), 0.4);
    flex-shrink: 0;
    animation: swatch-pulse 4s ease-in-out infinite;
  }
  
  @keyframes swatch-pulse {
    0%,100% { box-shadow: 0 0 20px rgba(var(--r), var(--g), var(--b), 0.35); }
    50%      { box-shadow: 0 0 32px rgba(var(--r), var(--g), var(--b), 0.6); }
  }
  
  /* ── CARD ── */
  .card {
    background: var(--bg2);
    border: 1px solid var(--border);
    border-radius: 16px;
    overflow: hidden;
  }
  
  .card-title {
    padding: 14px 18px 12px;
    font-size: 0.6rem;
    letter-spacing: 0.22em;
    color: var(--accent-d);
    text-transform: uppercase;
    border-bottom: 1px solid var(--border);
  }
  
  .card-body { padding: 18px; }
  
  /* ── PICKER ── */
  .picker-wrap {
    position: relative;
    width: 100%;
    aspect-ratio: 1 / 0.68;
    border-radius: 10px;
    overflow: hidden;
    cursor: crosshair;
    margin-bottom: 14px;
    touch-action: none;
    border: 1px solid var(--border2);
  }
  
  #pickerCanvas { width: 100%; height: 100%; display: block; }
  
  .picker-cursor {
    position: absolute;
    width: 20px; height: 20px;
    border-radius: 50%;
    border: 2.5px solid #fff;
    box-shadow: 0 0 0 1.5px rgba(0,0,0,0.5), 0 2px 8px rgba(0,0,0,0.4);
    transform: translate(-50%, -50%);
    pointer-events: none;
    top: 20%; left: 80%;
    transition: top 0.04s, left 0.04s;
  }
  
  /* Hue slider */
  .hue-track {
    width: 100%;
    height: 18px;
    border-radius: 9px;
    background: linear-gradient(to right,
      hsl(0,100%,50%), hsl(36,100%,50%), hsl(60,100%,50%),
      hsl(120,100%,50%), hsl(180,100%,50%), hsl(240,100%,50%),
      hsl(300,100%,50%), hsl(360,100%,50%));
    position: relative;
    margin-bottom: 16px;
    cursor: pointer;
    touch-action: none;
    border: 1px solid var(--border2);
  }
  
  .hue-thumb {
    position: absolute;
    top: 50%;
    transform: translate(-50%, -50%);
    width: 22px; height: 22px;
    border-radius: 50%;
    background: #fff;
    border: 2.5px solid rgba(0,0,0,0.3);
    box-shadow: 0 1px 6px rgba(0,0,0,0.5);
    pointer-events: none;
    left: 0%;
  }
  
  /* RGB inputs */
  .rgb-row {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
    margin-bottom: 14px;
  }
  
  .rgb-field { display: flex; flex-direction: column; gap: 4px; }
  
  .rgb-field label {
    font-size: 0.58rem;
    letter-spacing: 0.16em;
    color: var(--text3);
    text-align: center;
    text-transform: uppercase;
  }
  
  .rgb-field input {
    background: var(--bg3);
    border: 1px solid var(--border2);
    border-radius: 8px;
    color: var(--text);
    font-size: 0.95rem;
    font-family: 'Courier New', monospace;
    text-align: center;
    padding: 10px 4px;
    width: 100%;
    outline: none;
    -webkit-appearance: none;
    transition: border-color 0.2s;
  }
  
  .rgb-field input:focus { border-color: var(--accent-d); }
  
  .hex-row {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-bottom: 16px;
  }
  
  .hex-row label {
    font-size: 0.58rem;
    letter-spacing: 0.16em;
    color: var(--text3);
    text-transform: uppercase;
    flex-shrink: 0;
  }
  
  .hex-row input {
    flex: 1;
    background: var(--bg3);
    border: 1px solid var(--border2);
    border-radius: 8px;
    color: var(--text);
    font-size: 0.9rem;
    font-family: 'Courier New', monospace;
    text-align: center;
    padding: 10px 8px;
    outline: none;
    -webkit-appearance: none;
    text-transform: uppercase;
    transition: border-color 0.2s;
  }
  .hex-row input:focus { border-color: var(--accent-d); }
  
  /* ── BTN PRIMARY ── */
  .btn-primary {
    width: 100%;
    padding: 15px;
    background: var(--accent);
    color: #0a0a0c;
    border: none;
    border-radius: 10px;
    font-size: 0.72rem;
    font-weight: 700;
    letter-spacing: 0.22em;
    text-transform: uppercase;
    cursor: pointer;
    transition: background 0.2s, transform 0.1s, opacity 0.2s;
    -webkit-tap-highlight-color: transparent;
  }
  .btn-primary:active { transform: scale(0.97); opacity: 0.85; }
  
  /* ── MODALITÀ ── */
  .modes-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }
  
  .mode-card {
    background: var(--bg3);
    border: 1px solid var(--border);
    border-radius: 12px;
    padding: 14px 14px 12px;
    cursor: pointer;
    transition: background 0.2s, border-color 0.2s, transform 0.1s;
    -webkit-tap-highlight-color: transparent;
    user-select: none;
    display: flex;
    flex-direction: column;
    gap: 4px;
  }
  
  .mode-card:last-child:nth-child(odd) {
    grid-column: 1 / -1;
  }
  
  .mode-card:active { transform: scale(0.97); }
  
  .mode-card.active {
    background: rgba(200,170,122,0.1);
    border-color: rgba(200,170,122,0.4);
  }
  
  .mode-name {
    font-size: 0.78rem;
    font-weight: 600;
    letter-spacing: 0.06em;
    color: var(--text);
    transition: color 0.2s;
  }
  
  .mode-card.active .mode-name { color: var(--accent2); }
  
  .mode-desc {
    font-size: 0.62rem;
    color: var(--text3);
    letter-spacing: 0.02em;
  }
  
  .mode-pip {
    width: 5px; height: 5px;
    border-radius: 50%;
    background: var(--text3);
    margin-top: 6px;
    transition: background 0.2s, box-shadow 0.2s;
  }
  
  .mode-card.active .mode-pip {
    background: var(--accent);
    box-shadow: 0 0 6px var(--accent);
  }
  
  /* ── AVANZATE ── */
  .section-toggle {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 14px 18px;
    cursor: pointer;
    user-select: none;
    -webkit-tap-highlight-color: transparent;
  }
  
  .section-toggle .title {
    font-size: 0.6rem;
    letter-spacing: 0.22em;
    color: var(--accent-d);
    text-transform: uppercase;
  }
  
  .chevron {
    width: 14px; height: 14px;
    border-right: 1.5px solid var(--text3);
    border-bottom: 1.5px solid var(--text3);
    transform: rotate(45deg) translateY(-3px);
    transition: transform 0.3s ease;
    margin-top: 2px;
  }
  
  .chevron.open {
    transform: rotate(-135deg) translateY(-3px);
  }
  
  .advanced-body {
    display: none;
    padding: 0 18px 18px;
    border-top: 1px solid var(--border);
    padding-top: 18px;
  }
  .advanced-body.open { display: block; }
  
  /* sliders */
  .param-row {
    margin-bottom: 20px;
  }
  
  .param-label-row {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    margin-bottom: 8px;
  }
  
  .param-label {
    font-size: 0.65rem;
    letter-spacing: 0.14em;
    color: var(--text2);
    text-transform: uppercase;
  }
  
  .param-val {
    font-size: 0.75rem;
    font-family: 'Courier New', monospace;
    color: var(--accent);
  }
  
  input[type=range] {
    -webkit-appearance: none;
    appearance: none;
    width: 100%;
    height: 4px;
    border-radius: 2px;
    background: var(--bg4);
    outline: none;
    cursor: pointer;
    accent-color: var(--accent);
  }
  
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 22px; height: 22px;
    border-radius: 50%;
    background: var(--accent);
    border: none;
    box-shadow: 0 1px 6px rgba(0,0,0,0.5);
    transition: transform 0.15s;
  }
  
  input[type=range]:active::-webkit-slider-thumb { transform: scale(1.15); }
  
  input[type=range]::-moz-range-thumb {
    width: 22px; height: 22px;
    border-radius: 50%;
    background: var(--accent);
    border: none;
    box-shadow: 0 1px 6px rgba(0,0,0,0.5);
  }
  
  .number-field {
    display: flex;
    flex-direction: column;
    gap: 6px;
    margin-bottom: 16px;
  }
  
  .number-field label {
    font-size: 0.63rem;
    letter-spacing: 0.14em;
    color: var(--text2);
    text-transform: uppercase;
  }
  
  .number-field input {
    background: var(--bg3);
    border: 1px solid var(--border2);
    border-radius: 8px;
    color: var(--text);
    font-size: 1rem;
    font-family: 'Courier New', monospace;
    padding: 11px 14px;
    width: 100%;
    outline: none;
    -webkit-appearance: none;
    transition: border-color 0.2s;
  }
  .number-field input:focus { border-color: var(--accent-d); }

  .toggle-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 20px;
    background: var(--bg3);
    padding: 12px 16px;
    border-radius: 12px;
    border: 1px solid var(--border);
  }
  
  .toggle-label {
    font-size: 0.7rem;
    letter-spacing: 0.12em;
    text-transform: uppercase;
    color: var(--text2);
  }
  
  .toggle-desc {
    font-size: 0.58rem;
    color: var(--text3);
    margin-top: 4px;
  }
  
  .toggle-switch {
    position: relative;
    width: 52px;
    height: 28px;
    background: var(--bg4);
    border-radius: 50px;
    cursor: pointer;
    transition: background 0.2s;
    border: 1px solid var(--border2);
  }
  
  .toggle-switch.active {
    background: var(--accent);
  }
  
  .toggle-knob {
    position: absolute;
    top: 2px;
    left: 2px;
    width: 22px;
    height: 22px;
    background: white;
    border-radius: 50%;
    transition: transform 0.2s;
    box-shadow: 0 1px 4px rgba(0,0,0,0.4);
  }
  
  .toggle-switch.active .toggle-knob {
    transform: translateX(24px);
  }
  
  .rpm-note {
    font-size: 0.6rem;
    color: var(--text3);
    letter-spacing: 0.04em;
    padding: 10px 12px;
    background: rgba(255,255,255,0.025);
    border-radius: 8px;
    border-left: 2px solid var(--accent-d);
    margin-bottom: 18px;
    line-height: 1.5;
  }
  
  /* ── FOOTER ── */
  footer {
    text-align: center;
    padding: 0px 20px 24px;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 14px;
  }
  
  .footer-text {
    font-size: 0.58rem;
    letter-spacing: 0.24em;
    color: var(--text3);
    text-transform: uppercase;
  }
  
  .monogram {
    opacity: 0.45;
    transition: opacity 0.3s;
  }
  .monogram:hover { opacity: 0.32; }
  
  /* ── TOAST ── */
  #toast {
    position: fixed;
    bottom: 24px;
    left: 50%;
    transform: translateX(-50%) translateY(80px);
    background: var(--bg3);
    border: 1px solid var(--border2);
    border-radius: 10px;
    padding: 11px 20px;
    font-size: 0.65rem;
    letter-spacing: 0.16em;
    color: var(--text2);
    text-transform: uppercase;
    z-index: 1000;
    transition: transform 0.3s cubic-bezier(.34,1.56,.64,1), opacity 0.3s;
    opacity: 0;
    white-space: nowrap;
    backdrop-filter: blur(10px);
    -webkit-backdrop-filter: blur(10px);
    pointer-events: none;
  }
  
  #toast.show {
    transform: translateX(-50%) translateY(0);
    opacity: 1;
  }
  
  #toast.ok { border-color: rgba(122,200,160,0.4); color: var(--ok); }
  #toast.err { border-color: rgba(200,122,122,0.4); color: var(--err); }
  
  /* ── DIVIDER ── */
  .divider {
    height: 1px;
    background: var(--border);
    margin: 2px 0;
  }
  </style>
  </head>
  <body>
  
  <header>
    <div class="logo-group">
      <div class="logo">TRACE LIGHTING</div>
      <div class="logo-sub">Ambient control</div>
    </div>
    <div class="conn-indicator">
      <div class="conn-dot" id="connDot"></div>
      <div class="conn-label" id="connLabel">Online</div>
    </div>
  </header>
  
  <main>
  
    <!-- HERO -->
    <div class="hero" id="hero">
      <div class="hero-bg" id="heroBg"></div>
      <div class="hero-glow" id="heroGlow"></div>
      <div style="position:relative; margin-bottom: 8px;">
        <div class="hero-label">Colore attivo</div>
      </div>
      <div class="hero-inner">
        <div>
          <div class="hero-hex" id="heroHex">#FFFFFF</div>
        </div>
        <div class="hero-swatch" id="heroSwatch"></div>
      </div>
    </div>
  
    <!-- PICKER -->
    <div class="card">
      <div class="card-title">Seleziona colore</div>
      <div class="card-body">
        <div class="picker-wrap" id="pickerWrap">
          <canvas id="pickerCanvas"></canvas>
          <div class="picker-cursor" id="pickerCursor"></div>
        </div>
  
        <!-- Hue slider -->
        <div class="hue-track" id="hueTrack">
          <div class="hue-thumb" id="hueThumb"></div>
        </div>
  
        <!-- RGB -->
        <div class="rgb-row">
          <div class="rgb-field">
            <label>R</label>
            <input type="number" id="rIn" min="0" max="255" value="255">
          </div>
          <div class="rgb-field">
            <label>G</label>
            <input type="number" id="gIn" min="0" max="255" value="255">
          </div>
          <div class="rgb-field">
            <label>B</label>
            <input type="number" id="bIn" min="0" max="255" value="255">
          </div>
        </div>
  
        <!-- HEX -->
        <div class="hex-row">
          <label>Hex</label>
          <input type="text" id="hexIn" value="#FFFFFF" maxlength="7" spellcheck="false">
        </div>
  
        <button class="btn-primary" id="applyColor">Applica colore</button>
      </div>
    </div>
  
    <!-- MODALITÀ -->
    <div class="card">
      <div class="card-title">Modalità</div>
      <div class="card-body">
        <div class="modes-grid">
          <div class="mode-card" data-mode="STATIC">
            <div class="mode-name">Statico</div>
            <div class="mode-desc">Colore fisso</div>
            <div class="mode-pip"></div>
          </div>
          <div class="mode-card" data-mode="FADING">
            <div class="mode-name">Fading</div>
            <div class="mode-desc">Transizione morbida</div>
            <div class="mode-pip"></div>
          </div>
          <div class="mode-card" data-mode="BREATHING">
            <div class="mode-name">Respiro</div>
            <div class="mode-desc">Luminosità pulsante</div>
            <div class="mode-pip"></div>
          </div>
        </div>
      </div>
    </div>
  
    <!-- IMPOSTAZIONI AVANZATE -->
    <div class="card">
      <div class="section-toggle" id="advToggle">
        <div class="title">Impostazioni avanzate</div>
        <div class="chevron" id="advChevron"></div>
      </div>
      <div class="advanced-body" id="advBody">
  
        <div class="param-row">
          <div class="param-label-row">
            <div class="param-label">Velocità effetto</div>
            <div class="param-val" id="speedVal">50</div>
          </div>
          <input type="range" id="speedSlider" min="0" max="100" value="50">
        </div>
  
        <div class="param-row">
          <div class="param-label-row">
            <div class="param-label">Luminosità</div>
            <div class="param-val" id="brightnessVal">100%</div>
          </div>
          <input type="range" id="brightnessSlider" min="0" max="100" value="100">
        </div>
  
        <div class="divider" style="margin-bottom:18px;"></div>

        <!-- toggle Avviso RPM -->
        <div class="toggle-row" id="rpmWarningToggle">
          <div>
            <div class="toggle-label">Avviso RPM</div>
            <div class="toggle-desc">Lampeggio rosso oltre la soglia</div>
          </div>
          <div class="toggle-switch" id="rpmWarningSwitch">
            <div class="toggle-knob"></div>
          </div>
        </div>

        <div class="rpm-note">
          Se attivo, i LED lampeggiano rosso quando gli RPM superano la soglia impostata.
        </div>
  
        <div class="number-field">
          <label>Soglia avviso RPM</label>
          <input type="number" id="rpmThreshold" min="0" max="15000" value="6000">
        </div>
  
        <button class="btn-primary" id="applyParams">Salva impostazioni</button>
      </div>
    </div>

    <!-- CARD LOGS -->
    <div class="card">
      <div class="card-title">Registrazioni</div>
      <div class="card-body">
        <button class="btn-primary" id="downloadLogsBtn" style="background: var(--bg3); color: var(--text2); border: 1px solid var(--border2);">Download logs</button>
        <div id="logsDisabledMsg" style="display:none; font-size:0.65rem; margin-top:12px; color: var(--err); text-align:center;">Disponibile solo a registrazione ferma</div>
      </div>
    </div>
  
  </main>
  
  <footer>
    <div class="footer-text">TRACE · ESP32-S3</div>
  
    <!-- MONOGRAM SVG PLACEHOLDER -->
    <!-- Sostituisci il contenuto di questo SVG con il tuo monogramma personalizzato -->
    <!-- Mantieni width/height e opacity per preservare il look discreto -->
    <div class="monogram">
      <svg width="128" height="128" viewBox="0 0 1254 1254" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path style="fill:var(--accent); stroke:none;" d="M619 70.4244C596.348 73.3983 576.128 78.7502 558 93.6157C551.432 99.0017 545.408 105.266 540.235 112C538.033 114.867 535.587 120.184 531.826 121.211C528.924 122.004 525.509 119.948 523 118.741C517.77 116.227 512.595 114.024 507 112.427C492.236 108.214 476.115 107.609 461 110.46C438.036 114.791 416.583 127.416 401.289 145C396.219 150.83 391.879 157.319 388.015 164C386.384 166.82 385.06 171.364 381.867 172.799C378.757 174.197 374.154 172.206 371 171.611C364.286 170.345 357.824 169.923 351 170.004C333.057 170.217 315.956 176.908 301 186.436C282.599 198.159 268.106 216.65 260.425 237C257.951 243.555 256.396 250.105 255.247 257C254.731 260.098 254.784 265.057 252.4 267.397C250.113 269.641 245.909 268.998 243 269C237.066 269.005 230.778 269.291 225 270.76C207.248 275.272 192.179 285.122 180.3 299C166.981 314.561 158.499 335.527 157.09 356C156.578 363.422 156.422 371.628 157.425 379C157.898 382.475 159.966 388.388 157.958 391.582C156.376 394.097 152.569 394.941 150 396.012C144.474 398.315 138.972 401.01 134 404.36C119.802 413.925 108.712 427.669 101.259 443C92.188 461.661 89.8923 484.688 93.2855 505C94.3395 511.31 95.6909 517.843 97.4282 524C98.282 527.026 100.376 530.794 99.8789 534C98.9104 540.247 90.1711 546.626 86.8148 552C79.0936 564.364 74.4924 577.592 72.4282 592C69.179 614.681 73.4772 640.072 82.4275 661C85.2371 667.57 88.6514 673.938 92.4252 680C94.3267 683.055 97.3652 686.358 97.9722 690C98.5288 693.339 96.6172 696.857 95.7454 700C94.2287 705.468 92.7357 711.348 92.1705 717C89.9402 739.307 93.6463 761.483 102.427 782C107.035 792.765 113.651 802.981 121.084 812C125.505 817.365 131.488 822.125 135.157 828C137.974 832.509 139.326 838.128 141.428 843C144.119 849.235 147.791 855.276 151.436 861C162.968 879.11 179.345 894.247 196 907.551C234.085 937.974 279.337 959.337 314 994C333.182 1013.18 349.403 1036.86 349.946 1065C350.418 1089.48 337.284 1112.82 354.529 1135C363.677 1146.77 377.839 1151.5 392 1153.42C419.182 1157.12 446.542 1145.91 474 1149.29C483.349 1150.43 494.21 1152.26 500.467 1160.04C507.015 1168.19 506.694 1177.41 517 1183.09C531.39 1191.03 547.724 1183.78 563 1186.33C585.149 1190.02 604.566 1210.23 628 1207.71C637.779 1206.66 646.947 1204.44 656 1200.57C667.513 1195.65 678.524 1188.7 691 1186.31C706.783 1183.29 722.674 1190.78 738 1183.46C748.564 1178.41 748.392 1169.24 754.558 1161C760.309 1153.32 769.855 1150.48 779 1149.29C807.357 1145.6 836.004 1157.79 864 1153.25C877.768 1151.01 891.204 1146.45 900.1 1134.99C917.252 1112.89 903.959 1086.7 905.039 1062C906.079 1038.22 919.406 1017.53 934.428 1000C947.287 984.993 963.98 972.712 980 961.291C1008.37 941.065 1038.46 923.307 1065 900.572C1080.22 887.53 1094.11 871.675 1103.69 854C1108.19 845.706 1110.6 836.178 1115.27 828.089C1118.47 822.558 1124.18 818.082 1128.11 813C1135.35 803.63 1141.2 793.712 1146.22 783C1154.57 765.178 1158.7 742.625 1156.83 723C1156.19 716.237 1155.01 709.589 1153.37 703C1152.45 699.288 1150.45 694.843 1150.6 691C1150.73 687.614 1153.22 684.741 1154.95 682C1157.8 677.486 1160.47 672.857 1162.69 668C1169.56 653.006 1175.23 635.603 1175.96 619C1176.76 600.753 1176.1 580.825 1168.22 564C1165.3 557.774 1162.26 551.503 1158.1 546C1155.38 542.4 1150.53 538.633 1149.7 534C1149.01 530.132 1151.55 525.651 1152.57 522C1154.46 515.261 1155.86 507.95 1156.71 501C1159.2 480.762 1156.17 459.511 1147.69 441C1140.61 425.554 1129.3 411.941 1115 402.699C1110.01 399.475 1104.61 396.933 1099 395.003C1095.98 393.963 1091.73 393.132 1090.31 389.867C1088.93 386.674 1090.68 382.308 1090.91 379C1091.49 370.547 1091.49 361.453 1090.91 353C1089.37 330.652 1079.43 309.582 1064.71 293.004C1054.37 281.357 1038.31 272.398 1023 269.461C1017.01 268.311 1011.08 268.003 1005 268C1001.99 267.999 997.944 268.589 995.514 266.397C992.465 263.646 993.17 258.667 992.714 255C991.829 247.891 989.708 240.737 987.302 234C980.545 215.084 966.657 197.252 950 186.089C933.965 175.344 915.401 168.908 896 169.001C890.134 169.029 883.714 169.538 878 170.899C874.868 171.645 870.415 174.017 867.213 172.672C864.054 171.345 862.693 166.789 861.219 164C857.889 157.697 854.025 151.548 849.539 146C837.306 130.869 820.618 118.93 802 113.029C783.842 107.274 764.495 106.251 746 110.873C738.991 112.625 732.48 115.044 726 118.248C723.148 119.657 719.458 122.213 716.184 120.716C712.598 119.076 710.324 113.985 707.985 111C703.518 105.3 698.382 100.014 693 95.1705C674.209 78.2593 644.507 67.0757 619 70.4244M612 76.2863C639.917 72.8573 668.603 80.9852 690 99.3002C695.695 104.175 700.8 109.971 705.244 116C707.993 119.729 710.394 124.534 715.004 126.254C719.201 127.819 723.31 125.578 727 123.752C732.883 120.842 738.669 118.234 745 116.427C761.831 111.625 780.243 111.84 797 117.026C815.772 122.835 833.016 133.506 845.334 149C849.555 154.31 853.17 159.958 856.244 166C858.395 170.228 860.421 175.629 865.015 177.782C868.469 179.4 872.552 177.734 876 176.873C881.86 175.411 887.941 174.153 894 174.015C912.29 173.596 930.872 179.327 946 189.52C963.97 201.627 976.935 219.604 983.921 240C985.744 245.323 987.218 251.403 987.826 257C988.326 261.597 987.567 267.058 991.303 270.566C994.206 273.291 998.31 272.994 1002 273C1008.42 273.011 1014.67 272.893 1021 274.095C1036.65 277.065 1051.93 286.142 1062.41 298.039C1075.61 313.021 1085.08 333.836 1085.96 354C1086.32 362.121 1086.67 370.926 1085.71 379C1085.23 383.107 1083.3 389.048 1085.22 392.945C1087.1 396.764 1091.36 397.564 1095 398.864C1100.17 400.712 1105.28 403.007 1110 405.823C1123.97 414.159 1135.85 427.18 1142.69 442C1152.07 462.317 1154.1 485.012 1150.7 507C1149.82 512.69 1148.41 518.518 1146.66 524C1145.59 527.327 1143.38 531.434 1143.76 535C1144.42 541.143 1152.83 546.91 1156.14 552C1163.02 562.594 1167.79 574.501 1169.73 587C1173.4 610.81 1170.43 635.624 1161.55 658C1158.78 664.975 1155.54 671.627 1151.57 678C1149.28 681.68 1145.78 685.542 1145.27 690C1144.37 697.829 1149.94 707.073 1150.83 715C1153.51 739 1150.42 763.093 1139.74 785C1135.07 794.579 1129.31 803.77 1122.54 812C1118.9 816.417 1113.83 820.031 1110.93 825C1106.26 833.034 1104.23 842.744 1099.69 851C1089.34 869.833 1074.29 885.948 1058 899.725C1017.43 934.034 965.517 956.378 930.425 997C914.715 1015.19 900.621 1037.16 900.015 1062C899.461 1084.67 912.068 1110.79 896.471 1131C884.223 1146.87 864.521 1149.36 846 1148.09C823.062 1146.52 800.016 1141.3 777 1144.29C743.765 1148.61 715.25 1164.16 686 1179.26C669.069 1188 652.31 1199.34 633 1201.71C617.686 1203.59 604.387 1197.94 591 1191.25C571.8 1181.65 553.594 1169.59 534 1160.86C511.209 1150.71 486.111 1143.33 461 1143.02C446.004 1142.83 430.899 1146.41 416 1147.83C396.029 1149.73 372.491 1149.49 358.9 1131.99C341.501 1109.58 357.169 1085.66 354.83 1061C351.846 1029.54 333.768 1006.51 312 985.004C296.875 970.059 278.6 958.4 261 946.645C237.565 930.993 213.699 915.624 192 897.565C176.702 884.833 162.683 870.195 152.453 853C147.225 844.213 144.897 833.395 139.298 825C135.201 818.857 129.083 813.801 124.449 808C116.441 797.975 109.894 786.87 105.011 775C97.5912 756.965 95.2319 736.389 97.1705 717C97.7431 711.274 99.208 705.453 101.001 700C102.109 696.628 104.148 692.627 103.397 689C102.637 685.335 99.6596 682.086 97.6952 679C94.2381 673.57 90.9911 667.854 88.3086 662C78.8489 641.355 74.1887 615.623 77.4282 593C79.3072 579.879 83.0631 567.189 90.3449 556C93.3156 551.435 96.5641 547.088 100.166 543C102.236 540.65 105.072 538.294 105.498 535C105.994 531.165 103.486 526.578 102.344 523C100.483 517.167 99.3904 511.044 98.439 505C95.2543 484.768 97.1229 463.589 106.109 445C113.414 429.889 123.985 417.293 138 408.004C143.071 404.643 148.403 402.203 154 399.86C157.314 398.473 161.165 397.363 162.968 393.945C164.681 390.699 163.302 386.448 163 383C162.24 374.32 161.657 365.742 162.039 357C162.921 336.871 172.165 316.089 185.289 301C197.255 287.243 212.425 279.076 230 275.12C237.16 273.508 251.179 276.951 256.566 271.566C259.58 268.554 259.45 262.968 260 259C260.948 252.16 262.594 245.456 265.065 239C273.362 217.323 289.113 199.283 309 187.453C322.844 179.218 339.832 174.338 356 175.039C361.628 175.284 367.531 176.042 373 177.374C376.778 178.294 381.203 179.848 384.895 177.82C389.037 175.546 390.691 169.935 392.782 166C396.221 159.526 400.716 153.588 405.439 148C419.799 131.01 441.235 119.383 463 115.424C478.004 112.695 494.472 113.471 509 118.356C513.755 119.954 518.549 122.007 523 124.32C526.081 125.92 529.339 128.402 532.996 127.214C536.824 125.97 539.325 122.043 541.626 119C545.835 113.432 549.878 107.877 555.001 103.089C570.702 88.4177 590.657 78.9076 612 76.2863M538 154C533.682 147.995 529.095 142.35 523 138.043C472.583 102.417 396 143.203 396 204C392.001 200.035 387.958 196.113 383 193.349C350.448 175.202 308.795 193.53 287.899 221C271.423 242.66 267.251 269.913 274 296C242.735 269.421 198.133 294.352 182.27 326C167.886 354.699 173.382 383.683 185 412C161.103 401.718 134.658 420.21 122.229 440C107.959 462.722 104.924 491.614 113.696 517C117.306 527.446 122.991 535.877 129 545C111.475 545.048 100.316 558.805 94.0116 574C82.6804 601.309 86.8803 636.569 101.427 662C105.964 669.931 111.301 677.182 117.425 683.961C120.572 687.444 124.249 690.175 127 694C119.184 694.87 114.743 701.034 112.065 708C107.154 720.774 107.232 734.637 109.296 748C112.581 769.269 123.093 788.576 136.615 805C141.801 811.299 148.81 816.333 153.486 823C157.539 828.778 158.664 837.51 161.784 844C167.511 855.915 176.447 867.753 186.039 876.83C215.021 904.255 249.475 924.513 282 947.28C306.498 964.428 330.803 983.661 347.305 1009C360.21 1028.82 365.993 1049.47 364.961 1073C364.116 1092.27 354.344 1112.06 371.04 1127.9C380.333 1136.71 392.877 1138 405 1138C426.784 1137.99 448.049 1132.58 470 1134.09C506.914 1136.62 541.72 1157.23 573 1175.28C588.947 1184.49 606.814 1198.8 626 1199C648.499 1199.23 670.266 1180.93 689 1170.28C720.926 1152.13 756.876 1135.19 794 1133.09C813.203 1132.01 832.721 1138 852 1138C863.351 1138 875.31 1135.95 883.816 1127.7C900.559 1111.46 889.902 1087.7 889.039 1068C888.143 1047.54 895.545 1027.11 906.436 1010C943.107 952.405 1012.43 927.226 1061 881.911C1072.07 871.583 1082.4 858.581 1089.19 845C1092.6 838.176 1093.36 829.218 1097.56 823C1103.4 814.369 1112.23 807.477 1118.52 799C1129.35 784.414 1136.41 766.835 1139.54 749C1140.12 745.681 1139.85 742.339 1140.17 739C1141.52 724.858 1141.02 697.636 1123 694C1151.54 665.378 1167.62 625.451 1158.35 585C1154.27 567.22 1143.63 545.943 1123 545C1128.17 535.722 1133.73 527.249 1136.97 517C1145.04 491.511 1142.33 460.43 1127.25 438C1114.29 418.72 1086.47 399.695 1063 412C1074.79 383.677 1080.09 355.222 1067.14 326C1054.88 298.341 1021.8 276.678 991 285.785C984.339 287.755 978.386 291.741 973 296C975.206 285.309 977.669 275.021 976.911 264C975.059 237.118 959.451 212.339 937 197.864C918.808 186.135 892.416 181.135 872 189.873C864.309 193.164 856.958 197.376 853 205C851.21 198.945 851.831 192.263 850.55 186C848.58 176.364 843.977 166.956 838.254 159C816.859 129.256 773.069 114.967 739 130.312C731.311 133.775 724.135 138.129 718.174 144.17C715.152 147.232 713.09 151.03 710 154C708.719 139.332 698.058 124.135 687.996 114.001C649.892 75.6211 584.612 80.6827 552.378 124C545.986 132.59 539.609 143.26 538 154M683 354C687.924 339.454 687.995 322.195 690.13 307C694.857 273.371 698.811 239.515 704.246 206C707.031 188.83 706.052 168.17 715.468 153C722.434 141.777 733.784 135.037 746 131.001C786.014 117.781 836.562 142.034 847.105 184C852.367 204.944 844.15 223.291 838.026 243C830.241 268.05 822.221 293.091 813.999 318C809.137 332.732 802.265 348.662 800 364C805.384 351.853 808.559 338.613 812.67 326C820.933 300.648 829.236 275.291 837.667 250C842.856 234.433 846.561 212.714 859.04 201.184C869.037 191.948 881.812 189 895 189C936.245 189 969.897 222.357 973.83 263C976.251 288.018 964.272 309.982 952.424 331C938.41 355.863 922.972 381.647 912 408C919.403 397.602 924.823 385.284 930.781 374C938.78 358.849 947.251 343.894 955.719 329C962.836 316.482 968.906 302.407 981 293.789C990.635 286.924 1003.68 286.013 1015 287.921C1048.66 293.593 1070.52 325.139 1071.96 358C1072.64 373.623 1070.19 389.692 1063.69 404C1059.02 414.281 1052.12 423.048 1045.37 432C1031.24 450.771 1014.63 469.582 1003 490C1017.22 473.568 1029.7 455.28 1042.88 438C1049.59 429.209 1056.48 417.973 1067 413.468C1082.58 406.792 1101.41 415.354 1113 426.09C1137.11 448.434 1143.11 483.024 1134.84 514C1129.4 534.395 1115.44 549.228 1102.25 565C1088.33 581.639 1074.46 598.324 1060.59 615C1052.69 624.487 1043.96 633.819 1037 644C1058.03 622.529 1076.26 597.786 1095.87 575C1102.98 566.735 1111.17 552.388 1122 548.858C1135.3 544.523 1146.81 561.977 1151.13 572C1163.13 599.868 1159.16 634.504 1145.11 661C1133.9 682.135 1115.19 696.16 1098 712.089C1073.27 735.002 1048.85 758.154 1025 782C1032.47 776.735 1038.32 769.308 1045 763.089C1060.93 748.253 1076.82 733.396 1093 718.831C1098.96 713.467 1104.73 707.819 1111 702.808C1114.15 700.291 1117.73 697.196 1122 697.217C1130.79 697.259 1134.53 708.013 1136.11 715C1140.35 733.694 1137.04 754.326 1130.19 772C1119.24 800.271 1096.92 819.828 1074 838.46C1045.45 861.669 1014.59 883.552 988 909C997.884 902.462 1006.8 894.086 1016 886.611C1030.54 874.792 1045.36 863.308 1060 851.6C1069.69 843.847 1080.36 832.477 1092 828C1085.58 859.001 1058.04 882.701 1034 900.87C987.869 935.741 927.901 963.74 899.781 1017C891.775 1032.16 885.303 1049.55 886.039 1067C886.947 1088.53 899.984 1115.71 876 1129.91C867.267 1135.07 857.789 1135 848 1135C828.578 1135 809.515 1130.77 790 1131C766.342 1131.29 747.064 1142.97 725 1149C735.632 1139.01 751.305 1132.23 764 1125.14C777.868 1117.4 791.791 1109.86 806 1102.75C812.817 1099.34 819.661 1095.11 827 1093C822.229 1092.96 818.214 1095.65 814 1097.75L794 1107.75C774.603 1117.45 755.608 1128.06 737 1139.2C726.648 1145.4 715.495 1154.34 704 1158C709.655 1149.41 716.967 1138.51 704.989 1131.01C703.078 1129.81 701.171 1129.39 699 1129C698.462 1117.25 690.266 1111.09 679 1111C676.721 1095.86 663.807 1092.61 651 1096C644.13 1072.21 609.973 1072.3 603 1096C589.047 1092.31 577.608 1096.82 574 1112C562.893 1109.45 554.92 1118.53 554 1129C546.263 1130.23 539.61 1136.63 541.537 1145C542.47 1149.05 545.348 1152.26 547 1156C534.221 1149.96 522.14 1141.28 510 1134C491.437 1122.86 472.358 1112.43 453 1102.75C443.424 1097.97 433.29 1091.96 423 1089C442.036 1100.11 462.6 1108.81 482 1119.31C497.991 1127.96 512.988 1138.48 529 1147C524.367 1147.96 519.288 1144.48 515 1142.81C505.242 1139.02 495.321 1135.41 485 1133.46C457.502 1128.27 431.448 1135 404 1135C389.773 1135 375.3 1132.67 367.533 1119C360.147 1106 365.434 1089.65 367.561 1076C368.594 1069.37 368.477 1061.68 367.83 1055C364.731 1022.98 342.668 995.204 320 974.089C286.068 942.483 243.888 921.509 208 892.389C186.637 875.055 166.706 855.146 159 828C169.872 830.826 180.208 842.547 189 849.35C203.731 860.748 218.433 872.194 233 883.801C241.892 890.886 250.518 898.729 260 905C250.388 895.802 239.383 887.909 229 879.6C212.002 865.997 195.028 852.369 178 838.801C162.048 826.09 146.111 813.343 133.654 797C118.06 776.541 107.865 747.928 111.579 722C112.823 713.323 116.734 695.049 129 696.495C138.439 697.609 148.243 709.209 155 715.155C166.706 725.455 178.43 735.719 190 746.17C197.234 752.703 204.043 760.395 212 766C190.887 744.887 168.545 724.862 146 705.282C130.586 691.894 113.853 679.543 104.258 661C90.4961 634.402 84.4169 598.907 98.3704 571C103.824 560.093 115.738 544.187 130 548.653C140.44 551.922 149.538 566.025 156.424 574C174.779 595.257 192.358 617.944 212 638C204.742 627.386 195.511 617.752 187.13 608C173.991 592.711 161.021 577.28 147.87 562C133.619 545.442 119.792 530.796 114.375 509C106.751 478.323 115.629 443.627 141 423.669C151.49 415.417 167.603 408.163 181 413.479C191.256 417.549 197.472 427.595 203.884 436C216.087 451.994 227.841 468.793 241 484C225.251 456.359 199.641 434.611 185.37 406C179.721 394.673 176.95 381.565 176.09 369C173.847 336.251 193.659 302.176 225 291.344C237.997 286.852 253.948 286.112 266 293.711C278.362 301.505 284.505 316.617 291.306 329C304.966 353.874 321.089 378.793 332 405C331.195 398.333 326.693 391.8 323.424 386C319.908 379.762 316.942 373.24 313.424 367C306.027 353.877 299.254 340.324 292.219 327C280.626 305.042 269.994 284.638 274.285 259C280.164 223.88 310.559 194.143 346 189.845C360.988 188.027 378.83 190.574 389.815 202.093C401.392 214.234 404.834 235.502 410 251C419.082 278.246 428.436 305.503 436.72 333C439.71 342.925 444.26 352.766 446 363C446.765 357.829 444.265 352.894 442.67 348C439.86 339.38 437.525 330.605 434.667 322C425.729 295.09 417.604 267.91 408.667 241C402.883 223.587 395.22 204.567 400.428 186C411.767 145.583 457.591 120.264 498 130.13C512.676 133.713 526.985 141.292 534.482 155C539.269 163.752 539.498 174.359 541.08 184C544.134 202.609 546.581 221.335 549.271 240C554.891 279.001 561.146 317.945 566.282 357C569.21 379.272 573.232 401.604 575 424C577.038 416.907 574.442 408.219 573.424 401C571.595 388.022 569.888 375.009 568.285 362C562.443 314.584 555.69 267.258 548.728 220C546.666 206.001 544.607 192.022 542.718 178C541.525 169.15 539.527 159.827 542.04 151C544.786 141.355 549.145 132.963 555.236 125C587.495 82.824 651.463 79.7001 687.96 118C694.534 124.899 699.854 133.239 703.572 142C712.596 163.265 704.405 189.062 701.271 211C696.803 242.268 692.61 273.64 688.845 305C686.9 321.201 683.337 337.673 683 354M625 181L625 410C627.085 405.031 626 398.35 626 393C626 381 625.943 368.999 626.001 357C626.193 317.082 627 277.001 627 237L627 199C627 193.148 628.104 186.182 625 181M484 235L484 238C484.696 236.446 484.696 236.554 484 235M760 237C754.292 266.431 747.01 295.708 740.576 325C737.467 339.153 732.631 354.529 732 369C736.599 361.372 736.89 349.664 738.881 341C744.23 317.73 748.77 294.27 754.119 271C755.804 263.667 757.38 256.323 759.116 249C760.042 245.093 761.493 240.826 760 237M485 239L485 242C485.696 240.446 485.696 240.554 485 239M486 243C487.381 260.349 493.463 278.131 497.626 295C505.732 327.85 514.164 360.863 521 394C522.512 390.124 520.885 385.949 519.989 382C518.021 373.323 515.937 364.659 513.884 356C507.647 329.683 501.934 303.242 495.373 277C492.752 266.512 491.637 252.229 486 243M883 307L882 311C883.303 309.332 883.553 309.008 883 307M363 309C367.955 329.859 379.872 350.189 388 370C388.883 364.957 384.823 358.725 383.05 354C379.75 345.205 375.723 336.707 372.201 328C369.666 321.735 367.318 314.231 363 309M881 312C872.652 332.078 863.486 351.801 855.4 372C852.241 379.89 847.966 387.723 846 396C850.89 390.566 852.973 381.703 855.85 375C861.853 361.012 868.091 347.113 873.799 333C876.16 327.163 882.104 318.3 881 312M682 355L681 367C682.956 363.32 683.629 358.891 682 355M447 363C447.276 366.161 448.042 368.979 449 372C449.961 368.749 448.773 365.821 447 363M799 364C798.232 367.357 797.3 370.56 797 374C798.933 370.923 800.245 367.51 799 364M389 371L390 375C390.393 373.076 390.146 372.64 389 371M450.333 373.667C450.278 373.722 450.222 374.778 450.667 374.333C450.722 374.278 450.778 373.222 450.333 373.667M451.333 376.667C451.278 376.722 451.222 377.778 451.667 377.333C451.722 377.278 451.778 376.222 451.333 376.667M392.333 378.667C392.278 378.722 392.222 379.778 392.667 379.333C392.722 379.278 392.778 378.222 392.333 378.667M522 395C522.18 399.122 523.17 402.975 524 407C525.525 403.084 523.949 398.586 522 395M333 405C333.939 408.26 335.296 411.068 337 414C337.513 410.48 335.265 407.61 333 405M983 406C974.656 419.584 962.498 432.982 957 448C965.426 440.278 971.366 427.676 977.424 418C979.667 414.418 983.634 410.367 983 406M525 408L525 411C525.696 409.446 525.696 409.554 525 408M911 408L908 415C910.151 412.848 911.505 411.07 911 408M265 410C270.324 424.066 281.602 437.552 290 450C290.364 445.779 286.527 441.504 284.319 438C278.543 428.837 273.013 417.343 265 410M376 410L376 415C389.588 415.112 404.582 419.238 411.583 432C416.371 440.727 414.623 450.502 415.039 460C415.914 479.983 416 499.951 416 520C411.71 513.248 416.613 502.846 407 500C406.997 495.435 407.553 489.656 402.895 487.038C400.088 485.46 396.805 486.334 394.224 484.041C391.852 481.934 391.394 478.542 389.428 476.108C385.064 470.706 378.21 470.522 372 472C368.406 464.379 363.236 465.047 356 464.085C350.645 463.373 346.301 462.596 341 464.363C337.402 465.562 334.475 467.893 331 469.273C324.424 471.884 316.623 471.007 310.015 474.456C304.165 477.51 304.468 483.008 301.066 487.725C298.955 490.651 294.877 491.743 292.09 493.928C287.894 497.219 284.809 501.925 283.221 507C280.711 515.019 283.914 522.956 282 531C280.805 528.834 280.309 527.447 280 525C267.934 526.903 271.765 542.622 280 547C276.192 553.799 275.35 562.898 284 566C282.214 573.594 284.876 578.555 293 579C293.359 585.551 296.573 588.835 303 590C303.749 599.848 315.422 604.723 324 606C321.821 608.562 319.63 613.31 316.621 614.879C312.593 616.979 305.253 612.184 302.093 609.895C293.912 603.97 290.189 593.122 282 586.669C264.635 572.986 243.499 572.473 223 567.373C215.889 565.604 205.337 560.496 198.019 562.322C193.436 563.466 192.521 567.975 193.855 572C195.628 577.348 199.913 581.028 205 583C200.308 594.076 208.371 601.425 218 606C212.699 617.017 222.06 624.137 232 627C226.41 637.488 236.608 643.18 246 644C238.868 654.853 249.657 658.994 259 659C255.556 670.657 264.097 674.138 274 671C274.109 680.253 279.529 683.589 288 680C290.675 687.438 296.718 686.216 302 682C301.677 697.661 292.075 708.188 284.453 721C276.5 734.368 271.06 751.285 273.3 767C275.154 780.008 280.424 793.158 291.001 801.481C295.422 804.961 302.792 806.337 306.397 810.498C309.073 813.587 308.646 820.11 310.054 824C312.36 830.373 315.259 836.349 319.004 842C320.953 844.941 324.735 848.46 325.414 851.985C326.183 855.971 321.576 857.645 319.093 859.633C316.154 861.987 313.849 865.235 313.225 869C311.821 877.482 316.569 883.69 319.637 891C321.246 894.837 321.117 899.083 322.482 903C326.09 913.352 333.52 924.178 346 922C344.364 926.248 342.417 930.384 343.808 935C345.986 942.231 353.047 944.95 357.787 950.093C360.76 953.319 362.067 957.502 364.637 960.985C367.174 964.421 370.632 966.089 373.91 968.633C380.484 973.734 386.227 977.879 393.848 970.701C400.849 964.106 394.125 956.298 391.164 950C389.54 946.547 390.256 942.649 389.482 939C388.412 933.955 385.399 929.19 385.144 924C384.733 915.648 389.32 907.326 390 899L473 900L473 894C457.294 893.676 444.789 885.704 440.899 870C437.55 856.479 439 841.821 439 828L439 757L439 721C439 715.781 437.914 709.156 439.318 704.105C441.679 695.609 456.308 697.464 461.567 690.906C469.058 681.565 469.5 670.111 467.374 659C466.276 653.263 466.447 648.854 460 647C463.683 637.452 468.141 626.429 467.871 616C467.59 605.147 462.246 594.783 456.329 586C452.468 580.269 446.926 569.25 439 571L439 469C455.062 496.519 467.024 527.069 480.309 556C498.558 595.744 516.964 635.504 535.745 675C543.918 692.19 551.964 709.553 559.576 727C562.562 733.845 565.611 745.815 572 750C574.626 740.816 579.048 731.88 582.6 723C590.369 703.579 598.482 684.288 606.576 665C625.951 618.826 644.725 572.375 663.603 526C671.976 505.433 683.146 484.402 689 463C692.11 473.522 690 487.045 690 498L690 568L690 798L690 843C690 851.221 690.739 859.85 689.572 868C687.301 883.854 679.952 893.953 663 894L663 900L744 900C768.18 900 792.53 900.27 816 893.573C833.63 888.542 847.659 879.554 863 870C863.001 882.565 865.861 894.662 867.92 907C868.881 912.76 871 919.144 869.76 925C868.822 929.429 866.606 933.594 865.478 938C864.569 941.554 864.66 945.579 863.404 949C860.747 956.241 853.494 962.37 860.148 970.367C868.381 980.261 874.544 972.453 882.039 967.309C888.674 962.755 890.583 956.986 895.529 951.093C898.452 947.611 902.649 945.878 905.895 942.79C912.358 936.643 911.045 929.73 909 922C927.829 925.286 929.689 904.222 934.88 892C938.002 884.649 943.128 878.53 941.671 870C941.057 866.413 939.437 862.998 936.786 860.478C933.988 857.818 928.122 855.831 929.857 850.995C931.031 847.725 934.03 844.846 935.971 842C939.164 837.319 941.925 832.226 944.124 827C946.063 822.394 946.162 814.306 949.6 810.699C952.65 807.498 958.323 806.312 962 803.779C970.361 798.02 976.181 789.478 979.561 780C985.572 763.143 984.018 743.258 976.688 727C971.612 715.739 962.699 705.627 961.184 693C960.871 690.391 960.104 684.801 962.603 683.002C964.793 681.426 968.483 683.139 970.996 682.754C977.247 681.796 977.953 676.238 978 671C988.122 675.864 995.415 668.269 993 658C1002.93 660.928 1010.76 653.9 1006 644C1016.16 643.107 1022.48 637.429 1020 627C1029.1 623.412 1038.69 616.667 1033 606C1038.83 603.454 1045.15 599.348 1047.3 593C1048.4 589.743 1047.3 586.107 1048.89 583C1050.63 579.599 1054.02 577.335 1055.91 574C1059.14 568.317 1057.43 561.715 1050 562.193C1043 562.643 1035.89 565.8 1029 567.199C1009.83 571.095 989.984 573.721 974 585.899C962.162 594.918 958.369 609.799 944 616C941.711 611.405 938.537 607.698 935 604C939.275 603.518 941.288 602.363 942 598C947.326 597.4 948.829 595.23 949 590C956.649 589.211 956.259 585.375 960.587 580.495C962.242 578.629 964.748 577.825 966.258 575.776C967.961 573.465 967.595 570.716 968.584 568.17C969.602 565.55 971.821 563.632 972.852 560.985C974.887 555.758 972.592 550.971 971 546C976.824 543.127 980.323 532.896 976.107 527.153C972.793 522.64 970.305 525.87 971 530C964.786 526.571 968.404 518.066 967.907 512C967.025 501.251 958.915 491.118 948 490C947.804 487.608 947.547 485.286 946.751 482.999C942.584 471.026 930.899 472.528 921 469.642C916.771 468.409 913.305 465.322 909 464.044C901.15 461.713 890.267 463.609 883 467C873.976 448.031 853.147 434.803 835 425.753C804.918 410.751 771.843 410 739 410L662 410L662 415C665.398 415.009 668.679 415.089 672 415.9C680.67 418.017 688.063 424.311 686.786 434C685.973 440.169 682.281 446.335 679.85 452C675.44 462.274 671.153 472.62 667 483C648.422 529.436 629.256 575.672 610.397 622C601.219 644.545 590.802 666.937 583 690C572.252 671.515 564.619 650.445 555.691 631C532.712 580.948 510.194 530.6 486.258 481C480.037 468.107 473.966 455.076 468.139 442C465.849 436.862 461.476 430.917 462.269 425.001C463.222 417.883 471.002 415.812 477 415L477 410L376 410M849 499C840.404 500.75 840.987 506.884 840.081 514C839.731 516.751 838.285 519.198 838.151 522C837.765 530.118 840.368 540.213 847 545C845.317 556.399 839.982 566.062 842.68 578C844.492 586.021 851.844 592.175 852.867 600C855.013 616.425 853 634.425 853 651C850.834 648.799 849.118 644.361 845.949 643.497C838.943 641.587 839.708 651.365 841 655C839.182 653.856 837.293 651.65 834.961 651.827C831.727 652.073 829.808 655.154 829.224 658.015C827.87 664.637 825.503 674.477 828.199 681C830.588 686.779 833.656 693.736 839.001 697.297C842.966 699.939 851.148 698.916 852.682 704.189C854.536 710.562 853 719.379 853 726L853 759C853 764.347 853.974 770.823 852.49 776C850.281 783.708 844.763 789.867 843.015 798C841.534 804.889 842.74 812.527 845.464 819C847.724 824.368 851.344 827.859 849.839 834C840.874 870.596 807.706 887.854 773 891.17C762.358 892.187 750.377 893.154 740 889.775C728.976 886.187 729.12 872.786 729.004 863C728.691 836.675 729 810.327 729 784L729 525L729 456C729 446.63 726.999 433.447 732.089 425.044C735.039 420.175 740.791 419 746 418.084C757.263 416.103 769.937 417.112 781 419.873C807.97 426.606 832.708 440.568 846.218 466C848.792 470.845 852.397 476.526 853.332 482C853.961 485.682 851.584 487.886 850.16 491C848.985 493.568 849.025 496.239 849 499M576 427L576 432C576.83 429.97 576.83 429.03 576 427M577 435L577 440C577.83 437.97 577.83 437.03 577 435M359 470C359 468.203 359.014 468.478 360 467C365.123 468.337 368.533 470.793 370 476C377.896 474.12 387.091 474.623 389 484C384.058 479.49 380.852 477.062 374 477C376.914 477.995 380.253 478.442 382.896 480.089C385.398 481.648 386.84 484.297 389.209 486.007C393.952 489.429 398.316 490.297 401 496C398.47 495.352 397.115 494.573 395 493C395.983 494.276 400.053 499.316 401.558 495.625C402.33 493.732 400.605 490.761 400 489C405.332 493.706 401.524 496.139 402.518 501.741C403.412 506.775 407.254 508.004 406 514C408.582 509.793 408.001 507.372 406 503C414.168 507.772 406.486 515.431 406 522C407.186 520.964 407.604 520.661 409 520C408.669 524.331 407.57 526.396 403 526C403.406 523.908 403.217 523.98 405 523C402.778 519.176 399.733 516.406 397 513C399.814 513.642 400.76 513.771 403 512C400.156 512.495 398.694 512.098 396 511C397.509 502.452 391.062 488.845 382 487C387.116 490.331 391.899 496.127 393.951 501.999C395.74 507.117 392.609 509.757 388 508C390.383 501.724 388.504 498.765 384 494C387.885 499.98 390.601 506.645 382 510C382 508.203 382.014 508.478 383 507C375.316 502.686 374.256 496.125 364 498C366.87 498.851 370.316 498.646 372.985 500.028C380.953 504.155 379.434 513.448 376 520C374.839 517.487 374.46 514.489 372.427 512.434C367.79 507.743 362.905 512.417 358 513.581C354.948 514.305 351.977 513.708 349 513C356.718 519.704 361.507 509.151 368.941 511.647C377.109 514.39 371.685 524.385 366 526C366.985 524.509 368.525 523.285 369.397 521.737C370.99 518.909 369.27 515.839 365.981 515.708C361.013 515.509 357.311 519.8 352 518.336C346.14 516.722 342.667 511.421 338 508C344.916 516.417 348.899 530.266 338.815 538.196C335.227 541.018 330.26 541.647 326 543C327.922 545.905 328.272 546.773 327 550L331 548C328.598 555.198 321.698 552.303 317 549C319.668 552.584 325.809 554.819 327.164 559.055C330.678 570.042 315.278 571.513 309 568C314.786 574.557 326.383 571.755 328.858 562.999C329.665 560.146 328.103 557.538 327 555C332.415 558.164 331.265 564.543 327.606 568.945C322.24 575.398 309.956 577.55 305.318 568.907C304.165 566.759 304.11 564.366 304 562C302.974 564.626 302.559 566.321 303.88 569.002C304.891 571.053 308.093 573.967 306.248 576.397C304.114 579.208 299.604 577.286 297 576.567C290.637 574.81 285.358 573.979 287 566C289.007 569.111 290.538 572.162 293 575C290.314 567.434 288.884 562.923 294 556C291.794 557.532 289.945 559.138 288 561C284.111 557.565 284.07 554.653 286 550C280.889 554.53 283.585 557.763 286 563C273.203 558.73 287.293 546.038 291 541C285.836 544.408 280.208 545.113 277 539C280.036 540.079 282.825 541.396 286 542L286 540C280.285 539.569 276.885 536.734 276 531C282.171 535.404 286.195 532.228 287 525C290.241 529.745 288.87 534.268 284 537C292.852 535.451 289.533 527.353 290.316 521C290.794 517.114 292.539 513.58 294 510C291.031 514.088 288.553 517.906 288 523C281.726 512.527 289.386 498.032 300 493.995C304.047 492.456 308.749 493 313 493C313.019 486.218 314.67 477.364 322.015 474.604C325.261 473.384 328.734 474.363 332 472.892C341.924 468.421 348.394 464.669 359 470M938 492C942.558 492.094 947.859 492.362 952 494.456C961.414 499.216 966.958 511.969 964 522C963.126 519.948 963.047 518.253 963 516C961.521 516.986 961.797 517 960 517C958.654 512.468 956.229 509.406 953 506C955.487 510.131 958.422 514.199 959.467 519C960.131 522.05 959.156 524.98 959.564 528C960.165 532.449 963.58 535.533 967 538C962.983 533.75 959.627 529.669 963 524C965.303 531.203 966.434 534.673 974 530C973.645 536.461 970.32 539.776 964 541C967.817 541.717 970.412 540.272 974 539C969.642 544.498 964.702 544.763 959 541C963.343 546.663 977.038 556.034 966 563C967.898 556.914 968.493 553.185 963 549C966.523 553.43 967.145 556.168 964 561L958 557C959.67 559.493 961.64 561.868 961.772 565.004C962.182 574.678 951.821 579.759 944 576L944 574C948.859 569.847 950.076 566.051 948 560C947.822 562.952 948.253 566.247 946.82 568.956C942.072 577.93 928.757 575.477 924.009 567.957C921.402 563.828 920.755 556.999 925 554C924.454 557.278 922.953 560.753 924.361 563.999C927.71 571.718 937.391 574.357 943 568C936.8 571.469 923.155 570.606 925.478 560.018C926.345 556.063 930.162 554.894 932.957 552.586C936.985 549.258 937.796 545.929 937 541C935.934 544.476 935.854 548.649 932.656 550.987C929.869 553.024 924.77 552.954 923.827 548.893C923.275 546.513 924.994 544.015 926 542C921.874 541.222 917.539 540.842 914.015 538.347C903.36 530.804 906.187 516.961 913 508C908.46 511.458 905.048 516.823 899 517.772C894.196 518.525 887.46 513.349 883.333 516.043C879.274 518.693 883.484 522.53 886 524C879.168 526.366 875.632 515.711 881.394 512.194C884.681 510.189 887.971 512.755 891 513.93C894.806 515.406 898.763 515.452 902 513C899.326 513.635 896.753 514.184 894 513.603C887.407 512.209 879.611 506.697 876.939 516.959C876.595 518.279 876.891 519.682 877 521C871.287 516.01 870.16 505.569 877.109 500.742C880.004 498.732 883.651 499.01 887 499C879.659 495.148 872.549 500.33 871 508C867.3 506.135 866.308 504.089 866 500C864.337 503.919 865.475 505.739 868 509C862.455 510.506 858.815 507.592 858 502L854 509L855 507C856.085 511.593 854.234 515.131 850 517C851.457 519.985 851.859 522.108 848 522L851 526C845.76 526.986 843.665 523.934 843 519C844.922 519.901 844.999 520.076 846 522C847.361 515.913 839.742 508.769 846 504C845.249 507.269 845.249 509.731 846 513C847.523 508.002 847.318 503.687 853 502C851.7 494.971 853.3 487.898 862 489C864.278 479.526 871.599 475.222 881 478C882.403 472.102 885.942 468.675 892 468C890.894 470.513 889.477 472.678 888 475C891.143 472.292 893.828 468.576 898 467.434C906.09 465.218 911.973 469.905 919 472.637C921.925 473.774 925.125 472.992 927.999 474.333C934.786 477.5 937.835 484.994 938 492M347 469C349.815 470.215 352.967 470.435 356 471C353.196 468.875 350.466 469.012 347 469M895 470C897.905 470.894 900.948 470.129 904 470C900.917 468.718 898.208 469.238 895 470M359 470C361.12 471.721 362.265 472.035 365 472C366.218 474.118 366.867 474.845 369 476C366.614 471.653 363.894 470.268 359 470M884 476L890 471C886.453 470.51 885.115 472.895 884 476M353 475C356.842 476.628 361.838 477.295 366 478C362.005 474.106 358.289 473.745 353 475M887 479C891.089 477.64 894.644 476.202 899 476C894.671 473.094 889.559 474.574 887 479M314 476C311.739 480.943 308 484.479 307 490C302.919 484.359 309.25 478.805 314 476M910 476C912.24 478.784 915.993 479.186 918.985 481.133C923.39 484 925.966 488.287 928 493C930.65 483.337 917.617 477.031 910 476M936 476C941.848 479.107 943.848 483.544 944 490C942.053 486.053 940.867 482.501 938 479C939.893 483.056 940.904 486.512 941 491L939 491C938.884 486.054 935.321 480.49 936 476M321 493C326.791 486.366 330.623 479.673 340 478C331.158 474.402 322.409 485.22 321 493M315 492L323 478C317.126 480.807 315.286 485.753 315 492M927 478C929.793 482.28 932.337 486.126 934 491C936.161 485.404 931.877 480.329 927 478M865 485C868.769 483.438 872.058 481.168 876 480C871.227 478.161 867.768 481.286 865 485M358 481C362.231 482.793 367.488 483.713 372 485C368.059 480.488 363.687 479.992 358 481M313 481C312.105 484.382 311.18 487.489 311 491L309 491C309.215 486.86 310.041 483.965 313 481M881 486C884.677 484.076 887.832 482.429 892 482C887.387 480.083 883.283 481.65 881 486M380 482C381.341 483.274 382.403 484.041 384 485C382.771 483.13 382.132 482.776 380 482M241 484C243.627 490.206 248.781 496.752 254 501C250.95 494.875 246.314 488.326 241 484M325 497C330.165 494.243 333.95 489.209 339 486C332.552 484.288 327.739 492.196 325 497M863 493C865.893 490.414 868.718 488.089 872 486C867.034 484.977 863.741 488.313 863 493M907 485C909.928 486.646 913.24 487.495 915.996 489.498C918.835 491.562 920.949 494.253 924 496C920.789 488.438 915.024 485.172 907 485M365 487C369.199 489.842 372.704 492.022 376 496C376.626 489.384 370.73 487.298 365 487M874 496L875 497C876.875 496.453 878.255 495.89 880 495C878.341 494.594 877.758 494.691 876 495C880.064 489.531 884.362 487.827 891 489C884.319 483.938 876.393 488.981 874 496M854 498C856.069 495.597 857.437 492.776 859 490C855.422 491.877 854.443 494.032 854 498M1002 490C998.981 493.805 995.897 497.513 994 502C997.419 499.217 1002.23 494.626 1002 490M332 501C336.333 498.04 340.018 494.852 345 493C339.316 491.034 334.254 496.21 332 501M862 499C864.507 497.205 866.205 495.507 868 493C864.716 494.343 863.244 495.665 862 499M288 514C294.255 506.451 296.582 499.008 307 496C297.355 492.58 288.83 505.893 288 514M945 495C952.27 501.109 957.766 504.518 961 514C964.238 504.869 952.792 496.406 945 495M866 499L867 500L866 499M338 504C342.087 505.208 346.368 502.914 350 501C345.879 500.059 341.642 502.088 338 504M858 500L858 502C859.635 501.455 859.455 501.635 860 500L858 500M902 500C904.934 502.835 907.976 503.646 912 504C909.012 501.765 905.62 500.838 902 500M384 501L383 507C385.597 505.148 385.798 503.596 384 501M860 502C860.526 504.543 860.878 505.535 863 507C862.251 504.746 861.656 503.686 860 502M367 503C369.46 504.288 371.74 505.338 374 507L372 507C373.836 509.447 375.237 510.701 378 512C376.409 505.909 373.301 503.205 367 503M391 503L391 509C392.582 506.943 392.826 505.613 393 503L391 503M868.667 503.333C868.222 503.778 869.278 503.722 869.333 503.667C869.778 503.222 868.722 503.278 868.667 503.333M874 513L879 509C877.502 508.521 877.65 508.6 876 509C878.089 506.349 879.801 505.086 883 504C876.982 502.046 874.279 507.916 874 513M398 505L399 509C399.751 506.953 399.656 506.404 398 505M849 505C848.014 506.479 848 506.203 848 508C849.766 507.021 849.955 506.778 849 505M892 506C895.596 510.536 904.137 513.959 908 508C901.756 509.159 897.784 508.564 892 506M375 507L374 508L375 507M858 507L858 511C857.29 509.241 857.29 508.759 858 507M344 509C348.423 513.144 354.012 512.315 358 508C352.931 509.341 349.215 509.965 344 509M389 510L389 515C390.947 514.547 392.269 514.006 394 513C399.147 522.054 400.4 532.066 402.895 542C407.04 558.502 413.748 572.587 404.475 589C401.096 594.979 397.011 603.362 390.996 606.991C386.979 609.415 382.443 608.747 378 609.439C361.716 611.977 342.057 609.885 334 593C324.9 599.332 313.017 587.19 312 578C315.267 578.466 317.945 579.805 321 581C319.814 579.814 319.416 579.472 318 579C323.694 577.785 326.088 583.109 332 579C325.241 574.588 332.026 569.658 333.438 564C334.229 560.833 333.176 558.125 333.015 555C332.815 551.105 333.835 547.963 333 544C345.094 540.408 347.858 530.26 347 519C349.832 520.09 352.879 521.709 356 521.424C360.526 521.011 364.288 518.244 369 518C367.297 521.17 365.455 522.09 362 523C368.387 535.663 380.181 519.655 382 512C384.58 511.973 385.138 511.771 387 510L389 510M302 536C303.471 532.495 302.661 528.681 303.61 525C304.96 519.76 308.377 515.61 311 511C301.066 516.045 301.247 526.465 302 536M852 511C852.406 513.092 852.217 513.02 854 514C853.309 512.612 853.016 512.139 852 511M869 512C870.446 518.034 873.336 523.087 879.004 526.079C880.978 527.122 889.263 529.071 888.37 524.19C887.989 522.111 884.116 520.056 883 518L884 517C892.317 519.395 896.313 521.298 905 519C902.897 530.367 908.337 542.625 921 544C920.147 549.148 917.875 556.983 918.623 562C919.273 566.365 923.193 569.684 923.554 573.998C923.77 576.581 922.183 578.846 921 581C923.863 581.561 926.123 581.476 929 581L925 588C927.018 587.66 927.968 587.677 930 588L929 584L932 583C931.45 577.581 935.771 577.761 940 579C937.94 587.149 929.307 598.739 920 594C911.784 609.837 893.865 612.236 878 610.147C871.239 609.256 865.315 609.724 860.418 603.895C858.453 601.556 857.467 598.583 855.882 596C850.627 587.438 844.976 579.63 845.259 569C845.494 560.207 849.59 552.511 850.83 544C851.91 536.595 852.746 529.17 855.029 522C855.917 519.207 856.904 515.516 859.394 513.692C861.807 511.924 866.113 512.479 869 512M941 512C944.174 520.72 947.784 526.388 946 536C951.381 528.897 947.212 517.147 941 512M847 513C848.223 514.195 848.42 514.316 850 515C848.861 513.984 848.389 513.691 847 513M405 514L406 515L405 514M194 515C196.846 521.723 202.445 527.414 207.081 533C210.85 537.542 214.227 542.487 219 546C212.876 535.355 203.247 523.081 194 515M1053 516L1054 517L1053 516M1051 518C1043.54 528.062 1034.22 537.085 1028 548C1034.97 542.872 1040.03 534.733 1045.4 528C1047.76 525.042 1051.2 521.898 1051 518M402 519C402.986 520.478 403 520.203 403 522C401.711 520.557 401.599 520.766 402 519M412 519C411.943 525.257 410.808 529.235 404 530L404 528C409.05 526.547 409.85 523.322 412 519M842 519C842.649 523.853 844.714 526.668 849 529C843.114 530.847 840.048 523.798 842 519M849.333 519.667C849.278 519.722 849.222 520.778 849.667 520.333C849.722 520.278 849.778 519.222 849.333 519.667M933 520C935.155 526.341 940.767 530.192 942 537L944 537C943.256 529.787 938.311 524.595 933 520M316 521C311.013 526.032 306.746 530.716 306 538L308 538C309.096 531.853 316.123 526.565 316 521M295 522C294.275 525.919 294.082 529.1 295 533C296.431 529.59 296.431 525.41 295 522M915 522C915.426 531.302 920.121 535.939 929 538L927 546C929.221 544.325 929.562 542.743 930 540C932.481 545.524 929.953 549.081 924 547C928.917 552.735 934.824 547.598 932.257 541.018C930.184 535.704 923.996 535.195 920.228 531.61C917.445 528.963 916.928 525.151 915 522M954 522L954 533C955.431 529.59 955.431 525.41 954 522M331 524C327.179 531.678 311.707 540.72 322.064 549.258C323.249 550.235 324.592 550.517 326 551C323.882 548.465 320.322 545.668 320.662 542C321.358 534.494 333.534 532.683 331 524M416 525C416 551.21 417.854 578.124 416 604C411.28 602.712 406.868 600.719 402 600C406.508 589.281 413.302 581.337 413.544 569C413.69 561.564 408.871 555.118 408.526 548C408.339 544.143 411.635 541.453 412.813 538C414.428 533.267 412.63 529.243 416 525M910 525C910.412 532.61 914.716 537.835 922 540C920.25 537.317 917.529 536.107 915.339 533.787C912.932 531.235 911.856 527.894 910 525M341 526C339.603 533.709 333.994 534.547 330 540C336.31 538.483 343.88 533.455 341 526M922 528C923.662 531.376 925.445 532.78 929 534C926.893 531.585 924.67 529.776 922 528M335 530C332.46 532.285 329.884 534.172 327 536C330.895 537.019 335.088 534.095 335 530M410 531C409.961 535.318 409.968 538.682 406 541L409 534C407.6 535.13 406.584 536.218 405 537L405 532L410 531M845 531C846.707 531.857 847.109 531.982 849 532L849 537C846.876 536.101 846.899 536.124 846 534C846.035 536.949 847.025 539.206 848 542C844.23 539.011 841.808 535.286 845 531M938 535C939.638 542.671 941.537 546.42 938 554C945.451 549.757 943.248 540.198 938 535M310 536C307.287 540.844 304.569 549.163 310 553C309.312 550.374 308.295 547.756 308.622 545C309.003 541.791 310.676 539.217 310 536M856 538C858.663 541.632 864.122 543.665 868 546C865.534 540.84 861.521 538.831 856 538M323 539C322.236 541.746 322.236 543.255 323 546C324.088 543.749 324.519 541.465 325 539L323 539M385 546C388.778 544.052 391.954 541.505 396 540C390.942 538.051 387.108 541.588 385 546M294 542C292.402 544.303 291.073 546.419 290 549C293.055 547.398 294.934 545.548 294 542M313 542C311.396 547.595 312.898 552.049 318 555L313 542M953 542C955.66 547.597 959.297 553.284 965 556C962.504 551.107 957.833 544.643 953 542M855 549C854.068 556.194 863.188 563.421 868 556C862.635 555.721 858.892 554.277 857 549L855 549M950 549L950 555L954 556C952.829 553.438 951.82 551.133 950 549M298 550C298.077 552.645 297.997 554.266 300 556L300 550L298 550M398 550C394.217 553.576 391.278 555.711 386 556C391.045 562.626 397.372 556.217 400 551L398 550M286 557L292 552C288.349 551.254 286.942 553.747 286 557M384 551C383.427 554.232 383.3 556.784 384 560C385.256 557.006 385.256 553.994 384 551M869 553L869 560C870.059 557.466 870.059 555.534 869 553M346 564L363 555C356.308 552.627 349.579 558.964 346 564M889 554C893.762 557.563 899.9 559.672 905 563C901.922 556.554 895.859 554.161 889 554M950 557C951.74 563.575 951.468 566.687 949 573C954.074 568.978 956.115 561.19 950 557M299 558C296.379 563.447 296.067 569.008 302 572C298.681 566.377 298.88 564.021 301 558L299 558M304 562C305.753 560.67 306.67 559.753 308 558C305.751 559.154 305.154 559.751 304 562M324 558C322.897 563.404 319.574 566.526 314 565C319.755 570.803 325.294 564.331 326 558L324 558M928 558C926.262 565.212 933.329 570.768 939 565C932.602 565.98 930.776 563.265 928 558M932 558L931 562C932.87 560.771 933.224 560.132 934 558L932 558M321.333 559.667C321.278 559.722 321.222 560.778 321.667 560.333C321.722 560.278 321.778 559.222 321.333 559.667M958 563C957.093 568.713 954.254 572.541 949 575C955.434 577.011 960.229 568.479 958 563M293 564C293.21 570.466 296.393 574.98 303 576C298.764 572.174 296.066 568.753 293 564M887 564C887.497 574.7 893.876 575.669 903 575C899.299 569.286 891.351 572.282 889 564L887 564M314 620C312.315 629.483 307.458 638.406 305.618 648C303.996 656.457 304.803 667.095 301.656 674.999C300.258 678.511 294.333 685.059 291.603 678.851C289.715 674.555 291.892 669.187 293 665C290.529 668.884 288.984 674.681 284.896 677.214C280.743 679.786 277.328 676.181 277.202 671.996C277.045 666.767 280.441 662.254 283 658C279.106 661.883 274.67 667.389 269.001 668.562C264.679 669.456 261.07 666.486 261.641 661.999C262.184 657.74 265.339 654.132 268 651C262.925 653.109 257.738 656.192 252.019 654.566C241.793 651.659 251.693 642.699 257 641C250.447 639.58 244.307 641.736 238.019 637.914C234.267 635.634 231.89 630.986 237.059 628.738C241.721 626.712 248.017 627.003 253 627C243.561 624.21 233.639 626.142 225.094 619.776C222.4 617.769 218.188 613.058 221.608 609.757C223.852 607.591 227.375 608.798 230 609.339C236.522 610.686 243.348 611.714 250 612C238.673 607.751 226.434 607.832 216 600.895C212.611 598.642 207.07 594.597 206.89 590.059C206.685 584.889 213.178 586.77 216 587.671C226.281 590.95 238.419 592.191 248 597C246.319 601.085 248.434 603.038 252 605C251.873 608.932 252.918 610.602 256 613C254.702 619.183 256.593 622.703 263 624C261.496 629.846 263.213 633.014 269 635C268.348 640.907 271.218 643.564 277 644C277.834 649.384 280.587 651.597 286 651C287.734 655.818 290.281 656.746 295 655C296.709 660.609 300.483 659.47 303 655C300.962 656.395 299.373 657.349 297 658C296.181 654.438 296.345 651.579 297 648C295.958 649.738 294.964 652.066 293.366 653.382C287.301 658.377 289.075 647.543 290 645C288.449 646.434 286.95 648.094 284.98 648.978C276.716 652.685 280.568 642.047 283 639C280.304 640.355 277.225 642.303 274.059 641.638C267.427 640.246 272.942 634.641 276 633C272.931 632.662 268.915 633.624 266.153 631.971C259.371 627.915 268.198 624.208 272 624C268.26 622.894 263.569 623.661 260.149 621.682C252.006 616.97 263.511 615.003 267 615C263.459 613.88 248.067 611.091 257 606C253.14 604.158 250.025 602.895 248 599C251.843 597.896 255.085 598.506 259 599C256.372 597.836 253.731 597 251 596.112C249.242 595.54 239.834 590.89 245.337 588.696C251.656 586.177 261.185 594.775 263 600C266.049 598.384 267.921 599.656 271 601C262.479 594.432 253.605 588.455 243 586C242.434 588.399 242.049 590.539 242 593C230.698 591.541 218.875 587.846 209 582.124C202.426 578.316 197.427 573.902 197 566C205.129 566.067 212.132 569.248 220 570.739C238.87 574.315 259.913 576.459 276 587.721C285.777 594.566 290.228 605.154 299.015 612.787C303.316 616.524 308.678 618.265 314 620M365 566C362.853 568.035 360.731 570.248 357.998 571.506C354.82 572.97 352.082 571.967 350 575C358.508 576.07 362.946 575.094 367 567L365 566M1054 566C1052.14 584.018 1025.62 589.983 1011 593L1010 587C1016.26 583.847 1023.17 581.579 1030 580C1025.03 579.265 1020.46 582.858 1016 580C1013.06 585.612 1006.4 586.556 1001 589.12C996.814 591.11 993.141 593.949 989 596L992 598C994.989 591.924 1003.31 587.61 1010 589C1007.78 595.205 1001.99 596.747 996 598L1005 599C1002.36 604.361 997.657 605.537 992 606C994.729 606.81 997.15 606.979 1000 607C998.967 613.089 993.412 613.763 988 614L988 616C991.328 616.039 999.216 618.018 992.847 621.822C990.041 623.498 986.13 622.998 983 623C984.115 623.793 985.787 624.703 986.722 625.756C993.586 633.487 979.982 632.451 976 631C977.445 632.335 979.164 633.658 980.278 635.309C985.393 642.896 974.944 641.437 971 640C975.405 648.017 971.395 652.249 964 646C964.422 648.283 965.46 651.428 964.383 653.663C962.533 657.501 958.92 653.003 957.97 650.981C955.659 646.062 955.221 640.162 953.484 635C951.64 629.521 948.597 624.544 947 619C961.504 612.74 965.918 597.39 978 588.239C993.256 576.685 1013.76 574.227 1032 570.551C1039.42 569.056 1046.37 566.158 1054 566M964 567C965.246 572.16 963.116 574.942 958 576C960.327 573.083 962.141 570.206 964 567M389 570C389.69 576.602 385.349 577.491 381 581C379.814 579.814 379.416 579.472 378 579L379 583C384.971 581.416 396.483 577.335 389 570M864 570C860.366 577.248 871.36 586.525 876 580C868.709 579.756 866.199 576.405 864 570M446 580C443.87 577.721 442.299 575.819 441 573C446.853 575.86 450.262 582.55 453.55 588C458.576 596.331 463.328 605.065 463.907 615C464.433 624.04 461.112 633.652 458 642C455.894 641.493 453.342 640.471 451.212 641.388C448.578 642.523 447.744 645.685 446.187 647.831C444.325 650.396 441.879 652.435 440 655C436.489 646.633 439 633.076 439 624L446 629L443 626C444.922 624.708 446.84 623.858 449 623C439.586 622.237 437.338 616.749 429.91 612.225C427.387 610.687 421.558 609.99 420.461 606.87C418.829 602.225 430.996 603.644 433 604C428.107 600.804 423.45 599.727 418 598C421.234 591.837 427.998 592.571 434 594C429.444 590.269 421.75 589.988 419 585C426.416 582.297 431.82 583.075 439 586C434.973 580.786 427.207 580.008 424 575C433.919 571.905 437.382 575.606 446 580M324 575C325.752 576.59 326.794 577.954 328 580C325.483 578.753 323.917 577.895 324 575M928 575C927.349 577.434 926.84 578.283 925 580L926 575L928 575M944 578C947.225 579.709 949.357 580.414 953 580C950.675 583.962 947.985 583.594 944 582C948.634 585.221 951.546 584.354 955 580C956.535 587.533 946.467 586.88 942 585C947.388 591.779 943.988 596.343 936 593C939.382 592.17 941.881 593.231 944 590C942.365 590.545 942.545 590.365 942 592L938 591C939.506 590.317 940.314 590.174 942 590C940.276 585.784 941.857 581.915 944 578M223 579C228.132 582.839 236.711 584.918 243 586L237 580C232.291 582.114 227.84 579.712 223 579M309 579C310.356 583.324 311.43 586.533 307 589L310 585C304.16 586.083 298.936 586.673 297 580C301.136 580.04 304.926 579.661 309 579M933 579C934.229 580.87 934.868 581.224 937 582C936.422 579.303 935.738 579.211 933 579M314 580L315 581L314 580M321 581C322.33 582.753 323.247 583.67 325 585C323.814 586.186 323.472 586.584 323 588L329 587C326.709 583.922 324.641 582.239 321 581M299 582C300.223 583.195 300.42 583.316 302 584C300.861 582.985 300.388 582.691 299 582M304 583C305.809 583.574 306.069 583.465 308 583C306.341 582.594 305.758 582.691 304 583M395 582C389.867 586.413 380.261 588.024 378 595C385.164 592.908 394.509 589.752 397 582L395 582M858 582C860.472 589.902 869.748 592.882 877 595C874.354 588.312 864.349 584.385 858 582M934 583L933 585C934.263 584.029 934.392 584.306 934 583M1003 611C1009.2 612.706 1015.73 610.053 1022 609.286C1024.3 609.005 1028.29 608.313 1030.21 610.028C1033.63 613.079 1028.31 618.237 1025.96 619.895C1017.78 625.659 1007.6 626 998 626C1002.68 627.385 1013.27 625.976 1016.29 630.318C1018.36 633.302 1015.36 636.437 1012.96 637.995C1006.53 642.164 999.861 640.568 993 639C995.57 640.516 998.516 641.81 1000.77 643.789C1005.27 647.733 1004.5 654.309 997.995 654.877C992.201 655.382 987.404 652.348 983 649C985.562 652.927 989.403 657.087 989.728 661.999C990.173 668.697 983.527 669.842 979 666.566C976.054 664.435 973.563 661.561 971 659C972.847 663.527 975.834 668.938 974.552 673.995C971.878 684.54 964.407 675.853 963 670C962.19 672.729 962.021 675.151 962 678L960 678C960 670.453 959.398 663.422 958 656C959.922 656.901 959.999 657.076 961 659C964.609 657.02 966.177 655.045 967 651C972.265 651.857 975.46 649.47 975 644C981.072 643.667 984.306 641.333 983 635C988.629 633.854 991.045 630.698 990 625C995.561 622.81 997.925 620.037 997 614C1000.46 611.344 1001.48 609.352 1001 605C1004.67 603.188 1006.67 601.065 1005 597C1013.82 592.572 1024.59 591.446 1034 588.333C1036.56 587.486 1042.07 584.866 1043.84 588.333C1045.87 592.304 1039.6 597.438 1037 599.471C1027.74 606.691 1014.53 610.082 1003 611M314 589C312.212 590.421 311.294 590.697 309 591C310.75 589.745 311.857 589.386 314 589M307 590C310.356 592.082 312.207 591.885 316 591C313.672 593.375 305.634 595.735 307 590M386 595C389.764 596.226 392.059 594.148 394 591C391.126 592.045 388.618 593.414 386 595M861 591C862.85 594.559 865.195 595.867 869 597C867.068 593.891 864.247 592.599 861 591M321 595C319.135 598.7 317.089 599.692 313 600C314.269 595.626 316.504 594.007 321 595M934 594C942.261 598.213 932.033 602.819 934 594M318 595L314 598C316.479 597.912 318.089 597.707 318 595M935 596C935.406 598.092 935.217 598.02 937 599C936.316 597.42 936.195 597.223 935 596M325 598C323.608 599.433 322.767 600.045 321 601L322 597C323.797 597 323.522 597.014 325 598M335 597C337.066 599.815 339.185 602.044 342 604.139C343.401 605.181 345.749 606.175 346.333 607.917C346.683 608.96 344.145 608.971 344 609C356.435 612.934 368.046 614.637 381 613C377.397 616.597 372.928 617.363 368 618C372.993 619.258 377.173 617.29 382 616L381 620C382.469 617.952 383.759 615.288 385.635 613.586C387.421 611.967 389.926 611.569 391.997 610.432C394.89 608.844 397.073 605.745 400.17 604.614C404.924 602.879 413.543 606.324 415.397 611.044C416.971 615.053 416 620.76 416 625L416 656L412 656C412 643.604 409.225 633.815 406 622C405.617 627.712 407.895 633.35 408.7 639C409.513 644.705 408.714 650.349 408 656C399.573 656.968 396.758 651.48 391 646.043C387.732 642.957 382.829 641.011 380.074 637.529C373.114 628.734 369.988 619.461 358 616C362.888 619.595 367.625 622.992 371.211 628C373.765 631.566 374.743 636.342 377.56 639.621C380.875 643.48 386.212 645.673 389.961 649.174C394.085 653.025 398.594 657.347 396 663C408.291 658.267 417.581 661.387 430 662.815C434.04 663.279 437.969 662.358 442 663C439.404 655.638 448.823 649.946 451 643C458.167 645.264 454.625 651.187 452 656C455.519 654.77 457.071 653.169 459 650C464.179 652.614 463.801 656.01 462 661C468.164 665 463.869 677.727 461.451 682.999C457.375 691.884 452.009 691.334 444 694.944C434.256 699.335 425.792 703.309 415 704.711C393.758 707.47 374.149 700.522 358 686.7C350.366 680.166 344.228 671.796 336 666C337.848 668.265 341.464 671.459 337 672C343.93 676.297 348.182 682.379 354.001 687.961C360.285 693.989 367.988 699.543 376 703C372.184 704.368 367.909 702.758 364 702C371.176 705.762 378.559 704.8 386 707C382.837 708.327 379.414 708 376 708C381.528 710.332 385.383 709.486 391 708C390.099 709.922 389.924 709.999 388 711C389.479 711.986 389.203 712 391 712C389.816 716.803 389.017 721.444 387 726C392.929 731.181 386.637 745.846 383 751C377.463 748.087 372.031 745.018 366 743.208C361.427 741.836 356.517 741.758 352 740.316C347.896 739.007 344.244 736.985 340 736C342.578 737.605 345.234 738.75 348 740C344.616 741.522 340.642 742.167 337 743C345.135 743.894 352.91 743.503 361 745.745C389.641 753.683 427.971 783.545 411.005 817C407.881 823.16 402.855 827.148 398.376 832.17C389.032 842.646 380.058 853.058 370 862.996C364.81 868.124 357.461 872.516 354 879L348 879C350.166 880.195 351.553 880.691 354 881C354.097 892.667 355.992 904.323 356 916L353 916L353 909L351 909C350.972 912.422 350.625 915.637 350 919C348.203 919 348.478 918.986 347 918C347.359 915.489 347.402 913.502 347 911C345.747 914.662 345.439 917.896 342 920C341.988 916.533 342.125 913.804 340 911C340.723 914.793 339.855 916.331 336 917C336.297 914.2 336.445 911.783 336 909L334 915L332 915C330.307 910.167 327.293 905.826 325.803 901C324.792 897.724 325.219 894.23 324.006 891C320.266 881.04 310.728 867.819 325 861L325 868C327.142 861.838 330.247 856.173 332.333 850C338.893 830.584 344.396 804.173 370 803L370 801C366.578 801.028 363.363 801.375 360 802C362.057 799.307 364.452 797.241 367 795C358.792 799.677 351.824 809.158 343 812.397C326.58 818.423 310.11 801.853 299 793L309 805C283.905 799.833 273.978 770.959 278.389 748C279.806 740.624 282.892 733.661 286.222 727C289.441 720.562 293.835 715.096 297.611 709C298.967 706.811 301.957 700.76 303 706C304.171 702.04 304 698.113 304 694C305.838 697.064 306.15 700.55 307 704C308.491 695.188 305.476 685.955 306.09 677C307.935 650.055 313.53 626.856 330 605L334 606C332.238 602.685 331.97 600.614 333 597L335 597M921 606C931.74 601.345 940.237 616.225 944.561 624C958.447 648.965 956.159 677.875 954 705C955.588 701.756 956.312 698.539 957 695C958.745 699.167 957.346 703.621 957 708L959 704C975.865 728.956 988.441 762.668 969.279 790C963.93 797.629 957.13 802.831 948 805L957 793C947.018 802.975 929.827 818.15 914 812.397C905.951 809.472 900.534 800.908 893 797C895.286 798.896 897.205 800.644 899 803L889 802C895.464 805.407 902.335 807.277 907.907 812.326C918.473 821.901 920.015 837.003 923.378 850C925.037 856.412 929.682 862.266 928 869C929.942 866.44 929.965 864.192 930 861C943.757 868.312 935.084 880.729 930.979 891C929.567 894.534 929.403 898.403 928.211 902C926.742 906.437 924.466 910.521 923 915L921 915C920.438 913.199 920.176 911.878 920 910L918 910L919 917L918 918L916 918L914 911C913.1 914.041 913.008 916.831 913 920C909.17 917.888 909.049 915.161 909 911C907.51 913.78 907.089 915.848 907 919L904 919L904 910L902 910L902 916L899 916C899.032 904.324 900.992 892.681 901 881L907 881C898.036 877.281 891.755 869.755 885 863C875.963 853.963 868.435 844.448 860.91 834.17C858.766 831.241 855.919 828.944 853.789 825.996C847.858 817.791 846.292 808.921 847.093 799C847.946 788.443 854.834 779.245 862.004 772C878.028 755.808 898.999 745 922 745C918.932 743.522 916.398 743.079 913 743C914.962 740.029 917.684 739.135 921 738C915.824 737.711 912.71 740.967 908 742.529C898.888 745.552 890.641 747.454 882 752C878.558 740.357 881 725.133 881 713C886.115 713.557 891.056 712.279 896 711C891.36 710.586 886.712 711.961 882 712C888.185 708.688 897.008 706.571 904 706C902.466 704.977 902.598 705.195 902 704C909.115 699.145 915.289 693.933 920.486 687C924.257 681.97 926.289 676.123 932 673C930.246 672.415 931.005 672.332 929 673C930.022 670.317 931.277 668.306 933 666C925.877 671.243 922.23 679.333 916.674 686C906.655 698.023 892.203 707.35 876 706.87C866.772 706.597 858.883 701.438 851 697.417C846.478 695.111 841.085 695.907 837.433 691.775C835.707 689.823 835.169 687.316 834.125 685C831.806 679.853 828.122 668.814 834 665C832.224 660.088 831.423 656.134 837 654C839.615 656.877 841.139 657.637 845 657C844.241 655.489 839.639 646.009 844.889 646.921C846.787 647.251 848.113 649.702 849.258 651.019C853.115 655.452 857.51 658.731 856 665C864.994 664.975 872.961 662.322 882 664C878.315 656.352 885.953 650.33 889.686 644.17C891.849 640.601 891.393 635.907 892.927 632C895.929 624.351 900.941 619.296 907 614C899.519 617.56 893.735 623.437 890.313 631C887.28 637.704 886.452 643.897 882 650C880.448 646.301 881 641.983 881 638C881 630.1 880.161 621.724 882 614C891.081 615.68 900.434 613.276 908 607.956C911.303 605.634 916.463 598.271 920.681 598.346C924.459 598.413 921.653 604.451 921 606M931 597C931.58 600.76 929.353 602.556 927 599C928.416 599.472 928.814 599.814 930 601C930.174 599.315 930.317 598.506 931 597M986 599C987.58 598.317 987.777 598.195 989 597C987.42 597.684 987.223 597.805 986 599M328 599C326.532 601.306 325.567 602.051 323 603C324.624 600.956 325.643 600.104 328 599M990 599C986.401 603.265 984.583 607.534 980 611C981.243 614.46 979.666 615.76 977 618C976.817 622.088 974.871 623.208 971 624C973.091 628.498 970.149 629.464 966 629C967.488 634.42 964.401 637.293 959 637L957 642C959.364 641.518 959.518 641.364 960 639C965.19 638.619 969.132 636.363 970 631C978.055 628.565 981.133 617.234 984.826 610.664C986.916 606.945 990.3 603.475 990 599M993.667 599.333C993.222 599.778 994.278 599.722 994.333 599.667C994.778 599.222 993.722 599.278 993.667 599.333M238 600C239.248 600.685 239.549 600.749 241 601C239.752 600.315 239.452 600.251 238 600M264 600C265.027 604.381 267.067 608.667 271 611C270.084 616.488 273.908 622.868 279 625C278.641 629.171 280.136 630.639 284 632C282.778 630.267 281.991 629.849 280 629C280.955 627.233 281.566 626.393 283 625C279.601 623.025 277.191 622.245 277 618C274.329 615.65 273.464 614.543 274 611C271.269 609.419 265.744 605.222 268 602L264 600M1008 601C1010.52 601.99 1012.36 601.617 1015 601C1012.54 600.112 1010.59 600.533 1008 601M242 601C243.248 601.685 243.549 601.749 245 602C243.751 601.315 243.452 601.251 242 601M273 602C274.48 605.166 276.168 606.951 279 609C277.255 606.223 275.543 604.065 273 602M981 602C979.242 604.012 978.084 605.581 977 608C979.279 606.209 980.831 604.937 981 602M258 606C259.769 606.779 261.036 606.912 263 607C261.231 606.221 259.964 606.088 258 606M278 611C279.33 612.753 280.247 613.67 282 615C280.846 612.751 280.249 612.154 278 611M356.667 615.333C356.222 615.778 357.278 615.722 357.333 615.667C357.778 615.222 356.722 615.278 356.667 615.333M882 616C884.248 617.799 886.163 618.493 889 619C886.794 617.183 884.802 616.606 882 616M242 617C244.89 618.213 247.874 617.998 251 618C248.11 616.787 245.126 617.002 242 617M282 617C283.402 619.274 284.515 619.987 287 621C285.435 619.119 284.201 618.058 282 617M1003 617C1005.05 617.874 1006.75 617.953 1009 618C1006.95 617.126 1005.25 617.047 1003 617M380 620L381 621L380 620M404 621L405 622L404 621M287 629C284.614 633.667 285.582 636.568 291 637C292.783 642.86 302.685 645.987 306 640C304.077 640.902 304.001 641.076 303 643L301 643L300 638L293 640L294 634C288.613 635.9 288.316 633.574 287 629M958 630L959 631L958 630M298.667 631.333C298.222 631.778 299.278 631.722 299.333 631.667C299.778 631.222 298.722 631.278 298.667 631.333M954.667 632.333C954.222 632.778 955.278 632.722 955.333 632.667C955.778 632.222 954.722 632.278 954.667 632.333M306.333 633.667C306.278 633.722 306.222 634.778 306.667 634.333C306.722 634.278 306.778 633.222 306.333 633.667M212 638C214.977 644.499 220.688 649.56 225.246 655C232.643 663.829 239.852 675.003 249 682C243.763 672.897 235.954 665.048 229.247 657C223.887 650.568 218.764 642.979 212 638M257.667 639.333C257.222 639.778 258.278 639.722 258.333 639.667C258.778 639.222 257.722 639.278 257.667 639.333M257 646C259.948 646.518 262.297 645.254 265 644C262.094 643.513 259.724 644.893 257 646M988 643C990.162 645.171 992.059 646.129 995 647C992.813 644.968 990.845 643.906 988 643M1036 644C1029.26 651.321 1023.28 659.349 1016.92 667C1011.81 673.141 1006 679.069 1002 686C1012.28 678.604 1020.15 665.72 1028.25 656C1031.02 652.668 1036.11 648.487 1036 644M452 645L453 646L452 645M271 648L268 650C269.926 650.338 270.709 649.976 271 648M844 648L845 649L844 648M273 655C271.029 656.941 269.355 658.609 268 661C270.408 659.263 272.302 657.905 273 655M979 655C980.324 657.687 981.313 658.676 984 660C982.451 657.864 981.136 656.549 979 655M334 664C334.545 665.635 334.365 665.455 336 666C335.455 664.365 335.635 664.545 334 664M283 670C284.457 668.897 284.897 668.457 286 667C283.876 667.899 283.899 667.876 283 670M967 667C967.684 668.58 967.805 668.777 969 670C968.317 668.42 968.195 668.223 967 667M182 691C191.867 705.007 206.768 717.045 218.985 729.039C224.954 734.9 230.352 743.359 238 747C230.436 736.262 219.282 727.282 210 718C201.035 709.035 192.371 698.306 182 691M385 692C386.45 693.608 387.051 694.064 389 695C387.607 693.566 386.767 692.955 385 692M890 694C888.257 695.111 887.881 695.27 887 697C889.108 696.415 890.801 696.153 890 694M1012 753C1022.74 745.436 1031.72 734.283 1041 725C1050.28 715.717 1061.44 706.738 1069 696C1061.9 699.309 1056.48 706.516 1051 712L1025 738C1020.38 742.623 1014.79 747.017 1012 753M415 709L415 781C410.75 776.227 407.458 770.643 402.91 766.089C398.044 761.217 392.219 757.451 387 753C390.066 747 392.429 740.822 392.791 734C392.969 730.635 391.877 727.37 392.199 724C392.688 718.887 394.773 714.203 395 709L415 709M372 734C373.395 736.038 374.349 737.627 375 740C378.824 738.761 380.392 735.558 382 732C378.428 733.367 375.867 734.44 372 734M884 736C884.547 737.875 885.11 739.255 886 741C887.081 739.542 887.436 738.752 888 737C886.359 736.869 885.544 736.631 884 736M212 766C217.476 773.774 225.249 779.429 232 786.015C241.574 795.354 251.065 806.297 262 814C253.436 801.842 240.692 792.184 230 781.961C224.353 776.561 219.158 769.338 212 766M1024 782C1015.4 790.422 1002.14 798.985 997 810C1004.06 805.026 1009.9 798.1 1016 792C1018.92 789.075 1023.01 786.077 1024 782M892 796L893 797L892 796M339 818C335.041 819.661 330.263 819 326 819C329.96 820.171 333.887 820 338 820C335.096 823.142 332.21 823.804 328 824C330.729 824.81 333.15 824.979 336 825C333.333 832.153 332.524 841.922 328 848C320.196 837.018 312.604 824.845 312 811C321.186 813.177 329.503 816.793 339 818M943 811C942.768 822.165 936.512 840.629 926 846C925.418 838.873 923.171 831.79 921 825L927 825C923.973 823.998 921.757 823.725 920 821C923.735 820.969 927.305 820.467 931 820C926.739 818.741 922.423 819 918 819C921.755 817.196 925.992 817.146 930 816.076C934.508 814.873 938.701 812.764 943 811M415 820L415 835C415 855.116 419.144 889.931 392 894C392.455 883.571 395.05 873.582 394.999 863C394.97 856.943 392.563 849.837 394.854 844C398.284 835.262 408.205 826.205 415 820M390 849C391.21 864.328 390.763 878.883 387.489 894C385.503 903.17 380.467 914.608 381.086 924C381.359 928.146 383.781 931.988 384.761 936C385.918 940.738 385.825 946.412 387.596 950.911C388.921 954.277 391.782 956.485 393 960L388 956C390.663 959.493 393.006 962.525 393 967C390.63 964.185 388.935 961.301 386 959C388.672 962.576 390.632 965.471 390 970L384 962C385.059 964.12 388.567 970.402 383.863 970.963C380.344 971.383 376.761 965.662 373.971 963.92C367.544 959.906 366.025 953.664 361.471 948.04C356.828 942.307 346.341 939.002 347.484 930.002C347.792 927.578 349.088 925.238 350 923L352 923L355 927C354.132 924.645 353.426 922.469 353 920C359.898 916.302 358.561 911.794 358.209 905C358.052 901.973 358.803 899.037 358.536 896C358.136 891.441 356.271 885.527 357.333 881.039C358.382 876.605 363.745 873.236 367 870.424C374.885 863.611 381.485 854.998 390 849M869 851C875.1 857.229 880.604 863.953 887 869.911C890.297 872.982 895.616 875.978 897.396 880.286C898.845 883.792 897.33 888.437 896.754 892C895.916 897.177 896.66 901.852 896.545 907C896.479 909.947 895.456 913.104 897.027 915.855C898.318 918.116 900.762 918.981 903 920C902.261 922.567 901.318 924.673 900 927C901.667 925.553 903.119 924.13 905 923C906.028 926.887 907.776 930.989 906.257 934.999C904.169 940.511 898.273 942.615 894.326 946.468C890.653 950.052 888.877 954.839 885.896 958.906C883.556 962.098 880.256 963.087 877.375 965.545C874.334 968.139 872.925 970.634 869 972C867.812 967.817 868.479 965.507 871 962C868.061 964.27 866.242 966.211 866 970C862.51 966.898 865.63 962.97 868 960L861 967C861.626 962.32 863.869 959.443 867 956L861 960C862.557 956.656 865.358 954.272 866.973 951C868.592 947.72 868.053 943.535 868.899 940C870.412 933.678 874.39 927.624 873.946 921C872.772 903.503 867 886.659 867 869C867 863.131 866.643 856.463 869 851M260 905C266.242 913.156 277.852 922.33 287 927C281.094 918.976 269.105 909.058 260 905M970 924C976.552 920.619 983.522 914.854 988 909C981.478 912.024 974.261 918.212 970 924M602 913L602 921C603.161 918.23 603.161 915.77 602 913M540 914C542.331 929.782 549.675 945.712 554.28 961C563.191 990.581 571.568 1020.28 580 1050C581.383 1046.1 579.401 1041.87 578.291 1038C576.193 1030.69 574.391 1023.31 572.291 1016C565.463 992.237 558.36 968.605 551.026 945C548.121 935.652 546.235 921.581 540 914M603 922C603 960.102 611.685 997.941 612 1036C615.369 1027.97 611.424 1016.48 611.039 1008C610.176 988.975 607.982 969.96 606.17 951C605.311 942.008 606.515 930.357 603 922M355 927L356 928L355 927M662 927C659.894 953.645 653.628 980.53 649.728 1007C648.316 1016.59 646.2 1026.29 646 1036C649.528 1028.66 649.334 1018.98 650.59 1011C653.475 992.675 656.071 974.304 659.08 956C660.378 948.108 665.053 934.484 662 927M899 927L900 928L899 927M719 934C709.076 959.585 699.373 985.227 689.947 1011C685.414 1023.39 678.929 1036.94 677 1050C682.092 1043.19 683.805 1032.93 686.808 1025C694.486 1004.73 701.408 984.158 709.399 964C712.021 957.385 714.459 950.667 716.95 944C718.174 940.724 719.951 937.429 719 934M787 935C765.789 966.442 745.315 998.477 725.8 1031C720.028 1040.62 714.266 1050.25 708.719 1060C705.591 1065.5 701.793 1070.9 700 1077C703.394 1073.64 705.36 1069.1 707.799 1065L721.4 1042C735.732 1018.11 750.233 994.195 765.667 971C771.181 962.713 776.479 954.286 782 946C784.196 942.704 787.592 939.078 787 935M475 941C483.417 959.006 495.591 975.796 505.576 993C520.888 1019.38 534.642 1046.69 550 1073C550.667 1068.99 547.763 1065.43 545.861 1062C541.854 1054.77 538.012 1047.36 534.245 1040C522.38 1016.82 508.793 994.323 495.4 972C489.483 962.139 483.906 948.379 475 941M420 964C427.108 974.395 436.528 983.548 444.845 993C458.647 1008.68 472.075 1024.74 485.197 1041C494.797 1052.89 504.192 1064.94 513.576 1077C518.487 1083.31 522.777 1090.93 529 1096C526.655 1090.45 522.474 1085.8 518.873 1081C511.792 1071.56 504.511 1062.27 497.211 1053C481.395 1032.91 464.912 1013.41 448.271 994C439.676 983.976 430.822 971.624 420 964M836 965C806.799 994.201 779.362 1025.56 752.579 1057C744.717 1066.23 736.971 1075.53 729.4 1085C725.366 1090.04 720.637 1095.08 718 1101C728.25 1093.14 735.712 1079.83 744.131 1070C763.755 1047.08 783.32 1023.94 804.036 1002C811.479 994.118 818.452 985.765 826.015 978C829.595 974.324 835.051 970.23 836 965M366 997C371.732 1003.94 380.809 1008.96 388 1014.37C401.78 1024.75 415.482 1035.28 429 1046C448.696 1061.62 468.131 1077.55 487 1094.16C497.839 1103.7 508.153 1114.75 520 1123C512.646 1112.56 501.354 1104.52 492 1095.91C468.417 1074.23 443.26 1054.26 418 1034.58C407.483 1026.38 396.68 1018.6 386 1010.63C379.7 1005.92 373.26 1000.07 366 997M888 997L889 998L888 997M884 1000C846.598 1026.79 810.016 1054.87 775 1084.73C764.171 1093.96 753.42 1103.22 743 1112.91C737.425 1118.09 731.419 1122.73 727 1129C741.344 1119.19 753.843 1105.67 767 1094.28C791.237 1073.31 816.525 1053.32 842 1033.88C852.313 1026.02 862.724 1018.27 873 1010.35C876.499 1007.65 882.806 1004.35 884 1000M645 1038L645 1043C645.83 1040.97 645.83 1040.03 645 1038M581 1051L581 1054C581.696 1052.45 581.696 1052.55 581 1051M676.333 1051.67C676.278 1051.72 676.222 1052.78 676.667 1052.33C676.722 1052.28 676.778 1051.22 676.333 1051.67M675.333 1054.67C675.278 1054.72 675.222 1055.78 675.667 1055.33C675.722 1055.28 675.778 1054.22 675.333 1054.67M582 1055L582 1058C582.696 1056.45 582.696 1056.55 582 1055M674.333 1057.67C674.278 1057.72 674.222 1058.78 674.667 1058.33C674.722 1058.28 674.778 1057.22 674.333 1057.67M583 1059L583 1062C583.696 1060.45 583.696 1060.55 583 1059M673.333 1060.67C673.278 1060.72 673.222 1061.78 673.667 1061.33C673.722 1061.28 673.778 1060.22 673.333 1060.67M672.333 1063.67C672.278 1063.72 672.222 1064.78 672.667 1064.33C672.722 1064.28 672.778 1063.22 672.333 1063.67M629 1191C631.607 1184.94 632.662 1178.34 634.424 1172C638.852 1156.07 642.864 1140.04 646.873 1124C648.485 1117.55 648.49 1103.86 654.147 1099.74C665.05 1091.81 678.602 1105.85 674.895 1117C672.823 1123.23 669.178 1129.14 666.247 1135C656.035 1155.42 643.065 1174.13 632 1194C646.015 1177.56 656.484 1156.91 666.769 1138C669.48 1133.02 672.237 1128.1 674.742 1123C676.06 1120.32 677.162 1117 680.133 1115.75C686.542 1113.04 693.871 1118.71 694.782 1125C695.474 1129.77 693.415 1134.08 690.91 1138C676.975 1159.78 655.779 1176.72 637 1194C654.954 1182.78 669.832 1165.64 683.83 1150C687.513 1145.89 690.642 1141.38 694.015 1137.02C695.511 1135.09 697.285 1132.69 700.014 1132.75C701.849 1132.79 703.521 1133.99 704.772 1135.23C712.045 1142.44 704.636 1151.6 698.996 1156.91C690.273 1165.12 680.007 1171.83 670 1178.34C659.449 1185.21 648.258 1192.65 636 1196L637 1194L631 1197L632 1194C630.814 1195.19 630.472 1195.58 630 1197C628.686 1196.23 628.768 1196.31 628 1195C620.271 1200.34 611.659 1190.8 606 1186.08C593.18 1175.38 581.129 1162.96 570.611 1150C566.121 1144.47 560.194 1138.13 558.533 1131C556.73 1123.26 564.8 1110.7 573.725 1116.18C578.881 1119.34 581.6 1130.71 584.247 1136C587.644 1142.79 591.388 1149.41 595.15 1156C601.386 1166.93 607.82 1178.4 616 1188C603.529 1165.6 586.949 1143.71 579.093 1119C575.369 1107.29 584.302 1095.43 596.995 1098.39C604.979 1100.26 604.132 1113.29 605.389 1120C608.097 1134.45 611.309 1148.8 615.151 1163C617.475 1171.59 620.81 1180.17 622 1189C623.047 1182.49 619.839 1175.37 618.424 1169C614.947 1153.34 611.328 1137.74 608.2 1122C606.828 1115.1 604.111 1105.99 605.544 1099C609.833 1078.08 644.29 1075.13 647.787 1098C649.318 1108.01 645.773 1119.35 643.373 1129C639.899 1142.97 636.568 1156.97 633.349 1171C631.827 1177.63 629.555 1184.2 629 1191M417 1086C418.953 1087.72 420.463 1088.34 423 1089C421.051 1087.43 419.412 1086.71 417 1086M827 1091C829.274 1091.41 830.781 1090.79 833 1090C830.726 1089.59 829.219 1090.21 827 1091M616 1195C610.426 1195.93 603.76 1190.36 599 1187.77C582.969 1179.05 566.102 1168.81 553.015 1155.96C547.765 1150.81 539.197 1136.96 551.039 1132.92C557.64 1130.66 563.87 1144.89 567.286 1149C581.527 1166.13 598.107 1181.74 616 1195M755 1152C752.895 1155.03 750.163 1157.72 748.442 1161C745.999 1165.66 745.607 1170.64 741.671 1174.53C731.253 1184.82 717.756 1180.96 705 1180.09C699.911 1179.74 695.025 1180.36 690 1181C699.104 1173.35 709.248 1170.24 721 1170C717.959 1169.1 715.169 1169.01 712 1169C723.108 1159.8 741.077 1154.88 755 1152M506 1153C519.081 1157.26 531.332 1162.73 544 1168C541.948 1168.87 540.253 1168.95 538 1169L538 1171C548.94 1171.01 555.375 1175.79 565 1180C560.752 1181.31 555.529 1179.78 551 1180.09C538.246 1180.96 524.722 1184.9 514.213 1174.67C509.32 1169.91 506.867 1161.14 504 1155C507.736 1156.86 510.847 1159.27 514 1162C511.462 1158.69 508.376 1156.43 505 1154L506 1153M749 1155C746.227 1157.12 744.037 1159.16 742 1162C745.038 1160.24 748.124 1158.51 749 1155M514 1162C515.33 1163.75 516.247 1164.67 518 1166C516.67 1164.25 515.753 1163.33 514 1162M616 1188C617.432 1191.38 619.159 1193.68 622 1196C620.536 1192.72 618.702 1190.37 616 1188M623 1189L623 1192C623.696 1190.45 623.696 1190.55 623 1189M628.333 1192.67C628.278 1192.72 628.222 1193.78 628.667 1193.33C628.722 1193.28 628.778 1192.22 628.333 1192.67z"/>
      </svg>
    </div>
    <!-- /MONOGRAM SVG PLACEHOLDER -->
  </footer>
  
  <div id="toast"></div>
  
  <script>
  // ─── STATE ────────────────────────────────────────────────────────────────────
  let hue = 0, sat = 1, val = 1;
  let pickerW = 0, pickerH = 0;
  
  // ─── UTILS ────────────────────────────────────────────────────────────────────
  function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
  
  function hsvToRgb(h, s, v) {
    const i = Math.floor(h / 60) % 6;
    const f = h / 60 - Math.floor(h / 60);
    const p = v * (1 - s), q = v * (1 - f * s), t = v * (1 - (1 - f) * s);
    const m = [[v,t,p],[q,v,p],[p,v,t],[p,q,v],[t,p,v],[v,p,q]][i];
    return { r: Math.round(m[0]*255), g: Math.round(m[1]*255), b: Math.round(m[2]*255) };
  }
  
  function rgbToHsv(r, g, b) {
    r /= 255; g /= 255; b /= 255;
    const max = Math.max(r,g,b), min = Math.min(r,g,b), d = max - min;
    let h = 0, s = max === 0 ? 0 : d / max, v = max;
    if (d !== 0) {
      if (max === r) h = ((g - b) / d % 6) * 60;
      else if (max === g) h = ((b - r) / d + 2) * 60;
      else h = ((r - g) / d + 4) * 60;
    }
    return { h: (h + 360) % 360, s, v };
  }
  
  function toHex2(n) { return n.toString(16).padStart(2,'0').toUpperCase(); }
  function rgbToHex(r,g,b) { return '#' + toHex2(r) + toHex2(g) + toHex2(b); }
  
  function hexToRgb(hex) {
    const m = hex.replace('#','').match(/.{2}/g);
    if (!m || m.length < 3) return null;
    const r = parseInt(m[0],16), g = parseInt(m[1],16), b = parseInt(m[2],16);
    if (isNaN(r)||isNaN(g)||isNaN(b)) return null;
    return { r, g, b };
  }
  
  // ─── PICKER CANVAS ────────────────────────────────────────────────────────────
  const canvas = document.getElementById('pickerCanvas');
  const ctx = canvas.getContext('2d');
  const wrap = document.getElementById('pickerWrap');
  const cursor = document.getElementById('pickerCursor');
  
  function drawPicker() {
    const w = canvas.width, h = canvas.height;
    // SV square for current hue
    const hsv = ctx.createLinearGradient(0, 0, w, 0);
    hsv.addColorStop(0, '#fff');
    hsv.addColorStop(1, `hsl(${hue},100%,50%)`);
    ctx.fillStyle = hsv;
    ctx.fillRect(0, 0, w, h);
    const dark = ctx.createLinearGradient(0, 0, 0, h);
    dark.addColorStop(0, 'rgba(0,0,0,0)');
    dark.addColorStop(1, '#000');
    ctx.fillStyle = dark;
    ctx.fillRect(0, 0, w, h);
  }
  
  function resizePicker() {
    const r = wrap.getBoundingClientRect();
    pickerW = r.width; pickerH = r.height;
    canvas.width = Math.round(pickerW * window.devicePixelRatio || 1);
    canvas.height = Math.round(pickerH * window.devicePixelRatio || 1);
    ctx.scale(window.devicePixelRatio || 1, window.devicePixelRatio || 1);
    drawPicker();
    moveCursorToSV();
  }
  
  function moveCursorToSV() {
    const x = sat * pickerW;
    const y = (1 - val) * pickerH;
    cursor.style.left = x + 'px';
    cursor.style.top  = y + 'px';
  }
  
  function pickerPointer(e) {
    const r = wrap.getBoundingClientRect();
    const cx = (e.touches ? e.touches[0].clientX : e.clientX) - r.left;
    const cy = (e.touches ? e.touches[0].clientY : e.clientY) - r.top;
    sat = clamp(cx / r.width, 0, 1);
    val = clamp(1 - cy / r.height, 0, 1);
    moveCursorToSV();
    drawPicker();
    syncFromHSV();
  }
  
  wrap.addEventListener('mousedown',  e => { e.preventDefault(); pickerPointer(e); document.addEventListener('mousemove', pickerPointer); document.addEventListener('mouseup', () => document.removeEventListener('mousemove', pickerPointer), {once:true}); });
  wrap.addEventListener('touchstart', e => { e.preventDefault(); pickerPointer(e); }, {passive:false});
  wrap.addEventListener('touchmove',  e => { e.preventDefault(); pickerPointer(e); }, {passive:false});
  
  // ─── HUE SLIDER ──────────────────────────────────────────────────────────────
  const hueTrack = document.getElementById('hueTrack');
  const hueThumb = document.getElementById('hueThumb');
  
  function moveHueThumb() {
    hueThumb.style.left = (hue / 360 * 100) + '%';
  }
  
  function huePointer(e) {
    const r = hueTrack.getBoundingClientRect();
    const cx = (e.touches ? e.touches[0].clientX : e.clientX) - r.left;
    hue = clamp(cx / r.width, 0, 1) * 360;
    moveHueThumb();
    drawPicker();
    syncFromHSV();
  }
  
  hueTrack.addEventListener('mousedown',  e => { e.preventDefault(); huePointer(e); document.addEventListener('mousemove', huePointer); document.addEventListener('mouseup', () => document.removeEventListener('mousemove', huePointer), {once:true}); });
  hueTrack.addEventListener('touchstart', e => { e.preventDefault(); huePointer(e); }, {passive:false});
  hueTrack.addEventListener('touchmove',  e => { e.preventDefault(); huePointer(e); }, {passive:false});
  
  // ─── SYNC ─────────────────────────────────────────────────────────────────────
  function syncFromHSV() {
    const {r, g, b} = hsvToRgb(hue, sat, val);
    updateUI(r, g, b, false);
  }
  
  function syncFromRGB() {
    const r = clamp(parseInt(document.getElementById('rIn').value) || 0, 0, 255);
    const g = clamp(parseInt(document.getElementById('gIn').value) || 0, 0, 255);
    const b = clamp(parseInt(document.getElementById('bIn').value) || 0, 0, 255);
    const hsv = rgbToHsv(r, g, b);
    hue = hsv.h; sat = hsv.s; val = hsv.v;
    moveHueThumb(); drawPicker(); moveCursorToSV();
    updateUI(r, g, b, false);
  }
  
  function syncFromHex() {
    const rgb = hexToRgb(document.getElementById('hexIn').value);
    if (!rgb) return;
    document.getElementById('rIn').value = rgb.r;
    document.getElementById('gIn').value = rgb.g;
    document.getElementById('bIn').value = rgb.b;
    syncFromRGB();
  }
  
  function updateUI(r, g, b, updateInputs = true) {
    const hex = rgbToHex(r, g, b);
    // Hero
    document.getElementById('heroHex').textContent = hex;
    document.documentElement.style.setProperty('--r', r);
    document.documentElement.style.setProperty('--g', g);
    document.documentElement.style.setProperty('--b', b);
    const heroSwatch = document.getElementById('heroSwatch');
    const heroBg     = document.getElementById('heroBg');
    const heroGlow   = document.getElementById('heroGlow');
    heroSwatch.style.background = `rgb(${r},${g},${b})`;
    heroSwatch.style.boxShadow  = `0 0 20px rgba(${r},${g},${b},0.45)`;
    heroBg.style.background     = `rgb(${r},${g},${b})`;
    heroGlow.style.background   = `radial-gradient(ellipse at 50% 110%, rgba(${r},${g},${b},0.5) 0%, transparent 70%)`;
  
    if (updateInputs) {
      document.getElementById('rIn').value = r;
      document.getElementById('gIn').value = g;
      document.getElementById('bIn').value = b;
      document.getElementById('hexIn').value = hex;
    } else {
      document.getElementById('hexIn').value = hex;
    }
  }
  
  // ─── INPUT EVENTS ─────────────────────────────────────────────────────────────
  ['rIn','gIn','bIn'].forEach(id => {
    document.getElementById(id).addEventListener('input', syncFromRGB);
  });
  document.getElementById('hexIn').addEventListener('input', function() {
    if (this.value.length === 7) syncFromHex();
  });
  
  // ─── APPLY COLOR ──────────────────────────────────────────────────────────────
  document.getElementById('applyColor').addEventListener('click', async () => {
    const r = clamp(parseInt(document.getElementById('rIn').value)||0,0,255);
    const g = clamp(parseInt(document.getElementById('gIn').value)||0,0,255);
    const b = clamp(parseInt(document.getElementById('bIn').value)||0,0,255);
    try {
      const res = await fetch('/api/color', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({r, g, b})
      });
      if (res.ok) { showToast('Colore applicato', 'ok'); setActiveMode('STATIC'); }
      else showToast('Errore', 'err');
    } catch { showToast('Connessione non disponibile', 'err'); setOffline(); }
  });
  
  // ─── MODES ────────────────────────────────────────────────────────────────────
  function setActiveMode(mode) {
    document.querySelectorAll('.mode-card').forEach(c => {
      c.classList.toggle('active', c.dataset.mode === mode);
    });
  }
  
  document.querySelectorAll('.mode-card').forEach(card => {
    card.addEventListener('click', async () => {
      const mode = card.dataset.mode;
      try {
        const res = await fetch('/api/mode', {
          method: 'POST',
          headers: {'Content-Type':'application/json'},
          body: JSON.stringify({mode})
        });
        if (res.ok) { setActiveMode(mode); showToast('Modalità attiva', 'ok'); }
        else showToast('Errore', 'err');
      } catch { showToast('Connessione non disponibile', 'err'); setOffline(); }
    });
  });
  
  // ─── ADVANCED SECTION ─────────────────────────────────────────────────────────
  document.getElementById('advToggle').addEventListener('click', () => {
    const body = document.getElementById('advBody');
    const chev = document.getElementById('advChevron');
    body.classList.toggle('open');
    chev.classList.toggle('open');
  });
  
  // Sliders
  document.getElementById('speedSlider').addEventListener('input', function() {
    document.getElementById('speedVal').textContent = this.value;
  });
  document.getElementById('brightnessSlider').addEventListener('input', function() {
    document.getElementById('brightnessVal').textContent = this.value + '%';
  });
  
  // Toggle RPM Warning
  const rpmWarningSwitch = document.getElementById('rpmWarningSwitch');
  let rpmWarningEnabled = true; // default from backend
  rpmWarningSwitch.addEventListener('click', () => {
    rpmWarningEnabled = !rpmWarningEnabled;
    if (rpmWarningEnabled) rpmWarningSwitch.classList.add('active');
    else rpmWarningSwitch.classList.remove('active');
  });

  // Apply advanced params (includes toggle)
  document.getElementById('applyParams').addEventListener('click', async () => {
    const params = {
      speed:        clamp(parseInt(document.getElementById('speedSlider').value)||0, 0, 100),
      brightness:   clamp(parseInt(document.getElementById('brightnessSlider').value)||0, 0, 100),
      rpmThreshold: clamp(parseInt(document.getElementById('rpmThreshold').value)||0, 0, 15000),
      rpmWarningEnabled: rpmWarningEnabled
    };
    try {
      const res = await fetch('/api/params', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify(params)
      });
      if (res.ok) showToast('Impostazioni salvate', 'ok');
      else showToast('Errore', 'err');
    } catch { showToast('Connessione non disponibile', 'err'); setOffline(); }
  });

  // Aggiornamento pulsante download logs
  const downloadBtn = document.getElementById('downloadLogsBtn');
  const logsMsg = document.getElementById('logsDisabledMsg');
  function updateDownloadButton(recording) {
    if (recording) {
      downloadBtn.disabled = true;
      downloadBtn.style.opacity = '0.5';
      downloadBtn.style.cursor = 'not-allowed';
      logsMsg.style.display = 'block';
    } else {
      downloadBtn.disabled = false;
      downloadBtn.style.opacity = '1';
      downloadBtn.style.cursor = 'pointer';
      logsMsg.style.display = 'none';
    }
  }
  
  // ─── TOAST ────────────────────────────────────────────────────────────────────
  let toastTimer;
  function showToast(msg, type) {
    const t = document.getElementById('toast');
    t.textContent = msg;
    t.className = 'show' + (type ? ' ' + type : '');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { t.className = ''; }, 2400);
  }
  
  // ─── CONNECTION STATE ─────────────────────────────────────────────────────────
  function setOffline() {
    document.getElementById('connDot').classList.add('error');
    document.getElementById('connLabel').textContent = 'Offline';
  }
  function setOnline() {
    document.getElementById('connDot').classList.remove('error');
    document.getElementById('connLabel').textContent = 'Online';
  }
  
  // ─── LOAD STATUS ──────────────────────────────────────────────────────────────
  async function loadStatus() {
    try {
      const res = await fetch('/api/status');
      if (!res.ok) { setOffline(); return; }
      const d = await res.json();
      const r = clamp(d.r || 0, 0, 255);
      const g = clamp(d.g || 0, 0, 255);
      const b = clamp(d.b || 0, 0, 255);
      document.getElementById('rIn').value = r;
      document.getElementById('gIn').value = g;
      document.getElementById('bIn').value = b;
      if (d.speed   !== undefined) { document.getElementById('speedSlider').value = d.speed; document.getElementById('speedVal').textContent = d.speed; }
      if (d.brightness !== undefined) { document.getElementById('brightnessSlider').value = d.brightness; document.getElementById('brightnessVal').textContent = d.brightness + '%'; }
      if (d.rpmThreshold !== undefined) document.getElementById('rpmThreshold').value = d.rpmThreshold;
      if (d.rpmWarningEnabled !== undefined) {
        rpmWarningEnabled = d.rpmWarningEnabled;
        if (rpmWarningEnabled) rpmWarningSwitch.classList.add('active');
        else rpmWarningSwitch.classList.remove('active');
      }
      syncFromRGB();
      if (d.mode) setActiveMode(d.mode);
      setOnline();
      updateDownloadButton(d.recordingActive)
    } catch {
      showToast('Connessione non disponibile', 'err');
      setOffline();
    }
  }
  
  // ─── INIT ─────────────────────────────────────────────────────────────────────
  window.addEventListener('resize', resizePicker);
  resizePicker();
  moveHueThumb();
  syncFromHSV();
  loadStatus();
  </script>
  </body>
  </html>
  
)rawhtml";

// ---------------------------------------------------------------------------
// Handler: GET /
// ---------------------------------------------------------------------------
static void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", UI_HTML);
}

// ---------------------------------------------------------------------------
// Handler: GET /api/status
// ---------------------------------------------------------------------------
static void handleGetStatus() {
    RgbParams p = rgbGetParams();
    RgbMode m = rgbGetMode();

    JsonDocument doc;
    doc["mode"] = RgbModeNames[m];
    doc["r"] = p.r;
    doc["g"] = p.g;
    doc["b"] = p.b;
    doc["speed"] = p.speed;
    doc["brightness"] = p.brightness;
    doc["rpmThreshold"] = p.rpmThreshold;
    // doc["rpmMax"] = p.rpmMax; // mantenuto per retrocompatibilità
    doc["rpmWarningEnabled"] = p.rpmWarningEnabled;
    doc["recordingActive"] = recordingActive;

    String out;
    serializeJson(doc, out);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
}

// ---------------------------------------------------------------------------
// Handler: POST /api/color  →  {"r":255,"g":0,"b":128}
// ---------------------------------------------------------------------------
static void handlePostColor(){
    if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"no body\"}"); return; }

    JsonDocument doc;
    if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
        server.send(400, "application/json", "{\"error\":\"invalid json\"}");
        return;
    }

    uint8_t r = constrain((int)doc["r"] | 0, 0, 255);
    uint8_t g = constrain((int)doc["g"] | 0, 0, 255);
    uint8_t b = constrain((int)doc["b"] | 0, 0, 255);
    rgbSetColor(r, g, b);
    
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------------------------------------------------------------------
// Handler: POST /api/mode  →  {"mode":"FADING"}
// ---------------------------------------------------------------------------
static void handlePostMode() {
    if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"no body\"}"); return; }
  
    JsonDocument doc;
    if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
      server.send(400, "application/json", "{\"error\":\"invalid json\"}"); return;
    }
  
    const char* modeStr = doc["mode"] | "";
    RgbMode newMode = RGB_STATIC; // fallback sicuro
    bool found = false;
    for (int i = 0; i < RGB_MODE_COUNT; i++) {
      if (strcmp(modeStr, RgbModeNames[i]) == 0) {
        newMode = (RgbMode)i;
        found = true;
        break;
      }
    }
    // Se la modalità non è riconosciuta, si usa RGB_STATIC senza errore
    // (così comandi obsoleti come "RPM_COLOR" o "RPM_WARNING" non bloccano)
    if (!found) {
      Serial.printf("[WebServer] Modalità sconosciuta: %s, fallback a STATIC\n", modeStr);
    }
  
    rgbSetMode(newMode);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------------------------------------------------------------------
// Handler: POST /api/params
// ---------------------------------------------------------------------------
static void handlePostParams(){
    if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"no body\"}"); return; }

    JsonDocument doc;
    if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
        server.send(400, "application/json", "{\"error\":\"invalid json\"}"); return;
    }

    RgbParams p = rgbGetParams();  // parti dallo stato corrente
    if (doc["speed"].is<int>())        p.speed        = constrain((int)doc["speed"],        0, 100);
    if (doc["brightness"].is<int>())   p.brightness   = constrain((int)doc["brightness"],   0, 100);
    if (doc["rpmThreshold"].is<int>()) p.rpmThreshold = constrain((int)doc["rpmThreshold"], 0, 15000);
    // if (doc["rpmMax"].is<int>())       p.rpmMax       = constrain((int)doc["rpmMax"],       1, 15000);
    if (doc["rpmWarningEnabled"].is<bool>()) p.rpmWarningEnabled = doc["rpmWarningEnabled"];
    rgbSetParams(p);

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
}

static const char LOGS_HTML[] PROGMEM = R"rawhtml(
  <!DOCTYPE html>
  <html lang="it">
  <head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>TRACE LOGS</title>
  <style>
  :root {
    --bg:       #0a0a0c;
    --bg2:      #111115;
    --bg3:      #18181e;
    --bg4:      #1f1f28;
    --border:   rgba(255,255,255,0.07);
    --border2:  rgba(255,255,255,0.12);
    --accent:   #c8aa7a;
    --accent2:  #e8cfa0;
    --accent-d: #8a7050;
    --text:     #e8e4dc;
    --text2:    #9a9490;
    --text3:    #504e4a;
    --ok:       #7ac8a0;
    --err:      #c87a7a;
  }

  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  html { scroll-behavior: smooth; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Trebuchet MS', 'Gill Sans', Optima, Candara, sans-serif;
    font-size: 15px;
    min-height: 100vh;
    -webkit-tap-highlight-color: transparent;
    overflow-x: hidden;
  }

  /* subtle grain */
  body::after {
    content: '';
    position: fixed;
    inset: 0;
    background-image: url("data:image/svg+xml,%3Csvg viewBox='0 0 200 200' xmlns='http://www.w3.org/2000/svg'%3E%3Cfilter id='n'%3E%3CfeTurbulence type='fractalNoise' baseFrequency='0.9' numOctaves='4' stitchTiles='stitch'/%3E%3C/filter%3E%3Crect width='100%25' height='100%25' filter='url(%23n)' opacity='0.035'/%3E%3C/svg%3E");
    pointer-events: none;
    z-index: 9998;
    opacity: 0.4;
  }

  /* ── HEADER ── */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 18px 20px 16px;
    border-bottom: 1px solid var(--border);
    position: sticky;
    top: 0;
    z-index: 200;
    background: rgba(10,10,12,0.92);
    backdrop-filter: blur(12px);
    -webkit-backdrop-filter: blur(12px);
  }

  .logo-group { display: flex; flex-direction: column; gap: 1px; }

  .logo {
    font-size: 0.78rem;
    font-weight: 600;
    letter-spacing: 0.32em;
    color: var(--accent);
    text-transform: uppercase;
  }

  .logo-sub {
    font-size: 0.58rem;
    letter-spacing: 0.18em;
    color: var(--text3);
    text-transform: uppercase;
  }

  .conn-indicator {
    display: flex;
    align-items: center;
    gap: 7px;
  }

  .conn-dot {
    width: 6px; height: 6px;
    border-radius: 50%;
    background: var(--ok);
    box-shadow: 0 0 6px var(--ok);
    transition: background 0.4s, box-shadow 0.4s;
  }
  .conn-dot.error { background: var(--err); box-shadow: 0 0 6px var(--err); }

  .conn-label {
    font-size: 0.58rem;
    letter-spacing: 0.14em;
    color: var(--text3);
    text-transform: uppercase;
  }

  /* ── MAIN ── */
  main {
    max-width: 440px;
    margin: 0 auto;
    padding: 24px 16px 12px;
    display: flex;
    flex-direction: column;
    gap: 14px;
  }

  /* ── HERO / SUMMARY ── */
  .hero {
    position: relative;
    border-radius: 20px;
    overflow: hidden;
    min-height: 130px;
    display: flex;
    flex-direction: column;
    justify-content: flex-end;
    padding: 20px;
    border: 1px solid var(--border);
  }

  .hero-bg {
    position: absolute;
    inset: 0;
    background: var(--accent);
    opacity: 0.06;
  }

  .hero-glow {
    position: absolute;
    inset: 0;
    background: radial-gradient(ellipse at 50% 120%,
      rgba(200,170,122,0.18) 0%,
      transparent 70%);
    pointer-events: none;
  }

  .hero-inner {
    position: relative;
    display: flex;
    align-items: flex-end;
    justify-content: space-between;
  }

  .hero-label {
    font-size: 0.6rem;
    letter-spacing: 0.2em;
    color: var(--text2);
    text-transform: uppercase;
    margin-bottom: 6px;
  }

  .hero-count {
    font-size: 2.4rem;
    font-weight: 300;
    letter-spacing: 0.04em;
    color: var(--text);
    font-family: 'Courier New', 'Lucida Console', monospace;
    line-height: 1;
  }

  .hero-count-label {
    font-size: 0.6rem;
    letter-spacing: 0.14em;
    color: var(--text3);
    text-transform: uppercase;
    margin-top: 4px;
  }

  .hero-icon {
    width: 48px; height: 48px;
    border-radius: 50%;
    background: rgba(200,170,122,0.1);
    border: 1px solid rgba(200,170,122,0.2);
    display: flex;
    align-items: center;
    justify-content: center;
    flex-shrink: 0;
  }

  .hero-icon svg {
    width: 22px; height: 22px;
    stroke: var(--accent);
    fill: none;
    stroke-width: 1.5;
    stroke-linecap: round;
    stroke-linejoin: round;
  }

  /* ── CARD ── */
  .card {
    background: var(--bg2);
    border: 1px solid var(--border);
    border-radius: 16px;
    overflow: hidden;
  }

  .card-title {
    padding: 14px 18px 12px;
    font-size: 0.6rem;
    letter-spacing: 0.22em;
    color: var(--accent-d);
    text-transform: uppercase;
    border-bottom: 1px solid var(--border);
  }

  .card-body { padding: 18px; }

  /* ── RECORDING BANNER ── */
  .recording-banner {
    display: none;
    align-items: center;
    gap: 10px;
    background: rgba(200,122,122,0.08);
    border: 1px solid rgba(200,122,122,0.25);
    border-radius: 10px;
    padding: 12px 16px;
    margin-bottom: 14px;
  }

  .recording-banner.visible { display: flex; }

  .rec-dot {
    width: 7px; height: 7px;
    border-radius: 50%;
    background: var(--err);
    box-shadow: 0 0 6px var(--err);
    flex-shrink: 0;
    animation: rec-pulse 1.2s ease-in-out infinite;
  }

  @keyframes rec-pulse {
    0%,100% { opacity: 1; }
    50%      { opacity: 0.35; }
  }

  .rec-text {
    font-size: 0.62rem;
    letter-spacing: 0.1em;
    color: var(--err);
    text-transform: uppercase;
  }

  /* ── FILE LIST ── */
  .file-list {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }

  .file-item {
    display: flex;
    align-items: center;
    justify-content: space-between;
    background: var(--bg3);
    border: 1px solid var(--border);
    border-radius: 12px;
    padding: 14px 16px;
    gap: 12px;
    transition: border-color 0.2s;
  }

  .file-item:hover { border-color: var(--border2); }

  .file-item-icon {
    width: 34px; height: 34px;
    border-radius: 8px;
    background: rgba(200,170,122,0.08);
    border: 1px solid rgba(200,170,122,0.15);
    display: flex;
    align-items: center;
    justify-content: center;
    flex-shrink: 0;
  }

  .file-item-icon svg {
    width: 16px; height: 16px;
    stroke: var(--accent-d);
    fill: none;
    stroke-width: 1.5;
    stroke-linecap: round;
    stroke-linejoin: round;
  }

  .file-info {
    flex: 1;
    min-width: 0;
  }

  .file-name {
    font-size: 0.78rem;
    color: var(--text);
    letter-spacing: 0.02em;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    font-family: 'Courier New', monospace;
  }

  .file-size {
    font-size: 0.58rem;
    color: var(--text3);
    letter-spacing: 0.1em;
    margin-top: 3px;
    text-transform: uppercase;
  }

  .file-actions {
    display: flex;
    gap: 6px;
    flex-shrink: 0;
  }

  .btn-icon {
    width: 34px; height: 34px;
    border-radius: 8px;
    border: 1px solid var(--border2);
    background: var(--bg4);
    display: flex;
    align-items: center;
    justify-content: center;
    cursor: pointer;
    transition: background 0.2s, border-color 0.2s, transform 0.1s;
    -webkit-tap-highlight-color: transparent;
  }

  .btn-icon:active { transform: scale(0.93); }

  .btn-icon:hover { background: rgba(255,255,255,0.05); border-color: var(--accent-d); }

  .btn-icon svg {
    width: 15px; height: 15px;
    stroke: var(--text2);
    fill: none;
    stroke-width: 1.5;
    stroke-linecap: round;
    stroke-linejoin: round;
    transition: stroke 0.2s;
  }

  .btn-icon:hover svg { stroke: var(--text); }

  .btn-icon.delete:hover { background: rgba(200,122,122,0.1); border-color: rgba(200,122,122,0.3); }
  .btn-icon.delete:hover svg { stroke: var(--err); }

  /* ── EMPTY STATE ── */
  .empty-state {
    display: none;
    flex-direction: column;
    align-items: center;
    gap: 10px;
    padding: 32px 16px;
    text-align: center;
  }

  .empty-state.visible { display: flex; }

  .empty-icon {
    width: 48px; height: 48px;
    border-radius: 50%;
    background: var(--bg3);
    border: 1px solid var(--border);
    display: flex;
    align-items: center;
    justify-content: center;
    margin-bottom: 4px;
  }

  .empty-icon svg {
    width: 22px; height: 22px;
    stroke: var(--text3);
    fill: none;
    stroke-width: 1.5;
    stroke-linecap: round;
    stroke-linejoin: round;
  }

  .empty-title {
    font-size: 0.75rem;
    color: var(--text2);
    letter-spacing: 0.08em;
  }

  .empty-sub {
    font-size: 0.6rem;
    color: var(--text3);
    letter-spacing: 0.04em;
  }

  /* ── LOADING STATE ── */
  .loading-state {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }

  .skeleton {
    background: var(--bg3);
    border: 1px solid var(--border);
    border-radius: 12px;
    height: 62px;
    animation: skeleton-pulse 1.6s ease-in-out infinite;
  }

  @keyframes skeleton-pulse {
    0%,100% { opacity: 0.6; }
    50%      { opacity: 0.3; }
  }

  /* ── REFRESH BTN ── */
  .btn-secondary {
    width: 100%;
    padding: 14px;
    background: transparent;
    color: var(--text2);
    border: 1px solid var(--border2);
    border-radius: 10px;
    font-size: 0.68rem;
    font-weight: 600;
    letter-spacing: 0.2em;
    text-transform: uppercase;
    cursor: pointer;
    transition: background 0.2s, border-color 0.2s, color 0.2s, transform 0.1s;
    -webkit-tap-highlight-color: transparent;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
  }

  .btn-secondary:active { transform: scale(0.97); }
  .btn-secondary:hover { background: rgba(255,255,255,0.04); border-color: var(--accent-d); color: var(--text); }

  .btn-secondary svg {
    width: 14px; height: 14px;
    stroke: currentColor;
    fill: none;
    stroke-width: 1.8;
    stroke-linecap: round;
    stroke-linejoin: round;
    transition: transform 0.5s ease;
  }

  .btn-secondary.spinning svg { transform: rotate(360deg); }

  /* ── BACK LINK ── */
  .back-link {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    font-size: 0.6rem;
    letter-spacing: 0.18em;
    color: var(--text3);
    text-transform: uppercase;
    text-decoration: none;
    padding: 4px 0;
    transition: color 0.2s;
  }

  .back-link:hover { color: var(--text2); }

  .back-link svg {
    width: 12px; height: 12px;
    stroke: currentColor;
    fill: none;
    stroke-width: 1.8;
    stroke-linecap: round;
    stroke-linejoin: round;
  }

  /* ── DIVIDER ── */
  .divider {
    height: 1px;
    background: var(--border);
    margin: 2px 0;
  }

  /* ── FOOTER ── */
  footer {
    text-align: center;
    padding: 0px 20px 24px;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 14px;
  }

  .footer-text {
    font-size: 0.58rem;
    letter-spacing: 0.24em;
    color: var(--text3);
    text-transform: uppercase;
  }

  .monogram {
    opacity: 0.45;
    transition: opacity 0.3s;
  }
  .monogram:hover { opacity: 0.32; }

  /* ── TOAST ── */
  #toast {
    position: fixed;
    bottom: 24px;
    left: 50%;
    transform: translateX(-50%) translateY(80px);
    background: var(--bg3);
    border: 1px solid var(--border2);
    border-radius: 10px;
    padding: 11px 20px;
    font-size: 0.65rem;
    letter-spacing: 0.16em;
    color: var(--text2);
    text-transform: uppercase;
    z-index: 1000;
    transition: transform 0.3s cubic-bezier(.34,1.56,.64,1), opacity 0.3s;
    opacity: 0;
    white-space: nowrap;
    backdrop-filter: blur(10px);
    -webkit-backdrop-filter: blur(10px);
    pointer-events: none;
  }

  #toast.show {
    transform: translateX(-50%) translateY(0);
    opacity: 1;
  }

  #toast.ok  { border-color: rgba(122,200,160,0.4); color: var(--ok); }
  #toast.err { border-color: rgba(200,122,122,0.4); color: var(--err); }

  /* ── MODAL CONFIRM ── */
  .modal-overlay {
    position: fixed;
    inset: 0;
    background: rgba(0,0,0,0.65);
    backdrop-filter: blur(6px);
    -webkit-backdrop-filter: blur(6px);
    z-index: 500;
    display: flex;
    align-items: flex-end;
    justify-content: center;
    opacity: 0;
    pointer-events: none;
    transition: opacity 0.25s ease;
    padding-bottom: 24px;
  }

  .modal-overlay.open {
    opacity: 1;
    pointer-events: all;
  }

  .modal-sheet {
    background: var(--bg2);
    border: 1px solid var(--border2);
    border-radius: 20px;
    padding: 24px 20px;
    width: calc(100% - 32px);
    max-width: 440px;
    transform: translateY(40px);
    transition: transform 0.3s cubic-bezier(.34,1.4,.64,1);
    display: flex;
    flex-direction: column;
    gap: 16px;
  }

  .modal-overlay.open .modal-sheet { transform: translateY(0); }

  .modal-title {
    font-size: 0.68rem;
    letter-spacing: 0.2em;
    color: var(--text2);
    text-transform: uppercase;
    text-align: center;
  }

  .modal-filename {
    font-size: 0.85rem;
    color: var(--text);
    font-family: 'Courier New', monospace;
    text-align: center;
    padding: 10px 14px;
    background: var(--bg3);
    border-radius: 8px;
    border: 1px solid var(--border);
    letter-spacing: 0.02em;
    word-break: break-all;
  }

  .modal-actions {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }

  .btn-cancel {
    padding: 14px;
    background: var(--bg3);
    color: var(--text2);
    border: 1px solid var(--border2);
    border-radius: 10px;
    font-size: 0.68rem;
    font-weight: 600;
    letter-spacing: 0.18em;
    text-transform: uppercase;
    cursor: pointer;
    transition: background 0.2s, transform 0.1s;
    -webkit-tap-highlight-color: transparent;
  }

  .btn-cancel:active { transform: scale(0.97); }

  .btn-delete-confirm {
    padding: 14px;
    background: rgba(200,122,122,0.15);
    color: var(--err);
    border: 1px solid rgba(200,122,122,0.35);
    border-radius: 10px;
    font-size: 0.68rem;
    font-weight: 700;
    letter-spacing: 0.18em;
    text-transform: uppercase;
    cursor: pointer;
    transition: background 0.2s, transform 0.1s;
    -webkit-tap-highlight-color: transparent;
  }

  .btn-delete-confirm:active { transform: scale(0.97); opacity: 0.85; }
  </style>
  </head>
  <body>

  <header>
    <div class="logo-group">
      <div class="logo">TRACE LOGS</div>
      <div class="logo-sub">Gestione registrazioni</div>
    </div>
    <div class="conn-indicator">
      <div class="conn-dot" id="connDot"></div>
      <div class="conn-label" id="connLabel">Online</div>
    </div>
  </header>

  <main>

    <!-- BACK LINK -->
    <a href="/" class="back-link">
      <svg viewBox="0 0 16 16"><polyline points="10 12 6 8 10 4"/></svg>
      Torna al controllo
    </a>

    <!-- HERO -->
    <div class="hero">
      <div class="hero-bg"></div>
      <div class="hero-glow"></div>
      <div class="hero-inner">
        <div>
          <div class="hero-label">File disponibili</div>
          <div class="hero-count" id="fileCount">--</div>
          <div class="hero-count-label">registrazioni CSV</div>
        </div>
        <div class="hero-icon">
          <svg viewBox="0 0 24 24">
            <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"/>
            <polyline points="14 2 14 8 20 8"/>
            <line x1="16" y1="13" x2="8" y2="13"/>
            <line x1="16" y1="17" x2="8" y2="17"/>
            <polyline points="10 9 9 9 8 9"/>
          </svg>
        </div>
      </div>
    </div>

    <!-- CARD FILE LIST -->
    <div class="card">
      <div class="card-title">File sulla SD</div>
      <div class="card-body">

        <!-- Registrazione attiva -->
        <div class="recording-banner" id="recordingBanner">
          <div class="rec-dot"></div>
          <div class="rec-text">Registrazione in corso — download non disponibile</div>
        </div>

        <!-- Scheletri di caricamento -->
        <div class="loading-state" id="loadingState">
          <div class="skeleton"></div>
          <div class="skeleton"></div>
          <div class="skeleton"></div>
        </div>

        <!-- Lista file -->
        <div class="file-list" id="fileList" style="display:none;"></div>

        <!-- Stato vuoto -->
        <div class="empty-state" id="emptyState">
          <div class="empty-icon">
            <svg viewBox="0 0 24 24">
              <circle cx="12" cy="12" r="10"/>
              <line x1="12" y1="8" x2="12" y2="12"/>
              <line x1="12" y1="16" x2="12.01" y2="16"/>
            </svg>
          </div>
          <div class="empty-title">Nessun file trovato</div>
          <div class="empty-sub">Nessuna registrazione CSV presente sulla scheda SD</div>
        </div>

      </div>
    </div>

    <!-- REFRESH -->
    <button class="btn-secondary" id="refreshBtn">
      <svg id="refreshIcon" viewBox="0 0 24 24">
        <polyline points="23 4 23 10 17 10"/>
        <path d="M20.49 15a9 9 0 1 1-2.12-9.36L23 10"/>
      </svg>
      Aggiorna lista
    </button>

  </main>

  <footer>
    <div class="footer-text">TRACE · ESP32-S3</div>
    <div class="monogram">
      <svg width="128" height="128" viewBox="0 0 1254 1254" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path style="fill:var(--accent); stroke:none;" d="M619 70.4244C596.348 73.3983 576.128 78.7502 558 93.6157C551.432 99.0017 545.408 105.266 540.235 112C538.033 114.867 535.587 120.184 531.826 121.211C528.924 122.004 525.509 119.948 523 118.741C517.77 116.227 512.595 114.024 507 112.427C492.236 108.214 476.115 107.609 461 110.46C438.036 114.791 416.583 127.416 401.289 145C396.219 150.83 391.879 157.319 388.015 164C386.384 166.82 385.06 171.364 381.867 172.799C378.757 174.197 374.154 172.206 371 171.611C364.286 170.345 357.824 169.923 351 170.004C333.057 170.217 315.956 176.908 301 186.436C282.599 198.159 268.106 216.65 260.425 237C257.951 243.555 256.396 250.105 255.247 257C254.731 260.098 254.784 265.057 252.4 267.397C250.113 269.641 245.909 268.998 243 269C237.066 269.005 230.778 269.291 225 270.76C207.248 275.272 192.179 285.122 180.3 299C166.981 314.561 158.499 335.527 157.09 356C156.578 363.422 156.422 371.628 157.425 379C157.898 382.475 159.966 388.388 157.958 391.582C156.376 394.097 152.569 394.941 150 396.012C144.474 398.315 138.972 401.01 134 404.36C119.802 413.925 108.712 427.669 101.259 443C92.188 461.661 89.8923 484.688 93.2855 505C94.3395 511.31 95.6909 517.843 97.4282 524C98.282 527.026 100.376 530.794 99.8789 534C98.9104 540.247 90.1711 546.626 86.8148 552C79.0936 564.364 74.4924 577.592 72.4282 592C69.179 614.681 73.4772 640.072 82.4275 661C85.2371 667.57 88.6514 673.938 92.4252 680C94.3267 683.055 97.3652 686.358 97.9722 690C98.5288 693.339 96.6172 696.857 95.7454 700C94.2287 705.468 92.7357 711.348 92.1705 717C89.9402 739.307 93.6463 761.483 102.427 782C107.035 792.765 113.651 802.981 121.084 812C125.505 817.365 131.488 822.125 135.157 828C137.974 832.509 139.326 838.128 141.428 843C144.119 849.235 147.791 855.276 151.436 861C162.968 879.11 179.345 894.247 196 907.551C234.085 937.974 279.337 959.337 314 994C333.182 1013.18 349.403 1036.86 349.946 1065C350.418 1089.48 337.284 1112.82 354.529 1135C363.677 1146.77 377.839 1151.5 392 1153.42C419.182 1157.12 446.542 1145.91 474 1149.29C483.349 1150.43 494.21 1152.26 500.467 1160.04C507.015 1168.19 506.694 1177.41 517 1183.09C531.39 1191.03 547.724 1183.78 563 1186.33C585.149 1190.02 604.566 1210.23 628 1207.71C637.779 1206.66 646.947 1204.44 656 1200.57C667.513 1195.65 678.524 1188.7 691 1186.31C706.783 1183.29 722.674 1190.78 738 1183.46C748.564 1178.41 748.392 1169.24 754.558 1161C760.309 1153.32 769.855 1150.48 779 1149.29C807.357 1145.6 836.004 1157.79 864 1153.25C877.768 1151.01 891.204 1146.45 900.1 1134.99C917.252 1112.89 903.959 1086.7 905.039 1062C906.079 1038.22 919.406 1017.53 934.428 1000C947.287 984.993 963.98 972.712 980 961.291C1008.37 941.065 1038.46 923.307 1065 900.572C1080.22 887.53 1094.11 871.675 1103.69 854C1108.19 845.706 1110.6 836.178 1115.27 828.089C1118.47 822.558 1124.18 818.082 1128.11 813C1135.35 803.63 1141.2 793.712 1146.22 783C1154.57 765.178 1158.7 742.625 1156.83 723C1156.19 716.237 1155.01 709.589 1153.37 703C1152.45 699.288 1150.45 694.843 1150.6 691C1150.73 687.614 1153.22 684.741 1154.95 682C1157.8 677.486 1160.47 672.857 1162.69 668C1169.56 653.006 1175.23 635.603 1175.96 619C1176.76 600.753 1176.1 580.825 1168.22 564C1165.3 557.774 1162.26 551.503 1158.1 546C1155.38 542.4 1150.53 538.633 1149.7 534C1149.01 530.132 1151.55 525.651 1152.57 522C1154.46 515.261 1155.86 507.95 1156.71 501C1159.2 480.762 1156.17 459.511 1147.69 441C1140.61 425.554 1129.3 411.941 1115 402.699C1110.01 399.475 1104.61 396.933 1099 395.003C1095.98 393.963 1091.73 393.132 1090.31 389.867C1088.93 386.674 1090.68 382.308 1090.91 379C1091.49 370.547 1091.49 361.453 1090.91 353C1089.37 330.652 1079.43 309.582 1064.71 293.004C1054.37 281.357 1038.31 272.398 1023 269.461C1017.01 268.311 1011.08 268.003 1005 268C1001.99 267.999 997.944 268.589 995.514 266.397C992.465 263.646 993.17 258.667 992.714 255C991.829 247.891 989.708 240.737 987.302 234C980.545 215.084 966.657 197.252 950 186.089C933.965 175.344 915.401 168.908 896 169.001C890.134 169.029 883.714 169.538 878 170.899C874.868 171.645 870.415 174.017 867.213 172.672C864.054 171.345 862.693 166.789 861.219 164C857.889 157.697 854.025 151.548 849.539 146C837.306 130.869 820.618 118.93 802 113.029C783.842 107.274 764.495 106.251 746 110.873C738.991 112.625 732.48 115.044 726 118.248C723.148 119.657 719.458 122.213 716.184 120.716C712.598 119.076 710.324 113.985 707.985 111C703.518 105.3 698.382 100.014 693 95.1705C674.209 78.2593 644.507 67.0757 619 70.4244M612 76.2863C639.917 72.8573 668.603 80.9852 690 99.3002C695.695 104.175 700.8 109.971 705.244 116C707.993 119.729 710.394 124.534 715.004 126.254C719.201 127.819 723.31 125.578 727 123.752C732.883 120.842 738.669 118.234 745 116.427C761.831 111.625 780.243 111.84 797 117.026C815.772 122.835 833.016 133.506 845.334 149C849.555 154.31 853.17 159.958 856.244 166C858.395 170.228 860.421 175.629 865.015 177.782C868.469 179.4 872.552 177.734 876 176.873C881.86 175.411 887.941 174.153 894 174.015C912.29 173.596 930.872 179.327 946 189.52C963.97 201.627 976.935 219.604 983.921 240C985.744 245.323 987.218 251.403 987.826 257C988.326 261.597 987.567 267.058 991.303 270.566C994.206 273.291 998.31 272.994 1002 273C1008.42 273.011 1014.67 272.893 1021 274.095C1036.65 277.065 1051.93 286.142 1062.41 298.039C1075.61 313.021 1085.08 333.836 1085.96 354C1086.32 362.121 1086.67 370.926 1085.71 379C1085.23 383.107 1083.3 389.048 1085.22 392.945C1087.1 396.764 1091.36 397.564 1095 398.864C1100.17 400.712 1105.28 403.007 1110 405.823C1123.97 414.159 1135.85 427.18 1142.69 442C1152.07 462.317 1154.1 485.012 1150.7 507C1149.82 512.69 1148.41 518.518 1146.66 524C1145.59 527.327 1143.38 531.434 1143.76 535C1144.42 541.143 1152.83 546.91 1156.14 552C1163.02 562.594 1167.79 574.501 1169.73 587C1173.4 610.81 1170.43 635.624 1161.55 658C1158.78 664.975 1155.54 671.627 1151.57 678C1149.28 681.68 1145.78 685.542 1145.27 690C1144.37 697.829 1149.94 707.073 1150.83 715C1153.51 739 1150.42 763.093 1139.74 785C1135.07 794.579 1129.31 803.77 1122.54 812C1118.9 816.417 1113.83 820.031 1110.93 825C1106.26 833.034 1104.23 842.744 1099.69 851C1089.34 869.833 1074.29 885.948 1058 899.725C1017.43 934.034 965.517 956.378 930.425 997C914.715 1015.19 900.621 1037.16 900.015 1062C899.461 1084.67 912.068 1110.79 896.471 1131C884.223 1146.87 864.521 1149.36 846 1148.09C823.062 1146.52 800.016 1141.3 777 1144.29C743.765 1148.61 715.25 1164.16 686 1179.26C669.069 1188 652.31 1199.34 633 1201.71C617.686 1203.59 604.387 1197.94 591 1191.25C571.8 1181.65 553.594 1169.59 534 1160.86C511.209 1150.71 486.111 1143.33 461 1143.02C446.004 1142.83 430.899 1146.41 416 1147.83C396.029 1149.73 372.491 1149.49 358.9 1131.99C341.501 1109.58 357.169 1085.66 354.83 1061C351.846 1029.54 333.768 1006.51 312 985.004C296.875 970.059 278.6 958.4 261 946.645C237.565 930.993 213.699 915.624 192 897.565C176.702 884.833 162.683 870.195 152.453 853C147.225 844.213 144.897 833.395 139.298 825C135.201 818.857 129.083 813.801 124.449 808C116.441 797.975 109.894 786.87 105.011 775C97.5912 756.965 95.2319 736.389 97.1705 717C97.7431 711.274 99.208 705.453 101.001 700C102.109 696.628 104.148 692.627 103.397 689C102.637 685.335 99.6596 682.086 97.6952 679C94.2381 673.57 90.9911 667.854 88.3086 662C78.8489 641.355 74.1887 615.623 77.4282 593C79.3072 579.879 83.0631 567.189 90.3449 556C93.3156 551.435 96.5641 547.088 100.166 543C102.236 540.65 105.072 538.294 105.498 535C105.994 531.165 103.486 526.578 102.344 523C100.483 517.167 99.3904 511.044 98.439 505C95.2543 484.768 97.1229 463.589 106.109 445C113.414 429.889 123.985 417.293 138 408.004C143.071 404.643 148.403 402.203 154 399.86C157.314 398.473 161.165 397.363 162.968 393.945C164.681 390.699 163.302 386.448 163 383C162.24 374.32 161.657 365.742 162.039 357C162.921 336.871 172.165 316.089 185.289 301C197.255 287.243 212.425 279.076 230 275.12C237.16 273.508 251.179 276.951 256.566 271.566C259.58 268.554 259.45 262.968 260 259C260.948 252.16 262.594 245.456 265.065 239C273.362 217.323 289.113 199.283 309 187.453C322.844 179.218 339.832 174.338 356 175.039C361.628 175.284 367.531 176.042 373 177.374C376.778 178.294 381.203 179.848 384.895 177.82C389.037 175.546 390.691 169.935 392.782 166C396.221 159.526 400.716 153.588 405.439 148C419.799 131.01 441.235 119.383 463 115.424C478.004 112.695 494.472 113.471 509 118.356C513.755 119.954 518.549 122.007 523 124.32C526.081 125.92 529.339 128.402 532.996 127.214C536.824 125.97 539.325 122.043 541.626 119C545.835 113.432 549.878 107.877 555.001 103.089C570.702 88.4177 590.657 78.9076 612 76.2863M538 154C533.682 147.995 529.095 142.35 523 138.043C472.583 102.417 396 143.203 396 204C392.001 200.035 387.958 196.113 383 193.349C350.448 175.202 308.795 193.53 287.899 221C271.423 242.66 267.251 269.913 274 296C242.735 269.421 198.133 294.352 182.27 326C167.886 354.699 173.382 383.683 185 412C161.103 401.718 134.658 420.21 122.229 440C107.959 462.722 104.924 491.614 113.696 517C117.306 527.446 122.991 535.877 129 545C111.475 545.048 100.316 558.805 94.0116 574C82.6804 601.309 86.8803 636.569 101.427 662C105.964 669.931 111.301 677.182 117.425 683.961C120.572 687.444 124.249 690.175 127 694C119.184 694.87 114.743 701.034 112.065 708C107.154 720.774 107.232 734.637 109.296 748C112.581 769.269 123.093 788.576 136.615 805C141.801 811.299 148.81 816.333 153.486 823C157.539 828.778 158.664 837.51 161.784 844C167.511 855.915 176.447 867.753 186.039 876.83C215.021 904.255 249.475 924.513 282 947.28C306.498 964.428 330.803 983.661 347.305 1009C360.21 1028.82 365.993 1049.47 364.961 1073C364.116 1092.27 354.344 1112.06 371.04 1127.9C380.333 1136.71 392.877 1138 405 1138C426.784 1137.99 448.049 1132.58 470 1134.09C506.914 1136.62 541.72 1157.23 573 1175.28C588.947 1184.49 606.814 1198.8 626 1199C648.499 1199.23 670.266 1180.93 689 1170.28C720.926 1152.13 756.876 1135.19 794 1133.09C813.203 1132.01 832.721 1138 852 1138C863.351 1138 875.31 1135.95 883.816 1127.7C900.559 1111.46 889.902 1087.7 889.039 1068C888.143 1047.54 895.545 1027.11 906.436 1010C943.107 952.405 1012.43 927.226 1061 881.911C1072.07 871.583 1082.4 858.581 1089.19 845C1092.6 838.176 1093.36 829.218 1097.56 823C1103.4 814.369 1112.23 807.477 1118.52 799C1129.35 784.414 1136.41 766.835 1139.54 749C1140.12 745.681 1139.85 742.339 1140.17 739C1141.52 724.858 1141.02 697.636 1123 694C1151.54 665.378 1167.62 625.451 1158.35 585C1154.27 567.22 1143.63 545.943 1123 545C1128.17 535.722 1133.73 527.249 1136.97 517C1145.04 491.511 1142.33 460.43 1127.25 438C1114.29 418.72 1086.47 399.695 1063 412C1074.79 383.677 1080.09 355.222 1067.14 326C1054.88 298.341 1021.8 276.678 991 285.785C984.339 287.755 978.386 291.741 973 296C975.206 285.309 977.669 275.021 976.911 264C975.059 237.118 959.451 212.339 937 197.864C918.808 186.135 892.416 181.135 872 189.873C864.309 193.164 856.958 197.376 853 205C851.21 198.945 851.831 192.263 850.55 186C848.58 176.364 843.977 166.956 838.254 159C816.859 129.256 773.069 114.967 739 130.312C731.311 133.775 724.135 138.129 718.174 144.17C715.152 147.232 713.09 151.03 710 154C708.719 139.332 698.058 124.135 687.996 114.001C649.892 75.6211 584.612 80.6827 552.378 124C545.986 132.59 539.609 143.26 538 154M683 354C687.924 339.454 687.995 322.195 690.13 307C694.857 273.371 698.811 239.515 704.246 206C707.031 188.83 706.052 168.17 715.468 153C722.434 141.777 733.784 135.037 746 131.001C786.014 117.781 836.562 142.034 847.105 184C852.367 204.944 844.15 223.291 838.026 243C830.241 268.05 822.221 293.091 813.999 318C809.137 332.732 802.265 348.662 800 364C805.384 351.853 808.559 338.613 812.67 326C820.933 300.648 829.236 275.291 837.667 250C842.856 234.433 846.561 212.714 859.04 201.184C869.037 191.948 881.812 189 895 189C936.245 189 969.897 222.357 973.83 263C976.251 288.018 964.272 309.982 952.424 331C938.41 355.863 922.972 381.647 912 408C919.403 397.602 924.823 385.284 930.781 374C938.78 358.849 947.251 343.894 955.719 329C962.836 316.482 968.906 302.407 981 293.789C990.635 286.924 1003.68 286.013 1015 287.921C1048.66 293.593 1070.52 325.139 1071.96 358C1072.64 373.623 1070.19 389.692 1063.69 404C1059.02 414.281 1052.12 423.048 1045.37 432C1031.24 450.771 1014.63 469.582 1003 490C1017.22 473.568 1029.7 455.28 1042.88 438C1049.59 429.209 1056.48 417.973 1067 413.468C1082.58 406.792 1101.41 415.354 1113 426.09C1137.11 448.434 1143.11 483.024 1134.84 514C1129.4 534.395 1115.44 549.228 1102.25 565C1088.33 581.639 1074.46 598.324 1060.59 615C1052.69 624.487 1043.96 633.819 1037 644C1058.03 622.529 1076.26 597.786 1095.87 575C1102.98 566.735 1111.17 552.388 1122 548.858C1135.3 544.523 1146.81 561.977 1151.13 572C1163.13 599.868 1159.16 634.504 1145.11 661C1133.9 682.135 1115.19 696.16 1098 712.089C1073.27 735.002 1048.85 758.154 1025 782C1032.47 776.735 1038.32 769.308 1045 763.089C1060.93 748.253 1076.82 733.396 1093 718.831C1098.96 713.467 1104.73 707.819 1111 702.808C1114.15 700.291 1117.73 697.196 1122 697.217C1130.79 697.259 1134.53 708.013 1136.11 715C1140.35 733.694 1137.04 754.326 1130.19 772C1119.24 800.271 1096.92 819.828 1074 838.46C1045.45 861.669 1014.59 883.552 988 909C997.884 902.462 1006.8 894.086 1016 886.611C1030.54 874.792 1045.36 863.308 1060 851.6C1069.69 843.847 1080.36 832.477 1092 828C1085.58 859.001 1058.04 882.701 1034 900.87C987.869 935.741 927.901 963.74 899.781 1017C891.775 1032.16 885.303 1049.55 886.039 1067C886.947 1088.53 899.984 1115.71 876 1129.91C867.267 1135.07 857.789 1135 848 1135C828.578 1135 809.515 1130.77 790 1131C766.342 1131.29 747.064 1142.97 725 1149C735.632 1139.01 751.305 1132.23 764 1125.14C777.868 1117.4 791.791 1109.86 806 1102.75C812.817 1099.34 819.661 1095.11 827 1093C822.229 1092.96 818.214 1095.65 814 1097.75L794 1107.75C774.603 1117.45 755.608 1128.06 737 1139.2C726.648 1145.4 715.495 1154.34 704 1158C709.655 1149.41 716.967 1138.51 704.989 1131.01C703.078 1129.81 701.171 1129.39 699 1129C698.462 1117.25 690.266 1111.09 679 1111C676.721 1095.86 663.807 1092.61 651 1096C644.13 1072.21 609.973 1072.3 603 1096C589.047 1092.31 577.608 1096.82 574 1112C562.893 1109.45 554.92 1118.53 554 1129C546.263 1130.23 539.61 1136.63 541.537 1145C542.47 1149.05 545.348 1152.26 547 1156C534.221 1149.96 522.14 1141.28 510 1134C491.437 1122.86 472.358 1112.43 453 1102.75C443.424 1097.97 433.29 1091.96 423 1089C442.036 1100.11 462.6 1108.81 482 1119.31C497.991 1127.96 512.988 1138.48 529 1147C524.367 1147.96 519.288 1144.48 515 1142.81C505.242 1139.02 495.321 1135.41 485 1133.46C457.502 1128.27 431.448 1135 404 1135C389.773 1135 375.3 1132.67 367.533 1119C360.147 1106 365.434 1089.65 367.561 1076C368.594 1069.37 368.477 1061.68 367.83 1055C364.731 1022.98 342.668 995.204 320 974.089C286.068 942.483 243.888 921.509 208 892.389C186.637 875.055 166.706 855.146 159 828C169.872 830.826 180.208 842.547 189 849.35C203.731 860.748 218.433 872.194 233 883.801C241.892 890.886 250.518 898.729 260 905C250.388 895.802 239.383 887.909 229 879.6C212.002 865.997 195.028 852.369 178 838.801C162.048 826.09 146.111 813.343 133.654 797C118.06 776.541 107.865 747.928 111.579 722C112.823 713.323 116.734 695.049 129 696.495C138.439 697.609 148.243 709.209 155 715.155C166.706 725.455 178.43 735.719 190 746.17C197.234 752.703 204.043 760.395 212 766C190.887 744.887 168.545 724.862 146 705.282C130.586 691.894 113.853 679.543 104.258 661C90.4961 634.402 84.4169 598.907 98.3704 571C103.824 560.093 115.738 544.187 130 548.653C140.44 551.922 149.538 566.025 156.424 574C174.779 595.257 192.358 617.944 212 638C204.742 627.386 195.511 617.752 187.13 608C173.991 592.711 161.021 577.28 147.87 562C133.619 545.442 119.792 530.796 114.375 509C106.751 478.323 115.629 443.627 141 423.669C151.49 415.417 167.603 408.163 181 413.479C191.256 417.549 197.472 427.595 203.884 436C216.087 451.994 227.841 468.793 241 484C225.251 456.359 199.641 434.611 185.37 406C179.721 394.673 176.95 381.565 176.09 369C173.847 336.251 193.659 302.176 225 291.344C237.997 286.852 253.948 286.112 266 293.711C278.362 301.505 284.505 316.617 291.306 329C304.966 353.874 321.089 378.793 332 405C331.195 398.333 326.693 391.8 323.424 386C319.908 379.762 316.942 373.24 313.424 367C306.027 353.877 299.254 340.324 292.219 327C280.626 305.042 269.994 284.638 274.285 259C280.164 223.88 310.559 194.143 346 189.845C360.988 188.027 378.83 190.574 389.815 202.093C401.392 214.234 404.834 235.502 410 251C419.082 278.246 428.436 305.503 436.72 333C439.71 342.925 444.26 352.766 446 363C446.765 357.829 444.265 352.894 442.67 348C439.86 339.38 437.525 330.605 434.667 322C425.729 295.09 417.604 267.91 408.667 241C402.883 223.587 395.22 204.567 400.428 186C411.767 145.583 457.591 120.264 498 130.13C512.676 133.713 526.985 141.292 534.482 155C539.269 163.752 539.498 174.359 541.08 184C544.134 202.609 546.581 221.335 549.271 240C554.891 279.001 561.146 317.945 566.282 357C569.21 379.272 573.232 401.604 575 424C577.038 416.907 574.442 408.219 573.424 401C571.595 388.022 569.888 375.009 568.285 362C562.443 314.584 555.69 267.258 548.728 220C546.666 206.001 544.607 192.022 542.718 178C541.525 169.15 539.527 159.827 542.04 151C544.786 141.355 549.145 132.963 555.236 125C587.495 82.824 651.463 79.7001 687.96 118C694.534 124.899 699.854 133.239 703.572 142C712.596 163.265 704.405 189.062 701.271 211C696.803 242.268 692.61 273.64 688.845 305C686.9 321.201 683.337 337.673 683 354M625 181L625 410C627.085 405.031 626 398.35 626 393C626 381 625.943 368.999 626.001 357C626.193 317.082 627 277.001 627 237L627 199C627 193.148 628.104 186.182 625 181M484 235L484 238C484.696 236.446 484.696 236.554 484 235M760 237C754.292 266.431 747.01 295.708 740.576 325C737.467 339.153 732.631 354.529 732 369C736.599 361.372 736.89 349.664 738.881 341C744.23 317.73 748.77 294.27 754.119 271C755.804 263.667 757.38 256.323 759.116 249C760.042 245.093 761.493 240.826 760 237M485 239L485 242C485.696 240.446 485.696 240.554 485 239M486 243C487.381 260.349 493.463 278.131 497.626 295C505.732 327.85 514.164 360.863 521 394C522.512 390.124 520.885 385.949 519.989 382C518.021 373.323 515.937 364.659 513.884 356C507.647 329.683 501.934 303.242 495.373 277C492.752 266.512 491.637 252.229 486 243M883 307L882 311C883.303 309.332 883.553 309.008 883 307M363 309C367.955 329.859 379.872 350.189 388 370C388.883 364.957 384.823 358.725 383.05 354C379.75 345.205 375.723 336.707 372.201 328C369.666 321.735 367.318 314.231 363 309M881 312C872.652 332.078 863.486 351.801 855.4 372C852.241 379.89 847.966 387.723 846 396C850.89 390.566 852.973 381.703 855.85 375C861.853 361.012 868.091 347.113 873.799 333C876.16 327.163 882.104 318.3 881 312M682 355L681 367C682.956 363.32 683.629 358.891 682 355M447 363C447.276 366.161 448.042 368.979 449 372C449.961 368.749 448.773 365.821 447 363M799 364C798.232 367.357 797.3 370.56 797 374C798.933 370.923 800.245 367.51 799 364M389 371L390 375C390.393 373.076 390.146 372.64 389 371M450.333 373.667C450.278 373.722 450.222 374.778 450.667 374.333C450.722 374.278 450.778 373.222 450.333 373.667M451.333 376.667C451.278 376.722 451.222 377.778 451.667 377.333C451.722 377.278 451.778 376.222 451.333 376.667M392.333 378.667C392.278 378.722 392.222 379.778 392.667 379.333C392.722 379.278 392.778 378.222 392.333 378.667M522 395C522.18 399.122 523.17 402.975 524 407C525.525 403.084 523.949 398.586 522 395M333 405C333.939 408.26 335.296 411.068 337 414C337.513 410.48 335.265 407.61 333 405M983 406C974.656 419.584 962.498 432.982 957 448C965.426 440.278 971.366 427.676 977.424 418C979.667 414.418 983.634 410.367 983 406M525 408L525 411C525.696 409.446 525.696 409.554 525 408M911 408L908 415C910.151 412.848 911.505 411.07 911 408M265 410C270.324 424.066 281.602 437.552 290 450C290.364 445.779 286.527 441.504 284.319 438C278.543 428.837 273.013 417.343 265 410M376 410L376 415C389.588 415.112 404.582 419.238 411.583 432C416.371 440.727 414.623 450.502 415.039 460C415.914 479.983 416 499.951 416 520C411.71 513.248 416.613 502.846 407 500C406.997 495.435 407.553 489.656 402.895 487.038C400.088 485.46 396.805 486.334 394.224 484.041C391.852 481.934 391.394 478.542 389.428 476.108C385.064 470.706 378.21 470.522 372 472C368.406 464.379 363.236 465.047 356 464.085C350.645 463.373 346.301 462.596 341 464.363C337.402 465.562 334.475 467.893 331 469.273C324.424 471.884 316.623 471.007 310.015 474.456C304.165 477.51 304.468 483.008 301.066 487.725C298.955 490.651 294.877 491.743 292.09 493.928C287.894 497.219 284.809 501.925 283.221 507C280.711 515.019 283.914 522.956 282 531C280.805 528.834 280.309 527.447 280 525C267.934 526.903 271.765 542.622 280 547C276.192 553.799 275.35 562.898 284 566C282.214 573.594 284.876 578.555 293 579C293.359 585.551 296.573 588.835 303 590C303.749 599.848 315.422 604.723 324 606C321.821 608.562 319.63 613.31 316.621 614.879C312.593 616.979 305.253 612.184 302.093 609.895C293.912 603.97 290.189 593.122 282 586.669C264.635 572.986 243.499 572.473 223 567.373C215.889 565.604 205.337 560.496 198.019 562.322C193.436 563.466 192.521 567.975 193.855 572C195.628 577.348 199.913 581.028 205 583C200.308 594.076 208.371 601.425 218 606C212.699 617.017 222.06 624.137 232 627C226.41 637.488 236.608 643.18 246 644C238.868 654.853 249.657 658.994 259 659C255.556 670.657 264.097 674.138 274 671C274.109 680.253 279.529 683.589 288 680C290.675 687.438 296.718 686.216 302 682C301.677 697.661 292.075 708.188 284.453 721C276.5 734.368 271.06 751.285 273.3 767C275.154 780.008 280.424 793.158 291.001 801.481C295.422 804.961 302.792 806.337 306.397 810.498C309.073 813.587 308.646 820.11 310.054 824C312.36 830.373 315.259 836.349 319.004 842C320.953 844.941 324.735 848.46 325.414 851.985C326.183 855.971 321.576 857.645 319.093 859.633C316.154 861.987 313.849 865.235 313.225 869C311.821 877.482 316.569 883.69 319.637 891C321.246 894.837 321.117 899.083 322.482 903C326.09 913.352 333.52 924.178 346 922C344.364 926.248 342.417 930.384 343.808 935C345.986 942.231 353.047 944.95 357.787 950.093C360.76 953.319 362.067 957.502 364.637 960.985C367.174 964.421 370.632 966.089 373.91 968.633C380.484 973.734 386.227 977.879 393.848 970.701C400.849 964.106 394.125 956.298 391.164 950C389.54 946.547 390.256 942.649 389.482 939C388.412 933.955 385.399 929.19 385.144 924C384.733 915.648 389.32 907.326 390 899L473 900L473 894C457.294 893.676 444.789 885.704 440.899 870C437.55 856.479 439 841.821 439 828L439 757L439 721C439 715.781 437.914 709.156 439.318 704.105C441.679 695.609 456.308 697.464 461.567 690.906C469.058 681.565 469.5 670.111 467.374 659C466.276 653.263 466.447 648.854 460 647C463.683 637.452 468.141 626.429 467.871 616C467.59 605.147 462.246 594.783 456.329 586C452.468 580.269 446.926 569.25 439 571L439 469C455.062 496.519 467.024 527.069 480.309 556C498.558 595.744 516.964 635.504 535.745 675C543.918 692.19 551.964 709.553 559.576 727C562.562 733.845 565.611 745.815 572 750C574.626 740.816 579.048 731.88 582.6 723C590.369 703.579 598.482 684.288 606.576 665C625.951 618.826 644.725 572.375 663.603 526C671.976 505.433 683.146 484.402 689 463C692.11 473.522 690 487.045 690 498L690 568L690 798L690 843C690 851.221 690.739 859.85 689.572 868C687.301 883.854 679.952 893.953 663 894L663 900L744 900C768.18 900 792.53 900.27 816 893.573C833.63 888.542 847.659 879.554 863 870C863.001 882.565 865.861 894.662 867.92 907C868.881 912.76 871 919.144 869.76 925C868.822 929.429 866.606 933.594 865.478 938C864.569 941.554 864.66 945.579 863.404 949C860.747 956.241 853.494 962.37 860.148 970.367C868.381 980.261 874.544 972.453 882.039 967.309C888.674 962.755 890.583 956.986 895.529 951.093C898.452 947.611 902.649 945.878 905.895 942.79C912.358 936.643 911.045 929.73 909 922C927.829 925.286 929.689 904.222 934.88 892C938.002 884.649 943.128 878.53 941.671 870C941.057 866.413 939.437 862.998 936.786 860.478C933.988 857.818 928.122 855.831 929.857 850.995C931.031 847.725 934.03 844.846 935.971 842C939.164 837.319 941.925 832.226 944.124 827C946.063 822.394 946.162 814.306 949.6 810.699C952.65 807.498 958.323 806.312 962 803.779C970.361 798.02 976.181 789.478 979.561 780C985.572 763.143 984.018 743.258 976.688 727C971.612 715.739 962.699 705.627 961.184 693C960.871 690.391 960.104 684.801 962.603 683.002C964.793 681.426 968.483 683.139 970.996 682.754C977.247 681.796 977.953 676.238 978 671C988.122 675.864 995.415 668.269 993 658C1002.93 660.928 1010.76 653.9 1006 644C1016.16 643.107 1022.48 637.429 1020 627C1029.1 623.412 1038.69 616.667 1033 606C1038.83 603.454 1045.15 599.348 1047.3 593C1048.4 589.743 1047.3 586.107 1048.89 583C1050.63 579.599 1054.02 577.335 1055.91 574C1059.14 568.317 1057.43 561.715 1050 562.193C1043 562.643 1035.89 565.8 1029 567.199C1009.83 571.095 989.984 573.721 974 585.899C962.162 594.918 958.369 609.799 944 616C941.711 611.405 938.537 607.698 935 604C939.275 603.518 941.288 602.363 942 598C947.326 597.4 948.829 595.23 949 590C956.649 589.211 956.259 585.375 960.587 580.495C962.242 578.629 964.748 577.825 966.258 575.776C967.961 573.465 967.595 570.716 968.584 568.17C969.602 565.55 971.821 563.632 972.852 560.985C974.887 555.758 972.592 550.971 971 546C976.824 543.127 980.323 532.896 976.107 527.153C972.793 522.64 970.305 525.87 971 530C964.786 526.571 968.404 518.066 967.907 512C967.025 501.251 958.915 491.118 948 490C947.804 487.608 947.547 485.286 946.751 482.999C942.584 471.026 930.899 472.528 921 469.642C916.771 468.409 913.305 465.322 909 464.044C901.15 461.713 890.267 463.609 883 467C873.976 448.031 853.147 434.803 835 425.753C804.918 410.751 771.843 410 739 410L662 410L662 415C665.398 415.009 668.679 415.089 672 415.9C680.67 418.017 688.063 424.311 686.786 434C685.973 440.169 682.281 446.335 679.85 452C675.44 462.274 671.153 472.62 667 483C648.422 529.436 629.256 575.672 610.397 622C601.219 644.545 590.802 666.937 583 690C572.252 671.515 564.619 650.445 555.691 631C532.712 580.948 510.194 530.6 486.258 481C480.037 468.107 473.966 455.076 468.139 442C465.849 436.862 461.476 430.917 462.269 425.001C463.222 417.883 471.002 415.812 477 415L477 410L376 410M849 499C840.404 500.75 840.987 506.884 840.081 514C839.731 516.751 838.285 519.198 838.151 522C837.765 530.118 840.368 540.213 847 545C845.317 556.399 839.982 566.062 842.68 578C844.492 586.021 851.844 592.175 852.867 600C855.013 616.425 853 634.425 853 651C850.834 648.799 849.118 644.361 845.949 643.497C838.943 641.587 839.708 651.365 841 655C839.182 653.856 837.293 651.65 834.961 651.827C831.727 652.073 829.808 655.154 829.224 658.015C827.87 664.637 825.503 674.477 828.199 681C830.588 686.779 833.656 693.736 839.001 697.297C842.966 699.939 851.148 698.916 852.682 704.189C854.536 710.562 853 719.379 853 726L853 759C853 764.347 853.974 770.823 852.49 776C850.281 783.708 844.763 789.867 843.015 798C841.534 804.889 842.74 812.527 845.464 819C847.724 824.368 851.344 827.859 849.839 834C840.874 870.596 807.706 887.854 773 891.17C762.358 892.187 750.377 893.154 740 889.775C728.976 886.187 729.12 872.786 729.004 863C728.691 836.675 729 810.327 729 784L729 525L729 456C729 446.63 726.999 433.447 732.089 425.044C735.039 420.175 740.791 419 746 418.084C757.263 416.103 769.937 417.112 781 419.873C807.97 426.606 832.708 440.568 846.218 466C848.792 470.845 852.397 476.526 853.332 482C853.961 485.682 851.584 487.886 850.16 491C848.985 493.568 849.025 496.239 849 499M576 427L576 432C576.83 429.97 576.83 429.03 576 427M577 435L577 440C577.83 437.97 577.83 437.03 577 435M359 470C359 468.203 359.014 468.478 360 467C365.123 468.337 368.533 470.793 370 476C377.896 474.12 387.091 474.623 389 484C384.058 479.49 380.852 477.062 374 477C376.914 477.995 380.253 478.442 382.896 480.089C385.398 481.648 386.84 484.297 389.209 486.007C393.952 489.429 398.316 490.297 401 496C398.47 495.352 397.115 494.573 395 493C395.983 494.276 400.053 499.316 401.558 495.625C402.33 493.732 400.605 490.761 400 489C405.332 493.706 401.524 496.139 402.518 501.741C403.412 506.775 407.254 508.004 406 514C408.582 509.793 408.001 507.372 406 503C414.168 507.772 406.486 515.431 406 522C407.186 520.964 407.604 520.661 409 520C408.669 524.331 407.57 526.396 403 526C403.406 523.908 403.217 523.98 405 523C402.778 519.176 399.733 516.406 397 513C399.814 513.642 400.76 513.771 403 512C400.156 512.495 398.694 512.098 396 511C397.509 502.452 391.062 488.845 382 487C387.116 490.331 391.899 496.127 393.951 501.999C395.74 507.117 392.609 509.757 388 508C390.383 501.724 388.504 498.765 384 494C387.885 499.98 390.601 506.645 382 510C382 508.203 382.014 508.478 383 507C375.316 502.686 374.256 496.125 364 498C366.87 498.851 370.316 498.646 372.985 500.028C380.953 504.155 379.434 513.448 376 520C374.839 517.487 374.46 514.489 372.427 512.434C367.79 507.743 362.905 512.417 358 513.581C354.948 514.305 351.977 513.708 349 513C356.718 519.704 361.507 509.151 368.941 511.647C377.109 514.39 371.685 524.385 366 526C366.985 524.509 368.525 523.285 369.397 521.737C370.99 518.909 369.27 515.839 365.981 515.708C361.013 515.509 357.311 519.8 352 518.336C346.14 516.722 342.667 511.421 338 508C344.916 516.417 348.899 530.266 338.815 538.196C335.227 541.018 330.26 541.647 326 543C327.922 545.905 328.272 546.773 327 550L331 548C328.598 555.198 321.698 552.303 317 549C319.668 552.584 325.809 554.819 327.164 559.055C330.678 570.042 315.278 571.513 309 568C314.786 574.557 326.383 571.755 328.858 562.999C329.665 560.146 328.103 557.538 327 555C332.415 558.164 331.265 564.543 327.606 568.945C322.24 575.398 309.956 577.55 305.318 568.907C304.165 566.759 304.11 564.366 304 562C302.974 564.626 302.559 566.321 303.88 569.002C304.891 571.053 308.093 573.967 306.248 576.397C304.114 579.208 299.604 577.286 297 576.567C290.637 574.81 285.358 573.979 287 566C289.007 569.111 290.538 572.162 293 575C290.314 567.434 288.884 562.923 294 556C291.794 557.532 289.945 559.138 288 561C284.111 557.565 284.07 554.653 286 550C280.889 554.53 283.585 557.763 286 563C273.203 558.73 287.293 546.038 291 541C285.836 544.408 280.208 545.113 277 539C280.036 540.079 282.825 541.396 286 542L286 540C280.285 539.569 276.885 536.734 276 531C282.171 535.404 286.195 532.228 287 525C290.241 529.745 288.87 534.268 284 537C292.852 535.451 289.533 527.353 290.316 521C290.794 517.114 292.539 513.58 294 510C291.031 514.088 288.553 517.906 288 523C281.726 512.527 289.386 498.032 300 493.995C304.047 492.456 308.749 493 313 493C313.019 486.218 314.67 477.364 322.015 474.604C325.261 473.384 328.734 474.363 332 472.892C341.924 468.421 348.394 464.669 359 470M938 492C942.558 492.094 947.859 492.362 952 494.456C961.414 499.216 966.958 511.969 964 522C963.126 519.948 963.047 518.253 963 516C961.521 516.986 961.797 517 960 517C958.654 512.468 956.229 509.406 953 506C955.487 510.131 958.422 514.199 959.467 519C960.131 522.05 959.156 524.98 959.564 528C960.165 532.449 963.58 535.533 967 538C962.983 533.75 959.627 529.669 963 524C965.303 531.203 966.434 534.673 974 530C973.645 536.461 970.32 539.776 964 541C967.817 541.717 970.412 540.272 974 539C969.642 544.498 964.702 544.763 959 541C963.343 546.663 977.038 556.034 966 563C967.898 556.914 968.493 553.185 963 549C966.523 553.43 967.145 556.168 964 561L958 557C959.67 559.493 961.64 561.868 961.772 565.004C962.182 574.678 951.821 579.759 944 576L944 574C948.859 569.847 950.076 566.051 948 560C947.822 562.952 948.253 566.247 946.82 568.956C942.072 577.93 928.757 575.477 924.009 567.957C921.402 563.828 920.755 556.999 925 554C924.454 557.278 922.953 560.753 924.361 563.999C927.71 571.718 937.391 574.357 943 568C936.8 571.469 923.155 570.606 925.478 560.018C926.345 556.063 930.162 554.894 932.957 552.586C936.985 549.258 937.796 545.929 937 541C935.934 544.476 935.854 548.649 932.656 550.987C929.869 553.024 924.77 552.954 923.827 548.893C923.275 546.513 924.994 544.015 926 542C921.874 541.222 917.539 540.842 914.015 538.347C903.36 530.804 906.187 516.961 913 508C908.46 511.458 905.048 516.823 899 517.772C894.196 518.525 887.46 513.349 883.333 516.043C879.274 518.693 883.484 522.53 886 524C879.168 526.366 875.632 515.711 881.394 512.194C884.681 510.189 887.971 512.755 891 513.93C894.806 515.406 898.763 515.452 902 513C899.326 513.635 896.753 514.184 894 513.603C887.407 512.209 879.611 506.697 876.939 516.959C876.595 518.279 876.891 519.682 877 521C871.287 516.01 870.16 505.569 877.109 500.742C880.004 498.732 883.651 499.01 887 499C879.659 495.148 872.549 500.33 871 508C867.3 506.135 866.308 504.089 866 500C864.337 503.919 865.475 505.739 868 509C862.455 510.506 858.815 507.592 858 502L854 509L855 507C856.085 511.593 854.234 515.131 850 517C851.457 519.985 851.859 522.108 848 522L851 526C845.76 526.986 843.665 523.934 843 519C844.922 519.901 844.999 520.076 846 522C847.361 515.913 839.742 508.769 846 504C845.249 507.269 845.249 509.731 846 513C847.523 508.002 847.318 503.687 853 502C851.7 494.971 853.3 487.898 862 489C864.278 479.526 871.599 475.222 881 478C882.403 472.102 885.942 468.675 892 468C890.894 470.513 889.477 472.678 888 475C891.143 472.292 893.828 468.576 898 467.434C906.09 465.218 911.973 469.905 919 472.637C921.925 473.774 925.125 472.992 927.999 474.333C934.786 477.5 937.835 484.994 938 492M347 469C349.815 470.215 352.967 470.435 356 471C353.196 468.875 350.466 469.012 347 469M895 470C897.905 470.894 900.948 470.129 904 470C900.917 468.718 898.208 469.238 895 470M359 470C361.12 471.721 362.265 472.035 365 472C366.218 474.118 366.867 474.845 369 476C366.614 471.653 363.894 470.268 359 470M884 476L890 471C886.453 470.51 885.115 472.895 884 476M353 475C356.842 476.628 361.838 477.295 366 478C362.005 474.106 358.289 473.745 353 475M887 479C891.089 477.64 894.644 476.202 899 476C894.671 473.094 889.559 474.574 887 479M314 476C311.739 480.943 308 484.479 307 490C302.919 484.359 309.25 478.805 314 476M910 476C912.24 478.784 915.993 479.186 918.985 481.133C923.39 484 925.966 488.287 928 493C930.65 483.337 917.617 477.031 910 476M936 476C941.848 479.107 943.848 483.544 944 490C942.053 486.053 940.867 482.501 938 479C939.893 483.056 940.904 486.512 941 491L939 491C938.884 486.054 935.321 480.49 936 476M321 493C326.791 486.366 330.623 479.673 340 478C331.158 474.402 322.409 485.22 321 493M315 492L323 478C317.126 480.807 315.286 485.753 315 492M927 478C929.793 482.28 932.337 486.126 934 491C936.161 485.404 931.877 480.329 927 478M865 485C868.769 483.438 872.058 481.168 876 480C871.227 478.161 867.768 481.286 865 485M358 481C362.231 482.793 367.488 483.713 372 485C368.059 480.488 363.687 479.992 358 481M313 481C312.105 484.382 311.18 487.489 311 491L309 491C309.215 486.86 310.041 483.965 313 481M881 486C884.677 484.076 887.832 482.429 892 482C887.387 480.083 883.283 481.65 881 486M380 482C381.341 483.274 382.403 484.041 384 485C382.771 483.13 382.132 482.776 380 482M241 484C243.627 490.206 248.781 496.752 254 501C250.95 494.875 246.314 488.326 241 484M325 497C330.165 494.243 333.95 489.209 339 486C332.552 484.288 327.739 492.196 325 497M863 493C865.893 490.414 868.718 488.089 872 486C867.034 484.977 863.741 488.313 863 493M907 485C909.928 486.646 913.24 487.495 915.996 489.498C918.835 491.562 920.949 494.253 924 496C920.789 488.438 915.024 485.172 907 485M365 487C369.199 489.842 372.704 492.022 376 496C376.626 489.384 370.73 487.298 365 487M874 496L875 497C876.875 496.453 878.255 495.89 880 495C878.341 494.594 877.758 494.691 876 495C880.064 489.531 884.362 487.827 891 489C884.319 483.938 876.393 488.981 874 496M854 498C856.069 495.597 857.437 492.776 859 490C855.422 491.877 854.443 494.032 854 498M1002 490C998.981 493.805 995.897 497.513 994 502C997.419 499.217 1002.23 494.626 1002 490M332 501C336.333 498.04 340.018 494.852 345 493C339.316 491.034 334.254 496.21 332 501M862 499C864.507 497.205 866.205 495.507 868 493C864.716 494.343 863.244 495.665 862 499M288 514C294.255 506.451 296.582 499.008 307 496C297.355 492.58 288.83 505.893 288 514M945 495C952.27 501.109 957.766 504.518 961 514C964.238 504.869 952.792 496.406 945 495M866 499L867 500L866 499M338 504C342.087 505.208 346.368 502.914 350 501C345.879 500.059 341.642 502.088 338 504M858 500L858 502C859.635 501.455 859.455 501.635 860 500L858 500M902 500C904.934 502.835 907.976 503.646 912 504C909.012 501.765 905.62 500.838 902 500M384 501L383 507C385.597 505.148 385.798 503.596 384 501M860 502C860.526 504.543 860.878 505.535 863 507C862.251 504.746 861.656 503.686 860 502M367 503C369.46 504.288 371.74 505.338 374 507L372 507C373.836 509.447 375.237 510.701 378 512C376.409 505.909 373.301 503.205 367 503M391 503L391 509C392.582 506.943 392.826 505.613 393 503L391 503M868.667 503.333C868.222 503.778 869.278 503.722 869.333 503.667C869.778 503.222 868.722 503.278 868.667 503.333M874 513L879 509C877.502 508.521 877.65 508.6 876 509C878.089 506.349 879.801 505.086 883 504C876.982 502.046 874.279 507.916 874 513M398 505L399 509C399.751 506.953 399.656 506.404 398 505M849 505C848.014 506.479 848 506.203 848 508C849.766 507.021 849.955 506.778 849 505M892 506C895.596 510.536 904.137 513.959 908 508C901.756 509.159 897.784 508.564 892 506M375 507L374 508L375 507M858 507L858 511C857.29 509.241 857.29 508.759 858 507M344 509C348.423 513.144 354.012 512.315 358 508C352.931 509.341 349.215 509.965 344 509M389 510L389 515C390.947 514.547 392.269 514.006 394 513C399.147 522.054 400.4 532.066 402.895 542C407.04 558.502 413.748 572.587 404.475 589C401.096 594.979 397.011 603.362 390.996 606.991C386.979 609.415 382.443 608.747 378 609.439C361.716 611.977 342.057 609.885 334 593C324.9 599.332 313.017 587.19 312 578C315.267 578.466 317.945 579.805 321 581C319.814 579.814 319.416 579.472 318 579C323.694 577.785 326.088 583.109 332 579C325.241 574.588 332.026 569.658 333.438 564C334.229 560.833 333.176 558.125 333.015 555C332.815 551.105 333.835 547.963 333 544C345.094 540.408 347.858 530.26 347 519C349.832 520.09 352.879 521.709 356 521.424C360.526 521.011 364.288 518.244 369 518C367.297 521.17 365.455 522.09 362 523C368.387 535.663 380.181 519.655 382 512C384.58 511.973 385.138 511.771 387 510L389 510M302 536C303.471 532.495 302.661 528.681 303.61 525C304.96 519.76 308.377 515.61 311 511C301.066 516.045 301.247 526.465 302 536M852 511C852.406 513.092 852.217 513.02 854 514C853.309 512.612 853.016 512.139 852 511M869 512C870.446 518.034 873.336 523.087 879.004 526.079C880.978 527.122 889.263 529.071 888.37 524.19C887.989 522.111 884.116 520.056 883 518L884 517C892.317 519.395 896.313 521.298 905 519C902.897 530.367 908.337 542.625 921 544C920.147 549.148 917.875 556.983 918.623 562C919.273 566.365 923.193 569.684 923.554 573.998C923.77 576.581 922.183 578.846 921 581C923.863 581.561 926.123 581.476 929 581L925 588C927.018 587.66 927.968 587.677 930 588L929 584L932 583C931.45 577.581 935.771 577.761 940 579C937.94 587.149 929.307 598.739 920 594C911.784 609.837 893.865 612.236 878 610.147C871.239 609.256 865.315 609.724 860.418 603.895C858.453 601.556 857.467 598.583 855.882 596C850.627 587.438 844.976 579.63 845.259 569C845.494 560.207 849.59 552.511 850.83 544C851.91 536.595 852.746 529.17 855.029 522C855.917 519.207 856.904 515.516 859.394 513.692C861.807 511.924 866.113 512.479 869 512M941 512C944.174 520.72 947.784 526.388 946 536C951.381 528.897 947.212 517.147 941 512M847 513C848.223 514.195 848.42 514.316 850 515C848.861 513.984 848.389 513.691 847 513M405 514L406 515L405 514M194 515C196.846 521.723 202.445 527.414 207.081 533C210.85 537.542 214.227 542.487 219 546C212.876 535.355 203.247 523.081 194 515M1053 516L1054 517L1053 516M1051 518C1043.54 528.062 1034.22 537.085 1028 548C1034.97 542.872 1040.03 534.733 1045.4 528C1047.76 525.042 1051.2 521.898 1051 518M402 519C402.986 520.478 403 520.203 403 522C401.711 520.557 401.599 520.766 402 519M412 519C411.943 525.257 410.808 529.235 404 530L404 528C409.05 526.547 409.85 523.322 412 519M842 519C842.649 523.853 844.714 526.668 849 529C843.114 530.847 840.048 523.798 842 519M849.333 519.667C849.278 519.722 849.222 520.778 849.667 520.333C849.722 520.278 849.778 519.222 849.333 519.667M933 520C935.155 526.341 940.767 530.192 942 537L944 537C943.256 529.787 938.311 524.595 933 520M316 521C311.013 526.032 306.746 530.716 306 538L308 538C309.096 531.853 316.123 526.565 316 521M295 522C294.275 525.919 294.082 529.1 295 533C296.431 529.59 296.431 525.41 295 522M915 522C915.426 531.302 920.121 535.939 929 538L927 546C929.221 544.325 929.562 542.743 930 540C932.481 545.524 929.953 549.081 924 547C928.917 552.735 934.824 547.598 932.257 541.018C930.184 535.704 923.996 535.195 920.228 531.61C917.445 528.963 916.928 525.151 915 522M954 522L954 533C955.431 529.59 955.431 525.41 954 522M331 524C327.179 531.678 311.707 540.72 322.064 549.258C323.249 550.235 324.592 550.517 326 551C323.882 548.465 320.322 545.668 320.662 542C321.358 534.494 333.534 532.683 331 524M416 525C416 551.21 417.854 578.124 416 604C411.28 602.712 406.868 600.719 402 600C406.508 589.281 413.302 581.337 413.544 569C413.69 561.564 408.871 555.118 408.526 548C408.339 544.143 411.635 541.453 412.813 538C414.428 533.267 412.63 529.243 416 525M910 525C910.412 532.61 914.716 537.835 922 540C920.25 537.317 917.529 536.107 915.339 533.787C912.932 531.235 911.856 527.894 910 525M341 526C339.603 533.709 333.994 534.547 330 540C336.31 538.483 343.88 533.455 341 526M922 528C923.662 531.376 925.445 532.78 929 534C926.893 531.585 924.67 529.776 922 528M335 530C332.46 532.285 329.884 534.172 327 536C330.895 537.019 335.088 534.095 335 530M410 531C409.961 535.318 409.968 538.682 406 541L409 534C407.6 535.13 406.584 536.218 405 537L405 532L410 531M845 531C846.707 531.857 847.109 531.982 849 532L849 537C846.876 536.101 846.899 536.124 846 534C846.035 536.949 847.025 539.206 848 542C844.23 539.011 841.808 535.286 845 531M938 535C939.638 542.671 941.537 546.42 938 554C945.451 549.757 943.248 540.198 938 535M310 536C307.287 540.844 304.569 549.163 310 553C309.312 550.374 308.295 547.756 308.622 545C309.003 541.791 310.676 539.217 310 536M856 538C858.663 541.632 864.122 543.665 868 546C865.534 540.84 861.521 538.831 856 538M323 539C322.236 541.746 322.236 543.255 323 546C324.088 543.749 324.519 541.465 325 539L323 539M385 546C388.778 544.052 391.954 541.505 396 540C390.942 538.051 387.108 541.588 385 546M294 542C292.402 544.303 291.073 546.419 290 549C293.055 547.398 294.934 545.548 294 542M313 542C311.396 547.595 312.898 552.049 318 555L313 542M953 542C955.66 547.597 959.297 553.284 965 556C962.504 551.107 957.833 544.643 953 542M855 549C854.068 556.194 863.188 563.421 868 556C862.635 555.721 858.892 554.277 857 549L855 549M950 549L950 555L954 556C952.829 553.438 951.82 551.133 950 549M298 550C298.077 552.645 297.997 554.266 300 556L300 550L298 550M398 550C394.217 553.576 391.278 555.711 386 556C391.045 562.626 397.372 556.217 400 551L398 550M286 557L292 552C288.349 551.254 286.942 553.747 286 557M384 551C383.427 554.232 383.3 556.784 384 560C385.256 557.006 385.256 553.994 384 551M869 553L869 560C870.059 557.466 870.059 555.534 869 553M346 564L363 555C356.308 552.627 349.579 558.964 346 564M889 554C893.762 557.563 899.9 559.672 905 563C901.922 556.554 895.859 554.161 889 554M950 557C951.74 563.575 951.468 566.687 949 573C954.074 568.978 956.115 561.19 950 557M299 558C296.379 563.447 296.067 569.008 302 572C298.681 566.377 298.88 564.021 301 558L299 558M304 562C305.753 560.67 306.67 559.753 308 558C305.751 559.154 305.154 559.751 304 562M324 558C322.897 563.404 319.574 566.526 314 565C319.755 570.803 325.294 564.331 326 558L324 558M928 558C926.262 565.212 933.329 570.768 939 565C932.602 565.98 930.776 563.265 928 558M932 558L931 562C932.87 560.771 933.224 560.132 934 558L932 558M321.333 559.667C321.278 559.722 321.222 560.778 321.667 560.333C321.722 560.278 321.778 559.222 321.333 559.667M958 563C957.093 568.713 954.254 572.541 949 575C955.434 577.011 960.229 568.479 958 563M293 564C293.21 570.466 296.393 574.98 303 576C298.764 572.174 296.066 568.753 293 564M887 564C887.497 574.7 893.876 575.669 903 575C899.299 569.286 891.351 572.282 889 564L887 564M314 620C312.315 629.483 307.458 638.406 305.618 648C303.996 656.457 304.803 667.095 301.656 674.999C300.258 678.511 294.333 685.059 291.603 678.851C289.715 674.555 291.892 669.187 293 665C290.529 668.884 288.984 674.681 284.896 677.214C280.743 679.786 277.328 676.181 277.202 671.996C277.045 666.767 280.441 662.254 283 658C279.106 661.883 274.67 667.389 269.001 668.562C264.679 669.456 261.07 666.486 261.641 661.999C262.184 657.74 265.339 654.132 268 651C262.925 653.109 257.738 656.192 252.019 654.566C241.793 651.659 251.693 642.699 257 641C250.447 639.58 244.307 641.736 238.019 637.914C234.267 635.634 231.89 630.986 237.059 628.738C241.721 626.712 248.017 627.003 253 627C243.561 624.21 233.639 626.142 225.094 619.776C222.4 617.769 218.188 613.058 221.608 609.757C223.852 607.591 227.375 608.798 230 609.339C236.522 610.686 243.348 611.714 250 612C238.673 607.751 226.434 607.832 216 600.895C212.611 598.642 207.07 594.597 206.89 590.059C206.685 584.889 213.178 586.77 216 587.671C226.281 590.95 238.419 592.191 248 597C246.319 601.085 248.434 603.038 252 605C251.873 608.932 252.918 610.602 256 613C254.702 619.183 256.593 622.703 263 624C261.496 629.846 263.213 633.014 269 635C268.348 640.907 271.218 643.564 277 644C277.834 649.384 280.587 651.597 286 651C287.734 655.818 290.281 656.746 295 655C296.709 660.609 300.483 659.47 303 655C300.962 656.395 299.373 657.349 297 658C296.181 654.438 296.345 651.579 297 648C295.958 649.738 294.964 652.066 293.366 653.382C287.301 658.377 289.075 647.543 290 645C288.449 646.434 286.95 648.094 284.98 648.978C276.716 652.685 280.568 642.047 283 639C280.304 640.355 277.225 642.303 274.059 641.638C267.427 640.246 272.942 634.641 276 633C272.931 632.662 268.915 633.624 266.153 631.971C259.371 627.915 268.198 624.208 272 624C268.26 622.894 263.569 623.661 260.149 621.682C252.006 616.97 263.511 615.003 267 615C263.459 613.88 248.067 611.091 257 606C253.14 604.158 250.025 602.895 248 599C251.843 597.896 255.085 598.506 259 599C256.372 597.836 253.731 597 251 596.112C249.242 595.54 239.834 590.89 245.337 588.696C251.656 586.177 261.185 594.775 263 600C266.049 598.384 267.921 599.656 271 601C262.479 594.432 253.605 588.455 243 586C242.434 588.399 242.049 590.539 242 593C230.698 591.541 218.875 587.846 209 582.124C202.426 578.316 197.427 573.902 197 566C205.129 566.067 212.132 569.248 220 570.739C238.87 574.315 259.913 576.459 276 587.721C285.777 594.566 290.228 605.154 299.015 612.787C303.316 616.524 308.678 618.265 314 620M365 566C362.853 568.035 360.731 570.248 357.998 571.506C354.82 572.97 352.082 571.967 350 575C358.508 576.07 362.946 575.094 367 567L365 566M1054 566C1052.14 584.018 1025.62 589.983 1011 593L1010 587C1016.26 583.847 1023.17 581.579 1030 580C1025.03 579.265 1020.46 582.858 1016 580C1013.06 585.612 1006.4 586.556 1001 589.12C996.814 591.11 993.141 593.949 989 596L992 598C994.989 591.924 1003.31 587.61 1010 589C1007.78 595.205 1001.99 596.747 996 598L1005 599C1002.36 604.361 997.657 605.537 992 606C994.729 606.81 997.15 606.979 1000 607C998.967 613.089 993.412 613.763 988 614L988 616C991.328 616.039 999.216 618.018 992.847 621.822C990.041 623.498 986.13 622.998 983 623C984.115 623.793 985.787 624.703 986.722 625.756C993.586 633.487 979.982 632.451 976 631C977.445 632.335 979.164 633.658 980.278 635.309C985.393 642.896 974.944 641.437 971 640C975.405 648.017 971.395 652.249 964 646C964.422 648.283 965.46 651.428 964.383 653.663C962.533 657.501 958.92 653.003 957.97 650.981C955.659 646.062 955.221 640.162 953.484 635C951.64 629.521 948.597 624.544 947 619C961.504 612.74 965.918 597.39 978 588.239C993.256 576.685 1013.76 574.227 1032 570.551C1039.42 569.056 1046.37 566.158 1054 566M964 567C965.246 572.16 963.116 574.942 958 576C960.327 573.083 962.141 570.206 964 567M389 570C389.69 576.602 385.349 577.491 381 581C379.814 579.814 379.416 579.472 378 579L379 583C384.971 581.416 396.483 577.335 389 570M864 570C860.366 577.248 871.36 586.525 876 580C868.709 579.756 866.199 576.405 864 570M446 580C443.87 577.721 442.299 575.819 441 573C446.853 575.86 450.262 582.55 453.55 588C458.576 596.331 463.328 605.065 463.907 615C464.433 624.04 461.112 633.652 458 642C455.894 641.493 453.342 640.471 451.212 641.388C448.578 642.523 447.744 645.685 446.187 647.831C444.325 650.396 441.879 652.435 440 655C436.489 646.633 439 633.076 439 624L446 629L443 626C444.922 624.708 446.84 623.858 449 623C439.586 622.237 437.338 616.749 429.91 612.225C427.387 610.687 421.558 609.99 420.461 606.87C418.829 602.225 430.996 603.644 433 604C428.107 600.804 423.45 599.727 418 598C421.234 591.837 427.998 592.571 434 594C429.444 590.269 421.75 589.988 419 585C426.416 582.297 431.82 583.075 439 586C434.973 580.786 427.207 580.008 424 575C433.919 571.905 437.382 575.606 446 580M324 575C325.752 576.59 326.794 577.954 328 580C325.483 578.753 323.917 577.895 324 575M928 575C927.349 577.434 926.84 578.283 925 580L926 575L928 575M944 578C947.225 579.709 949.357 580.414 953 580C950.675 583.962 947.985 583.594 944 582C948.634 585.221 951.546 584.354 955 580C956.535 587.533 946.467 586.88 942 585C947.388 591.779 943.988 596.343 936 593C939.382 592.17 941.881 593.231 944 590C942.365 590.545 942.545 590.365 942 592L938 591C939.506 590.317 940.314 590.174 942 590C940.276 585.784 941.857 581.915 944 578M223 579C228.132 582.839 236.711 584.918 243 586L237 580C232.291 582.114 227.84 579.712 223 579M309 579C310.356 583.324 311.43 586.533 307 589L310 585C304.16 586.083 298.936 586.673 297 580C301.136 580.04 304.926 579.661 309 579M933 579C934.229 580.87 934.868 581.224 937 582C936.422 579.303 935.738 579.211 933 579M314 580L315 581L314 580M321 581C322.33 582.753 323.247 583.67 325 585C323.814 586.186 323.472 586.584 323 588L329 587C326.709 583.922 324.641 582.239 321 581M299 582C300.223 583.195 300.42 583.316 302 584C300.861 582.985 300.388 582.691 299 582M304 583C305.809 583.574 306.069 583.465 308 583C306.341 582.594 305.758 582.691 304 583M395 582C389.867 586.413 380.261 588.024 378 595C385.164 592.908 394.509 589.752 397 582L395 582M858 582C860.472 589.902 869.748 592.882 877 595C874.354 588.312 864.349 584.385 858 582M934 583L933 585C934.263 584.029 934.392 584.306 934 583M1003 611C1009.2 612.706 1015.73 610.053 1022 609.286C1024.3 609.005 1028.29 608.313 1030.21 610.028C1033.63 613.079 1028.31 618.237 1025.96 619.895C1017.78 625.659 1007.6 626 998 626C1002.68 627.385 1013.27 625.976 1016.29 630.318C1018.36 633.302 1015.36 636.437 1012.96 637.995C1006.53 642.164 999.861 640.568 993 639C995.57 640.516 998.516 641.81 1000.77 643.789C1005.27 647.733 1004.5 654.309 997.995 654.877C992.201 655.382 987.404 652.348 983 649C985.562 652.927 989.403 657.087 989.728 661.999C990.173 668.697 983.527 669.842 979 666.566C976.054 664.435 973.563 661.561 971 659C972.847 663.527 975.834 668.938 974.552 673.995C971.878 684.54 964.407 675.853 963 670C962.19 672.729 962.021 675.151 962 678L960 678C960 670.453 959.398 663.422 958 656C959.922 656.901 959.999 657.076 961 659C964.609 657.02 966.177 655.045 967 651C972.265 651.857 975.46 649.47 975 644C981.072 643.667 984.306 641.333 983 635C988.629 633.854 991.045 630.698 990 625C995.561 622.81 997.925 620.037 997 614C1000.46 611.344 1001.48 609.352 1001 605C1004.67 603.188 1006.67 601.065 1005 597C1013.82 592.572 1024.59 591.446 1034 588.333C1036.56 587.486 1042.07 584.866 1043.84 588.333C1045.87 592.304 1039.6 597.438 1037 599.471C1027.74 606.691 1014.53 610.082 1003 611M314 589C312.212 590.421 311.294 590.697 309 591C310.75 589.745 311.857 589.386 314 589M307 590C310.356 592.082 312.207 591.885 316 591C313.672 593.375 305.634 595.735 307 590M386 595C389.764 596.226 392.059 594.148 394 591C391.126 592.045 388.618 593.414 386 595M861 591C862.85 594.559 865.195 595.867 869 597C867.068 593.891 864.247 592.599 861 591M321 595C319.135 598.7 317.089 599.692 313 600C314.269 595.626 316.504 594.007 321 595M934 594C942.261 598.213 932.033 602.819 934 594M318 595L314 598C316.479 597.912 318.089 597.707 318 595M935 596C935.406 598.092 935.217 598.02 937 599C936.316 597.42 936.195 597.223 935 596M325 598C323.608 599.433 322.767 600.045 321 601L322 597C323.797 597 323.522 597.014 325 598M335 597C337.066 599.815 339.185 602.044 342 604.139C343.401 605.181 345.749 606.175 346.333 607.917C346.683 608.96 344.145 608.971 344 609C356.435 612.934 368.046 614.637 381 613C377.397 616.597 372.928 617.363 368 618C372.993 619.258 377.173 617.29 382 616L381 620C382.469 617.952 383.759 615.288 385.635 613.586C387.421 611.967 389.926 611.569 391.997 610.432C394.89 608.844 397.073 605.745 400.17 604.614C404.924 602.879 413.543 606.324 415.397 611.044C416.971 615.053 416 620.76 416 625L416 656L412 656C412 643.604 409.225 633.815 406 622C405.617 627.712 407.895 633.35 408.7 639C409.513 644.705 408.714 650.349 408 656C399.573 656.968 396.758 651.48 391 646.043C387.732 642.957 382.829 641.011 380.074 637.529C373.114 628.734 369.988 619.461 358 616C362.888 619.595 367.625 622.992 371.211 628C373.765 631.566 374.743 636.342 377.56 639.621C380.875 643.48 386.212 645.673 389.961 649.174C394.085 653.025 398.594 657.347 396 663C408.291 658.267 417.581 661.387 430 662.815C434.04 663.279 437.969 662.358 442 663C439.404 655.638 448.823 649.946 451 643C458.167 645.264 454.625 651.187 452 656C455.519 654.77 457.071 653.169 459 650C464.179 652.614 463.801 656.01 462 661C468.164 665 463.869 677.727 461.451 682.999C457.375 691.884 452.009 691.334 444 694.944C434.256 699.335 425.792 703.309 415 704.711C393.758 707.47 374.149 700.522 358 686.7C350.366 680.166 344.228 671.796 336 666C337.848 668.265 341.464 671.459 337 672C343.93 676.297 348.182 682.379 354.001 687.961C360.285 693.989 367.988 699.543 376 703C372.184 704.368 367.909 702.758 364 702C371.176 705.762 378.559 704.8 386 707C382.837 708.327 379.414 708 376 708C381.528 710.332 385.383 709.486 391 708C390.099 709.922 389.924 709.999 388 711C389.479 711.986 389.203 712 391 712C389.816 716.803 389.017 721.444 387 726C392.929 731.181 386.637 745.846 383 751C377.463 748.087 372.031 745.018 366 743.208C361.427 741.836 356.517 741.758 352 740.316C347.896 739.007 344.244 736.985 340 736C342.578 737.605 345.234 738.75 348 740C344.616 741.522 340.642 742.167 337 743C345.135 743.894 352.91 743.503 361 745.745C389.641 753.683 427.971 783.545 411.005 817C407.881 823.16 402.855 827.148 398.376 832.17C389.032 842.646 380.058 853.058 370 862.996C364.81 868.124 357.461 872.516 354 879L348 879C350.166 880.195 351.553 880.691 354 881C354.097 892.667 355.992 904.323 356 916L353 916L353 909L351 909C350.972 912.422 350.625 915.637 350 919C348.203 919 348.478 918.986 347 918C347.359 915.489 347.402 913.502 347 911C345.747 914.662 345.439 917.896 342 920C341.988 916.533 342.125 913.804 340 911C340.723 914.793 339.855 916.331 336 917C336.297 914.2 336.445 911.783 336 909L334 915L332 915C330.307 910.167 327.293 905.826 325.803 901C324.792 897.724 325.219 894.23 324.006 891C320.266 881.04 310.728 867.819 325 861L325 868C327.142 861.838 330.247 856.173 332.333 850C338.893 830.584 344.396 804.173 370 803L370 801C366.578 801.028 363.363 801.375 360 802C362.057 799.307 364.452 797.241 367 795C358.792 799.677 351.824 809.158 343 812.397C326.58 818.423 310.11 801.853 299 793L309 805C283.905 799.833 273.978 770.959 278.389 748C279.806 740.624 282.892 733.661 286.222 727C289.441 720.562 293.835 715.096 297.611 709C298.967 706.811 301.957 700.76 303 706C304.171 702.04 304 698.113 304 694C305.838 697.064 306.15 700.55 307 704C308.491 695.188 305.476 685.955 306.09 677C307.935 650.055 313.53 626.856 330 605L334 606C332.238 602.685 331.97 600.614 333 597L335 597M921 606C931.74 601.345 940.237 616.225 944.561 624C958.447 648.965 956.159 677.875 954 705C955.588 701.756 956.312 698.539 957 695C958.745 699.167 957.346 703.621 957 708L959 704C975.865 728.956 988.441 762.668 969.279 790C963.93 797.629 957.13 802.831 948 805L957 793C947.018 802.975 929.827 818.15 914 812.397C905.951 809.472 900.534 800.908 893 797C895.286 798.896 897.205 800.644 899 803L889 802C895.464 805.407 902.335 807.277 907.907 812.326C918.473 821.901 920.015 837.003 923.378 850C925.037 856.412 929.682 862.266 928 869C929.942 866.44 929.965 864.192 930 861C943.757 868.312 935.084 880.729 930.979 891C929.567 894.534 929.403 898.403 928.211 902C926.742 906.437 924.466 910.521 923 915L921 915C920.438 913.199 920.176 911.878 920 910L918 910L919 917L918 918L916 918L914 911C913.1 914.041 913.008 916.831 913 920C909.17 917.888 909.049 915.161 909 911C907.51 913.78 907.089 915.848 907 919L904 919L904 910L902 910L902 916L899 916C899.032 904.324 900.992 892.681 901 881L907 881C898.036 877.281 891.755 869.755 885 863C875.963 853.963 868.435 844.448 860.91 834.17C858.766 831.241 855.919 828.944 853.789 825.996C847.858 817.791 846.292 808.921 847.093 799C847.946 788.443 854.834 779.245 862.004 772C878.028 755.808 898.999 745 922 745C918.932 743.522 916.398 743.079 913 743C914.962 740.029 917.684 739.135 921 738C915.824 737.711 912.71 740.967 908 742.529C898.888 745.552 890.641 747.454 882 752C878.558 740.357 881 725.133 881 713C886.115 713.557 891.056 712.279 896 711C891.36 710.586 886.712 711.961 882 712C888.185 708.688 897.008 706.571 904 706C902.466 704.977 902.598 705.195 902 704C909.115 699.145 915.289 693.933 920.486 687C924.257 681.97 926.289 676.123 932 673C930.246 672.415 931.005 672.332 929 673C930.022 670.317 931.277 668.306 933 666C925.877 671.243 922.23 679.333 916.674 686C906.655 698.023 892.203 707.35 876 706.87C866.772 706.597 858.883 701.438 851 697.417C846.478 695.111 841.085 695.907 837.433 691.775C835.707 689.823 835.169 687.316 834.125 685C831.806 679.853 828.122 668.814 834 665C832.224 660.088 831.423 656.134 837 654C839.615 656.877 841.139 657.637 845 657C844.241 655.489 839.639 646.009 844.889 646.921C846.787 647.251 848.113 649.702 849.258 651.019C853.115 655.452 857.51 658.731 856 665C864.994 664.975 872.961 662.322 882 664C878.315 656.352 885.953 650.33 889.686 644.17C891.849 640.601 891.393 635.907 892.927 632C895.929 624.351 900.941 619.296 907 614C899.519 617.56 893.735 623.437 890.313 631C887.28 637.704 886.452 643.897 882 650C880.448 646.301 881 641.983 881 638C881 630.1 880.161 621.724 882 614C891.081 615.68 900.434 613.276 908 607.956C911.303 605.634 916.463 598.271 920.681 598.346C924.459 598.413 921.653 604.451 921 606M931 597C931.58 600.76 929.353 602.556 927 599C928.416 599.472 928.814 599.814 930 601C930.174 599.315 930.317 598.506 931 597M986 599C987.58 598.317 987.777 598.195 989 597C987.42 597.684 987.223 597.805 986 599M328 599C326.532 601.306 325.567 602.051 323 603C324.624 600.956 325.643 600.104 328 599M990 599C986.401 603.265 984.583 607.534 980 611C981.243 614.46 979.666 615.76 977 618C976.817 622.088 974.871 623.208 971 624C973.091 628.498 970.149 629.464 966 629C967.488 634.42 964.401 637.293 959 637L957 642C959.364 641.518 959.518 641.364 960 639C965.19 638.619 969.132 636.363 970 631C978.055 628.565 981.133 617.234 984.826 610.664C986.916 606.945 990.3 603.475 990 599M993.667 599.333C993.222 599.778 994.278 599.722 994.333 599.667C994.778 599.222 993.722 599.278 993.667 599.333M238 600C239.248 600.685 239.549 600.749 241 601C239.752 600.315 239.452 600.251 238 600M264 600C265.027 604.381 267.067 608.667 271 611C270.084 616.488 273.908 622.868 279 625C278.641 629.171 280.136 630.639 284 632C282.778 630.267 281.991 629.849 280 629C280.955 627.233 281.566 626.393 283 625C279.601 623.025 277.191 622.245 277 618C274.329 615.65 273.464 614.543 274 611C271.269 609.419 265.744 605.222 268 602L264 600M1008 601C1010.52 601.99 1012.36 601.617 1015 601C1012.54 600.112 1010.59 600.533 1008 601M242 601C243.248 601.685 243.549 601.749 245 602C243.751 601.315 243.452 601.251 242 601M273 602C274.48 605.166 276.168 606.951 279 609C277.255 606.223 275.543 604.065 273 602M981 602C979.242 604.012 978.084 605.581 977 608C979.279 606.209 980.831 604.937 981 602M258 606C259.769 606.779 261.036 606.912 263 607C261.231 606.221 259.964 606.088 258 606M278 611C279.33 612.753 280.247 613.67 282 615C280.846 612.751 280.249 612.154 278 611M356.667 615.333C356.222 615.778 357.278 615.722 357.333 615.667C357.778 615.222 356.722 615.278 356.667 615.333M882 616C884.248 617.799 886.163 618.493 889 619C886.794 617.183 884.802 616.606 882 616M242 617C244.89 618.213 247.874 617.998 251 618C248.11 616.787 245.126 617.002 242 617M282 617C283.402 619.274 284.515 619.987 287 621C285.435 619.119 284.201 618.058 282 617M1003 617C1005.05 617.874 1006.75 617.953 1009 618C1006.95 617.126 1005.25 617.047 1003 617M380 620L381 621L380 620M404 621L405 622L404 621M287 629C284.614 633.667 285.582 636.568 291 637C292.783 642.86 302.685 645.987 306 640C304.077 640.902 304.001 641.076 303 643L301 643L300 638L293 640L294 634C288.613 635.9 288.316 633.574 287 629M958 630L959 631L958 630M298.667 631.333C298.222 631.778 299.278 631.722 299.333 631.667C299.778 631.222 298.722 631.278 298.667 631.333M954.667 632.333C954.222 632.778 955.278 632.722 955.333 632.667C955.778 632.222 954.722 632.278 954.667 632.333M306.333 633.667C306.278 633.722 306.222 634.778 306.667 634.333C306.722 634.278 306.778 633.222 306.333 633.667M212 638C214.977 644.499 220.688 649.56 225.246 655C232.643 663.829 239.852 675.003 249 682C243.763 672.897 235.954 665.048 229.247 657C223.887 650.568 218.764 642.979 212 638M257.667 639.333C257.222 639.778 258.278 639.722 258.333 639.667C258.778 639.222 257.722 639.278 257.667 639.333M257 646C259.948 646.518 262.297 645.254 265 644C262.094 643.513 259.724 644.893 257 646M988 643C990.162 645.171 992.059 646.129 995 647C992.813 644.968 990.845 643.906 988 643M1036 644C1029.26 651.321 1023.28 659.349 1016.92 667C1011.81 673.141 1006 679.069 1002 686C1012.28 678.604 1020.15 665.72 1028.25 656C1031.02 652.668 1036.11 648.487 1036 644M452 645L453 646L452 645M271 648L268 650C269.926 650.338 270.709 649.976 271 648M844 648L845 649L844 648M273 655C271.029 656.941 269.355 658.609 268 661C270.408 659.263 272.302 657.905 273 655M979 655C980.324 657.687 981.313 658.676 984 660C982.451 657.864 981.136 656.549 979 655M334 664C334.545 665.635 334.365 665.455 336 666C335.455 664.365 335.635 664.545 334 664M283 670C284.457 668.897 284.897 668.457 286 667C283.876 667.899 283.899 667.876 283 670M967 667C967.684 668.58 967.805 668.777 969 670C968.317 668.42 968.195 668.223 967 667M182 691C191.867 705.007 206.768 717.045 218.985 729.039C224.954 734.9 230.352 743.359 238 747C230.436 736.262 219.282 727.282 210 718C201.035 709.035 192.371 698.306 182 691M385 692C386.45 693.608 387.051 694.064 389 695C387.607 693.566 386.767 692.955 385 692M890 694C888.257 695.111 887.881 695.27 887 697C889.108 696.415 890.801 696.153 890 694M1012 753C1022.74 745.436 1031.72 734.283 1041 725C1050.28 715.717 1061.44 706.738 1069 696C1061.9 699.309 1056.48 706.516 1051 712L1025 738C1020.38 742.623 1014.79 747.017 1012 753M415 709L415 781C410.75 776.227 407.458 770.643 402.91 766.089C398.044 761.217 392.219 757.451 387 753C390.066 747 392.429 740.822 392.791 734C392.969 730.635 391.877 727.37 392.199 724C392.688 718.887 394.773 714.203 395 709L415 709M372 734C373.395 736.038 374.349 737.627 375 740C378.824 738.761 380.392 735.558 382 732C378.428 733.367 375.867 734.44 372 734M884 736C884.547 737.875 885.11 739.255 886 741C887.081 739.542 887.436 738.752 888 737C886.359 736.869 885.544 736.631 884 736M212 766C217.476 773.774 225.249 779.429 232 786.015C241.574 795.354 251.065 806.297 262 814C253.436 801.842 240.692 792.184 230 781.961C224.353 776.561 219.158 769.338 212 766M1024 782C1015.4 790.422 1002.14 798.985 997 810C1004.06 805.026 1009.9 798.1 1016 792C1018.92 789.075 1023.01 786.077 1024 782M892 796L893 797L892 796M339 818C335.041 819.661 330.263 819 326 819C329.96 820.171 333.887 820 338 820C335.096 823.142 332.21 823.804 328 824C330.729 824.81 333.15 824.979 336 825C333.333 832.153 332.524 841.922 328 848C320.196 837.018 312.604 824.845 312 811C321.186 813.177 329.503 816.793 339 818M943 811C942.768 822.165 936.512 840.629 926 846C925.418 838.873 923.171 831.79 921 825L927 825C923.973 823.998 921.757 823.725 920 821C923.735 820.969 927.305 820.467 931 820C926.739 818.741 922.423 819 918 819C921.755 817.196 925.992 817.146 930 816.076C934.508 814.873 938.701 812.764 943 811M415 820L415 835C415 855.116 419.144 889.931 392 894C392.455 883.571 395.05 873.582 394.999 863C394.97 856.943 392.563 849.837 394.854 844C398.284 835.262 408.205 826.205 415 820M390 849C391.21 864.328 390.763 878.883 387.489 894C385.503 903.17 380.467 914.608 381.086 924C381.359 928.146 383.781 931.988 384.761 936C385.918 940.738 385.825 946.412 387.596 950.911C388.921 954.277 391.782 956.485 393 960L388 956C390.663 959.493 393.006 962.525 393 967C390.63 964.185 388.935 961.301 386 959C388.672 962.576 390.632 965.471 390 970L384 962C385.059 964.12 388.567 970.402 383.863 970.963C380.344 971.383 376.761 965.662 373.971 963.92C367.544 959.906 366.025 953.664 361.471 948.04C356.828 942.307 346.341 939.002 347.484 930.002C347.792 927.578 349.088 925.238 350 923L352 923L355 927C354.132 924.645 353.426 922.469 353 920C359.898 916.302 358.561 911.794 358.209 905C358.052 901.973 358.803 899.037 358.536 896C358.136 891.441 356.271 885.527 357.333 881.039C358.382 876.605 363.745 873.236 367 870.424C374.885 863.611 381.485 854.998 390 849M869 851C875.1 857.229 880.604 863.953 887 869.911C890.297 872.982 895.616 875.978 897.396 880.286C898.845 883.792 897.33 888.437 896.754 892C895.916 897.177 896.66 901.852 896.545 907C896.479 909.947 895.456 913.104 897.027 915.855C898.318 918.116 900.762 918.981 903 920C902.261 922.567 901.318 924.673 900 927C901.667 925.553 903.119 924.13 905 923C906.028 926.887 907.776 930.989 906.257 934.999C904.169 940.511 898.273 942.615 894.326 946.468C890.653 950.052 888.877 954.839 885.896 958.906C883.556 962.098 880.256 963.087 877.375 965.545C874.334 968.139 872.925 970.634 869 972C867.812 967.817 868.479 965.507 871 962C868.061 964.27 866.242 966.211 866 970C862.51 966.898 865.63 962.97 868 960L861 967C861.626 962.32 863.869 959.443 867 956L861 960C862.557 956.656 865.358 954.272 866.973 951C868.592 947.72 868.053 943.535 868.899 940C870.412 933.678 874.39 927.624 873.946 921C872.772 903.503 867 886.659 867 869C867 863.131 866.643 856.463 869 851M260 905C266.242 913.156 277.852 922.33 287 927C281.094 918.976 269.105 909.058 260 905M970 924C976.552 920.619 983.522 914.854 988 909C981.478 912.024 974.261 918.212 970 924M602 913L602 921C603.161 918.23 603.161 915.77 602 913M540 914C542.331 929.782 549.675 945.712 554.28 961C563.191 990.581 571.568 1020.28 580 1050C581.383 1046.1 579.401 1041.87 578.291 1038C576.193 1030.69 574.391 1023.31 572.291 1016C565.463 992.237 558.36 968.605 551.026 945C548.121 935.652 546.235 921.581 540 914M603 922C603 960.102 611.685 997.941 612 1036C615.369 1027.97 611.424 1016.48 611.039 1008C610.176 988.975 607.982 969.96 606.17 951C605.311 942.008 606.515 930.357 603 922M355 927L356 928L355 927M662 927C659.894 953.645 653.628 980.53 649.728 1007C648.316 1016.59 646.2 1026.29 646 1036C649.528 1028.66 649.334 1018.98 650.59 1011C653.475 992.675 656.071 974.304 659.08 956C660.378 948.108 665.053 934.484 662 927M899 927L900 928L899 927M719 934C709.076 959.585 699.373 985.227 689.947 1011C685.414 1023.39 678.929 1036.94 677 1050C682.092 1043.19 683.805 1032.93 686.808 1025C694.486 1004.73 701.408 984.158 709.399 964C712.021 957.385 714.459 950.667 716.95 944C718.174 940.724 719.951 937.429 719 934M787 935C765.789 966.442 745.315 998.477 725.8 1031C720.028 1040.62 714.266 1050.25 708.719 1060C705.591 1065.5 701.793 1070.9 700 1077C703.394 1073.64 705.36 1069.1 707.799 1065L721.4 1042C735.732 1018.11 750.233 994.195 765.667 971C771.181 962.713 776.479 954.286 782 946C784.196 942.704 787.592 939.078 787 935M475 941C483.417 959.006 495.591 975.796 505.576 993C520.888 1019.38 534.642 1046.69 550 1073C550.667 1068.99 547.763 1065.43 545.861 1062C541.854 1054.77 538.012 1047.36 534.245 1040C522.38 1016.82 508.793 994.323 495.4 972C489.483 962.139 483.906 948.379 475 941M420 964C427.108 974.395 436.528 983.548 444.845 993C458.647 1008.68 472.075 1024.74 485.197 1041C494.797 1052.89 504.192 1064.94 513.576 1077C518.487 1083.31 522.777 1090.93 529 1096C526.655 1090.45 522.474 1085.8 518.873 1081C511.792 1071.56 504.511 1062.27 497.211 1053C481.395 1032.91 464.912 1013.41 448.271 994C439.676 983.976 430.822 971.624 420 964M836 965C806.799 994.201 779.362 1025.56 752.579 1057C744.717 1066.23 736.971 1075.53 729.4 1085C725.366 1090.04 720.637 1095.08 718 1101C728.25 1093.14 735.712 1079.83 744.131 1070C763.755 1047.08 783.32 1023.94 804.036 1002C811.479 994.118 818.452 985.765 826.015 978C829.595 974.324 835.051 970.23 836 965M366 997C371.732 1003.94 380.809 1008.96 388 1014.37C401.78 1024.75 415.482 1035.28 429 1046C448.696 1061.62 468.131 1077.55 487 1094.16C497.839 1103.7 508.153 1114.75 520 1123C512.646 1112.56 501.354 1104.52 492 1095.91C468.417 1074.23 443.26 1054.26 418 1034.58C407.483 1026.38 396.68 1018.6 386 1010.63C379.7 1005.92 373.26 1000.07 366 997M888 997L889 998L888 997M884 1000C846.598 1026.79 810.016 1054.87 775 1084.73C764.171 1093.96 753.42 1103.22 743 1112.91C737.425 1118.09 731.419 1122.73 727 1129C741.344 1119.19 753.843 1105.67 767 1094.28C791.237 1073.31 816.525 1053.32 842 1033.88C852.313 1026.02 862.724 1018.27 873 1010.35C876.499 1007.65 882.806 1004.35 884 1000M645 1038L645 1043C645.83 1040.97 645.83 1040.03 645 1038M581 1051L581 1054C581.696 1052.45 581.696 1052.55 581 1051M676.333 1051.67C676.278 1051.72 676.222 1052.78 676.667 1052.33C676.722 1052.28 676.778 1051.22 676.333 1051.67M675.333 1054.67C675.278 1054.72 675.222 1055.78 675.667 1055.33C675.722 1055.28 675.778 1054.22 675.333 1054.67M582 1055L582 1058C582.696 1056.45 582.696 1056.55 582 1055M674.333 1057.67C674.278 1057.72 674.222 1058.78 674.667 1058.33C674.722 1058.28 674.778 1057.22 674.333 1057.67M583 1059L583 1062C583.696 1060.45 583.696 1060.55 583 1059M673.333 1060.67C673.278 1060.72 673.222 1061.78 673.667 1061.33C673.722 1061.28 673.778 1060.22 673.333 1060.67M672.333 1063.67C672.278 1063.72 672.222 1064.78 672.667 1064.33C672.722 1064.28 672.778 1063.22 672.333 1063.67M629 1191C631.607 1184.94 632.662 1178.34 634.424 1172C638.852 1156.07 642.864 1140.04 646.873 1124C648.485 1117.55 648.49 1103.86 654.147 1099.74C665.05 1091.81 678.602 1105.85 674.895 1117C672.823 1123.23 669.178 1129.14 666.247 1135C656.035 1155.42 643.065 1174.13 632 1194C646.015 1177.56 656.484 1156.91 666.769 1138C669.48 1133.02 672.237 1128.1 674.742 1123C676.06 1120.32 677.162 1117 680.133 1115.75C686.542 1113.04 693.871 1118.71 694.782 1125C695.474 1129.77 693.415 1134.08 690.91 1138C676.975 1159.78 655.779 1176.72 637 1194C654.954 1182.78 669.832 1165.64 683.83 1150C687.513 1145.89 690.642 1141.38 694.015 1137.02C695.511 1135.09 697.285 1132.69 700.014 1132.75C701.849 1132.79 703.521 1133.99 704.772 1135.23C712.045 1142.44 704.636 1151.6 698.996 1156.91C690.273 1165.12 680.007 1171.83 670 1178.34C659.449 1185.21 648.258 1192.65 636 1196L637 1194L631 1197L632 1194C630.814 1195.19 630.472 1195.58 630 1197C628.686 1196.23 628.768 1196.31 628 1195C620.271 1200.34 611.659 1190.8 606 1186.08C593.18 1175.38 581.129 1162.96 570.611 1150C566.121 1144.47 560.194 1138.13 558.533 1131C556.73 1123.26 564.8 1110.7 573.725 1116.18C578.881 1119.34 581.6 1130.71 584.247 1136C587.644 1142.79 591.388 1149.41 595.15 1156C601.386 1166.93 607.82 1178.4 616 1188C603.529 1165.6 586.949 1143.71 579.093 1119C575.369 1107.29 584.302 1095.43 596.995 1098.39C604.979 1100.26 604.132 1113.29 605.389 1120C608.097 1134.45 611.309 1148.8 615.151 1163C617.475 1171.59 620.81 1180.17 622 1189C623.047 1182.49 619.839 1175.37 618.424 1169C614.947 1153.34 611.328 1137.74 608.2 1122C606.828 1115.1 604.111 1105.99 605.544 1099C609.833 1078.08 644.29 1075.13 647.787 1098C649.318 1108.01 645.773 1119.35 643.373 1129C639.899 1142.97 636.568 1156.97 633.349 1171C631.827 1177.63 629.555 1184.2 629 1191M417 1086C418.953 1087.72 420.463 1088.34 423 1089C421.051 1087.43 419.412 1086.71 417 1086M827 1091C829.274 1091.41 830.781 1090.79 833 1090C830.726 1089.59 829.219 1090.21 827 1091M616 1195C610.426 1195.93 603.76 1190.36 599 1187.77C582.969 1179.05 566.102 1168.81 553.015 1155.96C547.765 1150.81 539.197 1136.96 551.039 1132.92C557.64 1130.66 563.87 1144.89 567.286 1149C581.527 1166.13 598.107 1181.74 616 1195M755 1152C752.895 1155.03 750.163 1157.72 748.442 1161C745.999 1165.66 745.607 1170.64 741.671 1174.53C731.253 1184.82 717.756 1180.96 705 1180.09C699.911 1179.74 695.025 1180.36 690 1181C699.104 1173.35 709.248 1170.24 721 1170C717.959 1169.1 715.169 1169.01 712 1169C723.108 1159.8 741.077 1154.88 755 1152M506 1153C519.081 1157.26 531.332 1162.73 544 1168C541.948 1168.87 540.253 1168.95 538 1169L538 1171C548.94 1171.01 555.375 1175.79 565 1180C560.752 1181.31 555.529 1179.78 551 1180.09C538.246 1180.96 524.722 1184.9 514.213 1174.67C509.32 1169.91 506.867 1161.14 504 1155C507.736 1156.86 510.847 1159.27 514 1162C511.462 1158.69 508.376 1156.43 505 1154L506 1153M749 1155C746.227 1157.12 744.037 1159.16 742 1162C745.038 1160.24 748.124 1158.51 749 1155M514 1162C515.33 1163.75 516.247 1164.67 518 1166C516.67 1164.25 515.753 1163.33 514 1162M616 1188C617.432 1191.38 619.159 1193.68 622 1196C620.536 1192.72 618.702 1190.37 616 1188M623 1189L623 1192C623.696 1190.45 623.696 1190.55 623 1189M628.333 1192.67C628.278 1192.72 628.222 1193.78 628.667 1193.33C628.722 1193.28 628.778 1192.22 628.333 1192.67z"/>
      </svg>
    </div>
  </footer>

  <div id="toast"></div>

  <!-- MODAL CONFERMA ELIMINAZIONE -->
  <div class="modal-overlay" id="deleteModal">
    <div class="modal-sheet">
      <div class="modal-title">Eliminare il file?</div>
      <div class="modal-filename" id="deleteModalName">—</div>
      <div class="modal-actions">
        <button class="btn-cancel" id="cancelDelete">Annulla</button>
        <button class="btn-delete-confirm" id="confirmDelete">Elimina</button>
      </div>
    </div>
  </div>

  <script>
  // ─── STATE ────────────────────────────────────────────────────────────────────
  let filesData = [];
  let pendingDeleteFile = null;
  let isRecording = false;

  // ─── UTILS ────────────────────────────────────────────────────────────────────
  function formatBytes(bytes) {
    if (bytes < 1024) return bytes + ' B';
    if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
    return (bytes / (1024 * 1024)).toFixed(2) + ' MB';
  }

  // ─── TOAST ────────────────────────────────────────────────────────────────────
  let toastTimer;
  function showToast(msg, type) {
    const t = document.getElementById('toast');
    t.textContent = msg;
    t.className = 'show' + (type ? ' ' + type : '');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { t.className = ''; }, 2400);
  }

  // ─── CONNECTION ───────────────────────────────────────────────────────────────
  function setOffline() {
    document.getElementById('connDot').classList.add('error');
    document.getElementById('connLabel').textContent = 'Offline';
  }
  function setOnline() {
    document.getElementById('connDot').classList.remove('error');
    document.getElementById('connLabel').textContent = 'Online';
  }

  // ─── RENDER FILE LIST ─────────────────────────────────────────────────────────
  function renderFiles(files, recording) {
    isRecording = recording;
    filesData = files;

    const loadingEl  = document.getElementById('loadingState');
    const listEl     = document.getElementById('fileList');
    const emptyEl    = document.getElementById('emptyState');
    const bannerEl   = document.getElementById('recordingBanner');
    const countEl    = document.getElementById('fileCount');

    loadingEl.style.display = 'none';

    // Banner registrazione attiva
    if (recording) {
      bannerEl.classList.add('visible');
    } else {
      bannerEl.classList.remove('visible');
    }

    countEl.textContent = files.length;

    if (files.length === 0) {
      listEl.style.display = 'none';
      emptyEl.classList.add('visible');
      return;
    }

    emptyEl.classList.remove('visible');
    listEl.style.display = 'flex';
    listEl.innerHTML = '';

    files.forEach(file => {
      const item = document.createElement('div');
      item.className = 'file-item';
      item.innerHTML = `
        <div class="file-item-icon">
          <svg viewBox="0 0 24 24">
            <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"/>
            <polyline points="14 2 14 8 20 8"/>
          </svg>
        </div>
        <div class="file-info">
          <div class="file-name">${file.name}</div>
          <div class="file-size">${formatBytes(file.size)}</div>
        </div>
        <div class="file-actions">
          <div class="btn-icon download-btn ${recording ? 'disabled' : ''}"
               title="Scarica" data-name="${file.name}"
               style="${recording ? 'opacity:0.35;pointer-events:none;' : ''}">
            <svg viewBox="0 0 24 24">
              <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
              <polyline points="7 10 12 15 17 10"/>
              <line x1="12" y1="15" x2="12" y2="3"/>
            </svg>
          </div>
          <div class="btn-icon delete ${recording ? 'disabled' : ''}"
               title="Elimina" data-name="${file.name}"
               style="${recording ? 'opacity:0.35;pointer-events:none;' : ''}">
            <svg viewBox="0 0 24 24">
              <polyline points="3 6 5 6 21 6"/>
              <path d="M19 6l-1 14a2 2 0 0 1-2 2H8a2 2 0 0 1-2-2L5 6"/>
              <path d="M10 11v6"/>
              <path d="M14 11v6"/>
              <path d="M9 6V4h6v2"/>
            </svg>
          </div>
        </div>
      `;
      listEl.appendChild(item);
    });

    // Event listeners download
    listEl.querySelectorAll('.download-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        if (isRecording) return;
        const name = btn.dataset.name;
        window.location.href = '/download?file=' + encodeURIComponent(name);
        showToast('Download avviato', 'ok');
      });
    });

    // Event listeners delete
    listEl.querySelectorAll('.btn-icon.delete').forEach(btn => {
      btn.addEventListener('click', () => {
        if (isRecording) return;
        openDeleteModal(btn.dataset.name);
      });
    });
  }

  // ─── LOAD FILES ───────────────────────────────────────────────────────────────
  async function loadFiles() {
    const loadingEl = document.getElementById('loadingState');
    const listEl    = document.getElementById('fileList');
    const emptyEl   = document.getElementById('emptyState');

    loadingEl.style.display = 'flex';
    listEl.style.display    = 'none';
    emptyEl.classList.remove('visible');

    try {
      const res = await fetch('/api/logs');
      if (res.status === 403) {
        // Registrazione attiva
        const d = await res.json();
        renderFiles([], true);
        setOnline();
        return;
      }
      if (!res.ok) { throw new Error('http ' + res.status); }
      const d = await res.json();
      if (!d.ok) { throw new Error(d.error || 'error'); }
      renderFiles(d.files || [], d.recording || false);
      setOnline();
    } catch(e) {
      loadingEl.style.display = 'none';
      showToast('Connessione non disponibile', 'err');
      setOffline();
    }
  }

  // ─── DELETE MODAL ─────────────────────────────────────────────────────────────
  function openDeleteModal(filename) {
    pendingDeleteFile = filename;
    document.getElementById('deleteModalName').textContent = filename;
    document.getElementById('deleteModal').classList.add('open');
  }

  function closeDeleteModal() {
    pendingDeleteFile = null;
    document.getElementById('deleteModal').classList.remove('open');
  }

  document.getElementById('cancelDelete').addEventListener('click', closeDeleteModal);

  document.getElementById('deleteModal').addEventListener('click', function(e) {
    if (e.target === this) closeDeleteModal();
  });

  document.getElementById('confirmDelete').addEventListener('click', async () => {
    if (!pendingDeleteFile) return;
    const name = pendingDeleteFile;
    closeDeleteModal();
    try {
      const res = await fetch('/api/delete-log', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ file: name })
      });
      const d = await res.json();
      if (d.ok) {
        showToast('File eliminato', 'ok');
        loadFiles();
      } else {
        showToast(d.error || 'Errore eliminazione', 'err');
      }
    } catch {
      showToast('Connessione non disponibile', 'err');
      setOffline();
    }
  });

  // ─── REFRESH BUTTON ───────────────────────────────────────────────────────────
  document.getElementById('refreshBtn').addEventListener('click', () => {
    const btn  = document.getElementById('refreshBtn');
    const icon = document.getElementById('refreshIcon');
    btn.classList.add('spinning');
    icon.style.transition = 'transform 0.5s ease';
    icon.style.transform  = 'rotate(360deg)';
    setTimeout(() => {
      icon.style.transition = 'none';
      icon.style.transform  = 'rotate(0deg)';
      btn.classList.remove('spinning');
    }, 520);
    loadFiles();
  });

  // ─── INIT ─────────────────────────────────────────────────────────────────────
  loadFiles();
  </script>
  </body>
  </html>
)rawhtml";

// Handler per GET /logs (pagina HTML)
static void handleLogsPage() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", LOGS_HTML);
}

// Handler per GET /api/logs (JSON)
static void handleGetLogsList() {
  if (recordingActive || fileOpen) {
    server.send(403, "application/json", "{\"ok\":false,\"recording\":true,\"error\":\"recording_active\"}");
    return;
  }
  // Aprire directory root
  File root = SD.open("/");
  if (!root) {
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"cannot_open_root\"}");
    return;
  }
  JsonDocument doc;
  doc["ok"] = true;
  doc["recording"] = false;
  JsonArray filesArray = doc["files"].to<JsonArray>();
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (name.length() > 0 && name[0] == '/') {
        name = name.substring(1);
      }
      if (isSafeCsvFilename(name)) {
        JsonObject fileObj = filesArray.add<JsonObject>();
        fileObj["name"] = name;
        fileObj["size"] = entry.size();
      }
    }
    entry.close();
  }
  root.close();
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// Handler per GET /download
static void handleDownloadFile() {
  if (recordingActive || fileOpen) {
    server.send(403, "text/plain", "Recording active: cannot download");
    return;
  }
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }
  String fileName = server.arg("file");
  if (!isSafeCsvFilename(fileName)) {
    server.send(400, "text/plain", "Invalid filename");
    return;
  }
  File file = SD.open("/" + fileName, FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + fileName + "\"");
  server.streamFile(file, "text/csv");
  file.close();
}

// Handler per POST /api/delete-log
static void handleDeleteLog() {
  if (recordingActive || fileOpen) {
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"recording_active\"}");
    return;
  }
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"no_body\"}");
    return;
  }
  JsonDocument doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid_json\"}");
    return;
  }
  const char* fileName = doc["file"];
  if (!fileName || !isSafeCsvFilename(String(fileName))) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid_filename\"}");
    return;
  }
  String fullPath = "/" + String(fileName);
  if (!SD.exists(fullPath)) {
    server.send(404, "application/json", "{\"ok\":false,\"error\":\"file_not_found\"}");
    return;
  }
  if (SD.remove(fullPath)) {
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"delete_failed\"}");
  }
}

// ---------------------------------------------------------------------------
// Handler: 404
// ---------------------------------------------------------------------------
static void handleNotFound() {
  server.send(404, "application/json", "{\"error\":\"not found\"}");
}

// ===========================================================================
// API pubblica
// ===========================================================================
void webServerInit() {
    // Avvia Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    // Serial attivo solo in modalità TWAI (produzione) — in UART sim inquinerebbe il canale OBD2
    #ifdef OBD2_TRANSPORT_TWAI
      Serial.print("[WiFi] AP avviato: ");
      Serial.println(AP_SSID);
      Serial.print("[WiFi] IP: ");
      Serial.println(WiFi.softAPIP());
    #endif
    
    // Registra routes
    server.on("/",            HTTP_GET,  handleRoot);
    server.on("/api/status",  HTTP_GET,  handleGetStatus);
    server.on("/api/color",   HTTP_POST, handlePostColor);
    server.on("/api/mode",    HTTP_POST, handlePostMode);
    server.on("/api/params",  HTTP_POST, handlePostParams);
    server.on("/logs",         HTTP_GET, handleLogsPage);
    server.on("/api/logs",     HTTP_GET, handleGetLogsList);
    server.on("/download",     HTTP_GET, handleDownloadFile);
    server.on("/api/delete-log", HTTP_POST, handleDeleteLog);
    // Richieste automatiche del browser — rispondi 204 per evitare i log di errore
    auto noContent = []() { server.send(204); };
    server.on("/favicon.ico",                     HTTP_GET, noContent);
    server.on("/apple-touch-icon.png",            HTTP_GET, noContent);
    server.on("/apple-touch-icon-precomposed.png",HTTP_GET, noContent);
    server.onNotFound(handleNotFound);

    server.begin();
    Serial.println("[WebServer] Avviato su porta 80");
}

void webServerHandle() {
    server.handleClient();
}
