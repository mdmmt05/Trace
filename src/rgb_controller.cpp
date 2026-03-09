// ---------------------------------------------------------------------------
// rgb_controller.cpp
// ---------------------------------------------------------------------------

#include "rgb_controller.h"
#include "shared_data.h"

// ---------------------------------------------------------------------------
// Nomi modalità (indice = valore enum)
// ---------------------------------------------------------------------------
const char* const RgbModeNames[RGB_MODE_COUNT] = {
  "STATIC",
  "FADING",
  "BREATHING",
  "RPM_COLOR",
  "RPM_WARNING"
};

// ---------------------------------------------------------------------------
// Canali LEDC (PWM hardware ESP32-S3)
// ---------------------------------------------------------------------------
static const int LEDC_FREQ     = 5000;  // Hz
static const int LEDC_RES      = 8;     // bit (0–255)
static const int LEDC_CH_R     = 0;
static const int LEDC_CH_G     = 1;
static const int LEDC_CH_B     = 2;

// ---------------------------------------------------------------------------
// Stato interno (tutto static = privato al file)
// ---------------------------------------------------------------------------
static int _pinR, _pinG, _pinB;
static RgbMode _mode = RGB_STATIC;
static RgbParams _params;

// ---------------------------------------------------------------------------
// Utility: scrivi sui LED tenendo conto di:
//   1. Common anode  → valore invertito
//   2. Luminosità globale (params.brightness)
// ---------------------------------------------------------------------------
static void _writeLed(uint8_t r, uint8_t g, uint8_t b) {
    float scale = _params.brightness / 100.0f;
    uint8_t rS = (uint8_t)(r * scale);
    uint8_t gS = (uint8_t)(g * scale);
    uint8_t bS = (uint8_t)(b * scale);

    ledcWrite(LEDC_CH_R, 255 - rS);
    ledcWrite(LEDC_CH_G, 255 - gS);
    ledcWrite(LEDC_CH_B, 255 - bS);
}

// ---------------------------------------------------------------------------
// Utility: conversione HSV → RGB (h: 0–360, s/v: 0.0–1.0)
// ---------------------------------------------------------------------------
static void _hsvToRgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
    int i = (int)(h / 60.0f) % 6;
    float f = (h / 60.0f) - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - f*s);
    float t = v * (1.0f - (1.0f - f) * s);

    float rf, gf, bf;
    switch (i) {
        case 0: rf = v; gf = t; bf = p; break;
        case 1: rf = q; gf = v; bf = p; break;
        case 2: rf = p; gf = v; bf = t; break;
        case 3: rf = p; gf = q; bf = v; break;
        case 4: rf = t; gf = p; bf = v; break;
        default: rf = v; gf = p; bf = q; break;
    }
    r = (uint8_t)constrain((int)(rf * 255), 0, 255);
    g = (uint8_t)constrain((int)(gf * 255), 0, 255);
    b = (uint8_t)constrain((int)(bf * 255), 0, 255);
}

// ---------------------------------------------------------------------------
// Algoritmo: STATIC
// ---------------------------------------------------------------------------
static void _updateStatic() {
    _writeLed(_params.r, _params.g, _params.b);
}

// ---------------------------------------------------------------------------
// Algoritmo: FADING
// Transizione lineare ciclica tra 7 colori.
// Velocità controllata da params.speed (0–100).
// ---------------------------------------------------------------------------
static void _updateFading() {
    static const uint8_t colors[][3] = {
        {255,   0,   0},   // Rosso
        {255, 128,   0},   // Arancione
        {255, 255,   0},   // Giallo
        {  0, 255,   0},   // Verde
        {  0,   0, 255},   // Blu
        {128,   0, 255},   // Viola
        {255,   0, 255}    // Magenta
    };
    const int NUM_COLORS = sizeof(colors) / sizeof(colors[0]);

    static unsigned long lastUpdate = 0;
    static int           step       = 0;
    static uint8_t       startColor[3] = {colors[0][0], colors[0][1], colors[0][2]};
    static uint8_t       cur[3]        = {colors[0][0], colors[0][1], colors[0][2]};
    static int           curIdx  = 0;
    static int           nextIdx = 1;

    // Mappa speed (0-100) -> delay per step (5 - 200 ms)
    unsigned long stepDelay = map(_params.speed, 0, 100, 200, 5);

    unsigned long now = millis();
    if (now - lastUpdate < stepDelay) return;
    lastUpdate = now;

    if (step == 0) {
        startColor[0] = cur[0];
        startColor[1] = cur[1];
        startColor[2] = cur[2];
    }
    step++;

    for (int i = 0; i < 3; i++) {
        int diff = (int)colors[nextIdx][i] - (int)startColor[i];
        cur[i] = (uint8_t)(startColor[i] + (diff * step) / 100);
    }
    _writeLed(cur[0], cur[1], cur[2]);

    if (step >= 100) {
        step = 0;
        curIdx = nextIdx;
        nextIdx = (nextIdx + 1) % NUM_COLORS;
    }
}

// ---------------------------------------------------------------------------
// Algoritmo: BREATHING
// Fade in-out sinusoidale sul colore base.
// ---------------------------------------------------------------------------
static void _updateBreathing() {
    static unsigned long lastUpdate = 0;
    static float phase = 0.0f;

    // Mappa speed (0-100) -> incremento fase per tick
    float phaseStep = map(_params.speed, 0, 100, 1, 20) / 1000.0f;

    unsigned long now = millis();
    if (now - lastUpdate < 10) return; // tick a 100Hz
    lastUpdate = now;

    phase += phaseStep;
    if (phase > TWO_PI) phase -= TWO_PI;

    // Sinusoide tra 0 e 1 (sempre positiva)
    float intensity = (sinf(phase) + 1.0f) / 2.0f;

    uint8_t r = (uint8_t)(_params.r * intensity);
    uint8_t g = (uint8_t)(_params.g * intensity);
    uint8_t b = (uint8_t)(_params.b * intensity);
    _writeLed(r, g, b);
}

// ---------------------------------------------------------------------------
// Algoritmo: RPM_COLOR
// Hue scala da blu (0 rpm) → verde → giallo → rosso (rpmMax).
// Saturazione e valore fissi. L'effetto è un "termometro" visivo dei giri.
// ---------------------------------------------------------------------------
static void _updateRpmColor() {
    int rpm = constrain(vehicleData.rpm, 0, _params.rpmMax);

    // Hue: 240° (blu) a 0° (rosso) — invertito perché HSV va rosso→blu
    float hue = map(rpm, 0, _params.rpmMax, 240, 0);

    uint8_t r, g, b;
    _hsvToRgb(hue, 1.0f, 1.0f, r, g, b);
    _writeLed(r, g, b);
}

// ---------------------------------------------------------------------------
// Algoritmo: RPM_WARNING
// Sotto soglia: colore base.
// Sopra soglia: lampeggio rosso rapido.
// ---------------------------------------------------------------------------
static void _updateRpmWarning() {
    if (vehicleData.rpm < _params.rpmThreshold) {
        _writeLed(_params.r, _params.g, _params.b);
        return;
    }

    // Lampeggio: 150ms on / 150ms off
    bool blink = (millis() / 150) % 2 == 0;
    if (blink) {
      _writeLed(255, 0, 0);
    } else {
      _writeLed(0, 0, 0);
    }
}

// ===========================================================================
// API pubblica
// ===========================================================================

void rgbInit(int pinR, int pinG, int pinB) {
    _pinR = pinR;
    _pinG = pinG;
    _pinB = pinB;
  
    ledcSetup(LEDC_CH_R, LEDC_FREQ, LEDC_RES);
    ledcSetup(LEDC_CH_G, LEDC_FREQ, LEDC_RES);
    ledcSetup(LEDC_CH_B, LEDC_FREQ, LEDC_RES);
  
    ledcAttachPin(_pinR, LEDC_CH_R);
    ledcAttachPin(_pinG, LEDC_CH_G);
    ledcAttachPin(_pinB, LEDC_CH_B);
  
    rgbOff();
}

void rgbUpdate() {
    switch (_mode) {
      case RGB_STATIC:      _updateStatic();      break;
      case RGB_FADING:      _updateFading();      break;
      case RGB_BREATHING:   _updateBreathing();   break;
      case RGB_RPM_COLOR:   _updateRpmColor();    break;
      case RGB_RPM_WARNING: _updateRpmWarning();  break;
      default: break;
    }
}

void rgbSetMode(RgbMode mode) {
    _mode = mode;
}
  
RgbMode rgbGetMode() {
    return _mode;
}
  
void rgbSetParams(const RgbParams& params) {
    _params = params;
}
  
RgbParams rgbGetParams() {
    return _params;
}
  
void rgbSetColor(uint8_t r, uint8_t g, uint8_t b) {
    _params.r = r;
    _params.g = g;
    _params.b = b;
    _mode = RGB_STATIC;
}
  
void rgbOff() {
    // Common anode: 255 = spento
    ledcWrite(LEDC_CH_R, 255);
    ledcWrite(LEDC_CH_G, 255);
    ledcWrite(LEDC_CH_B, 255);
}  