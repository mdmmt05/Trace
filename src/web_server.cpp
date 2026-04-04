// ---------------------------------------------------------------------------
// web_server.cpp
// ---------------------------------------------------------------------------

#include "web_server.h"
#include "rgb_controller.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// Credenziali Access Point
// ---------------------------------------------------------------------------
static const char* AP_SSID = "Hyundai Airlines";
static const char* AP_PASSWORD = "hyundai10";
static const IPAddress AP_IP(192, 168, 4, 1);

static WebServer server(80);

// ---------------------------------------------------------------------------
// HTML / CSS / JS della UI — tutto embedded come raw string
// Stile: dark cockpit, HUD militare, mobile-first, minimal e immediato
// Font: Rajdhani (display, carattere tecnico-militare) + Share Tech Mono (valori)
// ---------------------------------------------------------------------------
static const char UI_HTML[] PROGMEM = R"rawhtml(
    <!DOCTYPE html>
    <html lang="it">
    <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0">
    <title>COCKPIT // RGB</title>

    <style>
      :root {
        --bg:        #080a0d;
        --bg2:       #0d1117;
        --bg3:       #111820;
        --border:    #1e2d3d;
        --accent:    #00c8ff;
        --accent2:   #ff6b00;
        --text:      #c9d8e8;
        --text-dim:  #4a6070;
        --danger:    #ff2a2a;
        --ok:        #00ff88;
        --radius:    4px;
        --preview-r: 255;
        --preview-g: 255;
        --preview-b: 255;
      }
    
      *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
    
      html, body {
        background: var(--bg);
        color: var(--text);
        font-family: ui-rounded, -apple-system, 'Segoe UI', sans-serif;
        font-size: 16px;
        min-height: 100vh;
        -webkit-tap-highlight-color: transparent;
        overflow-x: hidden;
      }
    
      /* Scanline overlay per effetto CRT sottile */
      body::before {
        content: '';
        position: fixed;
        inset: 0;
        background: repeating-linear-gradient(
          0deg,
          transparent,
          transparent 2px,
          rgba(0,0,0,0.07) 2px,
          rgba(0,0,0,0.07) 4px
        );
        pointer-events: none;
        z-index: 9999;
      }
    
      /* ── HEADER ── */
      header {
        display: flex;
        align-items: center;
        justify-content: space-between;
        padding: 14px 20px;
        border-bottom: 1px solid var(--border);
        background: var(--bg2);
        position: sticky;
        top: 0;
        z-index: 100;
      }
    
      .logo {
        font-size: 1.1rem;
        font-weight: 700;
        letter-spacing: 0.15em;
        color: var(--accent);
        text-transform: uppercase;
      }
      .logo span { color: var(--text-dim); font-weight: 400; }
    
      .status-dot {
        width: 8px; height: 8px;
        border-radius: 50%;
        background: var(--ok);
        box-shadow: 0 0 8px var(--ok);
        animation: pulse 2s ease-in-out infinite;
      }
      @keyframes pulse {
        0%,100% { opacity: 1; }
        50%      { opacity: 0.4; }
      }
    
      /* ── LAYOUT ── */
      main {
        max-width: 480px;
        margin: 0 auto;
        padding: 20px 16px 40px;
        display: flex;
        flex-direction: column;
        gap: 16px;
      }
    
      /* ── CARD ── */
      .card {
        background: var(--bg2);
        border: 1px solid var(--border);
        border-radius: var(--radius);
        overflow: hidden;
      }
    
      .card-header {
        display: flex;
        align-items: center;
        gap: 10px;
        padding: 10px 16px;
        border-bottom: 1px solid var(--border);
        background: var(--bg3);
      }
      .card-header .label {
        font-size: 0.7rem;
        font-weight: 700;
        letter-spacing: 0.2em;
        text-transform: uppercase;
        color: var(--text-dim);
      }
      .card-header .tag {
        margin-left: auto;
        font-family: ui-monospace, 'SF Mono', Consolas, monospace;
        font-size: 0.65rem;
        color: var(--accent);
        border: 1px solid var(--accent);
        padding: 1px 6px;
        border-radius: 2px;
        opacity: 0.7;
      }
    
      .card-body { padding: 16px; }
    
      /* ── PREVIEW COLORE ── */
      .color-preview {
        height: 64px;
        border-radius: var(--radius);
        border: 1px solid var(--border);
        margin-bottom: 16px;
        transition: background 0.1s;
        background: rgb(var(--preview-r), var(--preview-g), var(--preview-b));
        position: relative;
        overflow: hidden;
      }
      .color-preview::after {
        content: '';
        position: absolute;
        inset: 0;
        background: linear-gradient(135deg, rgba(255,255,255,0.06) 0%, transparent 60%);
      }
    
      /* ── COLOR PICKER (tavoletta HSV) ── */
      .picker-wrap {
        position: relative;
        width: 100%;
        aspect-ratio: 1 / 0.6;
        border-radius: var(--radius);
        overflow: hidden;
        cursor: crosshair;
        margin-bottom: 12px;
        touch-action: none;
        border: 1px solid var(--border);
      }
      #pickerCanvas { width: 100%; height: 100%; display: block; }
    
      .picker-cursor {
        position: absolute;
        width: 16px; height: 16px;
        border-radius: 50%;
        border: 2px solid #fff;
        box-shadow: 0 0 0 1px rgba(0,0,0,0.5);
        transform: translate(-50%, -50%);
        pointer-events: none;
        top: 50%; left: 50%;
        transition: top 0.05s, left 0.05s;
      }
    
      /* Hue slider */
      .hue-slider-wrap {
        position: relative;
        margin-bottom: 16px;
      }
      #hueSlider {
        -webkit-appearance: none;
        appearance: none;
        width: 100%;
        height: 20px;
        border-radius: var(--radius);
        background: linear-gradient(to right,
          hsl(0,100%,50%), hsl(30,100%,50%), hsl(60,100%,50%),
          hsl(120,100%,50%), hsl(180,100%,50%), hsl(240,100%,50%),
          hsl(300,100%,50%), hsl(360,100%,50%));
        outline: none;
        border: 1px solid var(--border);
        cursor: pointer;
      }
      #hueSlider::-webkit-slider-thumb {
        -webkit-appearance: none;
        width: 20px; height: 20px;
        border-radius: 50%;
        background: #fff;
        border: 2px solid var(--bg);
        box-shadow: 0 0 4px rgba(0,0,0,0.6);
        cursor: pointer;
      }
    
      /* ── INPUTS RGB PRECISI ── */
      .rgb-inputs {
        display: grid;
        grid-template-columns: 1fr 1fr 1fr;
        gap: 8px;
      }
      .rgb-field { display: flex; flex-direction: column; gap: 4px; }
      .rgb-field label {
        font-size: 0.65rem;
        font-weight: 700;
        letter-spacing: 0.15em;
        text-transform: uppercase;
      }
      .rgb-field.r label { color: #ff4444; }
      .rgb-field.g label { color: #44ff88; }
      .rgb-field.b label { color: #4488ff; }
    
      .rgb-field input {
        background: var(--bg3);
        border: 1px solid var(--border);
        color: var(--text);
        font-family: ui-monospace, 'SF Mono', Consolas, monospace;
        font-size: 1rem;
        padding: 8px;
        border-radius: var(--radius);
        text-align: center;
        width: 100%;
        -webkit-appearance: none;
        appearance: none;
      }
      .rgb-field input:focus {
        outline: none;
        border-color: var(--accent);
        box-shadow: 0 0 0 2px rgba(0,200,255,0.15);
      }
    
      /* ── BOTTONE APPLICA ── */
      .btn-apply {
        width: 100%;
        margin-top: 14px;
        padding: 13px;
        background: transparent;
        border: 1px solid var(--accent);
        color: var(--accent);
        font-family: ui-rounded, -apple-system, 'Segoe UI', sans-serif;
        font-size: 0.85rem;
        font-weight: 700;
        letter-spacing: 0.2em;
        text-transform: uppercase;
        border-radius: var(--radius);
        cursor: pointer;
        transition: background 0.15s, box-shadow 0.15s;
        position: relative;
        overflow: hidden;
      }
      .btn-apply:active {
        background: rgba(0,200,255,0.12);
        box-shadow: 0 0 16px rgba(0,200,255,0.25);
      }
    
      /* ── MODALITÀ ── */
      .mode-grid {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 8px;
      }
      .mode-btn {
        padding: 14px 8px;
        background: var(--bg3);
        border: 1px solid var(--border);
        color: var(--text-dim);
        font-family: ui-rounded, -apple-system, 'Segoe UI', sans-serif;
        font-size: 0.8rem;
        font-weight: 600;
        letter-spacing: 0.1em;
        text-transform: uppercase;
        border-radius: var(--radius);
        cursor: pointer;
        transition: all 0.15s;
        text-align: center;
        position: relative;
      }
      .mode-btn .mode-icon {
        display: block;
        font-size: 1.4rem;
        margin-bottom: 4px;
      }
      .mode-btn:active { opacity: 0.7; }
      .mode-btn.active {
        border-color: var(--accent);
        color: var(--accent);
        background: rgba(0,200,255,0.07);
        box-shadow: 0 0 12px rgba(0,200,255,0.15);
      }
      .mode-btn.active-warn {
        border-color: var(--accent2);
        color: var(--accent2);
        background: rgba(255,107,0,0.07);
        box-shadow: 0 0 12px rgba(255,107,0,0.15);
      }
    
      /* ── SLIDERS PARAMETRI ── */
      .param-row {
        display: flex;
        flex-direction: column;
        gap: 6px;
        margin-bottom: 14px;
      }
      .param-row:last-child { margin-bottom: 0; }
    
      .param-label {
        display: flex;
        justify-content: space-between;
        align-items: baseline;
      }
      .param-label span:first-child {
        font-size: 0.7rem;
        font-weight: 700;
        letter-spacing: 0.15em;
        text-transform: uppercase;
        color: var(--text-dim);
      }
      .param-label .val {
        font-family: ui-monospace, 'SF Mono', Consolas, monospace;
        font-size: 0.8rem;
        color: var(--accent);
      }
    
      input[type=range] {
        -webkit-appearance: none;
        appearance: none;
        width: 100%;
        height: 4px;
        background: var(--border);
        border-radius: 2px;
        outline: none;
        cursor: pointer;
      }
      input[type=range]::-webkit-slider-thumb {
        -webkit-appearance: none;
        width: 18px; height: 18px;
        border-radius: 50%;
        background: var(--accent);
        border: 2px solid var(--bg);
        box-shadow: 0 0 6px rgba(0,200,255,0.4);
        cursor: pointer;
      }
    
      /* Slider numero (rpm threshold / rpmMax) */
      .number-input {
        background: var(--bg3);
        border: 1px solid var(--border);
        color: var(--text);
        font-family: ui-monospace, 'SF Mono', Consolas, monospace;
        font-size: 0.9rem;
        padding: 8px 10px;
        border-radius: var(--radius);
        width: 100%;
        -webkit-appearance: none;
        appearance: none;
      }
      .number-input:focus {
        outline: none;
        border-color: var(--accent);
      }
    
      /* ── FOOTER ── */
      footer {
        text-align: center;
        padding: 16px;
        font-size: 0.65rem;
        letter-spacing: 0.15em;
        color: var(--text-dim);
        text-transform: uppercase;
        border-top: 1px solid var(--border);
      }
    
      /* ── TOAST FEEDBACK ── */
      #toast {
        position: fixed;
        bottom: 24px;
        left: 50%;
        transform: translateX(-50%) translateY(20px);
        background: var(--bg3);
        border: 1px solid var(--accent);
        color: var(--accent);
        font-family: ui-monospace, 'SF Mono', Consolas, monospace;
        font-size: 0.75rem;
        letter-spacing: 0.1em;
        padding: 8px 20px;
        border-radius: var(--radius);
        opacity: 0;
        transition: opacity 0.2s, transform 0.2s;
        pointer-events: none;
        z-index: 1000;
        white-space: nowrap;
      }
      #toast.show {
        opacity: 1;
        transform: translateX(-50%) translateY(0);
      }
    </style>
    </head>
    <body>
    
    <header>
      <div class="logo">COCKPIT <span>// RGB</span></div>
      <div class="status-dot" id="statusDot"></div>
    </header>
    
    <main>
    
      <!-- ── SEZIONE COLORE ── -->
      <div class="card">
        <div class="card-header">
          <span class="label">Colore</span>
          <span class="tag" id="hexTag">#FFFFFF</span>
        </div>
        <div class="card-body">
    
          <div class="color-preview" id="colorPreview"></div>
    
          <!-- Tavoletta HSV -->
          <div class="picker-wrap" id="pickerWrap">
            <canvas id="pickerCanvas"></canvas>
            <div class="picker-cursor" id="pickerCursor"></div>
          </div>
    
          <!-- Slider hue -->
          <input type="range" id="hueSlider" min="0" max="360" value="0">
    
          <!-- Input RGB precisi -->
          <div class="rgb-inputs" style="margin-top:14px">
            <div class="rgb-field r">
              <label>R</label>
              <input type="number" id="rIn" min="0" max="255" value="255">
            </div>
            <div class="rgb-field g">
              <label>G</label>
              <input type="number" id="gIn" min="0" max="255" value="255">
            </div>
            <div class="rgb-field b">
              <label>B</label>
              <input type="number" id="bIn" min="0" max="255" value="0">
            </div>
          </div>
    
          <button class="btn-apply" id="applyColor">▶ APPLICA COLORE</button>
        </div>
      </div>
    
      <!-- ── SEZIONE MODALITÀ ── -->
      <div class="card">
        <div class="card-header">
          <span class="label">Modalità</span>
        </div>
        <div class="card-body">
          <div class="mode-grid">
            <button class="mode-btn" data-mode="STATIC">
              <span class="mode-icon">▧</span>Fisso
            </button>
            <button class="mode-btn" data-mode="FADING">
              <span class="mode-icon">◈</span>Fading
            </button>
            <button class="mode-btn" data-mode="BREATHING">
              <span class="mode-icon">◉</span>Respiro
            </button>
            <button class="mode-btn" data-mode="RPM_COLOR">
              <span class="mode-icon">◎</span>RPM Colore
            </button>
            <button class="mode-btn" data-mode="RPM_WARNING" style="grid-column:span 2">
              <span class="mode-icon">⚠</span>RPM Warning
            </button>
          </div>
        </div>
      </div>
    
      <!-- ── SEZIONE PARAMETRI ── -->
      <div class="card">
        <div class="card-header">
          <span class="label">Parametri</span>
        </div>
        <div class="card-body">
    
          <div class="param-row">
            <div class="param-label">
              <span>Velocità</span>
              <span class="val" id="speedVal">50</span>
            </div>
            <input type="range" id="speedSlider" min="0" max="100" value="50">
          </div>
    
          <div class="param-row">
            <div class="param-label">
              <span>Luminosità</span>
              <span class="val" id="brightnessVal">100%</span>
            </div>
            <input type="range" id="brightnessSlider" min="0" max="100" value="100">
          </div>
    
          <div class="param-row">
            <div class="param-label">
              <span>Soglia RPM Warning</span>
            </div>
            <input type="number" class="number-input" id="rpmThreshold" min="0" max="15000" value="6000" step="100">
          </div>
    
          <div class="param-row">
            <div class="param-label">
              <span>RPM Max (scala colore)</span>
            </div>
            <input type="number" class="number-input" id="rpmMax" min="1000" max="15000" value="8000" step="100">
          </div>
    
          <button class="btn-apply" id="applyParams">▶ APPLICA PARAMETRI</button>
        </div>
      </div>
    
    </main>
    
    <footer>ESP32-S3 · COCKPIT RGB v1.0</footer>
    <div id="toast">OK</div>
    
    <script>
    // ── STATE ──────────────────────────────────────────────────────────────────
    let hue = 0, sat = 1.0, val = 1.0;
    let currentMode = 'STATIC';
    
    // ── UTILS ──────────────────────────────────────────────────────────────────
    function hsvToRgb(h, s, v) {
      let r, g, b;
      const i = Math.floor(h / 60) % 6;
      const f = h / 60 - Math.floor(h / 60);
      const p = v * (1 - s), q = v * (1 - f * s), t = v * (1 - (1 - f) * s);
      switch(i) {
        case 0: r=v;g=t;b=p; break; case 1: r=q;g=v;b=p; break;
        case 2: r=p;g=v;b=t; break; case 3: r=p;g=q;b=v; break;
        case 4: r=t;g=p;b=v; break; default: r=v;g=p;b=q;
      }
      return [Math.round(r*255), Math.round(g*255), Math.round(b*255)];
    }
    
    function rgbToHex(r, g, b) {
      return '#' + [r,g,b].map(x => x.toString(16).padStart(2,'0').toUpperCase()).join('');
    }
    
    function showToast(msg, isErr) {
      const t = document.getElementById('toast');
      t.textContent = msg;
      t.style.borderColor = isErr ? 'var(--danger)' : 'var(--ok)';
      t.style.color = isErr ? 'var(--danger)' : 'var(--ok)';
      t.classList.add('show');
      setTimeout(() => t.classList.remove('show'), 1800);
    }
    
    // ── COLOR PICKER CANVAS ────────────────────────────────────────────────────
    const canvas = document.getElementById('pickerCanvas');
    const ctx = canvas.getContext('2d');
    const cursor = document.getElementById('pickerCursor');
    const wrap = document.getElementById('pickerWrap');
    
    function drawPicker() {
      const w = canvas.width, h = canvas.height;
      // Gradiente bianco→saturato (orizzontale)
      const gradS = ctx.createLinearGradient(0, 0, w, 0);
      gradS.addColorStop(0, `hsl(${hue},0%,100%)`);
      gradS.addColorStop(1, `hsl(${hue},100%,50%)`);
      ctx.fillStyle = gradS;
      ctx.fillRect(0, 0, w, h);
      // Gradiente trasparente→nero (verticale)
      const gradV = ctx.createLinearGradient(0, 0, 0, h);
      gradV.addColorStop(0, 'rgba(0,0,0,0)');
      gradV.addColorStop(1, 'rgba(0,0,0,1)');
      ctx.fillStyle = gradV;
      ctx.fillRect(0, 0, w, h);
    }
    
    function resizePicker() {
      canvas.width  = wrap.clientWidth;
      canvas.height = wrap.clientHeight;
      drawPicker();
      updateCursorPos();
    }
    
    function updateCursorPos() {
      cursor.style.left = (sat * canvas.width)  + 'px';
      cursor.style.top  = ((1 - val) * canvas.height) + 'px';
    }
    
    function pickAt(x, y) {
      const rect = wrap.getBoundingClientRect();
      sat = Math.max(0, Math.min(1, (x - rect.left) / rect.width));
      val = Math.max(0, Math.min(1, 1 - (y - rect.top) / rect.height));
      updateCursorPos();
      syncFromHSV();
    }
    
    wrap.addEventListener('mousedown', e => { pickAt(e.clientX, e.clientY); });
    wrap.addEventListener('mousemove', e => { if (e.buttons) pickAt(e.clientX, e.clientY); });
    wrap.addEventListener('touchstart', e => { e.preventDefault(); pickAt(e.touches[0].clientX, e.touches[0].clientY); }, {passive:false});
    wrap.addEventListener('touchmove',  e => { e.preventDefault(); pickAt(e.touches[0].clientX, e.touches[0].clientY); }, {passive:false});
    
    // ── HUE SLIDER ─────────────────────────────────────────────────────────────
    document.getElementById('hueSlider').addEventListener('input', function() {
      hue = parseInt(this.value);
      drawPicker();
      syncFromHSV();
    });
    
    // ── SYNC FUNCTIONS ─────────────────────────────────────────────────────────
    function syncFromHSV() {
      const [r, g, b] = hsvToRgb(hue, sat, val);
      document.getElementById('rIn').value = r;
      document.getElementById('gIn').value = g;
      document.getElementById('bIn').value = b;
      updatePreview(r, g, b);
    }
    
    function syncFromRGB() {
      const r = clamp(parseInt(document.getElementById('rIn').value)||0);
      const g = clamp(parseInt(document.getElementById('gIn').value)||0);
      const b = clamp(parseInt(document.getElementById('bIn').value)||0);
      // Converti RGB→HSV per aggiornare picker e slider
      const max = Math.max(r,g,b)/255, min = Math.min(r,g,b)/255;
      val = max;
      sat = max === 0 ? 0 : (max - min) / max;
      if (max === min) { hue = 0; }
      else {
        const d = (max - min);
        const rm=r/255,gm=g/255,bm=b/255;
        switch(max) {
          case rm: hue = 60*((gm-bm)/d % 6); break;
          case gm: hue = 60*((bm-rm)/d + 2); break;
          default: hue = 60*((rm-gm)/d + 4);
        }
        if (hue < 0) hue += 360;
      }
      document.getElementById('hueSlider').value = Math.round(hue);
      drawPicker();
      updateCursorPos();
      updatePreview(r, g, b);
    }
    
    function clamp(v) { return Math.max(0, Math.min(255, v)); }
    
    function updatePreview(r, g, b) {
      const hex = rgbToHex(r, g, b);
      document.getElementById('colorPreview').style.background = `rgb(${r},${g},${b})`;
      document.getElementById('hexTag').textContent = hex;
    }
    
    ['rIn','gIn','bIn'].forEach(id => {
      document.getElementById(id).addEventListener('input', syncFromRGB);
    });
    
    // ── APPLICA COLORE ─────────────────────────────────────────────────────────
    document.getElementById('applyColor').addEventListener('click', async () => {
      const r = clamp(parseInt(document.getElementById('rIn').value)||0);
      const g = clamp(parseInt(document.getElementById('gIn').value)||0);
      const b = clamp(parseInt(document.getElementById('bIn').value)||0);
      try {
        const res = await fetch('/api/color', {
          method: 'POST',
          headers: {'Content-Type':'application/json'},
          body: JSON.stringify({r, g, b})
        });
        if (res.ok) {
          showToast('COLORE APPLICATO');
          setActiveMode('STATIC');
        } else { showToast('ERRORE', true); }
      } catch(e) { showToast('CONNESSIONE PERSA', true); }
    });
    
    // ── MODALITÀ ───────────────────────────────────────────────────────────────
    function setActiveMode(mode) {
      currentMode = mode;
      document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.remove('active', 'active-warn');
        if (btn.dataset.mode === mode) {
          btn.classList.add(mode === 'RPM_WARNING' ? 'active-warn' : 'active');
        }
      });
    }
    
    document.querySelectorAll('.mode-btn').forEach(btn => {
      btn.addEventListener('click', async () => {
        const mode = btn.dataset.mode;
        try {
          const res = await fetch('/api/mode', {
            method: 'POST',
            headers: {'Content-Type':'application/json'},
            body: JSON.stringify({mode})
          });
          if (res.ok) { showToast('MODALITÀ: ' + mode); setActiveMode(mode); }
          else { showToast('ERRORE', true); }
        } catch(e) { showToast('CONNESSIONE PERSA', true); }
      });
    });
    
    // ── PARAMETRI SLIDERS ─────────────────────────────────────────────────────
    document.getElementById('speedSlider').addEventListener('input', function() {
      document.getElementById('speedVal').textContent = this.value;
    });
    document.getElementById('brightnessSlider').addEventListener('input', function() {
      document.getElementById('brightnessVal').textContent = this.value + '%';
    });
    
    document.getElementById('applyParams').addEventListener('click', async () => {
      const params = {
        speed:        parseInt(document.getElementById('speedSlider').value),
        brightness:   parseInt(document.getElementById('brightnessSlider').value),
        rpmThreshold: parseInt(document.getElementById('rpmThreshold').value),
        rpmMax:       parseInt(document.getElementById('rpmMax').value)
      };
      try {
        const res = await fetch('/api/params', {
          method: 'POST',
          headers: {'Content-Type':'application/json'},
          body: JSON.stringify(params)
        });
        if (res.ok) { showToast('PARAMETRI SALVATI'); }
        else { showToast('ERRORE', true); }
      } catch(e) { showToast('CONNESSIONE PERSA', true); }
    });
    
    // ── CARICA STATO INIZIALE ──────────────────────────────────────────────────
    async function loadStatus() {
      try {
        const res = await fetch('/api/status');
        if (!res.ok) return;
        const d = await res.json();
        document.getElementById('rIn').value = d.r;
        document.getElementById('gIn').value = d.g;
        document.getElementById('bIn').value = d.b;
        document.getElementById('speedSlider').value = d.speed;
        document.getElementById('speedVal').textContent = d.speed;
        document.getElementById('brightnessSlider').value = d.brightness;
        document.getElementById('brightnessVal').textContent = d.brightness + '%';
        document.getElementById('rpmThreshold').value = d.rpmThreshold;
        document.getElementById('rpmMax').value = d.rpmMax;
        syncFromRGB();
        setActiveMode(d.mode);
      } catch(e) {}
    }
    
    // ── INIT ───────────────────────────────────────────────────────────────────
    window.addEventListener('resize', resizePicker);
    resizePicker();
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
    doc["rpmMax"] = p.rpmMax;

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
    RgbMode newMode = RGB_STATIC;
    bool found = false;
    for (int i = 0; i < RGB_MODE_COUNT; i++) {
      if (strcmp(modeStr, RgbModeNames[i]) == 0) {
        newMode = (RgbMode)i;
        found = true;
        break;
      }
    }
    if (!found) { server.send(400, "application/json", "{\"error\":\"unknown mode\"}"); return; }
  
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
    if (doc["rpmMax"].is<int>())       p.rpmMax       = constrain((int)doc["rpmMax"],       1, 15000);
    rgbSetParams(p);

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
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
