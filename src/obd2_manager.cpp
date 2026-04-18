// ---------------------------------------------------------------------------
// obd2_manager.cpp
// Decodifica frame CAN OBD2 (Mode 01) e popola vehicleData.
//
// La logica di decodifica è identica al vecchio codice Arduino con MCP2515
// (funzione decodePIDResponse), adattata per girare sull'ESP32-S3.
//
// I due trasporti (UART simulatore / TWAI produzione) convergono tutti e due
// nella stessa funzione _decodeFrame(id, data[8]) — zero duplicazione.
// ---------------------------------------------------------------------------

#include "obd2_manager.h"
#include "shared_data.h"
#include <Arduino.h>
#include "time_sync_manager.h"

// Il blocco TWAI include l'header solo se serve — a livello di file, non dentro funzioni
#ifdef OBD2_TRANSPORT_TWAI
#include "driver/twai.h"
#endif

// ---------------------------------------------------------------------------
// Parametri TWAI (produzione) — modifica se necessario
// ---------------------------------------------------------------------------
#ifndef OBD2_TWAI_TX
#define OBD2_TWAI_TX 43
#endif
#ifndef OBD2_TWAI_RX
#define OBD2_TWAI_RX 44
#endif

// ID CAN OBD2 standard — uguali al vecchio codice Arduino
#define OBD2_REQUEST_ID  0x7DF
#define OBD2_RESPONSE_ID_MIN 0x7E8
#define OBD2_RESPONSE_ID_MAX 0x7EF

// PID da richiedere in round-robin
static const uint8_t PID_LIST[] = {0x0C, 0x0D, 0x04, 0x11, 0x05};
static const int PID_COUNT = sizeof(PID_LIST);

static bool _ready = false;
static int _pidIdx = 0;
static unsigned long _lastRequest = 0;
#define REQUEST_INTERVAL_MS 100 // un PID ogni 100ms -> ciclo completo 500ms (2Hz)

// ---------------------------------------------------------------------------
// Decodifica frame CAN — logica identica a decodePIDResponse() del vecchio Arduino
//
// Formato risposta OBD2 grezzo:
//   data[0] = numero byte aggiuntivi
//   data[1] = 0x41 (risposta mode 01)
//   data[2] = PID
//   data[3] = byte A
//   data[4] = byte B  (solo dove servono 2 byte, es. RPM)
// ---------------------------------------------------------------------------
static void _decodeFrame (uint32_t id, const uint8_t* data, uint8_t dlc) {
    // Filtra: solo risposte OBD2 (ID 0x7E8-0x7EF) con mode 0x41
    if (id < OBD2_RESPONSE_ID_MIN || id > OBD2_RESPONSE_ID_MAX) return;
    if (dlc < 3) return;
    if (data[1] != 0x41) return;

    uint8_t pid = data[2];
    uint8_t A = (dlc > 3) ? data[3] : 0;
    uint8_t B = (dlc > 4) ? data[4] : 0;

    uint64_t nowUs = timeSyncNowUs();   // timestamp di ricezione

    switch (pid) {
        case 0x0C:  // RPM = (A*256 + B) / 4  — identico al vecchio Arduino
          vehicleData.rpm = ((A * 256) + B) / 4;
          vehicleData.rpmTimestampUs = nowUs;
          break;
    
        case 0x0D:  // Velocità = A  (km/h diretta)
          vehicleData.speed = (float)A;
          vehicleData.speedTimestampUs = nowUs;
          break;
    
        case 0x04:  // Carico motore = A * 100 / 255
          vehicleData.load = (int)((A * 100) / 255);
          vehicleData.loadTimestampUs = nowUs;
          break;
    
        case 0x11:  // Posizione farfalla = A * 100 / 255
          vehicleData.throttle = (float)((A * 100) / 255);
          vehicleData.throttleTimestampUs = nowUs;
          break;
    
        case 0x05:  // Temperatura liquido raffreddamento = A - 40  (°C)
          vehicleData.coolant = (float)A - 40.0f;
          vehicleData.coolantTimestampUs = nowUs;
          break;
    }

    // Per ogni frame utile aggiorna anche il timestamp generale
    vehicleData.lastObdFrameMonoUs = nowUs;
}

// ---------------------------------------------------------------------------
// Costruisce e invia una richiesta OBD2 per un dato PID
// Formato: [02][01][PID][00][00][00][00][00]  — identico al vecchio Arduino
// ---------------------------------------------------------------------------
static void _sendRequest(uint8_t pid){
#ifdef OBD2_TRANSPORT_UART
    // Simulatore PC: manda la richiesta come stringa leggibile
    // Il simulatore Python la ignorerà (risponde in broadcast) ma è utile per debug
    Serial.printf("%03X 02 01 %02X 00 00 00 00 00\n", OBD2_REQUEST_ID, pid);
#elif defined(OBD2_TRANSPORT_TWAI)
    // Produzione: frame CAN via TWAI
    // Incluso solo se OBD2_USE_TWAI è definito, per non linkare il driver TWAI
    // nella build di simulazione
    twai_message_t msg = {};
    msg.identifier     = OBD2_REQUEST_ID;
    msg.data_length_code = 8;
    msg.data[0] = 0x02;
    msg.data[1] = 0x01;
    msg.data[2] = pid;
    // data[3..7] = 0x00 già per zero-init
    twai_transmit(&msg, pdMS_TO_TICKS(10));
#endif
}

// ===========================================================================
// TRASPORTO UART (simulazione PC)
// ===========================================================================
#ifdef OBD2_TRANSPORT_UART

// Buffer ricezione per righe dal simulatore
static String _rxBuf = "";

// Parsa una riga nel formato:
//   "7E8 04 41 0C 1A F8 00 00 00"
// — il formato che il vecchio Arduino avrebbe stampato su seriale
static void _parseUartLine(const String& line) {
    // Separa i token
    uint32_t id = 0;
    uint8_t  data[8] = {};
    uint8_t  dlc = 0;
  
    int pos = 0;
    int tokenIdx = 0;
    String token = "";
  
    // Scorri carattere per carattere
    for (int i = 0; i <= (int)line.length(); i++) {
      char c = (i < (int)line.length()) ? line[i] : ' ';
      if (c == ' ' || c == '\t') {
        if (token.length() > 0) {
          uint32_t val = strtoul(token.c_str(), nullptr, 16);
          if (tokenIdx == 0) {
            id = val;
          } else if (dlc < 8) {
            data[dlc++] = (uint8_t)val;
          }
          tokenIdx++;
          token = "";
        }
      } else {
        token += c;
      }
    }
  
    if (tokenIdx >= 4) {   // almeno ID + 3 byte dati (header + mode + pid)
      _decodeFrame(id, data, dlc);
    }
}

void obd2Init() {
    // Serial0 è già inizializzata nel main con Serial.begin()
    // Non la reinizializziamo — la condividiamo
    _ready = true;
    Serial.println("[OBD2] Modalità UART simulatore — Serial0 condivisa");
}

bool obd2IsReady() {return _ready; }

void obd2Update() {
    // Invia richiesta periodica
    unsigned long now = millis();
    if (now - _lastRequest >= REQUEST_INTERVAL_MS) {
      _lastRequest = now;
      _sendRequest(PID_LIST[_pidIdx]);
      _pidIdx = (_pidIdx + 1) % PID_COUNT;
    }
  
    // Leggi righe in arrivo dal simulatore
    // Nota: Serial.print dei log di sistema e queste letture coesistono
    // perché il simulatore Python filtra le righe che iniziano con "7E"
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') {
        _rxBuf.trim();
        if (_rxBuf.length() > 0) {
          _parseUartLine(_rxBuf);
        }
        _rxBuf = "";
      } else if (c != '\r') {
        _rxBuf += c;
        if (_rxBuf.length() > 64) _rxBuf = "";  // protezione overflow
      }
    }
}
  
#endif  // OBD2_TRANSPORT_UART

// ===========================================================================
// TRASPORTO TWAI (produzione)
// ===========================================================================
#ifdef OBD2_TRANSPORT_TWAI

void obd2Init() {
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)OBD2_TWAI_TX,
                                (gpio_num_t)OBD2_TWAI_RX,
                                TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();  // OBD2 standard
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
      twai_start() != ESP_OK) {
    Serial.println("[OBD2] TWAI init fallita");
    return;
  }
  _ready = true;
  Serial.printf("[OBD2] TWAI avviato TX=%d RX=%d 500kbps\n",
                OBD2_TWAI_TX, OBD2_TWAI_RX);
}

bool obd2IsReady() { return _ready; }

void obd2Update() {
  // Invia richiesta periodica
  unsigned long now = millis();
  if (now - _lastRequest >= REQUEST_INTERVAL_MS) {
    _lastRequest = now;
    _sendRequest(PID_LIST[_pidIdx]);
    _pidIdx = (_pidIdx + 1) % PID_COUNT;
  }

  // Leggi frame in arrivo
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    _decodeFrame(msg.identifier, msg.data, msg.data_length_code);
  }
}

#endif  // OBD2_TRANSPORT_TWAI