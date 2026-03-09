#pragma once

// ---------------------------------------------------------------------------
// obd2_manager.h
// Riceve frame CAN OBD2 e popola vehicleData.
//
// Modalità di trasporto (selezionabile a compile time):
//   OBD2_TRANSPORT_UART  — frame via Serial0 (USB nativa) dal simulatore PC
//   OBD2_TRANSPORT_TWAI  — frame via CAN bus hardware ESP32-S3 (produzione)
//
// Formato frame sul canale UART (simulatore):
//   "<ID_HEX> <B0> <B1> <B2> <B3> <B4> <B5> <B6> <B7>\n"
//   es: "7E8 04 41 0C 1A F8 00 00 00\n"
//   — identico a come il vecchio Arduino riceveva dal MCP2515 e stampava
//
// Decodifica PID supportati (Mode 01):
//   0x0C  RPM
//   0x0D  Velocità (km/h)
//   0x04  Carico motore (%)
//   0x11  Posizione farfalla (%)
//   0x05  Temperatura liquido raffreddamento (°C)
// Flusso:
//   1. obd2Init()    — inizializza UART e sequenza AT di setup
//   2. obd2Update()  — chiamare nel loop(); interroga un PID per volta
//                      in round-robin, non bloccante
//   3. obd2IsReady() — true dopo che la sequenza AT iniziale è completata
// ---------------------------------------------------------------------------

// Seleziona trasporto: definisci OBD2_USE_TWAI nel build per la produzione,
// altrimenti usa UART (Serial0) per la simulazione.
#ifndef OBD2_USE_TWAI
    #define OBD2_TRANSPORT_UART
#else
    #define OBD2_TRANSPORT_TWAI
#endif

// Inizializza UART e avvia handshake AT con l'ELM327.
void obd2Init();

// Avanza la macchina a stati — chiamare ogni loop() senza delay.
void obd2Update();

// True quando l'init AT è completato e i dati sono affidabili.
bool obd2IsReady();