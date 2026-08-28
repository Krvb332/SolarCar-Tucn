#pragma once

// Mitsuba motor controller link for Teensy 4.1.
//
// Sends the periodic Log_Req_RL1 request frame and decodes the three response
// frames into a single telemetry snapshot. Reception is interrupt driven so the
// motor data stays current no matter how long loop() is blocked by the
// bit-banged SoftwareSerial writes to the Nextion panel.
//
// Wiring: CAN1 on Teensy 4.1, pin 22 = CTX1, pin 23 = CRX1, through a CAN
// transceiver, 120 ohm termination at both ends of the bus, common ground with
// the controller.
//
// On ESP32 builds every entry point compiles to a no-op so the esp32dev
// environment keeps building.

#include <stdint.h>

#include "mitsuba_decode.h"

// Values on screen fall back to zero when Frame 0 has been silent this long.
constexpr uint32_t MITSUBA_STALE_MS = 1000;

struct MitsubaTelemetry {
  // Frame 0
  int16_t motorRpm = 0;
  float speedKmh = 0.0f;
  float batteryVoltage = 0.0f;
  float batteryCurrent = 0.0f;   // signed, negative while regenerating
  float batteryPower = 0.0f;     // V * I, keeps the sign of the current
  float fetTemperatureC = 0.0f;
  float motorCurrentPeak = 0.0f;

  // Frame 1
  float throttlePercent = 0.0f;
  float regenVrPercent = 0.0f;
  bool regenActive = false;
  bool powerMode = false;
  uint8_t driveAction = 0;

  // Frame 2
  uint32_t errorFlags = 0;
  uint8_t overheatLevel = 0;

  // Freshness
  uint32_t lastFrame0Ms = 0;
  uint32_t lastFrame1Ms = 0;
  uint32_t lastFrame2Ms = 0;
};

namespace mitsuba {

// Brings up CAN1 at 500 kbit/s and arms the receive interrupt.
void begin();

// Drives the request cadence and the request-profile detection. Must be called
// from loop() before any early return so the controller keeps being polled.
void service(uint32_t now);

// IRQ-safe copy of the latest telemetry. Returns false when no valid frame has
// ever been received.
bool snapshot(MitsubaTelemetry &out);

// True when a Frame 0 arrived recently enough to trust the values on screen.
bool isFresh(uint32_t now);

// The request ID currently in use, for serial diagnostics.
uint32_t activeRequestId();

// Total count of accepted response frames, for serial diagnostics.
uint32_t frameCount();

}  // namespace mitsuba
