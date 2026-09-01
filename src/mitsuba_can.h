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

// The bus rate is confirmed at 500 kbit/s, so the sweep stays off: walking
// other rates on a bus whose rate is known only sprays error frames at the
// controller for three quarters of every cycle. The candidate list is kept for
// the case where a different controller shows up; 1 Mbit/s is not in it because
// this bus is known not to run there.
constexpr bool MITSUBA_BAUD_AUTOSCAN = false;
constexpr uint32_t MITSUBA_BAUD_CANDIDATES[] = {500000, 250000, 125000};
constexpr uint8_t MITSUBA_BAUD_CANDIDATE_COUNT =
    sizeof(MITSUBA_BAUD_CANDIDATES) / sizeof(MITSUBA_BAUD_CANDIDATES[0]);

// Long enough for four request-profile switches at each candidate rate.
constexpr uint32_t MITSUBA_BAUD_DWELL_MS = 2000;

// A locked link that goes this long without a frame sweeps again.
constexpr uint32_t MITSUBA_BAUD_RESCAN_MS = 10000;

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

// How many received frames can wait for the main loop to print them.
constexpr uint8_t MITSUBA_RAW_FRAME_SLOTS = 8;

// One frame exactly as it came off CAN1, kept for the serial monitor.
struct MitsubaRawFrame {
  uint32_t id = 0;
  uint32_t timestampMs = 0;
  uint8_t length = 0;
  uint8_t data[8] = {};
  bool extended = false;
  bool accepted = false;
  // True for a frame this board put on the wire, false for one it received.
  bool transmitted = false;
  // 0, 1 or 2 for a decoded response frame, 0xFF for anything else.
  uint8_t frameIndex = 0xFF;
};

// Bus-level counters. These are what separate "the controller is not talking"
// from "it talks and we throw the frames away".
struct MitsubaLinkStats {
  uint32_t frame0 = 0;
  uint32_t frame1 = 0;
  uint32_t frame2 = 0;
  uint32_t otherId = 0;       // right bus, not one of the three response ids
  uint32_t rejected = 0;      // our id, but the decoder refused the payload
  uint32_t requestsSent = 0;
  uint32_t txAccepted = 0;    // requests that got a hardware transmit mailbox
  uint32_t txCompleted = 0;   // requests the controller reported as sent
  uint32_t txQueued = 0;      // requests parked in the library's software queue
  uint32_t rawOverflows = 0;  // frames that arrived while the buffer was full
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

// Snapshot of the bus counters.
MitsubaLinkStats stats();

// Pops one buffered frame for printing. Returns false when none is waiting.
// Call this from the main loop only; it is what keeps Serial out of the ISR.
bool takeRawFrame(MitsubaRawFrame &out);

// Prints what the CAN controller itself sees on the wire: sync state, fault
// confinement and the two error counters, read straight from the peripheral.
// It answers "is anything out there at all" even when no frame ever arrived.
// Main loop only; it prints.
void printBusState();

// True once CAN1 actually started.
bool isReady();

// Result of the boot-time internal loopback test. A pass means the FIFO, the
// filter, the interrupt and the frame handler all work, so a silent panel is a
// wiring or protocol problem rather than a firmware one.
bool loopbackSelfTestPassed();

// True while a request id is known to be producing answers.
bool isProfileLocked();

// The bit rate currently on the wire, and whether it has proven itself.
uint32_t activeBaudRate();
bool isBaudLocked();

// Writes the bit rate to the controller again. The FlexCAN library registers
// every bus object from its constructor and the first begin() re-applies its
// own (still zero) rate to the others, so both buses re-assert after bring-up.
void reapplyBaudRate();

// Milliseconds since that response frame last arrived, 0 if it never has.
// frameIndex is 0, 1 or 2.
uint32_t frameAgeMs(uint8_t frameIndex, uint32_t now);

// "stop", "forward" or "reverse" for the Frame 1 drive action field.
const char *driveActionName(uint8_t driveAction);

}  // namespace mitsuba
