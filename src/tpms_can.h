#pragma once

// TPMS receiver link.
//
// The wheel sensors get a bus of their own so the Mitsuba controller keeps CAN1
// to itself and nothing on this board has to arbitrate with it.
//
// Teensy 4.1 wiring: CAN2, pin 0 = CRX2, pin 1 = CTX2, through a CAN
// transceiver, 500 kbit/s, 120 ohm termination at both ends of the bus and a
// common ground with the TPMS receiver.
//
// On ESP32 builds the same frames arrive over the TWAI controller on GPIO 4 and
// GPIO 5, which is how the esp32dev environment already read these sensors.
//
// Frames are received from an interrupt on Teensy, so nothing in this module
// prints. Diagnostics are buffered here and drained by the main loop through
// takeRawFrame(), which is the only safe place to touch Serial.

#include <stdint.h>

// SAE J1939 broadcast the receiver uses for every wheel.
constexpr uint32_t TPMS_CAN_MESSAGE_ID = 0x18FEF433;
constexpr uint32_t TPMS_CAN_BAUD = 500000;

// Wheel ids run 1..4 in byte 0 of the frame. Slot 0 stays unused so a wheel id
// off the bus can index the table directly.
constexpr uint8_t TPMS_WHEEL_COUNT = 5;

// A wheel drops back to zero on the panel after this long without a frame.
constexpr uint32_t TPMS_SENSOR_TIMEOUT_MS = 2000;

// How many received frames can wait for the main loop to print them.
constexpr uint8_t TPMS_RAW_FRAME_SLOTS = 8;

// While no frame at all has been received, the link sweeps these bit rates.
// The receiver is a talker, not a responder, so the sweep can listen without
// ever driving the wire. Set TPMS_BAUD_AUTOSCAN false once the rate is known.
constexpr bool TPMS_BAUD_AUTOSCAN = true;
constexpr uint32_t TPMS_BAUD_CANDIDATES[] = {500000, 250000, 125000, 1000000};
constexpr uint8_t TPMS_BAUD_CANDIDATE_COUNT =
    sizeof(TPMS_BAUD_CANDIDATES) / sizeof(TPMS_BAUD_CANDIDATES[0]);

// Sensors report every second or so while the car moves, and far more rarely
// once it is parked, so each candidate needs a generous window.
constexpr uint32_t TPMS_BAUD_DWELL_MS = 4000;

struct TpmsWheel {
  float pressureBar = 0.0f;
  int16_t temperatureC = 0;
  float batteryV = 0.0f;
  bool leakingAir = false;
  bool extremeTemperature = false;
  uint32_t lastMessageMs = 0;
};

// One frame exactly as it came off the bus, kept for the serial monitor.
struct TpmsRawFrame {
  uint32_t id = 0;
  uint32_t timestampMs = 0;
  uint8_t length = 0;
  uint8_t data[8] = {};
  bool extended = false;
  bool accepted = false;
};

// Bus-level counters, all of them useful when the panel shows nothing.
struct TpmsLinkStats {
  uint32_t accepted = 0;      // decoded into a wheel
  uint32_t otherId = 0;       // right bus, wrong message id
  uint32_t malformed = 0;     // TPMS id but short frame or wheel id out of range
  uint32_t rawOverflows = 0;  // frames received while the print buffer was full
};

namespace tpms {

// Brings up the receiver link and arms reception.
void begin();

// Pumps the link on builds that need polling. Safe to call from every loop().
void service(uint32_t now);

// Copy of the latest reading for one wheel. Returns false for an unknown wheel
// id or for a sensor that has never been heard.
bool snapshot(uint8_t wheelId, TpmsWheel &out);

// True when that wheel reported recently enough to trust the value on screen.
bool isFresh(uint8_t wheelId, uint32_t now);

// Milliseconds since that wheel last reported, or 0 if it never has.
uint32_t wheelAgeMs(uint8_t wheelId, uint32_t now);

// How many frames this wheel has contributed since boot.
uint32_t wheelFrameCount(uint8_t wheelId);

// Total count of accepted TPMS frames, for serial diagnostics.
uint32_t frameCount();

// Snapshot of the bus counters.
TpmsLinkStats stats();

// Pops one buffered frame for printing. Returns false when none is waiting.
// Call this from the main loop only; it is what keeps Serial out of the ISR.
bool takeRawFrame(TpmsRawFrame &out);

// "front-left", "front-right", "rear-right", "rear-left" for wheel ids 1..4.
const char *wheelName(uint8_t wheelId);

// Prints what the CAN controller itself sees on the wire: sync state, fault
// confinement and the two error counters. This is read straight from the
// peripheral, so it answers "is anything out there at all" even when not one
// frame has ever been accepted. Main loop only; it prints.
void printBusState();

// The bit rate currently on the wire, and whether a frame has proven it.
uint32_t activeBaudRate();
bool isBaudLocked();

// Writes the bit rate to the controller again. The FlexCAN library registers
// every bus object from its constructor and the first begin() re-applies its
// own (still zero) rate to the others, so both buses re-assert after bring-up.
void reapplyBaudRate();

// True once the CAN controller actually started.
bool isReady();

// Result of the boot-time internal loopback test. A pass means the receive path
// on this board works, so silence is a wiring or bit rate problem.
bool loopbackSelfTestPassed();

}  // namespace tpms
