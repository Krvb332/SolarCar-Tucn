#include "mitsuba_can.h"

#if defined(ESP32)

// The Teensy FlexCAN peripheral does not exist here. Keep the symbols so
// main.cpp links unchanged in the esp32dev environment.
namespace mitsuba {
void begin() {}
void service(uint32_t) {}
bool snapshot(MitsubaTelemetry &) { return false; }
bool isFresh(uint32_t) { return false; }
uint32_t activeRequestId() { return 0; }
uint32_t frameCount() { return 0; }
}  // namespace mitsuba

#else

#include <Arduino.h>
#include <FlexCAN_T4.h>

namespace {

constexpr uint32_t MITSUBA_REQUEST_INTERVAL_MS = 100;

// While no profile is locked, alternate between the two known request IDs.
constexpr uint32_t MITSUBA_PROFILE_SWITCH_MS = 500;

// Drop back to profile detection after this long without a valid response.
constexpr uint32_t MITSUBA_PROFILE_RELOCK_MS = 2000;

// CAN1 is pin 22 (CTX1) and pin 23 (CRX1) on Teensy 4.1. Those are free: the
// Nextion uses 2 and 3, the ANT BMS uses 11 and 12.
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> mitsubaCan;

// Written by the receive interrupt, read by the main loop under noInterrupts().
volatile MitsubaTelemetry telemetry;
volatile bool haveTelemetry = false;
volatile uint32_t acceptedFrames = 0;

// Request profile state. Only the main loop touches these.
const uint32_t kRequestIds[2] = {MITSUBA_ID_REQUEST_A, MITSUBA_ID_REQUEST_B};
uint8_t requestProfile = 0;
// Which profile actually produced the request that is currently on the wire.
// Responses arrive about a millisecond after a request while profile switches
// happen on a 500 ms boundary, so latching this at send time stops a switch
// that lands between request and response from locking the wrong profile.
uint8_t profileOfLastRequest = 0;
bool profileLocked = false;
uint32_t lastRequestMs = 0;
uint32_t lastProfileSwitchMs = 0;
bool canReady = false;

void storeFrame0(const CAN_message_t &message) {
  MitsubaFrame0 decoded;
  if (!mitsubaDecodeFrame0(message.buf, message.len, decoded)) {
    return;
  }

  telemetry.motorRpm = decoded.rpm;
  telemetry.speedKmh = mitsubaSpeedKmhFromRpm(decoded.rpm);
  telemetry.batteryVoltage = decoded.batteryVoltage;
  telemetry.batteryCurrent = decoded.batteryCurrent;
  telemetry.batteryPower = decoded.batteryVoltage * decoded.batteryCurrent;
  telemetry.fetTemperatureC = decoded.fetTemperatureC;
  telemetry.motorCurrentPeak = decoded.motorCurrentPeak;
  telemetry.lastFrame0Ms = millis();

  haveTelemetry = true;
  ++acceptedFrames;
}

void storeFrame1(const CAN_message_t &message) {
  MitsubaFrame1 decoded;
  if (!mitsubaDecodeFrame1(message.buf, message.len, decoded)) {
    return;
  }

  telemetry.throttlePercent = decoded.throttlePercent;
  telemetry.regenVrPercent = decoded.regenVrPercent;
  telemetry.regenActive = decoded.regenActive;
  telemetry.powerMode = decoded.powerMode;
  telemetry.driveAction = decoded.driveAction;
  telemetry.lastFrame1Ms = millis();

  haveTelemetry = true;
  ++acceptedFrames;
}

void storeFrame2(const CAN_message_t &message) {
  MitsubaFrame2 decoded;
  if (!mitsubaDecodeFrame2(message.buf, message.len, decoded)) {
    return;
  }

  telemetry.errorFlags = decoded.errorFlags;
  telemetry.overheatLevel = decoded.overheatLevel;
  telemetry.lastFrame2Ms = millis();

  haveTelemetry = true;
  ++acceptedFrames;
}

// Called from the FlexCAN receive interrupt, via _mainHandler in mbCallbacks().
// Decoding is pure bit arithmetic, so it is cheap enough to run here, and it
// keeps the values fresh even while the main loop is stuck inside a bit-banged
// Nextion write or the BMS response window.
void onMitsubaFrame(const CAN_message_t &message) {
  if (!message.flags.extended) {
    return;
  }

  switch (message.id) {
    case MITSUBA_ID_FRAME0:
      storeFrame0(message);
      break;
    case MITSUBA_ID_FRAME1:
      storeFrame1(message);
      break;
    case MITSUBA_ID_FRAME2:
      storeFrame2(message);
      break;
    default:
      break;
  }
}

uint32_t lastFrameTimestamp() {
  uint32_t newest = 0;
  noInterrupts();
  const uint32_t f0 = telemetry.lastFrame0Ms;
  const uint32_t f1 = telemetry.lastFrame1Ms;
  const uint32_t f2 = telemetry.lastFrame2Ms;
  interrupts();

  if (f0 > newest) newest = f0;
  if (f1 > newest) newest = f1;
  if (f2 > newest) newest = f2;
  return newest;
}

void sendRequest(uint32_t now) {
  CAN_message_t request;
  request.id = kRequestIds[requestProfile];
  request.flags.extended = 1;
  request.len = 1;
  request.buf[0] = MITSUBA_REQUEST_ALL_FRAMES;

  mitsubaCan.write(request);
  profileOfLastRequest = requestProfile;
  lastRequestMs = now;
}

}  // namespace

namespace mitsuba {

void begin() {
  mitsubaCan.begin();
  mitsubaCan.setBaudRate(MITSUBA_CAN_BAUD);
  mitsubaCan.setMaxMB(16);
  mitsubaCan.enableFIFO();
  // Accept everything and filter in software, so another 500 kbit/s node on the
  // same bus (for example the TPMS receiver) can be added without touching the
  // hardware filters.
  mitsubaCan.setFIFOFilter(ACCEPT_ALL);
  // Arm the handler before enabling the interrupt, so a frame arriving during
  // bring-up cannot land while _mainHandler is still null.
  mitsubaCan.onReceive(onMitsubaFrame);
  mitsubaCan.enableFIFOInterrupt();

  canReady = true;
  lastProfileSwitchMs = millis();

  Serial.print("Mitsuba CAN1 up at ");
  Serial.print(MITSUBA_CAN_BAUD);
  Serial.println(" bit/s (pin 22 TX, pin 23 RX)");
  mitsubaCan.mailboxStatus();
}

void service(uint32_t now) {
  if (!canReady) {
    return;
  }

  // Deliberately no mitsubaCan.events() here. In FlexCAN_T4, struct2queueRx()
  // calls the handler straight from the interrupt while isEventsUsed is false,
  // and the first events() call latches that flag so frames are queued and
  // dispatched from the main loop instead. Calling events() would therefore
  // switch off the very interrupt path this module relies on, and it only pops
  // one frame per call on top of that.

  const uint32_t newestFrame = lastFrameTimestamp();
  const bool everSawFrame = newestFrame != 0;

  if (everSawFrame && (now - newestFrame) < MITSUBA_PROFILE_RELOCK_MS) {
    // A profile that produces answers is a profile worth keeping.
    if (!profileLocked) {
      requestProfile = profileOfLastRequest;
      profileLocked = true;
      Serial.print("Mitsuba request profile locked to 0x");
      Serial.println(kRequestIds[requestProfile], HEX);
    }
  } else if (profileLocked) {
    profileLocked = false;
    lastProfileSwitchMs = now;
    Serial.println("Mitsuba silent for >2s, re-entering request profile detection");
  }

  if (!profileLocked && (now - lastProfileSwitchMs) >= MITSUBA_PROFILE_SWITCH_MS) {
    requestProfile = static_cast<uint8_t>((requestProfile + 1) % 2);
    lastProfileSwitchMs = now;
  }

  if ((now - lastRequestMs) >= MITSUBA_REQUEST_INTERVAL_MS) {
    sendRequest(now);
  }
}

bool snapshot(MitsubaTelemetry &out) {
  noInterrupts();
  const bool valid = haveTelemetry;
  // Copy field by field: telemetry is volatile, so a struct assignment is not
  // available and a memcpy would drop the volatile qualifier.
  out.motorRpm = telemetry.motorRpm;
  out.speedKmh = telemetry.speedKmh;
  out.batteryVoltage = telemetry.batteryVoltage;
  out.batteryCurrent = telemetry.batteryCurrent;
  out.batteryPower = telemetry.batteryPower;
  out.fetTemperatureC = telemetry.fetTemperatureC;
  out.motorCurrentPeak = telemetry.motorCurrentPeak;
  out.throttlePercent = telemetry.throttlePercent;
  out.regenVrPercent = telemetry.regenVrPercent;
  out.regenActive = telemetry.regenActive;
  out.powerMode = telemetry.powerMode;
  out.driveAction = telemetry.driveAction;
  out.errorFlags = telemetry.errorFlags;
  out.overheatLevel = telemetry.overheatLevel;
  out.lastFrame0Ms = telemetry.lastFrame0Ms;
  out.lastFrame1Ms = telemetry.lastFrame1Ms;
  out.lastFrame2Ms = telemetry.lastFrame2Ms;
  interrupts();

  return valid;
}

bool isFresh(uint32_t now) {
  noInterrupts();
  const uint32_t lastFrame0 = telemetry.lastFrame0Ms;
  interrupts();

  return lastFrame0 != 0 && (now - lastFrame0) < MITSUBA_STALE_MS;
}

uint32_t activeRequestId() { return kRequestIds[requestProfile]; }

uint32_t frameCount() {
  noInterrupts();
  const uint32_t count = acceptedFrames;
  interrupts();
  return count;
}

}  // namespace mitsuba

#endif  // ESP32
