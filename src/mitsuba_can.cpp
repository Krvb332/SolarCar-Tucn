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
MitsubaLinkStats stats() { return MitsubaLinkStats(); }
bool takeRawFrame(MitsubaRawFrame &) { return false; }
void printBusState() {}
bool isReady() { return false; }
bool isProfileLocked() { return false; }
bool loopbackSelfTestPassed() { return false; }
uint32_t activeBaudRate() { return 0; }
bool isBaudLocked() { return false; }
void reapplyBaudRate() {}
uint32_t frameAgeMs(uint8_t, uint32_t) { return 0; }
const char *driveActionName(uint8_t) { return "unknown"; }
}  // namespace mitsuba

#else

#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "flexcan_diag.h"

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

// Diagnostics, also written from the interrupt. Nothing here ever prints; the
// main loop drains the ring through takeRawFrame().
volatile uint32_t frame0Count = 0;
volatile uint32_t frame1Count = 0;
volatile uint32_t frame2Count = 0;
volatile uint32_t otherIdFrames = 0;
volatile uint32_t rejectedFrames = 0;
volatile uint32_t rawOverflows = 0;
uint32_t requestsSent = 0;

// Single producer (the receive interrupt), single consumer (the main loop).
volatile MitsubaRawFrame rawFrames[MITSUBA_RAW_FRAME_SLOTS];
volatile uint8_t rawHead = 0;
volatile uint8_t rawTail = 0;

// Keeps every frame that reached CAN1, wanted or not, so the serial monitor can
// tell a silent controller apart from one answering on ids we do not expect.
void bufferRawFrame(const CAN_message_t &message, bool accepted, uint8_t frameIndex,
                    uint32_t timestampMs, bool transmitted) {
  const uint8_t next = static_cast<uint8_t>((rawHead + 1) % MITSUBA_RAW_FRAME_SLOTS);
  if (next == rawTail) {
    ++rawOverflows;
    return;
  }

  volatile MitsubaRawFrame &slot = rawFrames[rawHead];
  slot.id = message.id;
  slot.timestampMs = timestampMs;
  slot.length = message.len > 8 ? 8 : message.len;
  slot.extended = message.flags.extended != 0;
  slot.accepted = accepted;
  slot.transmitted = transmitted;
  slot.frameIndex = frameIndex;
  for (uint8_t i = 0; i < 8; ++i) {
    slot.data[i] = (i < slot.length) ? message.buf[i] : 0;
  }

  rawHead = next;
}

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

// Bit rate sweep state. Only the main loop touches these.
uint8_t baudIndex = 0;
bool baudLocked = false;
uint32_t lastBaudSwitchMs = 0;

uint32_t txAcceptedCount = 0;
uint32_t txQueuedCount = 0;
volatile uint32_t txCompletedCount = 0;
bool selfTestPassed = false;

// An id no node on this bus owns, so the loopback probe can never be mistaken
// for telemetry. It lands in the otherId bucket and is subtracted afterwards.
constexpr uint32_t MITSUBA_SELFTEST_ID = 0x1FABCDEFUL;

// CRX1 on Teensy 4.1, checked before the bus is brought up.
constexpr uint8_t MITSUBA_CAN_RX_PIN = 23;

bool storeFrame0(const CAN_message_t &message) {
  MitsubaFrame0 decoded;
  if (!mitsubaDecodeFrame0(message.buf, message.len, decoded)) {
    return false;
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
  ++frame0Count;
  return true;
}

bool storeFrame1(const CAN_message_t &message) {
  MitsubaFrame1 decoded;
  if (!mitsubaDecodeFrame1(message.buf, message.len, decoded)) {
    return false;
  }

  telemetry.throttlePercent = decoded.throttlePercent;
  telemetry.regenVrPercent = decoded.regenVrPercent;
  telemetry.regenActive = decoded.regenActive;
  telemetry.powerMode = decoded.powerMode;
  telemetry.driveAction = decoded.driveAction;
  telemetry.lastFrame1Ms = millis();

  haveTelemetry = true;
  ++acceptedFrames;
  ++frame1Count;
  return true;
}

bool storeFrame2(const CAN_message_t &message) {
  MitsubaFrame2 decoded;
  if (!mitsubaDecodeFrame2(message.buf, message.len, decoded)) {
    return false;
  }

  telemetry.errorFlags = decoded.errorFlags;
  telemetry.overheatLevel = decoded.overheatLevel;
  telemetry.lastFrame2Ms = millis();

  haveTelemetry = true;
  ++acceptedFrames;
  ++frame2Count;
  return true;
}

// Called from the FlexCAN receive interrupt, via _mainHandler in mbCallbacks().
// Decoding is pure bit arithmetic, so it is cheap enough to run here, and it
// keeps the values fresh even while the main loop is stuck inside a bit-banged
// Nextion write or the BMS response window.
void onMitsubaFrame(const CAN_message_t &message) {
  const uint32_t timestampMs = millis();
  bool accepted = false;
  uint8_t frameIndex = 0xFF;

  if (message.flags.extended) {
    switch (message.id) {
      case MITSUBA_ID_FRAME0:
        frameIndex = 0;
        accepted = storeFrame0(message);
        break;
      case MITSUBA_ID_FRAME1:
        frameIndex = 1;
        accepted = storeFrame1(message);
        break;
      case MITSUBA_ID_FRAME2:
        frameIndex = 2;
        accepted = storeFrame2(message);
        break;
      default:
        break;
    }
  }

  if (frameIndex == 0xFF) {
    ++otherIdFrames;
  } else if (!accepted) {
    // Right id, payload the decoder would not take. Almost always a short DLC.
    ++rejectedFrames;
  }

  bufferRawFrame(message, accepted, accepted ? frameIndex : 0xFF, timestampMs, false);
}

// Fired from the interrupt once a transmit mailbox has actually put its frame
// on the wire and seen it acknowledged. Handing a frame to write() only proves
// the driver took it; this is the proof that it left the board.
void onMitsubaTxComplete(const CAN_message_t &message) {
  ++txCompletedCount;
  bufferRawFrame(message, true, 0xFF, millis(), true);
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

  // A return of 1 means the frame went into a hardware transmit mailbox. Any
  // other value means the library parked it in a software queue, which only
  // drains from the interrupt, so counting these separates "the request never
  // left" from "the request left and nobody answered".
  if (mitsubaCan.write(request) == 1) {
    ++txAcceptedCount;
  } else {
    ++txQueuedCount;
  }

  profileOfLastRequest = requestProfile;
  lastRequestMs = now;
  ++requestsSent;
}

// Sends one frame to itself with the receiver internally wired to the
// transmitter. Nothing about the harness matters here, so a failure points
// squarely at this module's FIFO, filter, interrupt or handler setup.
void runLoopbackSelfTest() {
  const uint32_t before = otherIdFrames;
  const uint32_t txBefore = txCompletedCount;

  mitsubaCan.enableLoopBack(true);

  CAN_message_t probe;
  probe.id = MITSUBA_SELFTEST_ID;
  probe.flags.extended = 1;
  probe.len = 8;
  for (uint8_t i = 0; i < 8; ++i) {
    probe.buf[i] = static_cast<uint8_t>(0xA5 + i);
  }
  mitsubaCan.write(probe);

  const uint32_t start = millis();
  while ((millis() - start) < 50 && otherIdFrames == before) {
    // The frame comes back through the receive interrupt.
  }

  mitsubaCan.enableLoopBack(false);

  selfTestPassed = otherIdFrames != before;

  // Keep the probe out of the bus counters and out of the raw frame log.
  otherIdFrames = before;
  txCompletedCount = txBefore;
  rawHead = rawTail;

  Serial.print("Mitsuba CAN1 loopback self-test: ");
  Serial.println(selfTestPassed
                     ? "PASS, the receive path works end to end"
                     : "FAIL, frames cannot reach the handler on this board");
}

}  // namespace

namespace mitsuba {

void begin() {
  if (!flexcanRxLineIsUsable(MITSUBA_CAN_RX_PIN)) {
    flexcanReportStuckRxLine("[Mitsuba bus]", MITSUBA_CAN_RX_PIN);
    return;
  }

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
  // enableFIFO() already armed the transmit mailbox interrupts, so this handler
  // starts reporting completions straight away.
  mitsubaCan.onTransmit(onMitsubaTxComplete);
  mitsubaCan.enableFIFOInterrupt();

  runLoopbackSelfTest();

  canReady = true;
  lastProfileSwitchMs = millis();
  baudIndex = 0;
  baudLocked = !MITSUBA_BAUD_AUTOSCAN;
  lastBaudSwitchMs = millis();

  Serial.print("Mitsuba CAN1 up at ");
  Serial.print(MITSUBA_BAUD_CANDIDATES[baudIndex]);
  Serial.print(" bit/s (pin 22 TX, pin 23 RX)");
  Serial.println(MITSUBA_BAUD_AUTOSCAN ? ", bit rate sweep armed" : "");
  mitsubaCan.mailboxStatus();
}

uint32_t activeBaudRate() { return MITSUBA_BAUD_CANDIDATES[baudIndex]; }

bool isBaudLocked() { return baudLocked; }

void reapplyBaudRate() {
  if (!canReady) {
    return;
  }
  mitsubaCan.setBaudRate(MITSUBA_BAUD_CANDIDATES[baudIndex]);
}

// Walks the candidate bit rates until a response frame actually decodes.
//
// The sweep runs in normal mode rather than listen-only on purpose: the Mitsuba
// only answers when it is asked, so a silent listener would learn nothing. At a
// wrong rate the two nodes trade error frames, which is exactly the state the
// bus is already in when this runs, and each candidate only lasts two seconds.
void serviceBaudScan(uint32_t now) {
  if (baudLocked) {
    if (!MITSUBA_BAUD_AUTOSCAN) {
      return;
    }

    const uint32_t newestFrame = lastFrameTimestamp();
    if (newestFrame != 0 && (now - newestFrame) >= MITSUBA_BAUD_RESCAN_MS) {
      baudLocked = false;
      lastBaudSwitchMs = now;
      Serial.println("Mitsuba: silent for 10s, sweeping bit rates again");
    }
    return;
  }

  if (acceptedFrames > 0) {
    baudLocked = true;
    Serial.print("Mitsuba: bit rate locked at ");
    Serial.print(MITSUBA_BAUD_CANDIDATES[baudIndex]);
    Serial.println(" bit/s");
    return;
  }

  if ((now - lastBaudSwitchMs) < MITSUBA_BAUD_DWELL_MS) {
    return;
  }

  baudIndex = static_cast<uint8_t>((baudIndex + 1) % MITSUBA_BAUD_CANDIDATE_COUNT);
  lastBaudSwitchMs = now;
  mitsubaCan.setBaudRate(MITSUBA_BAUD_CANDIDATES[baudIndex]);

  // Clear the latched error bits so the next candidate is judged on its own.
  FLEXCANb_ESR1(CAN1) |= FLEXCANb_ESR1(CAN1);

  // Each rate deserves a fresh look at both request ids.
  profileLocked = false;
  requestProfile = 0;
  lastProfileSwitchMs = now;

  Serial.print("Mitsuba: no answer, trying ");
  Serial.print(MITSUBA_BAUD_CANDIDATES[baudIndex]);
  Serial.println(" bit/s");
}

void service(uint32_t now) {
  if (!canReady) {
    return;
  }

  serviceBaudScan(now);

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

MitsubaLinkStats stats() {
  MitsubaLinkStats out;
  noInterrupts();
  out.frame0 = frame0Count;
  out.frame1 = frame1Count;
  out.frame2 = frame2Count;
  out.otherId = otherIdFrames;
  out.rejected = rejectedFrames;
  out.rawOverflows = rawOverflows;
  interrupts();
  out.requestsSent = requestsSent;
  out.txAccepted = txAcceptedCount;
  out.txQueued = txQueuedCount;
  noInterrupts();
  out.txCompleted = txCompletedCount;
  interrupts();
  return out;
}

bool loopbackSelfTestPassed() { return selfTestPassed; }

bool takeRawFrame(MitsubaRawFrame &out) {
  noInterrupts();
  if (rawTail == rawHead) {
    interrupts();
    return false;
  }

  volatile MitsubaRawFrame &slot = rawFrames[rawTail];
  out.id = slot.id;
  out.timestampMs = slot.timestampMs;
  out.length = slot.length;
  out.extended = slot.extended;
  out.accepted = slot.accepted;
  out.transmitted = slot.transmitted;
  out.frameIndex = slot.frameIndex;
  for (uint8_t i = 0; i < 8; ++i) {
    out.data[i] = slot.data[i];
  }

  rawTail = static_cast<uint8_t>((rawTail + 1) % MITSUBA_RAW_FRAME_SLOTS);
  interrupts();
  return true;
}

uint32_t frameAgeMs(uint8_t frameIndex, uint32_t now) {
  noInterrupts();
  const uint32_t last = (frameIndex == 0)   ? telemetry.lastFrame0Ms
                        : (frameIndex == 1) ? telemetry.lastFrame1Ms
                                            : telemetry.lastFrame2Ms;
  interrupts();

  return last == 0 ? 0 : (now - last);
}

// Read straight out of the FlexCAN peripheral. The library only samples ESR1
// from its own interrupt, which never runs when the bus is completely silent,
// and a silent bus is exactly the case worth diagnosing.
void printBusState() {
  const uint32_t esr1 = FLEXCANb_ESR1(CAN1);
  const uint32_t ecr = FLEXCANb_ECR(CAN1);

  const bool synced = (esr1 & (1UL << 18)) != 0;
  const bool idle = (esr1 & (1UL << 7)) != 0;
  const bool receiving = (esr1 & (1UL << 3)) != 0;
  const bool transmitting = (esr1 & (1UL << 6)) != 0;
  const uint8_t faultConfinement = static_cast<uint8_t>((esr1 >> 4) & 0x3);
  const uint8_t rxErrors = static_cast<uint8_t>((ecr >> 8) & 0xFF);
  const uint8_t txErrors = static_cast<uint8_t>(ecr & 0xFF);
  const bool ackError = (esr1 & (1UL << 13)) != 0;

  Serial.print("[Mitsuba bus] sync=");
  Serial.print(synced ? "yes" : "NO");
  Serial.print(idle ? " idle" : "");
  Serial.print(receiving ? " receiving" : "");
  Serial.print(transmitting ? " transmitting" : "");

  Serial.print(faultConfinement == 0   ? " error-active"
               : faultConfinement == 1 ? " ERROR-PASSIVE"
                                       : " BUS-OFF");

  // The counters only fall on a successful frame, so a stuck value is a scar
  // from earlier rather than a fault happening now. The movement since the last
  // status line is what separates the two.
  static bool haveErrorBaseline = false;
  static uint8_t previousRxErrors = 0;
  static uint8_t previousTxErrors = 0;

  Serial.print(" rxErr=");
  Serial.print(rxErrors);
  if (haveErrorBaseline) {
    Serial.print(rxErrors == previousRxErrors ? "(frozen)" : "(MOVING)");
  }
  Serial.print(" txErr=");
  Serial.print(txErrors);
  if (haveErrorBaseline) {
    Serial.print(txErrors == previousTxErrors ? "(frozen)" : "(MOVING)");
  }

  previousRxErrors = rxErrors;
  previousTxErrors = txErrors;
  haveErrorBaseline = true;

  if (esr1 & (1UL << 10)) Serial.print(" STUFF_ERR");
  if (esr1 & (1UL << 11)) Serial.print(" FORM_ERR");
  if (esr1 & (1UL << 12)) Serial.print(" CRC_ERR");
  if (ackError) Serial.print(" ACK_ERR");
  if (esr1 & (1UL << 14)) Serial.print(" BIT0_ERR");
  if (esr1 & (1UL << 15)) Serial.print(" BIT1_ERR");

  Serial.print(" esr1=0x");
  Serial.println(esr1, HEX);

  flexcanPrintBitTiming("[Mitsuba bus]", CAN1, MITSUBA_BAUD_CANDIDATES[baudIndex]);

  // Receive errors outrank transmit errors here. A node only accumulates them
  // while another node is actually driving frames onto the wire, so they prove
  // the controller is alive and say the bits themselves are being misread. A
  // bit rate mismatch also produces transmit errors as a side effect, which is
  // why the acknowledge case has to be checked second.
  if (txErrors == 0 && txAcceptedCount > 0) {
    Serial.println("[Mitsuba bus] our requests are transmitted AND acknowledged, "
                   "so the wire, the terminators and the bit rate are all good. "
                   "Something on this bus hears us. If no response ever arrives, "
                   "the request id or payload is not what the controller expects, "
                   "or the controller answering the acknowledge is not the motor "
                   "controller");
  } else if (rxErrors > 0) {
    Serial.println("[Mitsuba bus] receive errors mean another node IS transmitting "
                   "and its bits do not decode. With the timing line above "
                   "confirming the rate, the remaining causes are termination "
                   "(exactly two 120 ohm resistors on the bus, one at each end), "
                   "a sample point the controller disagrees with, or CANH/CANL "
                   "shorted or swapped somewhere in the harness");
  } else if (ackError || txErrors > 0) {
    Serial.println("[Mitsuba bus] requests go out but nothing acknowledges them: "
                   "the controller is not on this bus, is unpowered, or the "
                   "transceiver is not driving CANH/CANL");
  } else if (!synced && rxErrors == 0 && txErrors == 0) {
    Serial.println("[Mitsuba bus] nothing is driving the wire: check transceiver "
                   "power, CANH/CANL not swapped, and the 120 ohm terminators");
  }
}

bool isReady() { return canReady; }

bool isProfileLocked() { return profileLocked; }

const char *driveActionName(uint8_t driveAction) {
  switch (driveAction) {
    case 0:
      return "stop";
    case 1:
      return "reserved";
    case 2:
      return "forward";
    case 3:
      return "reverse";
    default:
      return "unknown";
  }
}

}  // namespace mitsuba

#endif  // ESP32
