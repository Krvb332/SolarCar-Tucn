#include "tpms_can.h"

#include <Arduino.h>

namespace {

// Slot 0 stays unused so a wheel id off the bus indexes this directly.
volatile TpmsWheel wheels[TPMS_WHEEL_COUNT];
volatile uint32_t wheelFrames[TPMS_WHEEL_COUNT] = {};

volatile uint32_t acceptedFrames = 0;
volatile uint32_t otherIdFrames = 0;
volatile uint32_t malformedFrames = 0;
volatile uint32_t rawOverflows = 0;

// Single producer (the receive path) and single consumer (the main loop).
volatile TpmsRawFrame rawFrames[TPMS_RAW_FRAME_SLOTS];
volatile uint8_t rawHead = 0;
volatile uint8_t rawTail = 0;

bool canReady = false;

// Bit rate sweep state. Only the main loop touches these.
uint8_t baudIndex = 0;
bool baudLocked = false;
uint32_t lastBaudSwitchMs = 0;
bool selfTestPassed = false;

// An id no sensor uses, so the loopback probe cannot be mistaken for a wheel.
constexpr uint32_t TPMS_SELFTEST_ID = 0x1FABCDEFUL;

// Any frame at all proves the bit rate, even one carrying an id we do not want.
uint32_t framesSeen() {
  return acceptedFrames + otherIdFrames + malformedFrames;
}

// On Teensy the readings are written from the CAN receive interrupt, so the
// main loop has to take a short critical section to read them back. On ESP32
// they are written from the same task that reads them and no guard is needed.
inline void lockReadings() {
#if !defined(ARDUINO_ARCH_ESP32)
  noInterrupts();
#endif
}

inline void unlockReadings() {
#if !defined(ARDUINO_ARCH_ESP32)
  interrupts();
#endif
}

// Keeps every frame that reached this bus, wanted or not, so the serial monitor
// can tell "nothing is wired" apart from "the receiver uses another id".
void bufferRawFrame(uint32_t id, bool extended, const uint8_t *data, uint8_t length,
                    bool accepted, uint32_t timestampMs) {
  const uint8_t next = static_cast<uint8_t>((rawHead + 1) % TPMS_RAW_FRAME_SLOTS);
  if (next == rawTail) {
    ++rawOverflows;
    return;
  }

  volatile TpmsRawFrame &slot = rawFrames[rawHead];
  slot.id = id;
  slot.timestampMs = timestampMs;
  slot.length = length > 8 ? 8 : length;
  slot.extended = extended;
  slot.accepted = accepted;
  for (uint8_t i = 0; i < 8; ++i) {
    slot.data[i] = (i < slot.length) ? data[i] : 0;
  }

  rawHead = next;
}

// Frame layout: byte 0 is the wheel id, bytes 1..2 the raw pressure, byte 3 the
// temperature, byte 4 the sensor battery and byte 5 the alarm bits. Same
// scaling the standalone ESP32 sketch used.
//
// Runs in interrupt context on Teensy. It must never print.
void handleFrame(uint32_t id, bool extended, const uint8_t *data, uint8_t length) {
  const uint32_t timestampMs = millis();

  if (id != TPMS_CAN_MESSAGE_ID) {
    ++otherIdFrames;
    bufferRawFrame(id, extended, data, length, false, timestampMs);
    return;
  }

  const uint8_t wheelId = (length >= 1) ? data[0] : 0;
  if (length < 6 || wheelId < 1 || wheelId >= TPMS_WHEEL_COUNT) {
    ++malformedFrames;
    bufferRawFrame(id, extended, data, length, false, timestampMs);
    return;
  }

  const uint16_t rawPressure =
      static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[2]);

  volatile TpmsWheel &wheel = wheels[wheelId];
  wheel.pressureBar = (static_cast<float>(rawPressure) * 0.0101f) + 0.35f;
  wheel.temperatureC = static_cast<int16_t>(data[3]);
  wheel.batteryV = static_cast<float>(data[4]) * 0.1237f;
  wheel.leakingAir = bitRead(data[5], 0) != 0;
  wheel.extremeTemperature = bitRead(data[5], 4) != 0;
  wheel.lastMessageMs = timestampMs;

  ++wheelFrames[wheelId];
  ++acceptedFrames;
  bufferRawFrame(id, extended, data, length, true, timestampMs);
}

}  // namespace

#if defined(ARDUINO_ARCH_ESP32)

#include "driver/twai.h"

namespace {

constexpr gpio_num_t TPMS_CAN_TX_PIN = GPIO_NUM_5;
constexpr gpio_num_t TPMS_CAN_RX_PIN = GPIO_NUM_4;

}  // namespace

namespace tpms {

void begin() {
  twai_general_config_t generalConfig =
      TWAI_GENERAL_CONFIG_DEFAULT(TPMS_CAN_TX_PIN, TPMS_CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  canReady =
      twai_driver_install(&generalConfig, &timingConfig, &filterConfig) == ESP_OK &&
      twai_start() == ESP_OK;

  Serial.println(canReady ? "TPMS CAN started on TWAI GPIO4/GPIO5."
                          : "TPMS CAN failed to start.");
}

void service(uint32_t) {
  if (!canReady) {
    return;
  }

  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    handleFrame(message.identifier, message.extd != 0, message.data,
                message.data_length_code);
  }
}

void reapplyBaudRate() {}

uint32_t activeBaudRate() { return TPMS_CAN_BAUD; }

bool loopbackSelfTestPassed() { return false; }

bool isBaudLocked() { return true; }

void printBusState() {
  if (!canReady) {
    Serial.println("[TPMS bus] controller not started");
    return;
  }

  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) {
    Serial.println("[TPMS bus] status unavailable");
    return;
  }

  Serial.print("[TPMS bus] state=");
  switch (status.state) {
    case TWAI_STATE_STOPPED:
      Serial.print("stopped");
      break;
    case TWAI_STATE_RUNNING:
      Serial.print("running");
      break;
    case TWAI_STATE_BUS_OFF:
      Serial.print("BUS-OFF");
      break;
    default:
      Serial.print("recovering");
      break;
  }

  Serial.print(" rxErr=");
  Serial.print(status.rx_error_counter);
  Serial.print(" txErr=");
  Serial.print(status.tx_error_counter);
  Serial.print(" rxMissed=");
  Serial.print(status.rx_missed_count);
  Serial.print(" busErrors=");
  Serial.println(status.bus_error_count);
}

}  // namespace tpms

#else

#include <FlexCAN_T4.h>

#include "flexcan_diag.h"

namespace {

// CAN2 on Teensy 4.1 is pin 0 (CRX2) and pin 1 (CTX2). CAN1 on pins 22 and 23
// belongs to the Mitsuba controller and is not touched from here.
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> tpmsCan;

// CRX2, checked before the bus is brought up.
constexpr uint8_t TPMS_CAN_RX_PIN = 0;

// Runs from the FlexCAN receive interrupt so the readings stay current even
// while loop() is stuck inside a bit-banged Nextion write or the BMS response
// window. Same reasoning as the Mitsuba link on CAN1. No Serial calls here.
void onTpmsFrame(const CAN_message_t &message) {
  handleFrame(message.id, message.flags.extended != 0, message.buf, message.len);
}

// Sends one frame to itself with the receiver internally wired to the
// transmitter, so a failure points at this module rather than at the harness.
void runLoopbackSelfTest() {
  const uint32_t before = otherIdFrames;

  // Loopback needs the transmitter, so leave listen-only for the length of one
  // frame. Nothing reaches the bus: the receiver input is internally rewired.
  tpmsCan.setBaudRate(TPMS_BAUD_CANDIDATES[baudIndex]);
  tpmsCan.enableLoopBack(true);

  CAN_message_t probe;
  probe.id = TPMS_SELFTEST_ID;
  probe.flags.extended = 1;
  probe.len = 8;
  for (uint8_t i = 0; i < 8; ++i) {
    probe.buf[i] = static_cast<uint8_t>(0x5A + i);
  }
  tpmsCan.write(probe);

  const uint32_t start = millis();
  while ((millis() - start) < 50 && otherIdFrames == before) {
    // The frame comes back through the receive interrupt.
  }

  tpmsCan.enableLoopBack(false);
  tpmsCan.setBaudRate(TPMS_BAUD_CANDIDATES[baudIndex], baudLocked ? TX : LISTEN_ONLY);

  selfTestPassed = otherIdFrames != before;

  otherIdFrames = before;
  rawHead = rawTail;

  Serial.print("TPMS CAN2 loopback self-test: ");
  Serial.println(selfTestPassed
                     ? "PASS, the receive path works end to end"
                     : "FAIL, frames cannot reach the handler on this board");
}

}  // namespace

namespace tpms {

void begin() {
  if (!flexcanRxLineIsUsable(TPMS_CAN_RX_PIN)) {
    flexcanReportStuckRxLine("[TPMS bus]", TPMS_CAN_RX_PIN);
    return;
  }

  baudIndex = 0;
  baudLocked = !TPMS_BAUD_AUTOSCAN;

  tpmsCan.begin();
  // While sweeping, stay in listen-only: no acknowledge bits, no error frames,
  // nothing this board does can disturb a bus it has not identified yet.
  tpmsCan.setBaudRate(TPMS_BAUD_CANDIDATES[baudIndex], baudLocked ? TX : LISTEN_ONLY);
  tpmsCan.setMaxMB(16);
  tpmsCan.enableFIFO();
  // Accept everything and sort it out in handleFrame, so a frame with the wrong
  // id still shows up in the serial monitor instead of vanishing in hardware.
  tpmsCan.setFIFOFilter(ACCEPT_ALL);
  // Arm the handler before enabling the interrupt, so a frame arriving during
  // bring-up cannot land while _mainHandler is still null.
  tpmsCan.onReceive(onTpmsFrame);
  tpmsCan.enableFIFOInterrupt();

  runLoopbackSelfTest();

  canReady = true;
  lastBaudSwitchMs = millis();

  Serial.print("TPMS CAN2 up at ");
  Serial.print(TPMS_BAUD_CANDIDATES[baudIndex]);
  Serial.print(" bit/s (pin 1 TX, pin 0 RX)");
  Serial.println(TPMS_BAUD_AUTOSCAN ? ", listen-only bit rate sweep armed" : "");
  tpmsCan.mailboxStatus();
}

uint32_t activeBaudRate() { return TPMS_BAUD_CANDIDATES[baudIndex]; }

bool isBaudLocked() { return baudLocked; }

bool loopbackSelfTestPassed() { return selfTestPassed; }

void reapplyBaudRate() {
  if (!canReady) {
    return;
  }
  tpmsCan.setBaudRate(TPMS_BAUD_CANDIDATES[baudIndex], baudLocked ? TX : LISTEN_ONLY);
}

// Read straight out of the FlexCAN peripheral. The library only samples ESR1
// from its interrupt, which never runs when the bus is completely silent, and a
// silent bus is exactly the case worth diagnosing.
void printBusState() {
  const uint32_t esr1 = FLEXCANb_ESR1(CAN2);
  const uint32_t ecr = FLEXCANb_ECR(CAN2);

  const bool synced = (esr1 & (1UL << 18)) != 0;
  const bool idle = (esr1 & (1UL << 7)) != 0;
  const bool receiving = (esr1 & (1UL << 3)) != 0;
  const bool transmitting = (esr1 & (1UL << 6)) != 0;
  const uint8_t faultConfinement = static_cast<uint8_t>((esr1 >> 4) & 0x3);
  const uint8_t rxErrors = static_cast<uint8_t>((ecr >> 8) & 0xFF);
  const uint8_t txErrors = static_cast<uint8_t>(ecr & 0xFF);

  Serial.print("[TPMS bus] sync=");
  Serial.print(synced ? "yes" : "NO");
  Serial.print(idle ? " idle" : "");
  Serial.print(receiving ? " receiving" : "");
  Serial.print(transmitting ? " transmitting" : "");

  Serial.print(faultConfinement == 0   ? " error-active"
               : faultConfinement == 1 ? " ERROR-PASSIVE"
                                       : " BUS-OFF");

  Serial.print(" rxErr=");
  Serial.print(rxErrors);
  Serial.print(" txErr=");
  Serial.print(txErrors);

  if (esr1 & (1UL << 10)) Serial.print(" STUFF_ERR");
  if (esr1 & (1UL << 11)) Serial.print(" FORM_ERR");
  if (esr1 & (1UL << 12)) Serial.print(" CRC_ERR");
  if (esr1 & (1UL << 13)) Serial.print(" ACK_ERR");
  if (esr1 & (1UL << 14)) Serial.print(" BIT0_ERR");
  if (esr1 & (1UL << 15)) Serial.print(" BIT1_ERR");

  Serial.print(" esr1=0x");
  Serial.println(esr1, HEX);

  flexcanPrintBitTiming("[TPMS bus]", CAN2, TPMS_BAUD_CANDIDATES[baudIndex]);

  if (!synced && rxErrors == 0 && txErrors == 0) {
    Serial.println("[TPMS bus] nothing is driving the wire: check transceiver "
                   "power, CANH/CANL not swapped, and that the receiver is on");
  } else if (!synced || rxErrors > 0) {
    Serial.print("[TPMS bus] edges are there but the frames do not decode: "
                 "most likely the wrong bit rate. This build uses ");
    Serial.print(TPMS_CAN_BAUD);
    Serial.println(" bit/s; 250000 is the other common one");
  }
}

void service(uint32_t now) {
  // Deliberately no tpmsCan.events(). The first events() call latches
  // isEventsUsed inside FlexCAN_T4 and moves dispatch out of the interrupt and
  // into the main loop, which is exactly the path this module relies on. The
  // Mitsuba link on CAN1 avoids the same trap.

  if (!canReady || baudLocked) {
    // No rescan once a rate is proven. Sensors sleep when the car is parked, so
    // a long silence is normal here and must not tear a working link down.
    return;
  }

  if (framesSeen() > 0) {
    baudLocked = true;
    // Leave listen-only now: a bus where nobody acknowledges pushes the
    // receiver into error-passive, and it may stop transmitting altogether.
    tpmsCan.setBaudRate(TPMS_BAUD_CANDIDATES[baudIndex]);

    Serial.print("TPMS: bit rate locked at ");
    Serial.print(TPMS_BAUD_CANDIDATES[baudIndex]);
    Serial.println(" bit/s, acknowledging from now on");
    return;
  }

  if ((now - lastBaudSwitchMs) < TPMS_BAUD_DWELL_MS) {
    return;
  }

  baudIndex = static_cast<uint8_t>((baudIndex + 1) % TPMS_BAUD_CANDIDATE_COUNT);
  lastBaudSwitchMs = now;
  tpmsCan.setBaudRate(TPMS_BAUD_CANDIDATES[baudIndex], LISTEN_ONLY);

  // Clear the latched error bits so the next candidate is judged on its own.
  FLEXCANb_ESR1(CAN2) |= FLEXCANb_ESR1(CAN2);

  Serial.print("TPMS: nothing heard, listening at ");
  Serial.print(TPMS_BAUD_CANDIDATES[baudIndex]);
  Serial.println(" bit/s");
}

}  // namespace tpms

#endif  // ARDUINO_ARCH_ESP32

namespace tpms {

bool snapshot(uint8_t wheelId, TpmsWheel &out) {
  if (wheelId < 1 || wheelId >= TPMS_WHEEL_COUNT) {
    return false;
  }

  lockReadings();
  // Copy field by field: the table is volatile, so a struct assignment is not
  // available and a memcpy would drop the volatile qualifier.
  out.pressureBar = wheels[wheelId].pressureBar;
  out.temperatureC = wheels[wheelId].temperatureC;
  out.batteryV = wheels[wheelId].batteryV;
  out.leakingAir = wheels[wheelId].leakingAir;
  out.extremeTemperature = wheels[wheelId].extremeTemperature;
  out.lastMessageMs = wheels[wheelId].lastMessageMs;
  unlockReadings();

  return out.lastMessageMs != 0;
}

bool isFresh(uint8_t wheelId, uint32_t now) {
  if (wheelId < 1 || wheelId >= TPMS_WHEEL_COUNT) {
    return false;
  }

  lockReadings();
  const uint32_t lastMessageMs = wheels[wheelId].lastMessageMs;
  unlockReadings();

  return lastMessageMs != 0 && (now - lastMessageMs) <= TPMS_SENSOR_TIMEOUT_MS;
}

uint32_t wheelAgeMs(uint8_t wheelId, uint32_t now) {
  if (wheelId < 1 || wheelId >= TPMS_WHEEL_COUNT) {
    return 0;
  }

  lockReadings();
  const uint32_t lastMessageMs = wheels[wheelId].lastMessageMs;
  unlockReadings();

  return lastMessageMs == 0 ? 0 : (now - lastMessageMs);
}

uint32_t wheelFrameCount(uint8_t wheelId) {
  if (wheelId < 1 || wheelId >= TPMS_WHEEL_COUNT) {
    return 0;
  }

  lockReadings();
  const uint32_t count = wheelFrames[wheelId];
  unlockReadings();
  return count;
}

uint32_t frameCount() {
  lockReadings();
  const uint32_t count = acceptedFrames;
  unlockReadings();
  return count;
}

TpmsLinkStats stats() {
  TpmsLinkStats out;
  lockReadings();
  out.accepted = acceptedFrames;
  out.otherId = otherIdFrames;
  out.malformed = malformedFrames;
  out.rawOverflows = rawOverflows;
  unlockReadings();
  return out;
}

bool takeRawFrame(TpmsRawFrame &out) {
  lockReadings();
  if (rawTail == rawHead) {
    unlockReadings();
    return false;
  }

  volatile TpmsRawFrame &slot = rawFrames[rawTail];
  out.id = slot.id;
  out.timestampMs = slot.timestampMs;
  out.length = slot.length;
  out.extended = slot.extended;
  out.accepted = slot.accepted;
  for (uint8_t i = 0; i < 8; ++i) {
    out.data[i] = slot.data[i];
  }

  rawTail = static_cast<uint8_t>((rawTail + 1) % TPMS_RAW_FRAME_SLOTS);
  unlockReadings();
  return true;
}

const char *wheelName(uint8_t wheelId) {
  switch (wheelId) {
    case 1:
      return "front-left";
    case 2:
      return "front-right";
    case 3:
      return "rear-right";
    case 4:
      return "rear-left";
    default:
      return "unknown";
  }
}

bool isReady() { return canReady; }

}  // namespace tpms
