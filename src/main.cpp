#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32)
#include "driver/twai.h"
#else
#include <SoftwareSerial.h>
#endif
#include <string.h>

/*
  Teensy 4.1 <-> Nextion Intelligent NX8048P050-011C

  HMI inspected:
    C:\Users\Krab\Downloads\interfataSeptimiu.HMI

  Wiring used by this sketch:
    Teensy 4.1 pin 2, software TX  -> Nextion RX
    Teensy 4.1 pin 3, software RX  <- Nextion TX
    Teensy 4.1 pin 12, software TX -> ANT BMS RX
    Teensy 4.1 pin 11, software RX <- ANT BMS TX
    GND                    -> Nextion GND
    GND                    -> ANT BMS GND
    External 5 V supply    -> Nextion 5 V input

  Teensy 4.1 pins are 3.3 V and are not 5 V tolerant. If the Nextion TX line
  or BMS TX line outputs 5 V, level-shift it before connecting it to Teensy.

  The HMI uses page0 and xfloat numeric components. For xfloat fields with
  vvs1=1, write .val as value * 10. x1 has vvs1=0, so RPM is written directly.

  Extracted page/component addresses:
    page0 id=0
    p0    id=1   background picture
    x0    id=2   speed value, 1 decimal      t0  id=3  "Km/h"
    x1    id=4   RPM value, integer          t1  id=5  "RPM"
    tm0   id=6   HMI demo timer, disabled by Teensy
    x2    id=12  BMS Tmp, 1 decimal          t6  id=11 "Tmp"
    x3    id=13  BMS SOC, 1 decimal          t2  id=7  "SOC"
    x4    id=14  BMS Volt, 1 decimal         t3  id=8  "Volt"
    x5    id=15  BMS Amp, 1 decimal          t4  id=9  "Amp"
    x6    id=16  BMS Pow, 1 decimal          t5  id=10 "Pow"
    x11   id=22  MPPT Volt, 1 decimal        t11 id=21 "Volt"
    x10   id=20  secondary Volt, 1 decimal   t10 id=19 "Volt2"
    x9    id=18  MPPT Amp, 1 decimal         t9  id=17 "Amp"
    x7    id=24  MPPT Temp, 1 decimal        t12 id=25 "Temp"
    x8    id=26  MPPT Pow, 1 decimal         t8  id=27 "Pow"
    x12   id=30  drive voltage, 1 decimal
    x13   id=32  drive current, 1 decimal
    x14   id=34  drive power, 1 decimal
    x15   id=35  drive temperature, 1 decimal
    x16          TPMS front-left pressure
    x17          TPMS front-right pressure
    x18          TPMS middle wheel pressure
    z0    id=37  center gauge/progress value
*/

namespace {
constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t NEXTION_BAUD = 9600;
constexpr uint32_t BMS_BAUD = 19200;
constexpr uint32_t LOOPED_TELEMETRY_INTERVAL_MS = 250;
constexpr uint32_t PAGE_HEARTBEAT_INTERVAL_MS = 1000;
constexpr uint32_t BMS_REQUEST_INTERVAL_MS = 1000;
constexpr uint32_t BMS_RESPONSE_WINDOW_MS = 1000;
constexpr uint32_t BMS_STALE_LOG_INTERVAL_MS = 3000;
constexpr uint32_t TPMS_SENSOR_TIMEOUT_MS = 2000;
constexpr bool ENABLE_BMS_SERIAL_DEBUG = true;
constexpr bool BMS_DEBUG_PRINT_RAW_FRAME = true;
constexpr size_t BMS_DEBUG_RAW_BYTES_PER_LINE = 16;
constexpr float LOOPED_BMS_SOC_MIN = 1.0f;
constexpr float LOOPED_BMS_SOC_MAX = 100.0f;
constexpr float LOOPED_BMS_SOC_STEP = 1.0f;
constexpr float LOOPED_BMS_VOLTAGE_MAX = 150.0f;
constexpr float LOOPED_BMS_VOLTAGE_STEP = 0.8f;
constexpr float LOOPED_BMS_CURRENT_MAX = 100.0f;
constexpr float LOOPED_BMS_CURRENT_STEP = 0.6f;
constexpr float LOOPED_BMS_POWER_MAX = 3000.0f;
constexpr float LOOPED_BMS_POWER_STEP = 35.0f;
constexpr float LOOPED_BMS_TEMPERATURE_MAX = 100.0f;
constexpr float LOOPED_BMS_TEMPERATURE_STEP = 0.5f;
constexpr float LOOPED_MPPT_VOLTAGE_MAX = 150.0f;
constexpr float LOOPED_MPPT_VOLTAGE_STEP = 0.7f;
constexpr float LOOPED_MPPT_CURRENT_MAX = 50.0f;
constexpr float LOOPED_MPPT_CURRENT_STEP = 0.4f;
constexpr float LOOPED_MPPT_POWER_MAX = 2000.0f;
constexpr float LOOPED_MPPT_POWER_STEP = 25.0f;
constexpr float LOOPED_MPPT_TEMPERATURE_MAX = 100.0f;
constexpr float LOOPED_MPPT_TEMPERATURE_STEP = 0.5f;
constexpr float LOOPED_SECONDARY_VOLTAGE_MAX = 50.0f;
constexpr float LOOPED_SECONDARY_VOLTAGE_STEP = 0.4f;
constexpr float LOOPED_SPEED_MAX = 150.0f;
constexpr float LOOPED_SPEED_STEP = 0.8f;
constexpr uint32_t LOOPED_RPM_MAX = 2000;
constexpr uint32_t LOOPED_RPM_STEP = 50;
constexpr float LOOPED_DRIVE_POWER_MAX = 3000.0f;
constexpr float LOOPED_DRIVE_POWER_STEP = 35.0f;
constexpr float LOOPED_DRIVE_VOLTAGE_MAX = 150.0f;
constexpr float LOOPED_DRIVE_VOLTAGE_STEP = 0.8f;
constexpr float LOOPED_DRIVE_CURRENT_MAX = 100.0f;
constexpr float LOOPED_DRIVE_CURRENT_STEP = 0.6f;
constexpr float LOOPED_DRIVE_TEMPERATURE_MAX = 100.0f;
constexpr float LOOPED_DRIVE_TEMPERATURE_STEP = 0.5f;
constexpr float LOOPED_TPMS_PRESSURE_MAX = 3.0f;
constexpr float LOOPED_TPMS_PRESSURE_STEP = 0.03f;
constexpr uint32_t LOOPED_CENTER_GAUGE_MAX = 100;
constexpr uint32_t LOOPED_CENTER_GAUGE_STEP = 2;
constexpr uint8_t NEXTION_RX_PIN = 3;
constexpr uint8_t NEXTION_TX_PIN = 2;
constexpr uint8_t BMS_RX_PIN = 11;
constexpr uint8_t BMS_TX_PIN = 12;
#if defined(ARDUINO_ARCH_ESP32)
constexpr gpio_num_t TPMS_CAN_TX_PIN = GPIO_NUM_5;
constexpr gpio_num_t TPMS_CAN_RX_PIN = GPIO_NUM_4;
#endif
constexpr uint8_t NEXTION_TERMINATOR = 0xFF;
constexpr uint8_t PAGE0_ID = 0;
constexpr uint32_t TPMS_CAN_MESSAGE_ID = 0x18FEF433;
constexpr uint8_t ANT_REQUEST_CMD[6] = {0xDB, 0xDB, 0x00, 0x00, 0x00, 0x00};
constexpr uint8_t ANT_HEADER[4] = {0xAA, 0x55, 0xAA, 0xFF};
constexpr size_t ANT_FRAME_SIZE = 140;
constexpr uint8_t ANT_CELL_COUNT = 32;
constexpr uint8_t ANT_TEMP_COUNT = 6;
constexpr float PACK_VOLTAGE_SCALE = 0.1f;
constexpr float PACK_CURRENT_SCALE = 0.1f;
constexpr float MILLIVOLTS_TO_VOLTS = 0.001f;
constexpr float MICRO_AH_TO_AH = 0.000001f;

// ANT BMS frame byte addresses imported from bms.cpp.
constexpr size_t OFFSET_PACK_VOLTAGE = 4;
constexpr size_t OFFSET_CELLS = 6;
constexpr size_t OFFSET_PACK_CURRENT = 70;
constexpr size_t OFFSET_SOC = 74;
constexpr size_t OFFSET_CAP_TOTAL = 75;
constexpr size_t OFFSET_CAP_REMAIN = 79;
constexpr size_t OFFSET_CYCLES = 87;
constexpr size_t OFFSET_TEMPS = 91;
constexpr size_t OFFSET_MOS_CHARGE = 103;
constexpr size_t OFFSET_MOS_DISCHARGE = 104;

static_assert(OFFSET_CELLS + (ANT_CELL_COUNT * 2) <= ANT_FRAME_SIZE,
              "BMS cell block must fit inside the ANT frame");
static_assert(OFFSET_TEMPS + (ANT_TEMP_COUNT * 2) <= ANT_FRAME_SIZE,
              "BMS temperature block must fit inside the ANT frame");
static_assert(OFFSET_MOS_DISCHARGE < ANT_FRAME_SIZE,
              "BMS MOS status offsets must fit inside the ANT frame");

#if defined(ARDUINO_ARCH_ESP32)
HardwareSerial nextion(1);
HardwareSerial bmsSerial(2);
#else
SoftwareSerial nextion(NEXTION_RX_PIN, NEXTION_TX_PIN);
SoftwareSerial bmsSerial(BMS_RX_PIN, BMS_TX_PIN);
#endif

struct BmsTelemetry {
  float packVoltage = 0.0f;
  float packCurrent = 0.0f;
  float soc = 0.0f;
  float capTotalAh = 0.0f;
  float capRemainAh = 0.0f;
  uint16_t cycles = 0;
  float temps[ANT_TEMP_COUNT] = {};
  bool mosCharge = false;
  bool mosDischarge = false;
  float cells[ANT_CELL_COUNT] = {};
  uint8_t activeCellCount = 0;
  float minCell = 0.0f;
  float maxCell = 0.0f;
  float avgCell = 0.0f;
  float deltaCell = 0.0f;
  uint8_t minCellId = 0;
  uint8_t maxCellId = 0;
};

struct LoopedTelemetry {
  float bmsSoc = LOOPED_BMS_SOC_MIN;
  float bmsVoltage = 0.0f;
  float bmsCurrent = 0.0f;
  float bmsPower = 0.0f;
  float bmsTemperature = 0.0f;
  float mpptVoltage = 0.0f;
  float mpptCurrent = 0.0f;
  float mpptPower = 0.0f;
  float mpptTemperature = 0.0f;
  float secondaryVoltage = 0.0f;
  float speedKmh = 0.0f;
  uint32_t motorRpm = 0;
  float drivePower = 0.0f;
  float driveVoltage = 0.0f;
  float driveCurrent = 0.0f;
  float driveTemperature = 0.0f;
  float tpmsFrontLeft = 0.0f;
  float tpmsFrontRight = 0.0f;
  float tpmsMiddle = 0.0f;
  uint16_t centerGauge = 0;
};

struct TpmsWheel {
  float pressureBar = 0.0f;
  int temperatureC = 0;
  float batteryV = 0.0f;
  bool leakingAir = false;
  bool extremeTemperature = false;
  uint32_t lastMessageMs = 0;
};

BmsTelemetry bmsTelemetry = {};
LoopedTelemetry loopedTelemetry = {};
TpmsWheel tpmsWheels[5] = {};

uint32_t lastLoopedTelemetryUpdate = 0;
uint32_t lastPageHeartbeat = 0;
uint32_t lastBmsRequest = 0;
uint32_t lastBmsFrame = 0;
uint32_t lastBmsStatusLog = 0;
uint32_t bmsRequestCount = 0;
uint32_t bmsFrameCount = 0;
uint32_t bmsTimeoutCount = 0;
uint32_t bmsRxOverflowCount = 0;
uint32_t bmsDroppedNoiseBytes = 0;
uint32_t bmsEchoBytesDiscarded = 0;
size_t bmsBytesThisResponse = 0;
uint8_t bmsRxBuffer[512] = {};
size_t bmsRxLength = 0;
bool hasBmsTelemetry = false;
bool awaitingBmsResponse = false;
bool tpmsCanReady = false;

enum class ActiveSoftwarePort {
  None,
  Nextion,
  Bms,
};

ActiveSoftwarePort activeSoftwarePort = ActiveSoftwarePort::None;

struct NextionCachedValue {
  const char *component;
  int32_t value;
  bool valid;
};

NextionCachedValue nextionValueCache[] = {
    {"x0", 0, false},  {"x1", 0, false},  {"x2", 0, false},  {"x3", 0, false},
    {"x4", 0, false},  {"x5", 0, false},  {"x6", 0, false},  {"x7", 0, false},
    {"x8", 0, false},  {"x9", 0, false},  {"x10", 0, false}, {"x11", 0, false},
    {"x12", 0, false}, {"x13", 0, false}, {"x14", 0, false}, {"x15", 0, false},
    {"x16", 0, false}, {"x17", 0, false}, {"x18", 0, false},
    {"z0", 0, false},
};

void activateNextionPort() {
  if (activeSoftwarePort == ActiveSoftwarePort::Nextion) {
    return;
  }

  if (activeSoftwarePort == ActiveSoftwarePort::Bms) {
    bmsSerial.end();
  }

#if defined(ARDUINO_ARCH_ESP32)
  nextion.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#else
  nextion.begin(NEXTION_BAUD);
#endif
  activeSoftwarePort = ActiveSoftwarePort::Nextion;
}

void activateBmsPort() {
  if (activeSoftwarePort == ActiveSoftwarePort::Bms) {
    return;
  }

  if (activeSoftwarePort == ActiveSoftwarePort::Nextion) {
    nextion.end();
  }

#if defined(ARDUINO_ARCH_ESP32)
  bmsSerial.begin(BMS_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
#else
  bmsSerial.begin(BMS_BAUD);
#endif
  activeSoftwarePort = ActiveSoftwarePort::Bms;
}

void sendTerminator() {
  nextion.write(NEXTION_TERMINATOR);
  nextion.write(NEXTION_TERMINATOR);
  nextion.write(NEXTION_TERMINATOR);
}

void nextionCommand(const char *command) {
  activateNextionPort();
  nextion.print(command);
  sendTerminator();
}

void setValue(const char *component, int32_t value) {
  activateNextionPort();
  nextion.print(component);
  nextion.print(".val=");
  nextion.print(value);
  sendTerminator();
}

bool shouldSendValue(const char *component, int32_t value) {
  for (NextionCachedValue &cached : nextionValueCache) {
    if (strcmp(cached.component, component) != 0) {
      continue;
    }

    if (cached.valid && cached.value == value) {
      return false;
    }

    cached.value = value;
    cached.valid = true;
    return true;
  }

  return true;
}

void setValueIfChanged(const char *component, int32_t value) {
  if (shouldSendValue(component, value)) {
    setValue(component, value);
  }
}

void setAttributeValue(const char *attribute, int32_t value) {
  activateNextionPort();
  nextion.print(attribute);
  nextion.print('=');
  nextion.print(value);
  sendTerminator();
}

int32_t scaledValue(float value, uint8_t decimals) {
  float scale = 1.0f;
  for (uint8_t i = 0; i < decimals; ++i) {
    scale *= 10.0f;
  }

  const float scaled = value * scale;
  return static_cast<int32_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

void setXFloatIfChanged(const char *component, float value, uint8_t decimals) {
  setValueIfChanged(component, scaledValue(value, decimals));
}

void setXFloatFromBms(const char *component, float value, uint8_t decimals) {
  setValue(component, scaledValue(value, decimals));
}

float loopFloatValue(float value, float maxValue, float step, int8_t &direction,
                     float minValue = 0.0f) {
  value += step * static_cast<float>(direction);
  if (value >= maxValue) {
    value = maxValue;
    direction = -1;
  } else if (value <= minValue) {
    value = minValue;
    direction = 1;
  }

  return value;
}

uint32_t loopUnsignedValue(uint32_t value, uint32_t maxValue, uint32_t step,
                           int8_t &direction) {
  int32_t nextValue =
      static_cast<int32_t>(value) + (static_cast<int32_t>(step) * direction);

  if (nextValue >= static_cast<int32_t>(maxValue)) {
    nextValue = static_cast<int32_t>(maxValue);
    direction = -1;
  } else if (nextValue <= 0) {
    nextValue = 0;
    direction = 1;
  }

  return static_cast<uint32_t>(nextValue);
}

uint16_t readU16BE(const uint8_t *p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

int16_t readI16BE(const uint8_t *p) {
  return static_cast<int16_t>(readU16BE(p));
}

uint32_t readU32BE(const uint8_t *p) {
  return (static_cast<uint32_t>(p[0]) << 24) |
         (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) |
         static_cast<uint32_t>(p[3]);
}

int32_t readI32BE(const uint8_t *p) {
  return static_cast<int32_t>(readU32BE(p));
}

float absoluteFloat(float value) {
  return value < 0.0f ? -value : value;
}

float convertMilliVoltsToVolts(uint16_t milliVolts) {
  return static_cast<float>(milliVolts) * MILLIVOLTS_TO_VOLTS;
}

float convertTenthsToFloat(int32_t valueInTenths) {
  return static_cast<float>(valueInTenths) * PACK_CURRENT_SCALE;
}

float convertMicroAhToAh(uint32_t microAh) {
  return static_cast<float>(microAh) * MICRO_AH_TO_AH;
}

float computeAverageBmsTemperature(const BmsTelemetry &source) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < ANT_TEMP_COUNT; ++i) {
    sum += source.temps[i];
  }

  return sum / static_cast<float>(ANT_TEMP_COUNT);
}

void publishBmsTelemetry() {
  const float packCurrent = absoluteFloat(bmsTelemetry.packCurrent);
  const float packPower = absoluteFloat(bmsTelemetry.packVoltage * bmsTelemetry.packCurrent);
  const float averageTemperature = computeAverageBmsTemperature(bmsTelemetry);

  setXFloatFromBms("x3", bmsTelemetry.soc, 1);
  setXFloatFromBms("x4", bmsTelemetry.packVoltage, 1);
  setXFloatFromBms("x5", packCurrent, 1);
  setXFloatFromBms("x6", packPower, 1);
  setXFloatFromBms("x2", averageTemperature, 1);
}

void publishSimulatedBmsTelemetry() {
  setXFloatFromBms("x3", loopedTelemetry.bmsSoc, 1);
  setXFloatFromBms("x4", loopedTelemetry.bmsVoltage, 1);
  setXFloatFromBms("x5", loopedTelemetry.bmsCurrent, 1);
  setXFloatFromBms("x6", loopedTelemetry.bmsPower, 1);
  setXFloatFromBms("x2", loopedTelemetry.bmsTemperature, 1);
}

bool hasFreshBmsTelemetry(uint32_t now) {
  return hasBmsTelemetry && (now - lastBmsFrame < BMS_STALE_LOG_INTERVAL_MS);
}

void publishBmsDisplay(uint32_t now) {
  if (hasFreshBmsTelemetry(now)) {
    publishBmsTelemetry();
  } else {
    publishSimulatedBmsTelemetry();
  }
}

void publishLoopedTelemetry() {
  setXFloatIfChanged("x0", loopedTelemetry.speedKmh, 1);
  setValueIfChanged("x1", static_cast<int32_t>(loopedTelemetry.motorRpm));

  setXFloatIfChanged("x11", loopedTelemetry.mpptVoltage, 1);
  setXFloatIfChanged("x10", loopedTelemetry.secondaryVoltage, 1);
  setXFloatIfChanged("x9", loopedTelemetry.mpptCurrent, 1);
  setXFloatIfChanged("x7", loopedTelemetry.mpptTemperature, 1);
  setXFloatIfChanged("x8", loopedTelemetry.mpptPower, 1);

  setXFloatIfChanged("x12", loopedTelemetry.driveVoltage, 1);
  setXFloatIfChanged("x13", loopedTelemetry.driveCurrent, 1);
  setXFloatIfChanged("x14", loopedTelemetry.drivePower, 1);
  setXFloatIfChanged("x15", loopedTelemetry.driveTemperature, 1);

  setValueIfChanged("z0", loopedTelemetry.centerGauge);
}

bool hasFreshTpmsWheel(uint8_t wheelId, uint32_t now) {
  return tpmsWheels[wheelId].lastMessageMs != 0 &&
         (now - tpmsWheels[wheelId].lastMessageMs <= TPMS_SENSOR_TIMEOUT_MS);
}

void publishTpmsDisplay(uint32_t now) {
  const char *const pressureComponents[5] = {"", "x16", "x17", "x18", nullptr};
  const float simulatedPressures[5] = {
      0.0f,
      loopedTelemetry.tpmsFrontLeft,
      loopedTelemetry.tpmsFrontRight,
      loopedTelemetry.tpmsMiddle,
      0.0f,
  };

  for (uint8_t wheelId = 1; wheelId <= 4; ++wheelId) {
    if (pressureComponents[wheelId] == nullptr) {
      continue;
    }

    const float pressure =
        hasFreshTpmsWheel(wheelId, now) ? tpmsWheels[wheelId].pressureBar
                                        : simulatedPressures[wheelId];
    setXFloatIfChanged(pressureComponents[wheelId], pressure, 1);
  }
}

void initialiseTpmsCan() {
#if defined(ARDUINO_ARCH_ESP32)
  twai_general_config_t generalConfig =
      TWAI_GENERAL_CONFIG_DEFAULT(TPMS_CAN_TX_PIN, TPMS_CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  tpmsCanReady =
      twai_driver_install(&generalConfig, &timingConfig, &filterConfig) == ESP_OK &&
      twai_start() == ESP_OK;

  Serial.println(tpmsCanReady ? "TPMS CAN started." : "TPMS CAN failed to start.");
#else
  tpmsCanReady = false;
  Serial.println("TPMS TWAI is available only on ESP32 builds.");
#endif
}

void pollTpmsCan() {
#if defined(ARDUINO_ARCH_ESP32)
  if (!tpmsCanReady) {
    return;
  }

  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.identifier != TPMS_CAN_MESSAGE_ID || message.data_length_code < 6) {
      continue;
    }

    const uint8_t wheelId = message.data[0];
    if (wheelId < 1 || wheelId > 4) {
      continue;
    }

    const int rawPressure = (static_cast<int>(message.data[1]) << 8) | message.data[2];
    TpmsWheel &wheel = tpmsWheels[wheelId];
    wheel.pressureBar = (static_cast<float>(rawPressure) * 0.0101f) + 0.35f;
    wheel.temperatureC = message.data[3];
    wheel.batteryV = static_cast<float>(message.data[4]) * 0.1237f;
    wheel.leakingAir = bitRead(message.data[5], 0);
    wheel.extremeTemperature = bitRead(message.data[5], 4);
    wheel.lastMessageMs = millis();
  }
#endif
}

void updateLoopedTelemetry() {
  static int8_t directions[] = {
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  };
  uint8_t directionIndex = 0;

  loopedTelemetry.bmsSoc =
      loopFloatValue(loopedTelemetry.bmsSoc, LOOPED_BMS_SOC_MAX, LOOPED_BMS_SOC_STEP,
                     directions[directionIndex++], LOOPED_BMS_SOC_MIN);
  loopedTelemetry.bmsVoltage =
      loopFloatValue(loopedTelemetry.bmsVoltage, LOOPED_BMS_VOLTAGE_MAX,
                     LOOPED_BMS_VOLTAGE_STEP, directions[directionIndex++]);
  loopedTelemetry.bmsCurrent =
      loopFloatValue(loopedTelemetry.bmsCurrent, LOOPED_BMS_CURRENT_MAX,
                     LOOPED_BMS_CURRENT_STEP, directions[directionIndex++]);
  loopedTelemetry.bmsPower =
      loopFloatValue(loopedTelemetry.bmsPower, LOOPED_BMS_POWER_MAX, LOOPED_BMS_POWER_STEP,
                     directions[directionIndex++]);
  loopedTelemetry.bmsTemperature = loopFloatValue(
      loopedTelemetry.bmsTemperature, LOOPED_BMS_TEMPERATURE_MAX,
      LOOPED_BMS_TEMPERATURE_STEP, directions[directionIndex++]);
  loopedTelemetry.mpptVoltage =
      loopFloatValue(loopedTelemetry.mpptVoltage, LOOPED_MPPT_VOLTAGE_MAX,
                     LOOPED_MPPT_VOLTAGE_STEP, directions[directionIndex++]);
  loopedTelemetry.mpptCurrent =
      loopFloatValue(loopedTelemetry.mpptCurrent, LOOPED_MPPT_CURRENT_MAX,
                     LOOPED_MPPT_CURRENT_STEP, directions[directionIndex++]);
  loopedTelemetry.mpptPower =
      loopFloatValue(loopedTelemetry.mpptPower, LOOPED_MPPT_POWER_MAX,
                     LOOPED_MPPT_POWER_STEP, directions[directionIndex++]);
  loopedTelemetry.mpptTemperature = loopFloatValue(
      loopedTelemetry.mpptTemperature, LOOPED_MPPT_TEMPERATURE_MAX,
      LOOPED_MPPT_TEMPERATURE_STEP, directions[directionIndex++]);
  loopedTelemetry.secondaryVoltage = loopFloatValue(
      loopedTelemetry.secondaryVoltage, LOOPED_SECONDARY_VOLTAGE_MAX,
      LOOPED_SECONDARY_VOLTAGE_STEP, directions[directionIndex++]);
  loopedTelemetry.speedKmh =
      loopFloatValue(loopedTelemetry.speedKmh, LOOPED_SPEED_MAX, LOOPED_SPEED_STEP,
                     directions[directionIndex++]);
  loopedTelemetry.motorRpm =
      loopUnsignedValue(loopedTelemetry.motorRpm, LOOPED_RPM_MAX, LOOPED_RPM_STEP,
                        directions[directionIndex++]);
  loopedTelemetry.drivePower =
      loopFloatValue(loopedTelemetry.drivePower, LOOPED_DRIVE_POWER_MAX,
                     LOOPED_DRIVE_POWER_STEP, directions[directionIndex++]);
  loopedTelemetry.driveVoltage =
      loopFloatValue(loopedTelemetry.driveVoltage, LOOPED_DRIVE_VOLTAGE_MAX,
                     LOOPED_DRIVE_VOLTAGE_STEP, directions[directionIndex++]);
  loopedTelemetry.driveCurrent =
      loopFloatValue(loopedTelemetry.driveCurrent, LOOPED_DRIVE_CURRENT_MAX,
                     LOOPED_DRIVE_CURRENT_STEP, directions[directionIndex++]);
  loopedTelemetry.driveTemperature = loopFloatValue(
      loopedTelemetry.driveTemperature, LOOPED_DRIVE_TEMPERATURE_MAX,
      LOOPED_DRIVE_TEMPERATURE_STEP, directions[directionIndex++]);
  loopedTelemetry.tpmsFrontLeft =
      loopFloatValue(loopedTelemetry.tpmsFrontLeft, LOOPED_TPMS_PRESSURE_MAX,
                     LOOPED_TPMS_PRESSURE_STEP, directions[directionIndex++]);
  loopedTelemetry.tpmsFrontRight =
      loopFloatValue(loopedTelemetry.tpmsFrontRight, LOOPED_TPMS_PRESSURE_MAX,
                     LOOPED_TPMS_PRESSURE_STEP, directions[directionIndex++]);
  loopedTelemetry.tpmsMiddle =
      loopFloatValue(loopedTelemetry.tpmsMiddle, LOOPED_TPMS_PRESSURE_MAX,
                     LOOPED_TPMS_PRESSURE_STEP, directions[directionIndex++]);
  loopedTelemetry.centerGauge = static_cast<uint16_t>(
      loopUnsignedValue(loopedTelemetry.centerGauge, LOOPED_CENTER_GAUGE_MAX,
                        LOOPED_CENTER_GAUGE_STEP, directions[directionIndex++]));
}

void debugPrintHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void debugPrintHexBlock(const uint8_t *data, size_t length) {
  if (!ENABLE_BMS_SERIAL_DEBUG || !BMS_DEBUG_PRINT_RAW_FRAME) {
    return;
  }

  for (size_t i = 0; i < length; ++i) {
    if ((i % BMS_DEBUG_RAW_BYTES_PER_LINE) == 0) {
      Serial.println();
      Serial.print("[BMS raw ");
      if (i < 100) {
        Serial.print('0');
      }
      if (i < 10) {
        Serial.print('0');
      }
      Serial.print(i);
      Serial.print("] ");
    }

    debugPrintHexByte(data[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void clearBmsRxBuffer(const char *reason) {
  if (bmsRxLength > 0) {
    bmsDroppedNoiseBytes += static_cast<uint32_t>(bmsRxLength);
    if (ENABLE_BMS_SERIAL_DEBUG) {
      Serial.print("[BMS parser] cleared ");
      Serial.print(bmsRxLength);
      Serial.print(" stale byte(s)");
      if (reason != nullptr) {
        Serial.print(" before ");
        Serial.print(reason);
      }
      Serial.println();
    }
  }

  bmsRxLength = 0;
}

void debugPrintBmsTelemetry(const BmsTelemetry &source) {
  if (!ENABLE_BMS_SERIAL_DEBUG) {
    return;
  }

  Serial.print("[BMS decoded] Vpack=");
  Serial.print(source.packVoltage, 1);
  Serial.print("V Ipack=");
  Serial.print(source.packCurrent, 1);
  Serial.print("A SOC=");
  Serial.print(source.soc, 0);
  Serial.print("% Cap=");
  Serial.print(source.capRemainAh, 2);
  Serial.print('/');
  Serial.print(source.capTotalAh, 2);
  Serial.print("Ah Cycles=");
  Serial.print(source.cycles);
  Serial.print(" MOS(chg,dsg)=");
  Serial.print(source.mosCharge ? 1 : 0);
  Serial.print(',');
  Serial.println(source.mosDischarge ? 1 : 0);

  Serial.print("[BMS cells] active=");
  Serial.print(source.activeCellCount);
  Serial.print(" min=");
  Serial.print(source.minCell, 3);
  Serial.print("V #");
  Serial.print(source.minCellId);
  Serial.print(" max=");
  Serial.print(source.maxCell, 3);
  Serial.print("V #");
  Serial.print(source.maxCellId);
  Serial.print(" avg=");
  Serial.print(source.avgCell, 3);
  Serial.print("V delta=");
  Serial.print(source.deltaCell, 3);
  Serial.println("V");

  Serial.print("[BMS temps] ");
  for (uint8_t i = 0; i < ANT_TEMP_COUNT; ++i) {
    Serial.print("T");
    Serial.print(i + 1);
    Serial.print('=');
    Serial.print(source.temps[i], 1);
    if (i < (ANT_TEMP_COUNT - 1)) {
      Serial.print("C ");
    }
  }
  Serial.println("C");

  Serial.print("[BMS cell volts] ");
  for (uint8_t i = 0; i < ANT_CELL_COUNT; ++i) {
    if (source.cells[i] <= 0.0f) {
      continue;
    }

    Serial.print(i + 1);
    Serial.print('=');
    Serial.print(source.cells[i], 3);
    Serial.print("V ");
  }
  Serial.println();
}

void debugPrintBmsFrame(const uint8_t *frame, const BmsTelemetry &decoded) {
  if (!ENABLE_BMS_SERIAL_DEBUG) {
    return;
  }

  Serial.println();
  Serial.print("[BMS frame #");
  Serial.print(bmsFrameCount);
  Serial.print("] bytes=");
  Serial.print(ANT_FRAME_SIZE);
  Serial.print(" responseBytes=");
  Serial.print(bmsBytesThisResponse);
  Serial.print(" rxOverflow=");
  Serial.print(bmsRxOverflowCount);
  Serial.print(" droppedNoise=");
  Serial.println(bmsDroppedNoiseBytes);

  debugPrintHexBlock(frame, ANT_FRAME_SIZE);
  debugPrintBmsTelemetry(decoded);
}

void debugPrintBmsRequest() {
  if (!ENABLE_BMS_SERIAL_DEBUG) {
    return;
  }

  Serial.print("[BMS request #");
  Serial.print(bmsRequestCount);
  Serial.print("] t=");
  Serial.print(millis());
  Serial.print("ms cmd=");
  for (size_t i = 0; i < sizeof(ANT_REQUEST_CMD); ++i) {
    debugPrintHexByte(ANT_REQUEST_CMD[i]);
    if (i < sizeof(ANT_REQUEST_CMD) - 1) {
      Serial.print(' ');
    }
  }
  Serial.print(" pins RX=");
  Serial.print(BMS_RX_PIN);
  Serial.print(" TX=");
  Serial.print(BMS_TX_PIN);
  Serial.print(" baud=");
  Serial.print(BMS_BAUD);
  Serial.print(" rxIdle=");
  Serial.println(digitalRead(BMS_RX_PIN) == HIGH ? "HIGH" : "LOW");
}

void debugPrintBmsTimeout(uint32_t now) {
  if (!ENABLE_BMS_SERIAL_DEBUG) {
    return;
  }

  Serial.print("[BMS timeout #");
  Serial.print(++bmsTimeoutCount);
  Serial.print("] request #");
  Serial.print(bmsRequestCount);
  Serial.print(" waited=");
  Serial.print(now - lastBmsRequest);
  Serial.print("ms responseBytes=");
  Serial.print(bmsBytesThisResponse);
  Serial.print(" buffered=");
  Serial.print(bmsRxLength);
  Serial.print(" echoDiscarded=");
  Serial.print(bmsEchoBytesDiscarded);
  Serial.print(" rxIdle=");
  Serial.print(digitalRead(BMS_RX_PIN) == HIGH ? "HIGH" : "LOW");
  Serial.print(" hasTelemetry=");
  Serial.println(hasBmsTelemetry ? "yes" : "no");

  if (bmsRxLength > 0) {
    const size_t debugLength = (bmsRxLength < ANT_FRAME_SIZE) ? bmsRxLength : ANT_FRAME_SIZE;
    Serial.print("[BMS buffered preview]");
    debugPrintHexBlock(bmsRxBuffer, debugLength);
  }
}

int findBmsHeader(const uint8_t *data, size_t length) {
  if (length < sizeof(ANT_HEADER)) {
    return -1;
  }

  for (size_t i = 0; i <= length - sizeof(ANT_HEADER); ++i) {
    if (data[i] == ANT_HEADER[0] && data[i + 1] == ANT_HEADER[1] &&
        data[i + 2] == ANT_HEADER[2] && data[i + 3] == ANT_HEADER[3]) {
      return static_cast<int>(i);
    }
  }

  return -1;
}

bool isLikelyBmsRequestEcho(const uint8_t *data, size_t length) {
  if (length < 2 || length > sizeof(ANT_REQUEST_CMD)) {
    return false;
  }

  return data[0] == ANT_REQUEST_CMD[0] && data[1] == ANT_REQUEST_CMD[1];
}

void discardLikelyBmsRequestEcho() {
  if (!isLikelyBmsRequestEcho(bmsRxBuffer, bmsRxLength)) {
    return;
  }

  bmsEchoBytesDiscarded += static_cast<uint32_t>(bmsRxLength);
  if (ENABLE_BMS_SERIAL_DEBUG) {
    Serial.print("[BMS echo] discarded ");
    Serial.print(bmsRxLength);
    Serial.print(" byte request echo/noise:");
    debugPrintHexBlock(bmsRxBuffer, bmsRxLength);
  }

  bmsRxLength = 0;
}

void parseBmsCellBlock(const uint8_t *frame, BmsTelemetry &target) {
  float cellSum = 0.0f;
  bool hasMinMax = false;

  for (uint8_t i = 0; i < ANT_CELL_COUNT; ++i) {
    const size_t offset = OFFSET_CELLS + (i * 2);
    const uint16_t cellMilliVolts = readU16BE(&frame[offset]);
    if (cellMilliVolts == 0) {
      continue;
    }

    target.cells[i] = convertMilliVoltsToVolts(cellMilliVolts);
    target.activeCellCount++;
    cellSum += target.cells[i];

    if (!hasMinMax || target.cells[i] < target.minCell) {
      target.minCell = target.cells[i];
      target.minCellId = i + 1;
    }
    if (!hasMinMax || target.cells[i] > target.maxCell) {
      target.maxCell = target.cells[i];
      target.maxCellId = i + 1;
    }

    hasMinMax = true;
  }

  if (target.activeCellCount > 0) {
    target.avgCell = cellSum / static_cast<float>(target.activeCellCount);
    target.deltaCell = target.maxCell - target.minCell;
  }
}

void parseBmsTemperatureBlock(const uint8_t *frame, BmsTelemetry &target) {
  for (uint8_t i = 0; i < ANT_TEMP_COUNT; ++i) {
    const size_t offset = OFFSET_TEMPS + (i * 2);
    target.temps[i] = static_cast<float>(readI16BE(&frame[offset]));
  }
}

void parseBmsFrame(const uint8_t *frame) {
  BmsTelemetry parsed;

  parsed.packVoltage =
      static_cast<float>(readU16BE(&frame[OFFSET_PACK_VOLTAGE])) * PACK_VOLTAGE_SCALE;
  parseBmsCellBlock(frame, parsed);
  parsed.packCurrent = convertTenthsToFloat(readI32BE(&frame[OFFSET_PACK_CURRENT]));
  parsed.soc = static_cast<float>(frame[OFFSET_SOC]);
  parsed.capTotalAh = convertMicroAhToAh(readU32BE(&frame[OFFSET_CAP_TOTAL]));
  parsed.capRemainAh = convertMicroAhToAh(readU32BE(&frame[OFFSET_CAP_REMAIN]));
  parsed.cycles = readU16BE(&frame[OFFSET_CYCLES]);
  parseBmsTemperatureBlock(frame, parsed);
  parsed.mosCharge = frame[OFFSET_MOS_CHARGE] != 0;
  parsed.mosDischarge = frame[OFFSET_MOS_DISCHARGE] != 0;

  bmsTelemetry = parsed;
  hasBmsTelemetry = true;
  lastBmsFrame = millis();
  awaitingBmsResponse = false;
  ++bmsFrameCount;
  debugPrintBmsFrame(frame, bmsTelemetry);
  publishBmsTelemetry();
}

void compactBmsRxBuffer(size_t start, size_t count) {
  const size_t end = start + count;
  if (end >= bmsRxLength) {
    bmsRxLength = 0;
    return;
  }

  const size_t remaining = bmsRxLength - end;
  memmove(bmsRxBuffer, bmsRxBuffer + end, remaining);
  bmsRxLength = remaining;
}

void processBmsRxBuffer() {
  while (true) {
    discardLikelyBmsRequestEcho();

    const int headerIndex = findBmsHeader(bmsRxBuffer, bmsRxLength);
    if (headerIndex < 0) {
      if (bmsRxLength > (ANT_FRAME_SIZE * 2)) {
        bmsDroppedNoiseBytes += bmsRxLength;
        if (ENABLE_BMS_SERIAL_DEBUG) {
          Serial.print("[BMS parser] dropped ");
          Serial.print(bmsRxLength);
          Serial.println(" bytes with no ANT header");
        }
        bmsRxLength = 0;
      }
      return;
    }

    if (headerIndex > 0) {
      bmsDroppedNoiseBytes += static_cast<uint32_t>(headerIndex);
      if (ENABLE_BMS_SERIAL_DEBUG) {
        Serial.print("[BMS parser] resync dropped ");
        Serial.print(headerIndex);
        Serial.println(" byte(s) before ANT header");
      }
      compactBmsRxBuffer(0, static_cast<size_t>(headerIndex));
      continue;
    }

    if (bmsRxLength < ANT_FRAME_SIZE) {
      return;
    }

    parseBmsFrame(bmsRxBuffer);
    compactBmsRxBuffer(0, ANT_FRAME_SIZE);
  }
}

void pollBmsUart() {
  bool sawByte = false;

  while (bmsSerial.available() > 0) {
    const int value = bmsSerial.read();
    if (value < 0) {
      break;
    }

    sawByte = true;
    ++bmsBytesThisResponse;
    if (bmsRxLength < sizeof(bmsRxBuffer)) {
      bmsRxBuffer[bmsRxLength++] = static_cast<uint8_t>(value);
    } else {
      ++bmsRxOverflowCount;
      if (ENABLE_BMS_SERIAL_DEBUG) {
        Serial.println("[BMS RX] local buffer overflow; clearing buffered bytes");
      }
      bmsRxLength = 0;
    }
  }

  if (sawByte || bmsRxLength >= sizeof(ANT_HEADER)) {
    processBmsRxBuffer();
  }
}

void sendBmsRequestIfNeeded() {
  if (awaitingBmsResponse) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastBmsRequest >= BMS_REQUEST_INTERVAL_MS) {
    activateBmsPort();
    bmsSerial.flush();
    clearBmsRxBuffer("request");
    bmsSerial.write(ANT_REQUEST_CMD, sizeof(ANT_REQUEST_CMD));
    lastBmsRequest = now;
    bmsBytesThisResponse = 0;
    awaitingBmsResponse = true;
    ++bmsRequestCount;
    debugPrintBmsRequest();
  }
}

void changePage(const char *pageName) {
  activateNextionPort();
  nextion.print("page ");
  nextion.print(pageName);
  sendTerminator();
}

const char *componentName(uint8_t componentId) {
  switch (componentId) {
    case 0:
      return "page0";
    case 1:
      return "p0";
    case 2:
      return "x0_speed";
    case 3:
      return "t0_kmh";
    case 4:
      return "x1_rpm";
    case 5:
      return "t1_rpm";
    case 6:
      return "tm0";
    case 7:
      return "t2_soc";
    case 8:
      return "t3_volt";
    case 9:
      return "t4_amp";
    case 10:
      return "t5_pow";
    case 11:
      return "t6_tmp";
    case 12:
      return "x2_bms_tmp";
    case 13:
      return "x3_bms_soc";
    case 14:
      return "x4_bms_volt";
    case 15:
      return "x5_bms_amp";
    case 16:
      return "x6_bms_pow";
    case 17:
      return "t9_mppt_amp";
    case 18:
      return "x9_mppt_amp";
    case 19:
      return "t10_secondary_volt";
    case 20:
      return "x10_secondary_volt";
    case 21:
      return "t11_mppt_volt";
    case 22:
      return "x11_mppt_volt";
    case 23:
      return "t7_mppt";
    case 24:
      return "x7_mppt_temp";
    case 25:
      return "t12_mppt_temp";
    case 26:
      return "x8_mppt_pow";
    case 27:
      return "t8_mppt_pow";
    case 28:
      return "t13_bms";
    case 29:
      return "t14_drive_volt";
    case 30:
      return "x12_drive_volt";
    case 31:
      return "t15_drive_amp";
    case 32:
      return "x13_drive_amp";
    case 33:
      return "t16_drive_pow";
    case 34:
      return "x14_drive_pow";
    case 35:
      return "x15_drive_temp";
    case 36:
      return "t17_drive_temp";
    case 37:
      return "z0_center_gauge";
    default:
      return "unknown";
  }
}

void clearDisplayValues() {
  setXFloatIfChanged("x0", 0.0f, 1);
  setValueIfChanged("x1", 0);

  setXFloatIfChanged("x2", 0.0f, 1);
  setXFloatIfChanged("x3", 0.0f, 1);
  setXFloatIfChanged("x4", 0.0f, 1);
  setXFloatIfChanged("x5", 0.0f, 1);
  setXFloatIfChanged("x6", 0.0f, 1);

  setXFloatIfChanged("x11", 0.0f, 1);
  setXFloatIfChanged("x10", 0.0f, 1);
  setXFloatIfChanged("x9", 0.0f, 1);
  setXFloatIfChanged("x7", 0.0f, 1);
  setXFloatIfChanged("x8", 0.0f, 1);

  setXFloatFromBms("x12", 0.0f, 1);
  setXFloatFromBms("x13", 0.0f, 1);
  setXFloatFromBms("x14", 0.0f, 1);
  setXFloatFromBms("x15", 0.0f, 1);
  setXFloatIfChanged("x16", 0.0f, 1);
  setXFloatIfChanged("x17", 0.0f, 1);
  setXFloatIfChanged("x18", 0.0f, 1);

  setValueIfChanged("z0", 0);
}

void handleTouchEvent(uint8_t pageId, uint8_t componentId, bool pressed) {
  Serial.print("Nextion touch page=");
  Serial.print(pageId);
  Serial.print(" component=");
  Serial.print(componentId);
  Serial.print(" (");
  Serial.print(pageId == PAGE0_ID ? componentName(componentId) : "unknown page");
  Serial.print(") pressed=");
  Serial.println(pressed ? "yes" : "no");
}

void handleNextionReturnCode(uint8_t code) {
  switch (code) {
    case 0x00:
      Serial.println("Nextion invalid instruction");
      break;
    case 0x01:
      break;
    case 0x86:
      Serial.println("Nextion entered sleep");
      break;
    case 0x87:
      Serial.println("Nextion woke from sleep");
      break;
    case 0x88:
      Serial.println("Nextion startup ready");
      break;
    default:
      Serial.print("Nextion return code: 0x");
      Serial.println(code, HEX);
      break;
  }
}

void handleNextionPacket(const uint8_t *packet, size_t length) {
  if (length == 0) {
    return;
  }

  switch (packet[0]) {
    case 0x00:
      if (length == 3 && packet[1] == 0x00 && packet[2] == 0x00) {
        Serial.println("Nextion power-on preamble");
      } else {
        handleNextionReturnCode(packet[0]);
      }
      break;
    case 0x65:
      if (length >= 4) {
        handleTouchEvent(packet[1], packet[2], packet[3] != 0);
      }
      break;
    case 0x66:
      if (length >= 2) {
        Serial.print("Nextion page changed to ");
        Serial.println(packet[1]);
      }
      break;
    case 0x70:
      Serial.print("Nextion string: ");
      for (size_t i = 1; i < length; ++i) {
        Serial.write(packet[i]);
      }
      Serial.println();
      break;
    case 0x71:
      if (length >= 5) {
        const uint32_t value = static_cast<uint32_t>(packet[1]) |
                               (static_cast<uint32_t>(packet[2]) << 8) |
                               (static_cast<uint32_t>(packet[3]) << 16) |
                               (static_cast<uint32_t>(packet[4]) << 24);
        Serial.print("Nextion number: ");
        Serial.println(value);
      }
      break;
    default:
      if (length == 1) {
        handleNextionReturnCode(packet[0]);
      } else {
        Serial.print("Unhandled Nextion packet, first byte: 0x");
        Serial.println(packet[0], HEX);
      }
      break;
  }
}

void readNextion() {
  static uint8_t packet[64];
  static size_t packetLength = 0;
  static uint8_t terminatorCount = 0;

  while (nextion.available() > 0) {
    const uint8_t byteRead = nextion.read();

    if (byteRead == NEXTION_TERMINATOR) {
      ++terminatorCount;
      if (terminatorCount == 3) {
        handleNextionPacket(packet, packetLength);
        packetLength = 0;
        terminatorCount = 0;
      }
      continue;
    }

    while (terminatorCount > 0 && packetLength < sizeof(packet)) {
      packet[packetLength++] = NEXTION_TERMINATOR;
      --terminatorCount;
    }

    if (packetLength < sizeof(packet)) {
      packet[packetLength++] = byteRead;
    } else {
      packetLength = 0;
      terminatorCount = 0;
      Serial.println("Nextion packet overflow; packet dropped");
    }
  }
}

void initialiseNextion() {
  delay(500);
  activateNextionPort();

  nextionCommand("bkcmd=1");
  nextionCommand("recmod=0");
  nextionCommand("dim=100");
  changePage("page0");

  setAttributeValue("tm0.en", 0);
  clearDisplayValues();
}

void logBmsStatusIfNeeded(uint32_t now) {
  if (now - lastBmsStatusLog < BMS_STALE_LOG_INTERVAL_MS) {
    return;
  }

  if (!hasBmsTelemetry) {
    Serial.println("Waiting for ANT BMS frame...");
    lastBmsStatusLog = now;
    return;
  }

  if (now - lastBmsFrame >= BMS_STALE_LOG_INTERVAL_MS) {
    Serial.println("No fresh ANT BMS frame for >3s");
    lastBmsStatusLog = now;
  }
}

bool isBmsResponseWindowActive(uint32_t now) {
  if (!awaitingBmsResponse) {
    return false;
  }

  if (now - lastBmsRequest < BMS_RESPONSE_WINDOW_MS) {
    return true;
  }

  debugPrintBmsTimeout(now);
  clearBmsRxBuffer("next request");
  awaitingBmsResponse = false;
  return false;
}
}  // namespace

void setup() {

  Serial.begin(DEBUG_BAUD);

  initialiseNextion();
  initialiseTpmsCan();

  Serial.println("Teensy 4.1 Nextion page0 interface ready");
  Serial.print("ANT BMS reader on software serial RX=");
  Serial.print(BMS_RX_PIN);
  Serial.print(" TX=");
  Serial.print(BMS_TX_PIN);
  Serial.print(" baud=");
  Serial.println(BMS_BAUD);
  Serial.print("BMS serial debug=");
  Serial.println(ENABLE_BMS_SERIAL_DEBUG ? "enabled" : "disabled");
  Serial.println("Open the serial monitor at 115200 baud to watch BMS requests, raw frames, and decoded values.");
}

void loop() {
  pollTpmsCan();
  sendBmsRequestIfNeeded();

  if (isBmsResponseWindowActive(millis())) {
    pollBmsUart();
    logBmsStatusIfNeeded(millis());
    return;
  }

  activateNextionPort();
  readNextion();

  uint32_t now = millis();
  if (now - lastLoopedTelemetryUpdate >= LOOPED_TELEMETRY_INTERVAL_MS) {
    lastLoopedTelemetryUpdate = now;
    updateLoopedTelemetry();
    publishLoopedTelemetry();
    publishBmsDisplay(now);
    publishTpmsDisplay(now);
  }

  now = millis();
  if (now - lastPageHeartbeat >= PAGE_HEARTBEAT_INTERVAL_MS) {
    lastPageHeartbeat = now;
    nextionCommand("sendme");
  }

  logBmsStatusIfNeeded(millis());
}
