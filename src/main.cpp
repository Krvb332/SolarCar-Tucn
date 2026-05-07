#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>

/*
  Teensy 4.1 <-> Nextion Intelligent NX8048P050-011C

  HMI inspected:
    C:\Users\Krab\Downloads\interfataSeptimiu.HMI

  Wiring used by this sketch:
    Teensy 4.1 pin 2, software TX  -> Nextion RX
    Teensy 4.1 pin 3, software RX  <- Nextion TX
    Teensy 4.1 pin 17, Serial4 TX  -> ANT BMS RX
    Teensy 4.1 pin 16, Serial4 RX  <- ANT BMS TX
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
    x2    id=12  left Tmp, 1 decimal         t6  id=11 "Tmp"
    x3    id=13  left SOC, 1 decimal         t2  id=7  "SOC"
    x4    id=14  left Volt, 1 decimal        t3  id=8  "Volt"
    x5    id=15  left Amp, 1 decimal         t4  id=9  "Amp"
    x6    id=16  left Pow, 1 decimal         t5  id=10 "Pow"
    x11   id=22  right Volt, 1 decimal       t11 id=21 "Volt"
    x10   id=20  right SOC, 1 decimal        t10 id=19 "SOC"
    x9    id=18  right Amp, 1 decimal        t9  id=17 "Amp"
    x7    id=24  MPPT Temp, 1 decimal        t12 id=25 "Temp"
    x8    id=26  MPPT Pow, 1 decimal         t8  id=27 "Pow"
    x12   id=30  BMS Volt, 1 decimal         t14 id=29 "Volt"
    x13   id=32  BMS Amp, 1 decimal          t15 id=31 "Amp"
    x14   id=34  BMS Pow, 1 decimal          t16 id=33 "Pow"
    x15   id=35  BMS Tmp, 1 decimal          t17 id=36 "Tmp"
    z0    id=37  center gauge/progress value
*/

namespace {
constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t NEXTION_BAUD = 9600;
constexpr uint32_t BMS_BAUD = 19200;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;
constexpr uint32_t PAGE_HEARTBEAT_INTERVAL_MS = 1000;
constexpr uint32_t BMS_REQUEST_INTERVAL_MS = 1000;
constexpr uint32_t BMS_READ_SLICE_MS = 20;
constexpr float SPEED_MAX = 130.0f;
constexpr float SPEED_STEP = 0.8f;
constexpr uint32_t RPM_MAX = 1700;
constexpr uint32_t RPM_STEP = 50;
constexpr float TEMPERATURE_MAX = 100.0f;
constexpr float TEMPERATURE_STEP = 0.7f;
constexpr float SOC_MAX = 100.0f;
constexpr float SOC_STEP = 1.0f;
constexpr float VOLTAGE_MAX = 60.0f;
constexpr float VOLTAGE_STEP = 0.5f;
constexpr float CURRENT_MAX = 30.0f;
constexpr float CURRENT_STEP = 0.4f;
constexpr float POWER_MAX = 1800.0f;
constexpr float POWER_STEP = 25.0f;
constexpr uint32_t CENTER_GAUGE_MAX = 100;
constexpr uint32_t CENTER_GAUGE_STEP = 2;
constexpr uint8_t NEXTION_RX_PIN = 3;
constexpr uint8_t NEXTION_TX_PIN = 2;
constexpr uint8_t BMS_RX_PIN = 16;
constexpr uint8_t BMS_TX_PIN = 17;
constexpr uint8_t NEXTION_TERMINATOR = 0xFF;
constexpr uint8_t PAGE0_ID = 0;
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

SoftwareSerial nextion(NEXTION_RX_PIN, NEXTION_TX_PIN);
HardwareSerial &bmsSerial = Serial4;

struct Telemetry {
  float speedKmh;
  uint32_t motorRpm;

  float leftTemperature;
  float leftSoc;
  float leftVoltage;
  float leftCurrent;
  float leftPower;

  float rightVoltage;
  float rightSoc;
  float rightCurrent;
  float mpptTemperature;
  float mpptPower;

  float bmsVoltage;
  float bmsCurrent;
  float bmsPower;
  float bmsTemperature;

  uint16_t centerGauge;
};

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

Telemetry telemetry = {};
BmsTelemetry bmsTelemetry = {};

uint32_t lastTelemetryUpdate = 0;
uint32_t lastPageHeartbeat = 0;
uint32_t lastBmsRequest = 0;
uint32_t lastBmsFrame = 0;
uint8_t bmsRxBuffer[512] = {};
size_t bmsRxLength = 0;
bool hasBmsTelemetry = false;

void sendTerminator() {
  nextion.write(NEXTION_TERMINATOR);
  nextion.write(NEXTION_TERMINATOR);
  nextion.write(NEXTION_TERMINATOR);
}

void nextionCommand(const char *command) {
  nextion.print(command);
  sendTerminator();
}

void setValue(const char *component, int32_t value) {
  nextion.print(component);
  nextion.print(".val=");
  nextion.print(value);
  sendTerminator();
}

void setAttributeValue(const char *attribute, int32_t value) {
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

void setXFloat(const char *component, float value, uint8_t decimals) {
  setValue(component, scaledValue(value, decimals));
}

float loopFloatValue(float value, float maxValue, float step, int8_t &direction) {
  value += step * static_cast<float>(direction);
  if (value >= maxValue) {
    value = maxValue;
    direction = -1;
  } else if (value <= 0.0f) {
    value = 0.0f;
    direction = 1;
  }

  return value;
}

uint32_t loopUnsignedValue(uint32_t value, uint32_t maxValue, uint32_t step, int8_t &direction) {
  int32_t nextValue = static_cast<int32_t>(value) +
                      (static_cast<int32_t>(step) * direction);

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

void applyBmsTelemetryToDisplay() {
  telemetry.bmsVoltage = bmsTelemetry.packVoltage;
  telemetry.bmsCurrent = absoluteFloat(bmsTelemetry.packCurrent);
  telemetry.bmsPower = absoluteFloat(bmsTelemetry.packVoltage * bmsTelemetry.packCurrent);
  telemetry.bmsTemperature = computeAverageBmsTemperature(bmsTelemetry);
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
  applyBmsTelemetryToDisplay();
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
    const int headerIndex = findBmsHeader(bmsRxBuffer, bmsRxLength);
    if (headerIndex < 0) {
      if (bmsRxLength > (ANT_FRAME_SIZE * 2)) {
        bmsRxLength = 0;
      }
      return;
    }

    if (headerIndex > 0) {
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
  const uint32_t start = millis();

  while (millis() - start < BMS_READ_SLICE_MS) {
    while (bmsSerial.available() > 0) {
      const int value = bmsSerial.read();
      if (value < 0) {
        break;
      }

      if (bmsRxLength < sizeof(bmsRxBuffer)) {
        bmsRxBuffer[bmsRxLength++] = static_cast<uint8_t>(value);
      } else {
        bmsRxLength = 0;
      }
    }

    processBmsRxBuffer();
    delay(1);
  }
}

void sendBmsRequestIfNeeded() {
  const uint32_t now = millis();
  if (now - lastBmsRequest >= BMS_REQUEST_INTERVAL_MS) {
    bmsSerial.write(ANT_REQUEST_CMD, sizeof(ANT_REQUEST_CMD));
    bmsSerial.flush();
    lastBmsRequest = now;
  }
}

void changePage(const char *pageName) {
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
      return "x2_left_tmp";
    case 13:
      return "x3_left_soc";
    case 14:
      return "x4_left_volt";
    case 15:
      return "x5_left_amp";
    case 16:
      return "x6_left_pow";
    case 17:
      return "t9_right_amp";
    case 18:
      return "x9_right_amp";
    case 19:
      return "t10_right_soc";
    case 20:
      return "x10_right_soc";
    case 21:
      return "t11_right_volt";
    case 22:
      return "x11_right_volt";
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
      return "t14_bms_volt";
    case 30:
      return "x12_bms_volt";
    case 31:
      return "t15_bms_amp";
    case 32:
      return "x13_bms_amp";
    case 33:
      return "t16_bms_pow";
    case 34:
      return "x14_bms_pow";
    case 35:
      return "x15_bms_tmp";
    case 36:
      return "t17_bms_tmp";
    case 37:
      return "z0_center_gauge";
    default:
      return "unknown";
  }
}

void publishTelemetry() {
  setXFloat("x0", telemetry.speedKmh, 1);
  setValue("x1", static_cast<int32_t>(telemetry.motorRpm));

  setXFloat("x2", telemetry.leftTemperature, 1);
  setXFloat("x3", telemetry.leftSoc, 1);
  setXFloat("x4", telemetry.leftVoltage, 1);
  setXFloat("x5", telemetry.leftCurrent, 1);
  setXFloat("x6", telemetry.leftPower, 1);

  setXFloat("x11", telemetry.rightVoltage, 1);
  setXFloat("x10", telemetry.rightSoc, 1);
  setXFloat("x9", telemetry.rightCurrent, 1);
  setXFloat("x7", telemetry.mpptTemperature, 1);
  setXFloat("x8", telemetry.mpptPower, 1);

  setXFloat("x12", telemetry.bmsVoltage, 1);
  setXFloat("x13", telemetry.bmsCurrent, 1);
  setXFloat("x14", telemetry.bmsPower, 1);
  setXFloat("x15", telemetry.bmsTemperature, 1);

  setValue("z0", telemetry.centerGauge);
}

void simulateTelemetry() {
  static int8_t directions[] = {
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  };
  uint8_t directionIndex = 0;

  telemetry.speedKmh = loopFloatValue(telemetry.speedKmh, SPEED_MAX, SPEED_STEP,
                                      directions[directionIndex++]);
  telemetry.motorRpm = loopUnsignedValue(telemetry.motorRpm, RPM_MAX, RPM_STEP,
                                         directions[directionIndex++]);

  telemetry.leftTemperature = loopFloatValue(telemetry.leftTemperature, TEMPERATURE_MAX,
                                             TEMPERATURE_STEP, directions[directionIndex++]);
  telemetry.leftSoc = loopFloatValue(telemetry.leftSoc, SOC_MAX, SOC_STEP,
                                     directions[directionIndex++]);
  telemetry.leftVoltage = loopFloatValue(telemetry.leftVoltage, VOLTAGE_MAX, VOLTAGE_STEP,
                                         directions[directionIndex++]);
  telemetry.leftCurrent = loopFloatValue(telemetry.leftCurrent, CURRENT_MAX, CURRENT_STEP,
                                         directions[directionIndex++]);
  telemetry.leftPower = loopFloatValue(telemetry.leftPower, POWER_MAX, POWER_STEP,
                                       directions[directionIndex++]);

  telemetry.rightVoltage = loopFloatValue(telemetry.rightVoltage, VOLTAGE_MAX, VOLTAGE_STEP,
                                          directions[directionIndex++]);
  telemetry.rightSoc = loopFloatValue(telemetry.rightSoc, SOC_MAX, SOC_STEP,
                                      directions[directionIndex++]);
  telemetry.rightCurrent = loopFloatValue(telemetry.rightCurrent, CURRENT_MAX, CURRENT_STEP,
                                          directions[directionIndex++]);
  telemetry.mpptTemperature = loopFloatValue(telemetry.mpptTemperature, TEMPERATURE_MAX,
                                             TEMPERATURE_STEP, directions[directionIndex++]);
  telemetry.mpptPower = loopFloatValue(telemetry.mpptPower, POWER_MAX, POWER_STEP,
                                       directions[directionIndex++]);

  telemetry.centerGauge = static_cast<uint16_t>(loopUnsignedValue(
      telemetry.centerGauge, CENTER_GAUGE_MAX, CENTER_GAUGE_STEP, directions[directionIndex++]));
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

  nextionCommand("bkcmd=1");
  nextionCommand("recmod=0");
  nextionCommand("dim=100");
  changePage("page0");

  setAttributeValue("tm0.en", 0);
  publishTelemetry();
}
}  // namespace

void setup() {

  Serial.begin(DEBUG_BAUD);
  nextion.begin(NEXTION_BAUD);
  bmsSerial.begin(BMS_BAUD);

  initialiseNextion();

  Serial.println("Teensy 4.1 Nextion page0 interface ready");
  Serial.print("ANT BMS reader on Serial4 RX=");
  Serial.print(BMS_RX_PIN);
  Serial.print(" TX=");
  Serial.print(BMS_TX_PIN);
  Serial.print(" baud=");
  Serial.println(BMS_BAUD);
}

void loop() {
  const uint32_t now = millis();

  sendBmsRequestIfNeeded();
  pollBmsUart();
  readNextion();

  if (now - lastTelemetryUpdate >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryUpdate = now;
    simulateTelemetry();
    publishTelemetry();
  }

  if (now - lastPageHeartbeat >= PAGE_HEARTBEAT_INTERVAL_MS) {
    lastPageHeartbeat = now;
    nextionCommand("sendme");
  }
}
