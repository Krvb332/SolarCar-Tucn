#include <Arduino.h>
#include <SoftwareSerial.h>

/*
  Teensy 4.1 <-> Nextion Intelligent NX8048P050-011C

  HMI inspected:
    C:\Users\Krab\Downloads\interfataSeptimiu.HMI

  Wiring used by this sketch:
    Teensy 4.1 pin 2, software TX  -> Nextion RX
    Teensy 4.1 pin 3, software RX  <- Nextion TX
    GND                    -> Nextion GND
    External 5 V supply    -> Nextion 5 V input

  Teensy 4.1 pins are 3.3 V and are not 5 V tolerant. If the Nextion TX line
  outputs 5 V, level-shift it before connecting it to Teensy pin 3.

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
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;
constexpr uint32_t PAGE_HEARTBEAT_INTERVAL_MS = 1000;
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
constexpr uint8_t NEXTION_TERMINATOR = 0xFF;
constexpr uint8_t PAGE0_ID = 0;

SoftwareSerial nextion(NEXTION_RX_PIN, NEXTION_TX_PIN);

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

Telemetry telemetry = {};

uint32_t lastTelemetryUpdate = 0;
uint32_t lastPageHeartbeat = 0;

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

  telemetry.bmsVoltage = loopFloatValue(telemetry.bmsVoltage, VOLTAGE_MAX, VOLTAGE_STEP,
                                        directions[directionIndex++]);
  telemetry.bmsCurrent = loopFloatValue(telemetry.bmsCurrent, CURRENT_MAX, CURRENT_STEP,
                                        directions[directionIndex++]);
  telemetry.bmsPower = loopFloatValue(telemetry.bmsPower, POWER_MAX, POWER_STEP,
                                      directions[directionIndex++]);
  telemetry.bmsTemperature = loopFloatValue(telemetry.bmsTemperature, TEMPERATURE_MAX,
                                            TEMPERATURE_STEP, directions[directionIndex++]);

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

  initialiseNextion();

  Serial.println("Teensy 4.1 Nextion page0 interface ready");
}

void loop() {
  const uint32_t now = millis();

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
