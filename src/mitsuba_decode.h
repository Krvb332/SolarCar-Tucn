#pragma once

// Pure bit-unpacking for the Mitsuba motor controller CAN frames.
//
// Deliberately free of Arduino and FlexCAN dependencies so the exact same code
// can be compiled on a host and cross-checked against the working Python
// decoder in SolarCarDash (comm/mitsuba_serial.py).
//
// Authoritative signal definitions:
//   SolarCarDash/docs/protocols/can_protocol_rear_left_wheel.json
//
// Every frame is unpacked little-endian on the bit level: byte 0 is the least
// significant byte of a 64-bit word, then (word >> start_bit) & mask. Frames
// shorter than 8 bytes are zero padded, exactly like the Python side does with
// int.from_bytes(data + b"\0\0\0", "little").

#include <stddef.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// CAN identifiers (all extended, 29 bit, 500 kbit/s)
// ---------------------------------------------------------------------------

constexpr uint32_t MITSUBA_CAN_BAUD = 500000;

constexpr uint32_t MITSUBA_ID_FRAME0 = 0x08850225UL;  // Log_Res_Frm0_RL1, 8 bytes
constexpr uint32_t MITSUBA_ID_FRAME1 = 0x08950225UL;  // Log_Res_Frm1_RL1, 5 bytes
constexpr uint32_t MITSUBA_ID_FRAME2 = 0x08A50225UL;  // Log_Res_Frm2_RL1, 5 bytes

// Request Log_Req_RL1. The single payload byte is a bitmask:
// bit0 = send Frame 0, bit1 = send Frame 1, bit2 = send Frame 2.
constexpr uint32_t MITSUBA_ID_REQUEST_A = 0x08F85540UL;  // documented in the protocol JSON
constexpr uint32_t MITSUBA_ID_REQUEST_B = 0x08F89540UL;  // variant mentioned in docs/mitsuba.md
constexpr uint8_t MITSUBA_REQUEST_ALL_FRAMES = 0x07;

constexpr uint8_t MITSUBA_FRAME0_DLC = 8;
constexpr uint8_t MITSUBA_FRAME1_DLC = 5;
constexpr uint8_t MITSUBA_FRAME2_DLC = 5;

// ---------------------------------------------------------------------------
// Signal placement, straight out of the protocol JSON
// ---------------------------------------------------------------------------

// Frame 0
constexpr uint8_t MITSUBA_F0_BATTERY_VOLTAGE_BIT = 0;
constexpr uint8_t MITSUBA_F0_BATTERY_VOLTAGE_LEN = 10;
constexpr uint8_t MITSUBA_F0_BATTERY_CURRENT_BIT = 10;
constexpr uint8_t MITSUBA_F0_BATTERY_CURRENT_LEN = 9;
constexpr uint8_t MITSUBA_F0_CURRENT_DIRECTION_BIT = 19;
constexpr uint8_t MITSUBA_F0_MOTOR_CURRENT_PEAK_BIT = 20;
constexpr uint8_t MITSUBA_F0_MOTOR_CURRENT_PEAK_LEN = 10;
constexpr uint8_t MITSUBA_F0_FET_TEMPERATURE_BIT = 30;
constexpr uint8_t MITSUBA_F0_FET_TEMPERATURE_LEN = 5;
constexpr uint8_t MITSUBA_F0_RPM_BIT = 35;
constexpr uint8_t MITSUBA_F0_RPM_LEN = 12;
constexpr uint8_t MITSUBA_F0_PWM_DUTY_BIT = 47;
constexpr uint8_t MITSUBA_F0_PWM_DUTY_LEN = 10;
constexpr uint8_t MITSUBA_F0_LEAD_ANGLE_BIT = 57;
constexpr uint8_t MITSUBA_F0_LEAD_ANGLE_LEN = 7;

constexpr float MITSUBA_BATTERY_VOLTAGE_LSB = 0.5f;   // V per LSB
constexpr float MITSUBA_BATTERY_CURRENT_LSB = 1.0f;   // A per LSB
constexpr float MITSUBA_MOTOR_CURRENT_LSB = 1.0f;     // A per LSB
constexpr float MITSUBA_FET_TEMPERATURE_LSB = 5.0f;   // degC per LSB
constexpr float MITSUBA_PERCENT_LSB = 0.5f;           // % per LSB
constexpr float MITSUBA_LEAD_ANGLE_LSB = 0.5f;        // deg per LSB

// Frame 1
constexpr uint8_t MITSUBA_F1_POWER_MODE_BIT = 0;
constexpr uint8_t MITSUBA_F1_CTRL_MODE_BIT = 1;
constexpr uint8_t MITSUBA_F1_ACCEL_POSITION_BIT = 2;
constexpr uint8_t MITSUBA_F1_ACCEL_POSITION_LEN = 10;
constexpr uint8_t MITSUBA_F1_REGEN_VR_BIT = 12;
constexpr uint8_t MITSUBA_F1_REGEN_VR_LEN = 10;
constexpr uint8_t MITSUBA_F1_DIGIT_SW_BIT = 22;
constexpr uint8_t MITSUBA_F1_DIGIT_SW_LEN = 4;
constexpr uint8_t MITSUBA_F1_OUTPUT_TARGET_BIT = 26;
constexpr uint8_t MITSUBA_F1_OUTPUT_TARGET_LEN = 10;
constexpr uint8_t MITSUBA_F1_DRIVE_ACTION_BIT = 36;
constexpr uint8_t MITSUBA_F1_DRIVE_ACTION_LEN = 2;
constexpr uint8_t MITSUBA_F1_REGEN_STATUS_BIT = 38;

// Frame 2, error bit positions inside the frame word.
constexpr uint8_t MITSUBA_ERR_ANALOG_SENSOR = 0;
constexpr uint8_t MITSUBA_ERR_CURRENT_U = 1;
constexpr uint8_t MITSUBA_ERR_CURRENT_W = 2;
constexpr uint8_t MITSUBA_ERR_FET_THERM = 3;
constexpr uint8_t MITSUBA_ERR_BAT_VOLT_SENSOR = 5;
constexpr uint8_t MITSUBA_ERR_BAT_CURR_SENSOR = 6;
constexpr uint8_t MITSUBA_ERR_BAT_CURR_ADJ = 7;
constexpr uint8_t MITSUBA_ERR_MOT_CURR_ADJ = 8;
constexpr uint8_t MITSUBA_ERR_ACCEL_POS = 9;
constexpr uint8_t MITSUBA_ERR_CTRL_VOLT_SENSOR = 11;
constexpr uint8_t MITSUBA_ERR_POWER_SYS = 16;
constexpr uint8_t MITSUBA_ERR_OVER_CURRENT = 17;
constexpr uint8_t MITSUBA_ERR_OVER_VOLTAGE = 19;
constexpr uint8_t MITSUBA_ERR_OVER_CURRENT_LIMIT = 23;
constexpr uint8_t MITSUBA_ERR_MOTOR_SYS = 26;
constexpr uint8_t MITSUBA_ERR_MOTOR_LOCK = 27;
constexpr uint8_t MITSUBA_ERR_HALL_SHORT = 28;
constexpr uint8_t MITSUBA_ERR_HALL_OPEN = 29;
constexpr uint8_t MITSUBA_F2_OVERHEAT_LEVEL_BIT = 34;
constexpr uint8_t MITSUBA_F2_OVERHEAT_LEVEL_LEN = 2;

// Rear wheel is 16 in across, so 0.4064 m diameter and pi * 0.4064 = 1.2767 m
// of circumference. The Mitsuba is a direct drive in-wheel motor, so motor rpm
// equals wheel rpm and speed = rpm * 1.2767 * 60 / 1000.
// Same constant as SolarCarDash comm/mitsuba_serial.py:422.
constexpr float MITSUBA_RPM_TO_KMH = 0.0766f;

// ---------------------------------------------------------------------------
// Decoded frames
// ---------------------------------------------------------------------------

struct MitsubaFrame0 {
  int16_t rpm = 0;                 // signed, 1 rpm/LSB
  float batteryVoltage = 0.0f;     // V
  float batteryCurrent = 0.0f;     // A, negative when the direction bit is set
  float motorCurrentPeak = 0.0f;   // A
  float fetTemperatureC = 0.0f;    // degC
  float pwmDutyPercent = 0.0f;     // %
  float leadAngleDeg = 0.0f;       // deg
};

struct MitsubaFrame1 {
  bool powerMode = false;          // 0 eco, 1 power
  uint8_t ctrlMode = 0;            // 0 current mode, 1 PWM mode
  float throttlePercent = 0.0f;    // accelerator position, %
  float regenVrPercent = 0.0f;     // regeneration VR position, %
  uint8_t digitSwPosition = 0;
  float outputTarget = 0.0f;       // 0.5 A/LSB in current mode, 0.5 %/LSB in PWM mode
  uint8_t driveAction = 0;         // 0 stop, 1 RFU, 2 forward, 3 reverse
  bool regenActive = false;
};

struct MitsubaFrame2 {
  uint32_t errorFlags = 0;         // raw low 32 bits, use the MITSUBA_ERR_* positions
  uint8_t overheatLevel = 0;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Builds the little-endian frame word: buf[0] is the least significant byte.
inline uint64_t mitsubaLeBits(const uint8_t *buf, uint8_t len) {
  uint64_t value = 0;
  if (buf == nullptr) {
    return 0;
  }

  const uint8_t limit = len > 8 ? 8 : len;
  for (uint8_t i = 0; i < limit; ++i) {
    value |= static_cast<uint64_t>(buf[i]) << (8U * i);
  }
  return value;
}

inline uint32_t mitsubaBitsAt(uint64_t word, uint8_t startBit, uint8_t bitLength) {
  if (bitLength == 0 || bitLength >= 64 || startBit >= 64) {
    return 0;
  }

  const uint64_t mask = (1ULL << bitLength) - 1ULL;
  return static_cast<uint32_t>((word >> startBit) & mask);
}

// Sign-extends a value that occupies bitLength bits.
inline int32_t mitsubaSignExtend(uint32_t value, uint8_t bitLength) {
  if (bitLength == 0 || bitLength >= 32) {
    return static_cast<int32_t>(value);
  }

  const uint32_t signBit = 1UL << (bitLength - 1U);
  if ((value & signBit) != 0) {
    return static_cast<int32_t>(value) - static_cast<int32_t>(1UL << bitLength);
  }
  return static_cast<int32_t>(value);
}

// Speed shown on the dashboard is a magnitude, so reverse still reads as speed.
// The direction stays visible through the sign of the rpm field itself.
inline float mitsubaSpeedKmhFromRpm(int16_t rpm) {
  const int32_t magnitude = rpm < 0 ? -static_cast<int32_t>(rpm) : static_cast<int32_t>(rpm);
  return static_cast<float>(magnitude) * MITSUBA_RPM_TO_KMH;
}

// ---------------------------------------------------------------------------
// Decoders. Each returns false when the DLC does not match the specification,
// mirroring the length guards in the Python implementation.
// ---------------------------------------------------------------------------

inline bool mitsubaDecodeFrame0(const uint8_t *data, uint8_t len, MitsubaFrame0 &out) {
  if (data == nullptr || len != MITSUBA_FRAME0_DLC) {
    return false;
  }

  const uint64_t word = mitsubaLeBits(data, len);

  out.batteryVoltage =
      static_cast<float>(mitsubaBitsAt(word, MITSUBA_F0_BATTERY_VOLTAGE_BIT,
                                       MITSUBA_F0_BATTERY_VOLTAGE_LEN)) *
      MITSUBA_BATTERY_VOLTAGE_LSB;

  const float currentMagnitude =
      static_cast<float>(mitsubaBitsAt(word, MITSUBA_F0_BATTERY_CURRENT_BIT,
                                       MITSUBA_F0_BATTERY_CURRENT_LEN)) *
      MITSUBA_BATTERY_CURRENT_LSB;
  const bool currentIsNegative =
      mitsubaBitsAt(word, MITSUBA_F0_CURRENT_DIRECTION_BIT, 1) != 0;
  out.batteryCurrent = currentIsNegative ? -currentMagnitude : currentMagnitude;

  out.motorCurrentPeak =
      static_cast<float>(mitsubaBitsAt(word, MITSUBA_F0_MOTOR_CURRENT_PEAK_BIT,
                                       MITSUBA_F0_MOTOR_CURRENT_PEAK_LEN)) *
      MITSUBA_MOTOR_CURRENT_LSB;

  out.fetTemperatureC =
      static_cast<float>(mitsubaBitsAt(word, MITSUBA_F0_FET_TEMPERATURE_BIT,
                                       MITSUBA_F0_FET_TEMPERATURE_LEN)) *
      MITSUBA_FET_TEMPERATURE_LSB;

  out.rpm = static_cast<int16_t>(mitsubaSignExtend(
      mitsubaBitsAt(word, MITSUBA_F0_RPM_BIT, MITSUBA_F0_RPM_LEN), MITSUBA_F0_RPM_LEN));

  out.pwmDutyPercent =
      static_cast<float>(mitsubaBitsAt(word, MITSUBA_F0_PWM_DUTY_BIT,
                                       MITSUBA_F0_PWM_DUTY_LEN)) *
      MITSUBA_PERCENT_LSB;

  out.leadAngleDeg =
      static_cast<float>(mitsubaBitsAt(word, MITSUBA_F0_LEAD_ANGLE_BIT,
                                       MITSUBA_F0_LEAD_ANGLE_LEN)) *
      MITSUBA_LEAD_ANGLE_LSB;

  return true;
}

inline bool mitsubaDecodeFrame1(const uint8_t *data, uint8_t len, MitsubaFrame1 &out) {
  if (data == nullptr || len != MITSUBA_FRAME1_DLC) {
    return false;
  }

  const uint64_t word = mitsubaLeBits(data, len);

  out.powerMode = mitsubaBitsAt(word, MITSUBA_F1_POWER_MODE_BIT, 1) != 0;
  out.ctrlMode = static_cast<uint8_t>(mitsubaBitsAt(word, MITSUBA_F1_CTRL_MODE_BIT, 1));

  out.throttlePercent =
      static_cast<float>(mitsubaBitsAt(word, MITSUBA_F1_ACCEL_POSITION_BIT,
                                       MITSUBA_F1_ACCEL_POSITION_LEN)) *
      MITSUBA_PERCENT_LSB;

  out.regenVrPercent =
      static_cast<float>(mitsubaBitsAt(word, MITSUBA_F1_REGEN_VR_BIT,
                                       MITSUBA_F1_REGEN_VR_LEN)) *
      MITSUBA_PERCENT_LSB;

  out.digitSwPosition = static_cast<uint8_t>(
      mitsubaBitsAt(word, MITSUBA_F1_DIGIT_SW_BIT, MITSUBA_F1_DIGIT_SW_LEN));

  out.outputTarget =
      static_cast<float>(mitsubaBitsAt(word, MITSUBA_F1_OUTPUT_TARGET_BIT,
                                       MITSUBA_F1_OUTPUT_TARGET_LEN)) *
      MITSUBA_PERCENT_LSB;

  out.driveAction = static_cast<uint8_t>(
      mitsubaBitsAt(word, MITSUBA_F1_DRIVE_ACTION_BIT, MITSUBA_F1_DRIVE_ACTION_LEN));

  out.regenActive = mitsubaBitsAt(word, MITSUBA_F1_REGEN_STATUS_BIT, 1) != 0;

  return true;
}

inline bool mitsubaDecodeFrame2(const uint8_t *data, uint8_t len, MitsubaFrame2 &out) {
  if (data == nullptr || len != MITSUBA_FRAME2_DLC) {
    return false;
  }

  const uint64_t word = mitsubaLeBits(data, len);

  out.errorFlags = static_cast<uint32_t>(word & 0xFFFFFFFFULL);
  out.overheatLevel = static_cast<uint8_t>(mitsubaBitsAt(
      word, MITSUBA_F2_OVERHEAT_LEVEL_BIT, MITSUBA_F2_OVERHEAT_LEVEL_LEN));

  return true;
}

inline bool mitsubaErrorSet(uint32_t errorFlags, uint8_t bitPosition) {
  return ((errorFlags >> bitPosition) & 1UL) != 0;
}

// True when any specification-defined error bit is set, ignoring RFU bits.
inline bool mitsubaHasAnyError(uint32_t errorFlags) {
  constexpr uint32_t definedMask =
      (1UL << MITSUBA_ERR_ANALOG_SENSOR) | (1UL << MITSUBA_ERR_CURRENT_U) |
      (1UL << MITSUBA_ERR_CURRENT_W) | (1UL << MITSUBA_ERR_FET_THERM) |
      (1UL << MITSUBA_ERR_BAT_VOLT_SENSOR) | (1UL << MITSUBA_ERR_BAT_CURR_SENSOR) |
      (1UL << MITSUBA_ERR_BAT_CURR_ADJ) | (1UL << MITSUBA_ERR_MOT_CURR_ADJ) |
      (1UL << MITSUBA_ERR_ACCEL_POS) | (1UL << MITSUBA_ERR_CTRL_VOLT_SENSOR) |
      (1UL << MITSUBA_ERR_POWER_SYS) | (1UL << MITSUBA_ERR_OVER_CURRENT) |
      (1UL << MITSUBA_ERR_OVER_VOLTAGE) | (1UL << MITSUBA_ERR_OVER_CURRENT_LIMIT) |
      (1UL << MITSUBA_ERR_MOTOR_SYS) | (1UL << MITSUBA_ERR_MOTOR_LOCK) |
      (1UL << MITSUBA_ERR_HALL_SHORT) | (1UL << MITSUBA_ERR_HALL_OPEN);
  return (errorFlags & definedMask) != 0;
}
