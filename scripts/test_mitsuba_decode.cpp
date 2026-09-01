// Host harness around src/mitsuba_decode.h.
//
// Reads lines of "<frameIndex> <hexPayload>" from stdin and prints one line of
// comma separated name=value pairs per input line. It performs no assertions of
// its own: the comparison against the oracles lives in
// scripts/run_decode_differential.mjs, so the expected values never come from
// the same source as the code under test.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "mitsuba_decode.h"

namespace {

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Returns the byte count, or -1 when the hex string is malformed.
int parseHex(const char *text, uint8_t *out, size_t capacity) {
  const size_t length = strlen(text);
  if ((length % 2) != 0 || (length / 2) > capacity) {
    return -1;
  }

  for (size_t i = 0; i < length; i += 2) {
    const int hi = hexNibble(text[i]);
    const int lo = hexNibble(text[i + 1]);
    if (hi < 0 || lo < 0) {
      return -1;
    }
    out[i / 2] = static_cast<uint8_t>((hi << 4) | lo);
  }

  return static_cast<int>(length / 2);
}

}  // namespace

int main() {
  char line[256];

  while (fgets(line, sizeof(line), stdin) != nullptr) {
    int frameIndex = 0;
    char hex[128];
    if (sscanf(line, "%d %127s", &frameIndex, hex) != 2) {
      continue;
    }

    uint8_t payload[8];
    const int len = parseHex(hex, payload, sizeof(payload));
    if (len < 0) {
      printf("error=bad_hex\n");
      continue;
    }

    if (frameIndex == 0) {
      MitsubaFrame0 decoded;
      if (!mitsubaDecodeFrame0(payload, static_cast<uint8_t>(len), decoded)) {
        printf("error=rejected\n");
        continue;
      }
      printf(
          "rpm=%d,battery_voltage=%.6f,battery_current=%.6f,motor_current_peak=%.6f,"
          "fet_temp=%.6f,pwm_duty=%.6f,lead_angle=%.6f,speed_kmh=%.6f\n",
          static_cast<int>(decoded.rpm), decoded.batteryVoltage, decoded.batteryCurrent,
          decoded.motorCurrentPeak, decoded.fetTemperatureC, decoded.pwmDutyPercent,
          decoded.leadAngleDeg, mitsubaSpeedKmhFromRpm(decoded.rpm));
    } else if (frameIndex == 1) {
      MitsubaFrame1 decoded;
      if (!mitsubaDecodeFrame1(payload, static_cast<uint8_t>(len), decoded)) {
        printf("error=rejected\n");
        continue;
      }
      printf(
          "power_mode=%d,ctrl_mode=%d,throttle=%.6f,regen_vr=%.6f,digit_sw=%d,"
          "output_target=%.6f,drive_action=%d,regen_active=%d\n",
          decoded.powerMode ? 1 : 0, static_cast<int>(decoded.ctrlMode),
          decoded.throttlePercent, decoded.regenVrPercent,
          static_cast<int>(decoded.digitSwPosition), decoded.outputTarget,
          static_cast<int>(decoded.driveAction), decoded.regenActive ? 1 : 0);
    } else if (frameIndex == 2) {
      MitsubaFrame2 decoded;
      if (!mitsubaDecodeFrame2(payload, static_cast<uint8_t>(len), decoded)) {
        printf("error=rejected\n");
        continue;
      }
      printf(
          "analog_sensor=%d,current_u=%d,current_w=%d,fet_therm=%d,bat_volt_sensor=%d,"
          "bat_curr_sensor=%d,bat_curr_adj=%d,mot_curr_adj=%d,accel_pos=%d,"
          "ctrl_volt_sensor=%d,power_sys=%d,over_current=%d,over_voltage=%d,"
          "over_current_limit=%d,motor_sys=%d,motor_lock=%d,hall_short=%d,hall_open=%d,"
          "overheat_level=%d\n",
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_ANALOG_SENSOR),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_CURRENT_U),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_CURRENT_W),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_FET_THERM),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_BAT_VOLT_SENSOR),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_BAT_CURR_SENSOR),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_BAT_CURR_ADJ),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_MOT_CURR_ADJ),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_ACCEL_POS),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_CTRL_VOLT_SENSOR),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_POWER_SYS),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_OVER_CURRENT),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_OVER_VOLTAGE),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_OVER_CURRENT_LIMIT),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_MOTOR_SYS),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_MOTOR_LOCK),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_HALL_SHORT),
          mitsubaErrorSet(decoded.errorFlags, MITSUBA_ERR_HALL_OPEN),
          static_cast<int>(decoded.overheatLevel));
    } else {
      printf("error=bad_frame_index\n");
    }
  }

  return 0;
}
