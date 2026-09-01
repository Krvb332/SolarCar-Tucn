"""Oracle dumper for the Mitsuba decode differential test.

Runs the *working* Python decoder from the SolarCarDash repo over a shared set
of byte vectors and prints the decoded fields as JSON. The C++ decoder in
src/mitsuba_decode.h is then checked against this output, so the expected values
never originate from the code under test.

Usage: dump_python_decode.py <vectors-file> [solarcardash-root]

The vectors file holds one "<frameIndex> <hexPayload>" per line, the same file
that is fed to the native harness.
"""

import json
import queue
import sys
from pathlib import Path

FRAME0_FIELDS = {
    "rpm": "rpm",
    "fet_temp": "controller_temp",
    "motor_current_peak": "motor_current_peak",
}

FRAME1_FIELDS = {
    "power_mode": "power_mode",
    "ctrl_mode": "motor_ctrl_mode",
    "throttle": "throttle",
    "regen_vr": "regen_vr",
    "output_target": "output_target_val",
    "drive_action": "drive_action",
    "regen_active": "regen_active",
}

FRAME2_FIELDS = {
    "analog_sensor": "mc_err_analog_sensor",
    "current_u": "mc_err_current_u",
    "current_w": "mc_err_current_w",
    "fet_therm": "mc_err_fet_therm",
    "bat_volt_sensor": "mc_err_bat_volt_sensor",
    "bat_curr_sensor": "mc_err_bat_curr_sensor",
    "bat_curr_adj": "mc_err_bat_curr_adj",
    "mot_curr_adj": "mc_err_mot_curr_adj",
    "accel_pos": "mc_err_accel_pos",
    "ctrl_volt_sensor": "mc_err_ctrl_volt_sensor",
    "power_sys": "mc_err_power_sys",
    "over_current": "mc_err_over_current",
    "over_voltage": "mc_err_over_voltage",
    "over_current_limit": "mc_err_over_current_limit",
    "motor_sys": "mc_err_motor_sys",
    "motor_lock": "mc_err_motor_lock",
    "hall_short": "mc_err_hall_short",
    "hall_open": "mc_err_hall_open",
    "overheat_level": "mc_overheat_level",
}


def drain(module) -> None:
    """The upstream module publishes into a maxsize=1 queue with a blocking put."""
    while True:
        try:
            module._mitsuba_queue.get_nowait()
        except queue.Empty:
            return


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: dump_python_decode.py <vectors-file> [solarcardash-root]",
              file=sys.stderr)
        return 2

    vectors_path = Path(sys.argv[1])
    dash_root = Path(sys.argv[2]) if len(sys.argv) > 2 else Path.home() / "SolarCarDash"

    if not dash_root.is_dir():
        print(f"SolarCarDash not found at {dash_root}", file=sys.stderr)
        return 2

    sys.path.insert(0, str(dash_root))
    import comm.mitsuba_serial as oracle  # noqa: E402

    results = []
    for raw_line in vectors_path.read_text().splitlines():
        line = raw_line.strip()
        if not line:
            continue

        index_text, hex_text = line.split()
        frame_index = int(index_text)
        payload = list(bytes.fromhex(hex_text))

        if frame_index == 0:
            oracle._parse_frame0(payload)
            fields = FRAME0_FIELDS
        elif frame_index == 1:
            oracle._parse_frame1(payload)
            fields = FRAME1_FIELDS
        elif frame_index == 2:
            oracle._parse_frame2(payload)
            fields = FRAME2_FIELDS
        else:
            print(f"unknown frame index {frame_index}", file=sys.stderr)
            return 2

        drain(oracle)
        results.append({name: oracle._current_state[key] for name, key in fields.items()})

    json.dump(results, sys.stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
