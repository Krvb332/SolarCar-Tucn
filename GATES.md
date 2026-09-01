# Gates: Mitsuba CAN speed + RPM on Teensy 4.1

OWNS: src/**, scripts/**, platformio.ini, GATES.md

Scope: Teensy 4.1 requests and decodes Mitsuba motor-controller CAN frames over CAN1 at 500 kbit/s, shows real speed and RPM (plus drive voltage/current/power/FET temp) on the Nextion panel, and every simulated-data generator is removed from the firmware.

- [x] G1: Teensy 4.1 firmware compiles with the new CAN module linked in
  CHECK: /Users/razesusebastian-constantin/.platformio/penv/bin/pio run -e teensy41
  EXPECT: [SUCCESS]
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/razesusebastian-constantin/SolarCar-Tucn; path=905e7b81fee4/24 entries; EXPECT=matched; output-sha256=643b012697229bb1004c3661fd10c999873d1b52ccf49681ffc086cb1f0f465f; output-bytes=1631

- [x] G2: The ESP32 build is not broken by the Teensy-only CAN module
  CHECK: /Users/razesusebastian-constantin/.platformio/penv/bin/pio run -e esp32dev
  EXPECT: [SUCCESS]
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/razesusebastian-constantin/SolarCar-Tucn; path=905e7b81fee4/24 entries; EXPECT=matched; output-sha256=edfe5bd867b36ea322c4a3de9ff0f3ba6fcb8faf5c038ac143d5945fafde2425; output-bytes=5095

- [x] G3: No simulated-telemetry generator survives anywhere under src/
  CHECK: node scripts/check_no_simulation.mjs
  EXPECT: NO_SIMULATION_OK
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/razesusebastian-constantin/SolarCar-Tucn; path=905e7b81fee4/24 entries; EXPECT=matched; output-sha256=54a1fc8d491343eb295a4590da9847f844475364987bf3dc8e135b38c81fae9f; output-bytes=108

- [x] G4: C++ decode matches the SolarCarDash Python decoder on 1000 shared vectors (incl. negative RPM) and round-trips 1000 more against the protocol JSON
  CHECK: node scripts/run_decode_differential.mjs
  EXPECT: MITSUBA_DECODE_OK 2000/2000
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/razesusebastian-constantin/SolarCar-Tucn; path=905e7b81fee4/24 entries; EXPECT=matched; output-sha256=ca6b6f38610d6220fbdd9c581b134c0f67be52d439f4956106a484d34830eeb8; output-bytes=112

- [x] G5: Positive control - G4 actually fails on a corrupted start_bit, a corrupted scale, and a removed sign extension
  CHECK: node scripts/run_decode_control.mjs
  EXPECT: CONTROL_OK detected corruption 3/3
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/razesusebastian-constantin/SolarCar-Tucn; path=905e7b81fee4/24 entries; EXPECT=matched; output-sha256=d620c54bfa57a8516f656564fa2802aeb7eb5d4579a221a248c9c2efe5d34fa6; output-bytes=35

- [x] G6: x0/x1 are fed only from Mitsuba CAN telemetry and fall back to 0 when frames are stale
  CHECK: node scripts/check_display_wiring.mjs
  EXPECT: DISPLAY_WIRING_OK
  EVIDENCE: exit=0; shell=/bin/sh; cwd=/Users/razesusebastian-constantin/SolarCar-Tucn; path=905e7b81fee4/24 entries; EXPECT=matched; output-sha256=84f39d29c5a28dd346bee12b960c3292adbdac080e853bf1f034980e93c808ee; output-bytes=119

- [ ] G7: Bench check on the car - real frames move x0/x1, and pulling the CAN cable zeroes them within ~1 s
  EVIDENCE: pending
