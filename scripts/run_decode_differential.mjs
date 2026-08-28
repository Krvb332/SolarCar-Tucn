#!/usr/bin/env node
// Differential verification of src/mitsuba_decode.h against two independent
// oracles, neither of which is the code under test:
//
//   1. The working Python decoder in SolarCarDash (comm/mitsuba_serial.py),
//      over a shared set of byte vectors.
//   2. A round trip built from the protocol specification itself
//      (docs/protocols/can_protocol_rear_left_wheel.json): random field values
//      are encoded using the start_bit, bit_length and unit_lsb taken from that
//      JSON, then the decoder has to recover them. This covers the Frame 0
//      battery voltage and current signals, which the Python side never decodes.
//
// Usage: node scripts/run_decode_differential.mjs [--include <dir>]
// Exits non-zero and prints the offending vector on any mismatch.

import { execFileSync } from "node:child_process";
import { mkdtempSync, writeFileSync, readFileSync, existsSync } from "node:fs";
import { tmpdir, homedir } from "node:os";
import { join, resolve } from "node:path";

const VECTOR_COUNT = 1000;
const ROUNDTRIP_COUNT = 1000;
const EPSILON = 1e-4;

const args = process.argv.slice(2);
const includeIndex = args.indexOf("--include");
const includeDir = includeIndex >= 0 ? resolve(args[includeIndex + 1]) : resolve("src");
const dashRoot = process.env.SOLARCARDASH_ROOT
  ? resolve(process.env.SOLARCARDASH_ROOT)
  : join(homedir(), "SolarCarDash");

const work = mkdtempSync(join(tmpdir(), "mitsuba-decode-"));
const binary = join(work, "decode_test");

function fail(message) {
  console.error(`FAIL: ${message}`);
  process.exit(1);
}

// Deterministic PRNG so a failure is always reproducible.
function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function toHex(bytes) {
  return bytes.map((b) => b.toString(16).padStart(2, "0")).join("");
}

function bytesFromWord(word, length) {
  const bytes = [];
  for (let i = 0; i < length; i += 1) {
    bytes.push(Number((word >> BigInt(8 * i)) & 0xffn));
  }
  return bytes;
}

function parseNativeLine(line) {
  const out = {};
  for (const pair of line.split(",")) {
    const [key, value] = pair.split("=");
    out[key] = Number(value);
  }
  return out;
}

function runNative(vectors) {
  const vectorFile = join(work, "vectors.txt");
  writeFileSync(vectorFile, vectors.map(([i, hex]) => `${i} ${hex}`).join("\n") + "\n");

  const stdout = execFileSync(binary, {
    input: readFileSync(vectorFile),
    encoding: "utf8",
  });

  const lines = stdout.trim().split("\n");
  if (lines.length !== vectors.length) {
    fail(`native harness returned ${lines.length} lines for ${vectors.length} vectors`);
  }
  for (const line of lines) {
    if (line.startsWith("error=")) {
      fail(`native harness rejected a vector: ${line}`);
    }
  }
  return { vectorFile, decoded: lines.map(parseNativeLine) };
}

function close(a, b) {
  return Math.abs(a - b) <= EPSILON;
}

// --- build -----------------------------------------------------------------

if (!existsSync(join(includeDir, "mitsuba_decode.h"))) {
  fail(`mitsuba_decode.h not found in ${includeDir}`);
}

try {
  execFileSync(
    "clang++",
    ["-std=c++17", "-Wall", "-Wextra", "-O1", "-I", includeDir, "-o", binary,
     resolve("scripts/test_mitsuba_decode.cpp")],
    { stdio: ["ignore", "pipe", "pipe"] },
  );
} catch (error) {
  fail(`could not build the native harness: ${error.stderr?.toString() ?? error.message}`);
}

// --- oracle 1: the working Python decoder ----------------------------------

const rng = mulberry32(0x5011a2);
const vectors = [];
for (let i = 0; i < VECTOR_COUNT; i += 1) {
  const frameIndex = i % 3;
  const length = frameIndex === 0 ? 8 : 5;
  const bytes = Array.from({ length }, () => Math.floor(rng() * 256));
  vectors.push([frameIndex, toHex(bytes)]);
}

const { vectorFile, decoded: nativeDecoded } = runNative(vectors);

const pythonBin = join(dashRoot, ".venv/bin/python");
if (!existsSync(pythonBin)) {
  fail(`SolarCarDash python venv not found at ${pythonBin}`);
}

let pythonDecoded;
try {
  pythonDecoded = JSON.parse(
    execFileSync(pythonBin, [resolve("scripts/dump_python_decode.py"), vectorFile, dashRoot], {
      encoding: "utf8",
      cwd: dashRoot,
    }),
  );
} catch (error) {
  fail(`python oracle failed: ${error.stderr?.toString() ?? error.message}`);
}

if (pythonDecoded.length !== vectors.length) {
  fail(`python oracle returned ${pythonDecoded.length} results for ${vectors.length} vectors`);
}

let differentialPassed = 0;
let sawNegativeRpm = false;
for (let i = 0; i < vectors.length; i += 1) {
  const expected = pythonDecoded[i];
  const actual = nativeDecoded[i];

  for (const [key, want] of Object.entries(expected)) {
    const wantNumber = typeof want === "boolean" ? (want ? 1 : 0) : want;
    if (!(key in actual)) {
      fail(`vector ${i} (${vectors[i][0]} ${vectors[i][1]}): native output has no field ${key}`);
    }
    if (!close(actual[key], wantNumber)) {
      fail(
        `vector ${i} (frame ${vectors[i][0]}, ${vectors[i][1]}): ${key} ` +
          `python=${wantNumber} native=${actual[key]}`,
      );
    }
  }

  if (expected.rpm !== undefined && expected.rpm < 0) {
    sawNegativeRpm = true;
  }
  differentialPassed += 1;
}

if (!sawNegativeRpm) {
  fail("no negative rpm vector was generated, the 12 bit sign extension went untested");
}

// --- oracle 2: round trip against the protocol JSON ------------------------

const protocolPath = join(dashRoot, "docs/protocols/can_protocol_rear_left_wheel.json");
if (!existsSync(protocolPath)) {
  fail(`protocol specification not found at ${protocolPath}`);
}
const protocol = JSON.parse(readFileSync(protocolPath, "utf8"));

function signalsFor(idHex) {
  const message = protocol.messages.find(
    (m) => m.message_id_hex.toLowerCase() === idHex.toLowerCase(),
  );
  if (!message) {
    fail(`protocol specification has no message ${idHex}`);
  }
  const table = new Map();
  for (const signal of message.signals) {
    if (signal.name === "RFU") continue;
    const scaleMatch = /^([0-9]*\.?[0-9]+)/.exec(signal.unit_lsb ?? "");
    table.set(signal.name, {
      startBit: signal.start_bit,
      bitLength: signal.bit_length,
      scale: scaleMatch ? Number(scaleMatch[1]) : 1,
    });
  }
  return { table, dlc: message.data_length_bytes };
}

const frame0Spec = signalsFor("0x08850225");
const frame1Spec = signalsFor("0x08950225");
const frame2Spec = signalsFor("0x08A50225");

function place(word, spec, rawValue) {
  const mask = (1n << BigInt(spec.bitLength)) - 1n;
  return word | ((BigInt(rawValue) & mask) << BigInt(spec.startBit));
}

function rawFor(spec, random) {
  return Math.floor(random() * Number(1n << BigInt(spec.bitLength)));
}

// Field name in the native output -> signal name in the protocol JSON.
const FRAME0_MAP = {
  battery_voltage: "Battery Voltage",
  motor_current_peak: "Motor Current Peak Average",
  fet_temp: "FET Temperature",
  pwm_duty: "PWM DUTY",
  lead_angle: "Lead Angle",
};
const FRAME1_MAP = {
  throttle: "Accelerator Position",
  regen_vr: "Regeneration VR Position",
  digit_sw: "Digit SW Position",
  output_target: "Output Target Value",
  drive_action: "Drive Action Status",
  power_mode: "Power Mode",
  ctrl_mode: "Motor Control Mode",
  regen_active: "Regeneration Status",
};
const FRAME2_MAP = {
  analog_sensor: "Analog Sensor Error",
  current_u: "Motor Current Sensor U Error",
  current_w: "Motor Current Sensor W Error",
  fet_therm: "FET Thermistor Error",
  bat_volt_sensor: "Battery Voltage Sensor Error",
  bat_curr_sensor: "Battery Current Sensor Error",
  bat_curr_adj: "Battery Current Sensor Adjust Error",
  mot_curr_adj: "Motor Current Sensor Adjust Error",
  accel_pos: "Accelerator Position Error",
  ctrl_volt_sensor: "Controller Voltage Sensor Error",
  power_sys: "Power System Error",
  over_current: "Over Current Error",
  over_voltage: "Over Voltage Error",
  over_current_limit: "Over Current Limit",
  motor_sys: "Motor System Error",
  motor_lock: "Motor Lock",
  hall_short: "Hall Sensor Short",
  hall_open: "Hall Sensor Open",
  overheat_level: "Over Heat Level",
};

const rtRng = mulberry32(0xc0ffee);
const rtVectors = [];
const rtExpected = [];

for (let i = 0; i < ROUNDTRIP_COUNT; i += 1) {
  const frameIndex = i % 3;

  if (frameIndex === 0) {
    const spec = frame0Spec;
    let word = 0n;
    const expected = {};

    for (const [field, signalName] of Object.entries(FRAME0_MAP)) {
      const signal = spec.table.get(signalName);
      const raw = rawFor(signal, rtRng);
      word = place(word, signal, raw);
      expected[field] = raw * signal.scale;
    }

    // Motor speed is signed over its bit_length, so drive it across zero.
    const rpmSignal = spec.table.get("Motor Rotating Speed");
    const span = Number(1n << BigInt(rpmSignal.bitLength));
    const rpm = Math.floor(rtRng() * span) - span / 2;
    word = place(word, rpmSignal, rpm < 0 ? rpm + span : rpm);
    expected.rpm = rpm * rpmSignal.scale;

    // Battery current carries its sign in a separate direction bit.
    const currentSignal = spec.table.get("Battery Current");
    const directionSignal = spec.table.get("Battery Current Direction");
    const currentRaw = rawFor(currentSignal, rtRng);
    const negative = rtRng() < 0.5 ? 1 : 0;
    word = place(word, currentSignal, currentRaw);
    word = place(word, directionSignal, negative);
    expected.battery_current = (negative ? -currentRaw : currentRaw) * currentSignal.scale;

    rtVectors.push([0, toHex(bytesFromWord(word, spec.dlc))]);
    rtExpected.push(expected);
    continue;
  }

  const spec = frameIndex === 1 ? frame1Spec : frame2Spec;
  const map = frameIndex === 1 ? FRAME1_MAP : FRAME2_MAP;
  let word = 0n;
  const expected = {};

  for (const [field, signalName] of Object.entries(map)) {
    const signal = spec.table.get(signalName);
    const raw = rawFor(signal, rtRng);
    word = place(word, signal, raw);
    expected[field] = raw * signal.scale;
  }

  rtVectors.push([frameIndex, toHex(bytesFromWord(word, spec.dlc))]);
  rtExpected.push(expected);
}

const { decoded: rtDecoded } = runNative(rtVectors);

let roundtripPassed = 0;
for (let i = 0; i < rtVectors.length; i += 1) {
  for (const [key, want] of Object.entries(rtExpected[i])) {
    if (!close(rtDecoded[i][key], want)) {
      fail(
        `roundtrip ${i} (frame ${rtVectors[i][0]}, ${rtVectors[i][1]}): ${key} ` +
          `spec=${want} native=${rtDecoded[i][key]}`,
      );
    }
  }
  roundtripPassed += 1;
}

const total = differentialPassed + roundtripPassed;
console.log(`differential vs python decoder: ${differentialPassed}/${VECTOR_COUNT}`);
console.log(`roundtrip vs protocol json:     ${roundtripPassed}/${ROUNDTRIP_COUNT}`);
console.log(`MITSUBA_DECODE_OK ${total}/${VECTOR_COUNT + ROUNDTRIP_COUNT}`);
