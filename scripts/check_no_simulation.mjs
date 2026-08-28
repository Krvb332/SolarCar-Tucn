#!/usr/bin/env node
// Asserts that no simulated-telemetry generator survives anywhere under src/.
//
// This is a negative assertion, so the detector is first run against a fixture
// that deliberately contains the old simulation code. If the detector cannot
// find the fixture, the whole check is meaningless and fails loudly rather than
// reporting a hollow success.

import { readdirSync, readFileSync, statSync } from "node:fs";
import { join, resolve } from "node:path";

const BANNED = [
  { pattern: /\bLoopedTelemetry\b/, why: "simulated telemetry struct" },
  { pattern: /\bloopedTelemetry\b/, why: "simulated telemetry instance" },
  { pattern: /\bloopFloatValue\b/, why: "triangle wave generator" },
  { pattern: /\bloopUnsignedValue\b/, why: "triangle wave generator" },
  { pattern: /\bupdateLoopedTelemetry\b/, why: "simulation driver" },
  { pattern: /\bpublishLoopedTelemetry\b/, why: "simulated publisher" },
  { pattern: /\bpublishSimulatedBmsTelemetry\b/, why: "simulated BMS publisher" },
  { pattern: /\bLOOPED_[A-Z_]+/, why: "simulation tuning constant" },
  { pattern: /\bsimulatedPressures\b/, why: "simulated TPMS pressures" },
  { pattern: /\brandom\s*\(/, why: "random number source feeding telemetry" },
];

// The pre-change code, kept here purely as a positive control for the detector.
const FIXTURE = `
  struct LoopedTelemetry { float speedKmh = 0.0f; };
  LoopedTelemetry loopedTelemetry = {};
  constexpr float LOOPED_SPEED_MAX = 150.0f;
  void updateLoopedTelemetry() {
    loopedTelemetry.speedKmh = loopFloatValue(loopedTelemetry.speedKmh, LOOPED_SPEED_MAX, 0.8f, d);
  }
  void publishLoopedTelemetry() { setXFloatIfChanged("x0", loopedTelemetry.speedKmh, 1); }
  uint32_t gauge = loopUnsignedValue(loopedTelemetry.centerGauge, 100, 2, d);
  void publishSimulatedBmsTelemetry() {}
  const float simulatedPressures[5] = {};
  int noise = random(0, 100);
`;

function scan(text) {
  return BANNED.filter((rule) => rule.pattern.test(text));
}

function sourceFiles(dir) {
  const found = [];
  for (const entry of readdirSync(dir)) {
    const full = join(dir, entry);
    if (statSync(full).isDirectory()) {
      found.push(...sourceFiles(full));
    } else if (/\.(c|cc|cpp|h|hpp|ino)$/.test(entry)) {
      found.push(full);
    }
  }
  return found;
}

// Positive control first.
const fixtureHits = scan(FIXTURE);
if (fixtureHits.length !== BANNED.length) {
  const missed = BANNED.filter((rule) => !fixtureHits.includes(rule)).map((r) => r.why);
  console.error(
    `FAIL: the detector missed ${missed.length} pattern(s) in its own fixture: ${missed.join(", ")}`,
  );
  process.exit(1);
}

const files = sourceFiles(resolve("src"));
if (files.length === 0) {
  console.error("FAIL: no source files found under src/");
  process.exit(1);
}

let violations = 0;
for (const file of files) {
  const text = readFileSync(file, "utf8");
  text.split("\n").forEach((line, index) => {
    for (const rule of BANNED) {
      if (rule.pattern.test(line)) {
        console.error(`${file}:${index + 1}: ${rule.why}: ${line.trim()}`);
        violations += 1;
      }
    }
  });
}

if (violations > 0) {
  console.error(`FAIL: ${violations} simulation reference(s) still in src/`);
  process.exit(1);
}

console.log(`control: detector found all ${BANNED.length} patterns in the fixture`);
console.log(`scanned ${files.length} source file(s) under src/`);
console.log("NO_SIMULATION_OK");
