#!/usr/bin/env node
// Positive control for the decode gate.
//
// A green differential test only means something if the same test turns red on
// a real defect. This corrupts one signal placement in a throwaway copy of
// src/mitsuba_decode.h, reruns the differential against that copy, and requires
// it to fail. The real source file is never modified.

import { execFileSync } from "node:child_process";
import { mkdtempSync, copyFileSync, readFileSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, resolve } from "node:path";

const CORRUPTIONS = [
  // Shift the motor speed field by one bit.
  { from: "MITSUBA_F0_RPM_BIT = 35;", to: "MITSUBA_F0_RPM_BIT = 34;" },
  // Swap the battery voltage scale, which only the protocol JSON round trip sees.
  { from: "MITSUBA_BATTERY_VOLTAGE_LSB = 0.5f;", to: "MITSUBA_BATTERY_VOLTAGE_LSB = 1.0f;" },
  // Drop the sign extension on the 12 bit motor speed.
  { from: "const uint32_t signBit = 1UL << (bitLength - 1U);", to: "const uint32_t signBit = 0UL;" },
];

let detected = 0;

for (const corruption of CORRUPTIONS) {
  const work = mkdtempSync(join(tmpdir(), "mitsuba-control-"));
  const header = join(work, "mitsuba_decode.h");
  copyFileSync(resolve("src/mitsuba_decode.h"), header);

  const source = readFileSync(header, "utf8");
  if (!source.includes(corruption.from)) {
    console.error(`FAIL: control anchor not found in the header: ${corruption.from}`);
    process.exit(1);
  }
  writeFileSync(header, source.replace(corruption.from, corruption.to));

  let failedAsExpected = false;
  try {
    execFileSync("node", [resolve("scripts/run_decode_differential.mjs"), "--include", work], {
      stdio: ["ignore", "pipe", "pipe"],
    });
  } catch {
    failedAsExpected = true;
  }

  if (!failedAsExpected) {
    console.error(`FAIL: the differential test passed with a corrupted header (${corruption.from})`);
    process.exit(1);
  }

  detected += 1;
}

console.log(`CONTROL_OK detected corruption ${detected}/${CORRUPTIONS.length}`);
