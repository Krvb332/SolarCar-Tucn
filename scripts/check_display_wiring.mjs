#!/usr/bin/env node
// Asserts that the speed and RPM fields on the Nextion panel are fed from the
// Mitsuba CAN link and nothing else, that they fall back to zero when the link
// goes stale, and that the motor link is serviced before loop() can return
// early for the BMS response window.
//
// Every rule is first exercised against a fixture that violates it, so a green
// result cannot come from a rule that silently matches nothing.

import { readFileSync } from "node:fs";
import { resolve } from "node:path";

function extractFunction(source, signature) {
  const start = source.indexOf(signature);
  if (start < 0) return null;

  let depth = 0;
  let started = false;
  for (let i = start; i < source.length; i += 1) {
    if (source[i] === "{") {
      depth += 1;
      started = true;
    } else if (source[i] === "}") {
      depth -= 1;
      if (started && depth === 0) {
        return source.slice(start, i + 1);
      }
    }
  }
  return null;
}

const RULES = [
  {
    name: "x0 and x1 are written only by publishMotorDisplay and clearDisplayValues",
    check(source) {
      const owners = ["void publishMotorDisplay(uint32_t now) {", "void clearDisplayValues() {"]
        .map((sig) => extractFunction(source, sig))
        .filter(Boolean);
      if (owners.length !== 2) return "publishMotorDisplay or clearDisplayValues is missing";

      const writes = [...source.matchAll(/set(?:Value|XFloat)[A-Za-z]*\(\s*"(x0|x1)"/g)];
      const ownedWrites = owners
        .flatMap((body) => [...body.matchAll(/set(?:Value|XFloat)[A-Za-z]*\(\s*"(x0|x1)"/g)])
        .length;
      if (writes.length !== ownedWrites) {
        return `${writes.length - ownedWrites} write(s) to x0/x1 outside the two owning functions`;
      }
      return null;
    },
  },
  {
    name: "publishMotorDisplay sources its values from the Mitsuba snapshot",
    check(source) {
      const body = extractFunction(source, "void publishMotorDisplay(uint32_t now) {");
      if (!body) return "publishMotorDisplay not found";
      if (!/mitsuba::snapshot\(motor\)/.test(body)) return "no call to mitsuba::snapshot";
      for (const field of ["speedKmh", "motorRpm"]) {
        if (!body.includes(`motor.${field}`)) return `x0/x1 do not read motor.${field}`;
      }
      return null;
    },
  },
  {
    name: "stale Mitsuba data falls back to zero on every motor field",
    check(source) {
      const body = extractFunction(source, "void publishMotorDisplay(uint32_t now) {");
      if (!body) return "publishMotorDisplay not found";
      if (!/mitsuba::isFresh\(now\)/.test(body)) return "no freshness gate";

      const components = ["x0", "x1", "x12", "x13", "x14", "x15"];
      for (const component of components) {
        const line = body
          .split("\n")
          .find((l) => l.includes(`"${component}"`));
        if (!line) return `${component} is not published`;
        if (!/fresh\s*\?/.test(line) || !/:\s*0(\.0f)?\s*[,)]/.test(line)) {
          return `${component} does not fall back to zero when stale`;
        }
      }
      return null;
    },
  },
  {
    name: "loop() services the Mitsuba link before the BMS early return",
    check(source) {
      const body = extractFunction(source, "void loop() {");
      if (!body) return "loop() not found";

      const serviceAt = body.indexOf("mitsuba::service(");
      const returnAt = body.indexOf("return;");
      if (serviceAt < 0) return "loop() never calls mitsuba::service";
      if (returnAt < 0) return "loop() no longer has the BMS early return";
      if (serviceAt > returnAt) {
        return "mitsuba::service runs after the early return, so the motor stops being polled";
      }
      return null;
    },
  },
];

// Fixtures that must trip each rule, in the same order as RULES.
const FIXTURES = [
  `void publishMotorDisplay(uint32_t now) { setXFloatIfChanged("x0", motor.speedKmh, 1);
     setValueIfChanged("x1", (int32_t)motor.motorRpm); }
   void clearDisplayValues() { setXFloatIfChanged("x0", 0.0f, 1); setValueIfChanged("x1", 0); }
   void somethingElse() { setXFloatIfChanged("x0", 42.0f, 1); }`,
  `void publishMotorDisplay(uint32_t now) { setXFloatIfChanged("x0", fakeSpeed, 1); }
   void clearDisplayValues() { }`,
  `void publishMotorDisplay(uint32_t now) {
     MitsubaTelemetry motor; const bool fresh = mitsuba::snapshot(motor) && mitsuba::isFresh(now);
     setXFloatIfChanged("x0", motor.speedKmh, 1);
     setValueIfChanged("x1", fresh ? (int32_t)motor.motorRpm : 0);
     setXFloatIfChanged("x12", fresh ? motor.batteryVoltage : 0.0f, 1);
     setXFloatIfChanged("x13", fresh ? motor.batteryCurrent : 0.0f, 1);
     setXFloatIfChanged("x14", fresh ? motor.batteryPower : 0.0f, 1);
     setXFloatIfChanged("x15", fresh ? motor.fetTemperatureC : 0.0f, 1); }
   void clearDisplayValues() { }`,
  `void loop() { if (isBmsResponseWindowActive(millis())) { return; } mitsuba::service(millis()); }`,
];

let failures = 0;

for (let i = 0; i < RULES.length; i += 1) {
  const complaint = RULES[i].check(FIXTURES[i]);
  if (!complaint) {
    console.error(`FAIL: rule "${RULES[i].name}" did not trip on its own violating fixture`);
    failures += 1;
  }
}

if (failures > 0) {
  console.error("FAIL: the checker cannot detect the defects it claims to detect");
  process.exit(1);
}

const source = readFileSync(resolve("src/main.cpp"), "utf8");
for (const rule of RULES) {
  const complaint = rule.check(source);
  if (complaint) {
    console.error(`FAIL: ${rule.name}: ${complaint}`);
    failures += 1;
  }
}

if (failures > 0) {
  process.exit(1);
}

console.log(`control: all ${RULES.length} rules tripped on their violating fixtures`);
console.log(`checked ${RULES.length} wiring rules against src/main.cpp`);
console.log("DISPLAY_WIRING_OK");
