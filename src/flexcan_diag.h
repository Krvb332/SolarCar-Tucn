#pragma once

// Bring-up diagnostics shared by the two CAN links on this board.
//
// Asking FlexCAN for 500 kbit/s is not the same as running at 500 kbit/s. The
// library picks the prescaler and the three phase segments itself from whatever
// peripheral clock happens to be configured, and silently leaves the bit timing
// untouched when it cannot hit the request. These helpers read the registers
// back so the serial monitor reports what is genuinely on the wire.
//
// Teensy only. The ESP32 build never includes this header.

#include <Arduino.h>
#include <FlexCAN_T4.h>

struct FlexCanBitTiming {
  uint32_t clockHz = 0;
  uint16_t prescaler = 0;         // PRESDIV + 1
  uint8_t propSeg = 0;            // time quanta, PROPSEG + 1
  uint8_t phaseSeg1 = 0;          // time quanta, PSEG1 + 1
  uint8_t phaseSeg2 = 0;          // time quanta, PSEG2 + 1
  uint8_t syncJumpWidth = 0;      // time quanta, RJW + 1
  uint8_t quantaPerBit = 0;
  uint32_t bitRate = 0;           // what the controller actually clocks out
  uint8_t samplePointPercent = 0;
};

// The library's own getClock() is private and ignores the post divider, so this
// reproduces the full calculation: CCM_CSCMR2 holds the source select in bits
// 9:8 and the post divider in bits 7:2.
inline uint32_t flexcanClockHz() {
  static const uint8_t kSourceMhz[4] = {60, 24, 80, 0};
  const uint8_t source = static_cast<uint8_t>((CCM_CSCMR2 & 0x300) >> 8);
  const uint8_t postDivider = static_cast<uint8_t>((CCM_CSCMR2 >> 2) & 0x3F);

  return (static_cast<uint32_t>(kSourceMhz[source]) * 1000000UL) / (postDivider + 1UL);
}

inline FlexCanBitTiming flexcanReadBitTiming(uint32_t busBase) {
  const uint32_t ctrl1 = FLEXCANb_CTRL1(busBase);

  FlexCanBitTiming out;
  out.clockHz = flexcanClockHz();
  out.prescaler = static_cast<uint16_t>(((ctrl1 >> 24) & 0xFF) + 1);
  out.syncJumpWidth = static_cast<uint8_t>(((ctrl1 >> 22) & 0x3) + 1);
  out.phaseSeg1 = static_cast<uint8_t>(((ctrl1 >> 19) & 0x7) + 1);
  out.phaseSeg2 = static_cast<uint8_t>(((ctrl1 >> 16) & 0x7) + 1);
  out.propSeg = static_cast<uint8_t>((ctrl1 & 0x7) + 1);

  // One sync quantum, then the three programmable segments.
  out.quantaPerBit =
      static_cast<uint8_t>(1 + out.propSeg + out.phaseSeg1 + out.phaseSeg2);

  const uint32_t divisor =
      static_cast<uint32_t>(out.prescaler) * static_cast<uint32_t>(out.quantaPerBit);
  out.bitRate = divisor == 0 ? 0 : (out.clockHz / divisor);

  // The bus is sampled at the end of phase segment 1.
  const uint16_t beforeSample =
      static_cast<uint16_t>(1 + out.propSeg + out.phaseSeg1);
  out.samplePointPercent =
      out.quantaPerBit == 0
          ? 0
          : static_cast<uint8_t>((beforeSample * 100U) / out.quantaPerBit);

  return out;
}

// One line, printed next to the bus state. `requestedBaud` is what the firmware
// asked for, so a mismatch stands out without any arithmetic by the reader.
inline void flexcanPrintBitTiming(const char *tag, uint32_t busBase,
                                  uint32_t requestedBaud) {
  const FlexCanBitTiming timing = flexcanReadBitTiming(busBase);

  Serial.print(tag);
  Serial.print(" timing: asked=");
  Serial.print(requestedBaud);
  Serial.print(" actual=");
  Serial.print(timing.bitRate);
  Serial.print(" bit/s clk=");
  Serial.print(timing.clockHz / 1000000UL);
  Serial.print("MHz presc=");
  Serial.print(timing.prescaler);
  Serial.print(" tq=");
  Serial.print(timing.quantaPerBit);
  Serial.print(" (prop=");
  Serial.print(timing.propSeg);
  Serial.print(" pseg1=");
  Serial.print(timing.phaseSeg1);
  Serial.print(" pseg2=");
  Serial.print(timing.phaseSeg2);
  Serial.print(" rjw=");
  Serial.print(timing.syncJumpWidth);
  Serial.print(") sample=");
  Serial.print(timing.samplePointPercent);
  Serial.println('%');

  // A 5% error is already enough to lose every frame on a long message.
  const uint32_t tolerance = requestedBaud / 20;
  if (timing.bitRate + tolerance < requestedBaud ||
      timing.bitRate > requestedBaud + tolerance) {
    Serial.print(tag);
    Serial.println(" the controller is NOT running at the requested rate; the "
                   "library could not build that bit timing from this clock");
  }
}

// FlexCAN_T4 leaves freeze mode with an unbounded spin on FRZ_ACK
// (FlexCAN_T4.tpp, FLEXCAN_ExitFreezeMode), and the controller only clears that
// bit once it has seen an idle bus. Every bring-up call goes through it:
// begin(), setBaudRate(), setMaxMB(), enableFIFO(), setFIFOFilter(). A receive
// line held dominant therefore hangs setup() forever, with no serial output at
// all, which looks exactly like a dead board rather than a dead bus.
//
// The library configures the receive pad with 0x10B0, where PKE is clear, so
// the pin has no pull-up, no pull-down and no keeper. Sampling it through the
// internal pull-up before the bus is brought up separates a wire that is merely
// quiet from one that is stuck low.
//
// A busy bus still returns true: only a line that never goes recessive across
// the whole window is the one that hangs the freeze-mode exit.
inline bool flexcanRxLineIsUsable(uint8_t rxPin) {
  pinMode(rxPin, INPUT_PULLUP);
  delayMicroseconds(50);

  const uint32_t start = millis();
  while ((millis() - start) < 3) {
    if (digitalRead(rxPin) == HIGH) {
      pinMode(rxPin, INPUT);
      return true;
    }
  }

  pinMode(rxPin, INPUT);
  return false;
}

// Prints why a bus was skipped. Kept next to the check so both links report the
// same way and the serial monitor names the pin to put a probe on.
inline void flexcanReportStuckRxLine(const char *tag, uint8_t rxPin) {
  Serial.print(tag);
  Serial.print(" receive pin ");
  Serial.print(rxPin);
  Serial.println(" is held dominant (low). Bringing this bus up would spin "
                 "forever inside FlexCAN's freeze-mode exit and take the whole "
                 "board with it, so the link is left down. Check transceiver "
                 "power, CANH/CANL not shorted or swapped, and the two 120 ohm "
                 "terminators. The rest of the firmware keeps running.");
}
