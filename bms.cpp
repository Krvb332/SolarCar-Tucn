#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Beginner-friendly overview:
// - This file asks an ANT BMS for data over Serial2.
// - The BMS replies with fixed-size binary frames (140 bytes).
// - We decode those bytes into human-readable telemetry and print to USB Serial.
//
// How to read this file (recommended order):
// 1) setup() and loop() at the bottom (high-level flow).
// 2) sendBmsRequestIfNeeded() (how requests are sent).
// 3) pollBmsUart() + processRxBuffer() (how bytes are collected into frames).
// 4) parseFrame() and helper parsers (how bytes become telemetry values).
// 5) printTelemetry() helpers (how values are shown on Serial monitor).
// Note 1: This module follows a "request then parse reply" pattern common in UART protocols.
// Note 2: UART is byte-stream based, so frame boundaries are reconstructed in software.
// Note 3: A fixed frame size simplifies parsing because offsets can stay constant.
// Note 4: Constants near the top make protocol tuning safer and easier to review.
// Note 5: `static constexpr` values are compile-time constants with no runtime storage cost.
// Note 6: Naming constants with units (for example `_MS`) reduces timing mistakes.
// Note 7: The code separates transport logic from decoding logic for easier debugging.
// Note 8: Parsing into a struct gives one stable "snapshot" for printing and downstream logic.
// Note 9: Decoding with helper functions avoids copy-paste arithmetic bugs.
// Note 10: Big-endian decoding is explicit, so behavior is portable across CPUs.
// Note 11: `Serial` and `Serial2` are intentionally distinct channels with separate purposes.
// Note 12: USB serial is human-facing, while Serial2 is protocol-facing.
// Note 13: The ANT request command is represented as raw bytes to match the wire protocol exactly.
// Note 14: Header bytes are stored once and reused, which avoids magic values in loops.
// Note 15: Fixed-size arrays provide deterministic memory usage on microcontrollers.
// Note 16: Deterministic memory usage is valuable where heap fragmentation is risky.
// Note 17: Time-sliced reads keep the main loop responsive and avoid long blocking waits.
// Note 18: The code uses polling with small delays instead of interrupts for simplicity.
// Note 19: Polling is often enough when baud rates and frame rates are modest.
// Note 20: Telemetry conversion factors are centralized to keep units consistent.
// Note 21: Unit conversion helpers also make testing easier because formulas live in one place.
// Note 22: Offsets map protocol bytes to fields and act like lightweight documentation.
// Note 23: Keeping offsets symbolic allows easy updates if protocol revisions shift fields.
// Note 24: The telemetry struct stores both raw-derived values and computed summaries.
// Note 25: Derived fields like min, max, and delta support quick health checks.
// Note 26: Global state is limited and intentional, which is common in embedded loops.
// Note 27: `hasTelemetry` gates printing so startup noise does not look like valid data.
// Note 28: The receive buffer is larger than a frame to tolerate partial and noisy input.
// Note 29: A larger buffer also allows header resynchronization after line noise.
// Note 30: `readU16BE` demonstrates byte assembly with shifts and bitwise OR.
// Note 31: Casts in bit operations prevent accidental sign extension and narrowing.
// Note 32: Signed readers build on unsigned readers, reducing duplicated logic.
// Note 33: `readU32BE` handles 4-byte assembly in a way that is endian-stable.
// Note 34: Conversion helpers encode domain intent, not just arithmetic.
// Note 35: `findHeader` is a linear scan, which is fine for small embedded buffers.
// Note 36: The header scan guards length first to prevent out-of-bounds reads.
// Note 37: Returning `-1` for "not found" is a classic C-style sentinel.
// Note 38: `parseCellBlock` computes statistics in one pass for efficiency.
// Note 39: One-pass min/max/average avoids extra loops and keeps CPU use low.
// Note 40: Cell index plus one is used because user-facing cell IDs are usually 1-based.
// Note 41: Zero millivolts is treated as an unused slot by this protocol.
// Note 42: `hasMinMax` handles initialization without requiring large sentinel values.
// Note 43: Accumulating only active cells gives a meaningful average.
// Note 44: Delta (max - min) is a key balancing metric in battery packs.
// Note 45: Temperature parsing uses signed 16-bit reads for negative-capable values.
// Note 46: Parsing temperature channels with a loop keeps channel count configurable.
// Note 47: `parseFrame` stages decoding in logical steps for readability.
// Note 48: Parsing into a local `BmsTelemetry t` avoids partially updated global state.
// Note 49: Publishing telemetry only at the end gives an atomic-like update pattern.
// Note 50: `millis()` timestamps make staleness checks independent from print timing.
// Note 51: `compactBuffer` removes consumed bytes without reallocating memory.
// Note 52: `memmove` is used because source and destination ranges can overlap.
// Note 53: Resetting `rxLen` to zero is a fast recovery path after full consumption.
// Note 54: `processRxBuffer` loops until no complete frame remains in the buffer.
// Note 55: Parsing in a loop allows burst handling when multiple frames arrive quickly.
// Note 56: Dropping leading bytes before header is a classic resync strategy.
// Note 57: Buffer reset on excessive garbage avoids endless drift on bad streams.
// Note 58: Waiting for full frame length prevents parsing incomplete data.
// Note 59: `pollBmsUart` uses nested loops: outer for time slice, inner for available bytes.
// Note 60: Reading as `int` allows detection of negative return values from `read()`.
// Note 61: Overflow handling intentionally clears buffer to re-enter a known-good state.
// Note 62: Calling `processRxBuffer` frequently reduces latency from receive to decode.
// Note 63: Small `delay(1)` yields CPU time and avoids tight-spin starvation.
// Note 64: Request cadence is controlled with elapsed-time logic, not blocking delays.
// Note 65: Elapsed-time checks with subtraction are robust across `millis()` wraparound.
// Note 66: `Serial2.flush()` waits for transmit completion, helping predictable timing.
// Note 67: Printing helpers keep formatting concerns separate from parsing concerns.
// Note 68: `printCells` skips zero-valued entries so output reflects active channels.
// Note 69: Fixed precision in prints improves log comparability over time.
// Note 70: The summary line favors compactness for serial monitor readability.
// Note 71: Field labels (`Vpack`, `Ipack`) are short to minimize UART output volume.
// Note 72: Temperature output is generated in a simple comma-separated list.
// Note 73: `printTelemetry` composes multiple specialized print functions.
// Note 74: Composition makes it easier to add or remove output sections later.
// Note 75: Connection status logic distinguishes "never connected" from "stale data".
// Note 76: Startup waiting messages reassure users while the first frame is pending.
// Note 77: Stale warnings help detect cable faults or protocol mismatch at runtime.
// Note 78: Updating `lastFrameMs` after stale print rate-limits repeated warnings.
// Note 79: `setup` initializes both serial channels before active processing starts.
// Note 80: Startup logs print pins and baud to speed up wiring verification.
// Note 81: Explicitly stating expected frame shape helps quick field debugging.
// Note 82: `loop` is designed as a small scheduler with cooperative tasks.
// Note 83: Cooperative scheduling works well when each task stays short and bounded.
// Note 84: Printing is time-gated to avoid saturating USB serial bandwidth.
// Note 85: Protocol handling and user logging run concurrently within one loop.
// Note 86: This design avoids dynamic allocation, which is often safer on embedded targets.
// Note 87: The parser trusts frame structure once header and size checks pass.
// Note 88: If checksum support is later needed, `processRxBuffer` is the right insertion point.
// Note 89: The code favors explicitness over abstraction, which helps firmware maintainability.
// Note 90: Each helper is short, making step-through debugging straightforward.
// Note 91: `uint8_t` is used for raw bytes to keep binary intent clear.
// Note 92: `size_t` is used for indices to match array-size semantics.
// Note 93: Distinct constants separate protocol facts from runtime policy.
// Note 94: Runtime policy examples include print interval and stale timeout values.
// Note 95: Parsing order mirrors frame layout, reducing cognitive load when troubleshooting.
// Note 96: Single-responsibility helpers make unit-level reasoning easier.
// Note 97: Guard clauses (`return` early) keep control flow flat and readable.
// Note 98: This file demonstrates an end-to-end embedded data pipeline.
// Note 99: Pipeline stages are request, receive, frame sync, decode, and present.
// Note 100: The implementation is resilient to partial reads and line noise.
// Note 101: The code is intentionally conservative about state transitions.
// Note 102: Conservative transitions reduce surprising output for operators.
// Note 103: The design can be extended with alarms using existing telemetry fields.
// Note 104: Keep protocol constants versioned when integrating with different BMS firmwares.

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t BMS_BAUD = 19200;
static constexpr uint32_t NEXTION_BAUD = 9600;
// UART0 (GPIO1/GPIO3) is shared with USB serial monitor on ESP32 dev boards.
// - true  => Nextion uses TX0/RX0 (serial monitor output must stay OFF)
// - false => serial monitor can be used, but Nextion must be moved to another UART
static constexpr bool NEXTION_ON_UART0 = true;
static constexpr bool ENABLE_USB_DEBUG_OUTPUT = !NEXTION_ON_UART0;

static constexpr int BMS_RX_PIN = 16;
static constexpr int BMS_TX_PIN = 17;
static constexpr int NEXTION_RX_PIN = 3;
static constexpr int NEXTION_TX_PIN = 1;

static constexpr uint8_t ANT_REQUEST_CMD_2[6] = {0xDB, 0xDB, 0x00, 0x00, 0x00, 0x00};
static constexpr uint8_t ANT_HEADER[4] = {0xAA, 0x55, 0xAA, 0xFF};

static constexpr size_t ANT_FRAME_SIZE = 140;
static constexpr uint8_t ANT_CELL_COUNT = 32;
static constexpr uint8_t ANT_TEMP_COUNT = 6;
static constexpr uint32_t REQUEST_INTERVAL_MS = 1000;
static constexpr uint32_t READ_SLICE_MS = 20;
static constexpr uint32_t PRINT_INTERVAL_MS = 1000;
static constexpr uint32_t WAITING_MESSAGE_PERIOD_MS = 2000;
static constexpr uint32_t WAITING_MESSAGE_WINDOW_MS = 50;
static constexpr uint32_t STALE_FRAME_TIMEOUT_MS = 3000;
static constexpr uint8_t NEXTION_TERMINATOR = 0xFF;

static constexpr float PACK_VOLTAGE_SCALE = 0.1f;
static constexpr float PACK_CURRENT_SCALE = 0.1f;
static constexpr float MILLIVOLTS_TO_VOLTS = 0.001f;
static constexpr float MICRO_AH_TO_AH = 0.000001f;

// ANT frame fields used here (big-endian):
// [4..5]   pack voltage (0.1V)
// [6..69]  32x cell voltage (mV, 0 means unused slot)
// [70..73] pack current (0.1A, signed)
// [74]     SOC (%)
// [75..78] total capacity (uAh)
// [79..82] remaining capacity (uAh)
// [87..88] cycle count
// [91..102] 6x temperatures
// [103]    charge MOS state
// [104]    discharge MOS state
static constexpr size_t OFFSET_PACK_VOLTAGE = 4;
static constexpr size_t OFFSET_CELLS = 6;
static constexpr size_t OFFSET_PACK_CURRENT = 70;
static constexpr size_t OFFSET_SOC = 74;
static constexpr size_t OFFSET_CAP_TOTAL = 75;
static constexpr size_t OFFSET_CAP_REMAIN = 79;
static constexpr size_t OFFSET_CYCLES = 87;
static constexpr size_t OFFSET_TEMPS = 91;
static constexpr size_t OFFSET_MOS_CHARGE = 103;
static constexpr size_t OFFSET_MOS_DISCHARGE = 104;

struct BmsTelemetry {
	float packVoltage = 0.0f;
	float packCurrent = 0.0f;
	float soc = 0.0f;
	float capTotalAh = 0.0f;
	float capRemainAh = 0.0f;
	uint16_t cycles = 0;
	float temps[ANT_TEMP_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	bool mosCharge = false;
	bool mosDischarge = false;
	float cells[ANT_CELL_COUNT] = {0.0f};
	uint8_t activeCellCount = 0;
	float minCell = 0.0f;
	float maxCell = 0.0f;
	float avgCell = 0.0f;
	float deltaCell = 0.0f;
	uint8_t minCellId = 0;
	uint8_t maxCellId = 0;
};

// ESP-NOW structure (must match on receiver)
typedef struct struct_message {
	float packVoltage;
	float packCurrent;
	float soc;
	float capTotalAh;
	float capRemainAh;
	uint16_t cycles;
	float temps[ANT_TEMP_COUNT];
	uint8_t mosCharge;
	uint8_t mosDischarge;
	float cells[ANT_CELL_COUNT];
	uint8_t activeCellCount;
	float minCell;
	float maxCell;
	float avgCell;
	float deltaCell;
	uint8_t minCellId;
	uint8_t maxCellId;
} struct_message;

// MAC address for broadcast (FF:FF:FF:FF:FF:FF)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

static BmsTelemetry telemetry;
static bool hasTelemetry = false;

// Raw receive buffer: incoming bytes are appended here until a full frame is available.
static uint8_t rxBuffer[512];
static size_t rxLen = 0;
static uint32_t lastRequestMs = 0;
static uint32_t lastPrintMs = 0;
static uint32_t lastFrameMs = 0;

// Byte helpers:
// ANT frames are big-endian, so we rebuild integers from most-significant byte first.
static uint16_t readU16BE(const uint8_t* p) {
	return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

static int16_t readI16BE(const uint8_t* p) {
	return static_cast<int16_t>(readU16BE(p));
}

static uint32_t readU32BE(const uint8_t* p) {
	return (static_cast<uint32_t>(p[0]) << 24) |
				 (static_cast<uint32_t>(p[1]) << 16) |
				 (static_cast<uint32_t>(p[2]) << 8) |
				 static_cast<uint32_t>(p[3]);
}

static int32_t readI32BE(const uint8_t* p) {
	return static_cast<int32_t>(readU32BE(p));
}

// Unit conversion helpers keep formulas in one place.
static float convertMilliVoltsToVolts(const uint16_t milliVolts) {
	return static_cast<float>(milliVolts) * MILLIVOLTS_TO_VOLTS;
}

static float convertTenthsToFloat(const int32_t valueInTenths) {
	return static_cast<float>(valueInTenths) * PACK_CURRENT_SCALE;
}

static float convertMicroAhToAh(const uint32_t microAh) {
	return static_cast<float>(microAh) * MICRO_AH_TO_AH;
}

// Returns where the ANT header starts in the buffer, or -1 if not found.
static int findHeader(const uint8_t* data, size_t len) {
	if (len < 4) {
		return -1;
	}

	for (size_t i = 0; i <= len - 4; ++i) {
		if (data[i] == ANT_HEADER[0] && data[i + 1] == ANT_HEADER[1] &&
				data[i + 2] == ANT_HEADER[2] && data[i + 3] == ANT_HEADER[3]) {
			return static_cast<int>(i);
		}
	}

	return -1;
}

static void parseCellBlock(const uint8_t* frame, BmsTelemetry& t) {
	float cellSum = 0.0f;
	bool hasMinMax = false;

	// Step 1: read up to 32 cell values from the frame.
	for (uint8_t i = 0; i < ANT_CELL_COUNT; ++i) {
		const size_t offset = OFFSET_CELLS + (i * 2);
		const uint16_t cellMv = readU16BE(&frame[offset]);

		// cellMv == 0 means this cell slot is unused in this frame.
		if (cellMv > 0) {
			t.cells[i] = convertMilliVoltsToVolts(cellMv);
			t.activeCellCount++;
			cellSum += t.cells[i];

			// Step 2: track minimum and maximum cell values as we go.
			if (!hasMinMax || t.cells[i] < t.minCell) {
				t.minCell = t.cells[i];
				t.minCellId = i + 1;
			}
			if (!hasMinMax || t.cells[i] > t.maxCell) {
				t.maxCell = t.cells[i];
				t.maxCellId = i + 1;
			}

			hasMinMax = true;
		}
	}

	// Step 3: compute average and spread (delta) once all cells are scanned.
	if (t.activeCellCount > 0) {
		t.avgCell = cellSum / static_cast<float>(t.activeCellCount);
		t.deltaCell = t.maxCell - t.minCell;
	}
}

static void parseTemperatureBlock(const uint8_t* frame, BmsTelemetry& t) {
	// Temperatures are stored as signed 16-bit values.
	for (uint8_t i = 0; i < ANT_TEMP_COUNT; ++i) {
		const size_t offset = OFFSET_TEMPS + (i * 2);
		t.temps[i] = static_cast<float>(readI16BE(&frame[offset]));
	}
}

// Decode one full 140-byte ANT frame into telemetry.
static void parseFrame(const uint8_t* frame) {
	BmsTelemetry t;

	// Step 1: parse top-level pack values.
	t.packVoltage =
			static_cast<float>(readU16BE(&frame[OFFSET_PACK_VOLTAGE])) * PACK_VOLTAGE_SCALE;
	parseCellBlock(frame, t);

	// Step 2: parse current, SOC, capacities, cycles, temperatures.
	t.packCurrent = convertTenthsToFloat(readI32BE(&frame[OFFSET_PACK_CURRENT]));
	t.soc = static_cast<float>(frame[OFFSET_SOC]);
	t.capTotalAh = convertMicroAhToAh(readU32BE(&frame[OFFSET_CAP_TOTAL]));
	t.capRemainAh = convertMicroAhToAh(readU32BE(&frame[OFFSET_CAP_REMAIN]));
	t.cycles = readU16BE(&frame[OFFSET_CYCLES]);
	parseTemperatureBlock(frame, t);

	// Step 3: parse MOS states.
	t.mosCharge = frame[OFFSET_MOS_CHARGE] != 0;
	t.mosDischarge = frame[OFFSET_MOS_DISCHARGE] != 0;

	// Step 4: publish the fully decoded telemetry as the latest snapshot.
	telemetry = t;
	hasTelemetry = true;
	lastFrameMs = millis();
}

static void compactBuffer(size_t start, size_t count) {
	const size_t end = start + count;
	if (end >= rxLen) {
		rxLen = 0;
		return;
	}

	const size_t remaining = rxLen - end;
	memmove(rxBuffer, rxBuffer + end, remaining);
	rxLen = remaining;
}

static void processRxBuffer() {
	// Beginner view of buffer processing:
	// 1) Append bytes into rxBuffer.
	// 2) Search for header anywhere in buffered data.
	// 3) Drop noise before header.
	// 4) Parse exactly one fixed-size frame (140B), then compact and repeat.
	while (true) {
		const int headerIdx = findHeader(rxBuffer, rxLen);
		if (headerIdx < 0) {
			// No header yet. If garbage grows too much, clear and resync.
			if (rxLen > (ANT_FRAME_SIZE * 2)) {
				rxLen = 0;
			}
			return;
		}

		if (headerIdx > 0) {
			// Found header, but not at index 0: discard bytes before it.
			compactBuffer(0, static_cast<size_t>(headerIdx));
			continue;
		}

		if (rxLen < ANT_FRAME_SIZE) {
			// Header is aligned, but frame is incomplete. Wait for more bytes.
			return;
		}

		// Full frame ready.
		parseFrame(rxBuffer);
		compactBuffer(0, ANT_FRAME_SIZE);
	}
}

static void pollBmsUart() {
	const uint32_t start = millis();

	// Read for a short time slice so loop() stays responsive.
	while ((millis() - start) < READ_SLICE_MS) {
		while (Serial2.available() > 0) {
			const int value = Serial2.read();
			if (value < 0) {
				break;
			}

			if (rxLen < sizeof(rxBuffer)) {
				rxBuffer[rxLen++] = static_cast<uint8_t>(value);
			} else {
				// Overflow protection: reset and wait for next clean frame.
				rxLen = 0;
			}
		}

		processRxBuffer();
		delay(1);
	}
}

static void sendBmsRequestIfNeeded() {
	const uint32_t now = millis();
	if (now - lastRequestMs >= REQUEST_INTERVAL_MS) {
		// Ask the BMS for a new frame.
		Serial2.write(ANT_REQUEST_CMD_2, sizeof(ANT_REQUEST_CMD_2));
		Serial2.flush();
		lastRequestMs = now;
	}
}

static void printCells() {
	if (!ENABLE_USB_DEBUG_OUTPUT) {
		return;
	}

	Serial.print("Cells[");
	Serial.print(telemetry.activeCellCount);
	Serial.print("]: ");

	for (uint8_t i = 0; i < ANT_CELL_COUNT; ++i) {
		if (telemetry.cells[i] <= 0.0f) {
			continue;
		}

		Serial.print(i + 1);
		Serial.print('=');
		Serial.print(telemetry.cells[i], 3);
		Serial.print("V ");
	}
	Serial.println();
}

static void printSummaryLine() {
	if (!ENABLE_USB_DEBUG_OUTPUT) {
		return;
	}

	// Single-line status summary for quick monitoring.
	Serial.print("Vpack=");
	Serial.print(telemetry.packVoltage, 1);
	Serial.print("V | Ipack=");
	Serial.print(telemetry.packCurrent, 1);
	Serial.print("A | SOC=");
	Serial.print(telemetry.soc, 1);
	Serial.print("% | Cap=");
	Serial.print(telemetry.capRemainAh, 2);
	Serial.print('/');
	Serial.print(telemetry.capTotalAh, 2);
	Serial.print("Ah | Cycles=");
	Serial.print(telemetry.cycles);
	Serial.print(" | MOS(chg,dsg)=");
	Serial.print(telemetry.mosCharge ? "1" : "0");
	Serial.print(',');
	Serial.print(telemetry.mosDischarge ? "1" : "0");
	Serial.print(" | CellMin=");
	Serial.print(telemetry.minCell, 3);
	Serial.print("V(#");
	Serial.print(telemetry.minCellId);
	Serial.print(") | CellMax=");
	Serial.print(telemetry.maxCell, 3);
	Serial.print("V(#");
	Serial.print(telemetry.maxCellId);
	Serial.print(") | Delta=");
	Serial.print(telemetry.deltaCell, 3);
	Serial.println("V");
}

static void printTemperatureLine() {
	if (!ENABLE_USB_DEBUG_OUTPUT) {
		return;
	}

	// Print all available temperature channels.
	Serial.print("Temps: ");
	for (uint8_t i = 0; i < ANT_TEMP_COUNT; ++i) {
		Serial.print(telemetry.temps[i], 1);
		if (i < (ANT_TEMP_COUNT - 1)) {
			Serial.print(", ");
		}
	}
	Serial.println();
}

static void printTelemetry() {
	printSummaryLine();
	printTemperatureLine();
	printCells();
}

static void sendNextionTerminator() {
	Serial.write(NEXTION_TERMINATOR);
	Serial.write(NEXTION_TERMINATOR);
	Serial.write(NEXTION_TERMINATOR);
}

static void sendNextionNumber(const char* componentName, const int32_t value) {
	Serial.print(componentName);
	Serial.print(".val=");
	Serial.print(static_cast<long>(value));
	sendNextionTerminator();
}

static void sendNextionUnsignedNumber(const char* componentName, const uint32_t value) {
	Serial.print(componentName);
	Serial.print(".val=");
	Serial.print(static_cast<unsigned long>(value));
	sendNextionTerminator();
}

static void sendNextionPropertyInt(const char* componentName, const char* propertyName,
													const int32_t value) {
	Serial.print(componentName);
	Serial.print('.');
	Serial.print(propertyName);
	Serial.print('=');
	Serial.print(static_cast<long>(value));
	sendNextionTerminator();
}

static int32_t convertToNextionInt(const float value) {
	if (value >= 0.0f) {
		return static_cast<int32_t>(value + 0.5f);
	}
	return static_cast<int32_t>(value - 0.5f);
}

static float absoluteFloat(const float value) {
	return (value < 0.0f) ? -value : value;
}

static uint32_t convertToNextionUInt(const float value) {
	if (!(value >= 0.0f)) {
		return 0;
	}

	const float rounded = value + 0.5f;
	if (rounded >= static_cast<float>(0xFFFFFFFFu)) {
		return 0xFFFFFFFFu;
	}

	return static_cast<uint32_t>(rounded);
}

static float computeAverageTemperature(const BmsTelemetry& t) {
	float sum = 0.0f;
	for (uint8_t i = 0; i < ANT_TEMP_COUNT; ++i) {
		sum += t.temps[i];
	}
	return sum / static_cast<float>(ANT_TEMP_COUNT);
}

static void sendNextionTelemetry() {
	const int32_t socInt = convertToNextionInt(telemetry.soc);
	const int32_t vpackInt = convertToNextionInt(telemetry.packVoltage);
	const uint32_t amperageInt = convertToNextionUInt(absoluteFloat(telemetry.packCurrent));
	const uint32_t wattageInt =
			convertToNextionUInt(absoluteFloat(telemetry.packVoltage * telemetry.packCurrent));
	const int32_t avgTempInt = convertToNextionInt(computeAverageTemperature(telemetry));

	// Requested Nextion mapping:
	// n12 = SOC, n4 = Vpack, n3 = amperage, n2 = wattage, n15 = average temp.
	sendNextionNumber("n12", socInt);
	sendNextionNumber("n4", vpackInt);
	sendNextionUnsignedNumber("n3", amperageInt);
	sendNextionUnsignedNumber("n2", wattageInt);
	sendNextionNumber("n15", avgTempInt);
}

static void configureNextionNumericRanges() {
	// Ensure runtime values are inside widget ranges so Nextion does not ignore updates.
	sendNextionPropertyInt("n3", "minval", 0);
	sendNextionPropertyInt("n3", "maxval", 2000);
	sendNextionPropertyInt("n2", "minval", 0);
	sendNextionPropertyInt("n2", "maxval", 300000);
}

static void sendNextionStartupValues() {
	sendNextionNumber("n12", 0);
	sendNextionNumber("n4", 0);
	sendNextionNumber("n3", 0);
	sendNextionNumber("n2", 0);
	sendNextionNumber("n15", 0);
}

static void printConnectionStatus(const uint32_t now) {
	if (!ENABLE_USB_DEBUG_OUTPUT) {
		return;
	}

	// Keep existing cadence/conditions:
	// - before first frame: periodic waiting message
	// - after frames start: stale warning when no fresh frame for >3s
	if (!hasTelemetry && (now % WAITING_MESSAGE_PERIOD_MS < WAITING_MESSAGE_WINDOW_MS)) {
		Serial.println("Waiting for ANT BMS frame...");
	} else if (hasTelemetry && (now - lastFrameMs > STALE_FRAME_TIMEOUT_MS)) {
		Serial.println("No fresh ANT BMS frame for >3s");
		lastFrameMs = now;
	}
}

static void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
	if (ENABLE_USB_DEBUG_OUTPUT) {
		Serial.print("Last Packet Send Status: ");
		Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
	}
}

static void initEspNow() {
	WiFi.mode(WIFI_STA);

	if (esp_now_init() != ESP_OK) {
		if (ENABLE_USB_DEBUG_OUTPUT) {
			Serial.println("Error initializing ESP-NOW");
		}
		return;
	}

	esp_now_register_send_cb(OnDataSent);

	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
	peerInfo.channel = 0;
	peerInfo.encrypt = false;
	// peerInfo.ifidx = WIFI_IF_STA; // Explicitly set interface (though 0 is default)

	if (esp_now_add_peer(&peerInfo) != ESP_OK) {
		if (ENABLE_USB_DEBUG_OUTPUT) {
			Serial.println("Failed to add peer");
		}
		return;
	}
}

static void sendEspNowTelemetry() {
	struct_message msg;
	msg.packVoltage = telemetry.packVoltage;
	msg.packCurrent = telemetry.packCurrent;
	msg.soc = telemetry.soc;
	msg.capTotalAh = telemetry.capTotalAh;
	msg.capRemainAh = telemetry.capRemainAh;
	msg.cycles = telemetry.cycles;
	
	for (int i = 0; i < ANT_TEMP_COUNT; i++) {
		msg.temps[i] = telemetry.temps[i];
	}

	msg.mosCharge = telemetry.mosCharge ? 1 : 0;
	msg.mosDischarge = telemetry.mosDischarge ? 1 : 0;

	for (int i = 0; i < ANT_CELL_COUNT; i++) {
		msg.cells[i] = telemetry.cells[i];
	}

	msg.activeCellCount = telemetry.activeCellCount;
	msg.minCell = telemetry.minCell;
	msg.maxCell = telemetry.maxCell;
	msg.avgCell = telemetry.avgCell;
	msg.deltaCell = telemetry.deltaCell;
	msg.minCellId = telemetry.minCellId;
	msg.maxCellId = telemetry.maxCellId;

	esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&msg, sizeof(msg));

	if (result == ESP_OK) {
		if (ENABLE_USB_DEBUG_OUTPUT) {
			// Serial.println("Sent with success"); // Optional: limit logging
		}
	} else {
		if (ENABLE_USB_DEBUG_OUTPUT) {
			Serial.println("Error sending the data");
		}
	}
}

void setup() {
	// UART0 (GPIO3/GPIO1) is dedicated to Nextion commands.
	Serial.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
	delay(300);
	initEspNow();
	configureNextionNumericRanges();
	sendNextionStartupValues();

	// Serial2 is the UART connected to the BMS.
	Serial2.begin(BMS_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);

	if (ENABLE_USB_DEBUG_OUTPUT) {
		Serial.println();
		Serial.println("ESP32 ANT BMS reader started");
		Serial.print("Serial2 RX=");
		Serial.print(BMS_RX_PIN);
		Serial.print(" TX=");
		Serial.print(BMS_TX_PIN);
		Serial.print(" Baud=");
		Serial.println(BMS_BAUD);
		Serial.print("Nextion RX=");
		Serial.print(NEXTION_RX_PIN);
		Serial.print(" TX=");
		Serial.print(NEXTION_TX_PIN);
		Serial.print(" Baud=");
		Serial.println(NEXTION_BAUD);
		Serial.println("Sending DB DB request once per second; expecting 140-byte AA55AAFF frames");
	}
}

void loop() {
	// Main loop flow:
	// - send one request every second
	// - collect/parse UART data in short slices
	// - print telemetry once per second when valid data exists
	// - emit connection status messages
	sendBmsRequestIfNeeded();
	pollBmsUart();

	const uint32_t now = millis();
	if (hasTelemetry && (now - lastPrintMs >= PRINT_INTERVAL_MS)) {
		printTelemetry();
		sendNextionTelemetry();
		sendEspNowTelemetry();
		lastPrintMs = now;
	}

	printConnectionStatus(now);
}
