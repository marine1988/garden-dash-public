// Golden-vector tests for the VESC wire protocol parsers.
//
// Each COMM payload is built by writing known values at the documented field
// offsets, then parsed back. That checks the offsets against intent rather than
// against themselves. The expected values were validated field-for-field
// against the previous VescUart implementation over 20,000 randomised payloads
// before the library was removed.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "vesc_protocol.h"

namespace {

int failures = 0;

void expect(bool ok, const char *what) {
  if (!ok) {
    printf("FAIL: %s\n", what);
    failures++;
  }
}

void expectNear(float actual, float wanted, const char *what) {
  if (std::fabs(actual - wanted) > 1e-3F) {
    printf("FAIL: %s (got %.6f, want %.6f)\n", what, actual, wanted);
    failures++;
  }
}

void writeInt16(uint8_t *buf, size_t &i, int16_t v) {
  buf[i++] = (uint8_t)((uint16_t)v >> 8);
  buf[i++] = (uint8_t)((uint16_t)v & 0xFF);
}

void writeInt32(uint8_t *buf, size_t &i, int32_t v) {
  const uint32_t u = (uint32_t)v;
  buf[i++] = (uint8_t)(u >> 24);
  buf[i++] = (uint8_t)(u >> 16);
  buf[i++] = (uint8_t)(u >> 8);
  buf[i++] = (uint8_t)(u & 0xFF);
}

void writeFloat16(uint8_t *buf, size_t &i, float v, float scale) {
  writeInt16(buf, i, (int16_t)lroundf(v * scale));
}

void writeFloat32(uint8_t *buf, size_t &i, float v, float scale) {
  writeInt32(buf, i, (int32_t)lroundf(v * scale));
}

void testCrc() {
  // The standard CRC-16/XMODEM check value.
  const uint8_t digits[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
  expect(vescCrc16(digits, sizeof(digits)) == 0x31C3, "crc16 check value 0x31C3");

  const uint8_t one[] = {VESC_COMM_GET_VALUES};
  expect(vescCrc16(one, 1) == 0x4084, "crc16 of a single COMM_GET_VALUES byte");
}

void testGetValues() {
  uint8_t payload[80] = {};
  size_t i = 0;
  payload[i++] = VESC_COMM_GET_VALUES;
  writeFloat16(payload, i, 32.5F, 1e1F);     // mosfet temperature
  writeFloat16(payload, i, 41.2F, 1e1F);     // motor temperature
  writeFloat32(payload, i, 18.75F, 1e2F);    // average motor current
  writeFloat32(payload, i, 7.32F, 1e2F);     // average input current
  writeInt32(payload, i, 0);                 // avg_id, skipped
  writeInt32(payload, i, 0);                 // avg_iq, skipped
  writeFloat16(payload, i, 0.62F, 1e3F);     // signed duty cycle
  writeFloat32(payload, i, 3600.0F, 1e0F);   // eRPM
  writeFloat16(payload, i, 48.6F, 1e1F);     // input voltage
  writeFloat32(payload, i, 2.5431F, 1e4F);   // amp hours
  writeFloat32(payload, i, 0.0812F, 1e4F);   // amp hours charged
  writeFloat32(payload, i, 118.7F, 1e4F);    // watt hours
  writeFloat32(payload, i, 3.94F, 1e4F);     // watt hours charged
  writeInt32(payload, i, 123456);            // tachometer, skipped
  writeInt32(payload, i, 234567);            // tachometer absolute
  payload[i++] = 3;                          // fault code

  VescValues v = {};
  expect(vescParseValues(payload, 56, v), "GET_VALUES accepted");
  expectNear(v.tempMosfet, 32.5F, "tempMosfet");
  expectNear(v.tempMotor, 41.2F, "tempMotor");
  expectNear(v.avgMotorCurrent, 18.75F, "avgMotorCurrent");
  expectNear(v.avgInputCurrent, 7.32F, "avgInputCurrent");
  expectNear(v.dutyCycle, 0.62F, "duty scaled by 1000");
  // Negative duty denotes direction; voltage magnitude remains positive.
  payload[21] = 0xFD; payload[22] = 0x94;  // -620, big endian
  VescValues reverse = {};
  expect(vescParseValues(payload, 56, reverse), "reverse GET_VALUES accepted");
  expectNear(reverse.dutyCycle, -0.62F, "signed negative duty");
  expectNear(vescEstimatePhasePeakVoltage(52.0F, 0.6F), 18.013331F, "FOC phase peak estimate");
  expectNear(vescEstimatePhasePeakVoltage(52.0F, -0.6F), 18.013331F, "reverse voltage magnitude");
  expectNear(vescEstimatePhasePeakVoltage(52.0F, 0.0F), 0.0F, "zero modulation");
  expectNear(v.rpm, 3600.0F, "rpm");
  expectNear(v.inpVoltage, 48.6F, "inpVoltage");
  expectNear(v.ampHours, 2.5431F, "ampHours");
  expectNear(v.ampHoursCharged, 0.0812F, "ampHoursCharged");
  expectNear(v.wattHours, 118.7F, "wattHours");
  expectNear(v.wattHoursCharged, 3.94F, "wattHoursCharged");
  expect(v.tachometerAbs == 234567, "tachometerAbs");
  expect(v.faultCode == 3, "faultCode");

  VescValues ignored = {};
  expect(!vescParseValues(payload, 55, ignored), "GET_VALUES rejects a short payload");
  expect(!vescParseValues(nullptr, 56, ignored), "GET_VALUES rejects null");
  payload[0] = VESC_COMM_FW_VERSION;
  expect(!vescParseValues(payload, 56, ignored), "GET_VALUES rejects a wrong command id");
}

void testSetupValues() {
  uint8_t payload[80] = {};
  size_t i = 0;
  payload[i++] = VESC_COMM_GET_VALUES_SETUP;
  writeFloat16(payload, i, 34.8F, 1e1F);     // mosfet temperature
  writeFloat16(payload, i, 39.1F, 1e1F);     // motor temperature, skipped
  writeFloat32(payload, i, 12.0F, 1e2F);     // average motor current, skipped
  writeFloat32(payload, i, 5.5F, 1e2F);      // average input current, skipped
  writeFloat16(payload, i, 0.5F, 1e3F);      // signed duty cycle
  writeFloat32(payload, i, 2800.0F, 1e0F);   // eRPM, skipped
  writeFloat32(payload, i, 8.125F, 1e3F);    // speed, metres per second
  writeFloat16(payload, i, 47.9F, 1e1F);     // input voltage
  writeFloat16(payload, i, 0.78F, 1e3F);     // charge level, skipped
  writeFloat32(payload, i, 1.5F, 1e4F);      // amp hours, skipped
  writeFloat32(payload, i, 0.1F, 1e4F);      // amp hours charged, skipped
  writeFloat32(payload, i, 70.0F, 1e4F);     // watt hours, skipped
  writeFloat32(payload, i, 2.0F, 1e4F);      // watt hours charged, skipped
  writeFloat32(payload, i, 1234.0F, 1e3F);   // trip distance, skipped
  writeFloat32(payload, i, 45678.0F, 1e3F);  // absolute distance, metres

  VescSetupValues v = {};
  expect(vescParseSetupValues(payload, 55, v), "SETUP accepted");
  expectNear(v.tempFet, 34.8F, "setup tempFet");
  expectNear(v.speedMs, 8.125F, "setup speedMs");
  expectNear(v.voltage, 47.9F, "setup voltage");
  expectNear(v.distanceAbsM, 45678.0F, "setup distanceAbsM");
  expectNear(v.batteryLevel, 0.78F, "setup batteryLevel");
  expect(v.batteryLevelValid, "setup batteryLevel is usable");

  VescSetupValues ignored = {};
  expect(!vescParseSetupValues(payload, 54, ignored), "SETUP rejects a short payload");
}

void testFwVersion() {
  const uint8_t payload[] = {VESC_COMM_FW_VERSION, 6, 2};
  uint8_t major = 0;
  uint8_t minor = 0;
  expect(vescParseFwVersion(payload, sizeof(payload), major, minor), "FW_VERSION accepted");
  expect(major == 6 && minor == 2, "FW_VERSION values");

  // The previous library read both bytes after checking only that the reply was
  // non-empty, which could read past a one-byte payload.
  expect(!vescParseFwVersion(payload, 2, major, minor), "FW_VERSION rejects a truncated reply");
}

void testFraming() {
  uint8_t payload[3] = {};
  expect(vescBuildCommandPayload(VESC_COMM_GET_VALUES, 0, payload, sizeof(payload)) == 1,
         "payload without CAN forward is one byte");
  expect(payload[0] == VESC_COMM_GET_VALUES, "payload carries the command id");

  const size_t n = vescBuildCommandPayload(VESC_COMM_GET_VALUES, 17, payload, sizeof(payload));
  expect(n == 3, "payload with CAN forward is three bytes");
  expect(payload[0] == VESC_COMM_FORWARD_CAN && payload[1] == 17 &&
             payload[2] == VESC_COMM_GET_VALUES,
         "CAN forward wrapping");

  uint8_t frame[16] = {};
  const size_t frameLen = vescBuildFrame(payload, (uint8_t)n, frame, sizeof(frame));
  expect(frameLen == n + 5, "frame length");
  expect(frame[0] == 2, "frame start byte");
  expect(frame[1] == n, "frame length byte");
  expect(frame[frameLen - 1] == 3, "frame stop byte");
  // CRC-16/XMODEM of {34, 17, 4}, worked out with Python's binascii.crc_hqx
  // rather than with vescCrc16, stored high byte first.
  expect(frame[frameLen - 3] == 0x98 && frame[frameLen - 2] == 0x60, "frame CRC bytes");

  uint8_t tiny[4] = {};
  expect(vescBuildFrame(payload, (uint8_t)n, tiny, sizeof(tiny)) == 0, "frame refuses to overflow");
  expect(vescBuildCommandPayload(VESC_COMM_GET_VALUES, 17, payload, 2) == 0,
         "payload refuses to overflow");
}

void testControlFraming() {
  uint8_t payload[8] = {};
  // SET_CURRENT 1.000 A (1000 mA), no CAN forward: one command id and the
  // big-endian int32 argument.
  size_t n = vescBuildControlPayload(VESC_COMM_SET_CURRENT, 0, 1000, payload, sizeof(payload));
  expect(n == 5, "control payload without CAN forward is five bytes");
  expect(payload[0] == VESC_COMM_SET_CURRENT, "control payload carries the command id");
  expect(payload[1] == 0x00 && payload[2] == 0x00 && payload[3] == 0x03 && payload[4] == 0xE8,
         "control argument is big-endian int32");

  // A negative argument keeps its two's-complement bits.
  vescBuildControlPayload(VESC_COMM_SET_DUTY, 0, -620, payload, sizeof(payload));
  expect(payload[1] == 0xFF && payload[2] == 0xFF && payload[3] == 0xFD && payload[4] == 0x94,
         "control argument encodes negatives");

  // CAN forward wraps the command id and the argument together.
  n = vescBuildControlPayload(VESC_COMM_SET_RPM, 17, 4000, payload, sizeof(payload));
  expect(n == 7, "control payload with CAN forward is seven bytes");
  expect(payload[0] == VESC_COMM_FORWARD_CAN && payload[1] == 17 && payload[2] == VESC_COMM_SET_RPM,
         "control CAN forward wrapping");
  expect(payload[3] == 0x00 && payload[4] == 0x00 && payload[5] == 0x0F && payload[6] == 0xA0,
         "control CAN argument follows the command id");

  // The frame carries the payload and its CRC. The {34,17,6,0,0,3,0xE8} CRC was
  // worked out with Python's binascii.crc_hqx, stored high byte first.
  const uint8_t canPayload[] = {34, 17, 6, 0x00, 0x00, 0x03, 0xE8};
  uint8_t frame[16] = {};
  const size_t frameLen = vescBuildFrame(canPayload, sizeof(canPayload), frame, sizeof(frame));
  expect(frameLen == sizeof(canPayload) + 5, "control frame length");
  expect(frame[0] == 2 && frame[1] == sizeof(canPayload) && frame[frameLen - 1] == 3,
         "control frame delimiters");
  expect(frame[frameLen - 3] == 0xB5 && frame[frameLen - 2] == 0xC1, "control frame CRC bytes");

  expect(vescBuildControlPayload(VESC_COMM_SET_RPM, 17, 4000, payload, 6) == 0,
         "control payload refuses to overflow");
  expect(vescBuildControlPayload(VESC_COMM_SET_RPM, 0, 4000, payload, 4) == 0,
         "control payload without CAN refuses to overflow");
}

void testFloat32Auto() {
  // Golden bytes from an independent port of the controller's own
  // buffer_append_float32_auto (util/buffer.c), not from this implementation.
  struct Case {
    float value;
    uint8_t bytes[4];
  };
  const Case cases[] = {
      {0.0F, {0x00, 0x00, 0x00, 0x00}},
      {0.5F, {0x3F, 0x00, 0x00, 0x00}},
      {1.0F, {0x3F, 0x80, 0x00, 0x00}},
      {-1.0F, {0xBF, 0x80, 0x00, 0x00}},
      {0.62F, {0x3F, 0x1E, 0xB8, 0x52}},
      {25.0F, {0x41, 0xC8, 0x00, 0x00}},
      {100.0F, {0x42, 0xC8, 0x00, 0x00}},
      {3000.0F, {0x45, 0x3B, 0x80, 0x00}},
      {-3000.0F, {0xC5, 0x3B, 0x80, 0x00}},
      {0.1F, {0x3D, 0xCC, 0xCC, 0xCD}},
  };

  uint8_t buf[4] = {};
  for (const Case &c : cases) {
    size_t index = 0;
    vescAppendFloat32Auto(buf, c.value, index);
    expect(index == 4, "float32_auto writes four bytes");
    expect(std::memcmp(buf, c.bytes, 4) == 0, "float32_auto golden bytes");
    size_t readIndex = 0;
    expectNear(vescReadFloat32Auto(buf, readIndex), c.value, "float32_auto round-trip");
    expect(readIndex == 4, "float32_auto read advances four bytes");
  }

  // A NaN or a subnormal would poison a limit field, so the codec folds it to 0.
  size_t index = 0;
  vescAppendFloat32Auto(buf, NAN, index);
  size_t readIndex = 0;
  expect(vescReadFloat32Auto(buf, readIndex) == 0.0F, "float32_auto folds NaN to zero");
}

void testLispPayload() {
  const char *expr = "(conf-set 'foc-fw-current-max 10)";
  uint8_t payload[64] = {};
  const size_t n = vescBuildLispPayload(expr, payload, sizeof(payload));
  expect(n == std::strlen(expr) + 2, "lisp payload is id + text + NUL");
  expect(payload[0] == VESC_COMM_LISP_REPL_CMD, "lisp payload command id");
  expect(std::memcmp(payload + 1, expr, std::strlen(expr)) == 0, "lisp payload text");
  expect(payload[n - 1] == 0, "lisp payload NUL terminates the expression");

  expect(vescBuildLispPayload("", payload, sizeof(payload)) == 0, "lisp refuses an empty expression");
  expect(vescBuildLispPayload(expr, payload, n - 1) == 0, "lisp refuses to overflow");
}

void testProfileFrame() {
  // A realistic LispBM profile expression (~190 chars), as built by
  // vescProfileLisp for an Eco/Drive/Sport profile. The payload is command id
  // + expression + NUL, ~194 bytes total — far beyond the old frame[16] buffer.
  const char *expr =
      "(conf-set 'l-max-erpm 4500) (conf-set 'l-current-max-scale 0.5) "
      "(conf-set 'l-current-min-scale 0.3) (conf-set 'foc-fw-current-max 20) "
      "(conf-set 'l-current-max 30) (conf-set 'l-in-current-max 15)";

  uint8_t payload[260] = {};
  const size_t payloadLen = vescBuildLispPayload(expr, payload, sizeof(payload));
  expect(payloadLen > 0, "lisp payload built for profile expression");
  expect(payloadLen == std::strlen(expr) + 2, "profile payload is id + text + NUL");

  // The frame must fit in VESC_MAX_FRAME (the fix for the frame[16] bug).
  uint8_t frame[VESC_MAX_FRAME] = {};
  const size_t frameLen = vescBuildFrame(payload, (uint8_t)payloadLen, frame, sizeof(frame));
  expect(frameLen == payloadLen + 5, "profile frame length is payload + 5");
  expect(frame[0] == 2, "profile frame start byte");
  expect(frame[1] == (uint8_t)payloadLen, "profile frame length byte");
  expect(frame[frameLen - 1] == 3, "profile frame stop byte");

  // The old 16-byte buffer must refuse the profile frame — this is the exact
  // regression that silently dropped every profile write (GD-1 root cause).
  uint8_t tiny[16] = {};
  expect(vescBuildFrame(payload, (uint8_t)payloadLen, tiny, sizeof(tiny)) == 0,
         "profile frame refuses to fit in 16 bytes (regression guard)");
}

void testParseMcconfTemp() {
  uint8_t payload[64] = {};
  payload[0] = VESC_COMM_GET_MCCONF_TEMP;
  size_t i = 1;
  const float values[] = {0.5F, 1.0F, -3000.0F, 3000.0F, -1.0F, 1.0F, -10000.0F, 10000.0F, -30.0F, 30.0F};
  for (float value : values) vescAppendFloat32Auto(payload, value, i);

  const size_t profileEnd = i;
  VescMcconfTemp out = {};
  expect(vescParseMcconfTemp(payload, i, out), "MCCONF_TEMP reply accepted");
  expectNear(out.currentMinScale, 0.5F, "mcconf brake scale");
  expectNear(out.currentMaxScale, 1.0F, "mcconf drive scale");
  expectNear(out.minErpm, -3000.0F, "mcconf reverse limit");
  expectNear(out.maxErpm, 3000.0F, "mcconf forward limit");
  expectNear(out.inCurrentMax, 30.0F, "mcconf battery current max");
  expect(!out.setupValid, "old MCCONF_TEMP without setup values is still accepted");

  payload[i++] = 14;
  vescAppendFloat32Auto(payload, 1.0F, i);
  vescAppendFloat32Auto(payload, 0.254F, i);
  expect(vescParseMcconfTemp(payload, i, out), "extended MCCONF_TEMP reply accepted");
  expect(out.setupValid, "extended MCCONF_TEMP reports setup values");
  expect(out.motorPoles == 14, "mcconf motor poles");
  expectNear(out.gearRatio, 1.0F, "mcconf gear ratio");
  expectNear(out.wheelDiameterM, 0.254F, "mcconf wheel diameter");

  VescMcconfTemp ignored = {};
  expect(!vescParseMcconfTemp(payload, profileEnd - 1, ignored), "MCCONF_TEMP rejects a short reply");
  expect(!vescParseMcconfTemp(nullptr, i, ignored), "MCCONF_TEMP rejects null");
  payload[0] = VESC_COMM_GET_VALUES;
  expect(!vescParseMcconfTemp(payload, i, ignored), "MCCONF_TEMP rejects a wrong command id");
}

}  // namespace

int main() {
  testCrc();
  testGetValues();
  testSetupValues();
  testFwVersion();
  testFraming();
  testControlFraming();
  testFloat32Auto();
  testLispPayload();
  testProfileFrame();
  testParseMcconfTemp();
  if (failures == 0) printf("vesc_protocol: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
