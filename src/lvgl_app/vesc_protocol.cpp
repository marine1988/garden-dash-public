#include "vesc_protocol.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

float vescEstimatePhasePeakVoltage(float batteryVoltage, float dutyCycle) {
  // VESC FOC: |Vdq| = Vbus * |duty| / sqrt(3), the peak of a phase-to-neutral sine,
  // the same kind of amplitude as the motor current the VESC reports.
  return batteryVoltage > 0.0F && isfinite(dutyCycle)
      ? batteryVoltage * fminf(1.0F, fabsf(dutyCycle)) / sqrtf(3.0F) : 0.0F;
}

namespace {

// CRC-16/XMODEM: polynomial 0x1021, zero init, no reflection, no final xor.
// The bitwise form costs eight shifts per byte, which at ~75 bytes and 20 Hz is
// far below the noise floor on a 240 MHz core, and avoids carrying a table.
const uint16_t kCrcPoly = 0x1021;

int16_t readInt16(const uint8_t *buf, size_t &index) {
  const uint16_t raw = (uint16_t)((uint16_t)buf[index] << 8 | (uint16_t)buf[index + 1]);
  index += 2;
  return (int16_t)raw;
}

int32_t readInt32(const uint8_t *buf, size_t &index) {
  const uint32_t raw = (uint32_t)buf[index] << 24 | (uint32_t)buf[index + 1] << 16 |
                       (uint32_t)buf[index + 2] << 8 | (uint32_t)buf[index + 3];
  index += 4;
  return (int32_t)raw;
}

float readFloat16(const uint8_t *buf, float scale, size_t &index) {
  return (float)readInt16(buf, index) / scale;
}

float readFloat32(const uint8_t *buf, float scale, size_t &index) {
  return (float)readInt32(buf, index) / scale;
}

// Payload length needed to reach the last field each parser reads.
const size_t kValuesMinLen = 56;   // through the fault byte, matching the
                                   // length the previous library accepted
const size_t kSetupMinLen = 55;    // through absolute distance
const size_t kFwVersionMinLen = 3; // command id, major, minor

}  // namespace

uint16_t vescCrc16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)((uint16_t)buf[i] << 8);
    for (int bit = 0; bit < 8; bit++)
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ kCrcPoly) : (uint16_t)(crc << 1);
  }
  return crc;
}

size_t vescBuildCommandPayload(uint8_t command, uint8_t canId, uint8_t *out, size_t cap) {
  const size_t needed = canId != 0 ? 3u : 1u;
  if (!out || cap < needed) return 0;
  size_t n = 0;
  if (canId != 0) {
    out[n++] = VESC_COMM_FORWARD_CAN;
    out[n++] = canId;
  }
  out[n++] = command;
  return n;
}

size_t vescBuildControlPayload(uint8_t command, uint8_t canId, int32_t value, uint8_t *out, size_t cap) {
  const size_t needed = canId != 0 ? 7u : 5u;
  if (!out || cap < needed) return 0;
  size_t n = 0;
  if (canId != 0) {
    out[n++] = VESC_COMM_FORWARD_CAN;
    out[n++] = canId;
  }
  out[n++] = command;
  const uint32_t raw = (uint32_t)value;
  out[n++] = (uint8_t)(raw >> 24);
  out[n++] = (uint8_t)(raw >> 16);
  out[n++] = (uint8_t)(raw >> 8);
  out[n++] = (uint8_t)(raw & 0xFF);
  return n;
}

int32_t vescEncodeDuty(float duty) {
  if (!isfinite(duty)) return 0;
  const float clamped = fmaxf(-1.0F, fminf(1.0F, duty));
  return (int32_t)lroundf(clamped * VESC_DUTY_SCALE);
}

int32_t vescEncodeCurrent(float amps) {
  if (!isfinite(amps)) return 0;
  return (int32_t)lroundf(amps * VESC_CURRENT_SCALE);
}

size_t vescBuildFrame(const uint8_t *payload, size_t payloadLen, uint8_t *out, size_t cap) {
  if (!payload || !out || payloadLen == 0 || payloadLen > 0xFFFFu) return 0;
  const bool longFrame = payloadLen > 255u;
  const size_t frameLen = payloadLen + (longFrame ? 6u : 5u);
  if (cap < frameLen) return 0;
  size_t n = 0;
  if (longFrame) {
    out[n++] = 3;
    out[n++] = (uint8_t)(payloadLen >> 8);
    out[n++] = (uint8_t)(payloadLen & 0xFF);
  } else {
    out[n++] = 2;
    out[n++] = (uint8_t)payloadLen;
  }
  for (size_t i = 0; i < payloadLen; i++) out[n++] = payload[i];
  const uint16_t crc = vescCrc16(payload, payloadLen);
  out[n++] = (uint8_t)(crc >> 8);
  out[n++] = (uint8_t)(crc & 0xFF);
  out[n++] = 3;
  return n;
}

bool vescParseValues(const uint8_t *payload, size_t len, VescValues &out) {
  if (!payload || len < kValuesMinLen || payload[0] != VESC_COMM_GET_VALUES) return false;

  size_t i = 1;
  VescValues v = {};
  v.tempMosfet = readFloat16(payload, 1e1F, i);
  v.tempMotor = readFloat16(payload, 1e1F, i);
  v.avgMotorCurrent = readFloat32(payload, 1e2F, i);
  v.avgInputCurrent = readFloat32(payload, 1e2F, i);
  i += 4;  // avg_id
  i += 4;  // avg_iq
  v.dutyCycle = readFloat16(payload, 1e3F, i);
  v.rpm = readFloat32(payload, 1e0F, i);
  v.inpVoltage = readFloat16(payload, 1e1F, i);
  v.ampHours = readFloat32(payload, 1e4F, i);
  v.ampHoursCharged = readFloat32(payload, 1e4F, i);
  v.wattHours = readFloat32(payload, 1e4F, i);
  v.wattHoursCharged = readFloat32(payload, 1e4F, i);
  i += 4;  // tachometer, signed; only the absolute count is used
  v.tachometerAbs = readInt32(payload, i);
  v.faultCode = payload[i];

  out = v;
  return true;
}

bool vescParseSetupValues(const uint8_t *payload, size_t len, VescSetupValues &out) {
  if (!payload || len < kSetupMinLen || payload[0] != VESC_COMM_GET_VALUES_SETUP) return false;

  size_t i = 1;
  VescSetupValues v = {};
  v.tempFet = readFloat16(payload, 1e1F, i);
  i += 2;   // motor temperature
  i += 4;   // average motor current
  i += 4;   // average input current
  i += 2;   // duty cycle
  i += 4;   // eRPM
  v.speedMs = readFloat32(payload, 1e3F, i);
  v.voltage = readFloat16(payload, 1e1F, i);
  v.batteryLevel = readFloat16(payload, 1e3F, i);
  // The VESC divides remaining watt-hours by configured capacity. With no
  // battery configuration that is 0/0 and the float16 encoder writes 0, which
  // is indistinguishable from a truly empty pack; treat 0 as "not available"
  // and fall back to the display's voltage curve.
  v.batteryLevelValid = isfinite(v.batteryLevel) && v.batteryLevel > 0.0F && v.batteryLevel <= 1.0F;
  i += 4;   // amp hours
  i += 4;   // amp hours charged
  i += 4;   // watt hours
  i += 4;   // watt hours charged
  i += 4;   // distance this trip
  v.distanceAbsM = readFloat32(payload, 1e3F, i);

  out = v;
  return true;
}

bool vescParseFwVersion(const uint8_t *payload, size_t len, uint8_t &major, uint8_t &minor) {
  if (!payload || len < kFwVersionMinLen || payload[0] != VESC_COMM_FW_VERSION) return false;
  major = payload[1];
  minor = payload[2];
  return true;
}

void vescAppendFloat32Auto(uint8_t *out, float value, size_t &index) {
  // Byte-for-byte the controller's own buffer_append_float32_auto. Subnormals
  // are folded to zero because the split-significand form cannot represent them.
  if (!isfinite(value) || fabsf(value) < 1.5e-38F) value = 0.0F;

  int exponent = 0;
  const float sig = frexpf(value, &exponent);
  const float sigAbs = fabsf(sig);
  uint32_t sigInt = 0;
  if (sigAbs >= 0.5F) {
    sigInt = (uint32_t)((sigAbs - 0.5F) * 2.0F * 8388608.0F);
    exponent += 126;
  }
  uint32_t raw = ((uint32_t)(exponent & 0xFF) << 23) | (sigInt & 0x7FFFFFu);
  if (sig < 0.0F) raw |= 1u << 31;

  out[index++] = (uint8_t)(raw >> 24);
  out[index++] = (uint8_t)(raw >> 16);
  out[index++] = (uint8_t)(raw >> 8);
  out[index++] = (uint8_t)(raw);
}

float vescReadFloat32Auto(const uint8_t *buf, size_t &index) {
  const uint32_t raw = (uint32_t)buf[index] << 24 | (uint32_t)buf[index + 1] << 16 |
                       (uint32_t)buf[index + 2] << 8 | (uint32_t)buf[index + 3];
  index += 4;

  int exponent = (int)((raw >> 23) & 0xFF);
  const uint32_t sigInt = raw & 0x7FFFFFu;
  const bool negative = (raw & (1u << 31)) != 0;
  float sig = 0.0F;
  if (exponent != 0 || sigInt != 0) {
    sig = (float)sigInt / (8388608.0F * 2.0F) + 0.5F;
    exponent -= 126;
  }
  if (negative) sig = -sig;
  return ldexpf(sig, exponent);
}

size_t vescBuildLispPayload(const char *expression, uint8_t *out, size_t cap) {
  if (!expression || !out) return 0;
  const size_t len = strlen(expression);
  if (len == 0) return 0;
  const size_t needed = 1u + len + 1u;  // command id, text, NUL
  if (cap < needed) return 0;

  size_t n = 0;
  out[n++] = VESC_COMM_LISP_REPL_CMD;
  memcpy(out + n, expression, len);
  n += len;
  out[n++] = 0;
  return n;
}

bool vescParseMcconfConfig(const uint8_t *payload, size_t len, VescConfigValues &out) {
  if (!payload || len < 100 || payload[0] != VESC_COMM_GET_MCCONF) return false;
  const uint32_t signature = ((uint32_t)payload[1] << 24) | ((uint32_t)payload[2] << 16) |
                             ((uint32_t)payload[3] << 8) | (uint32_t)payload[4];
  if (signature != 0x1C17AFEEu) return false;

  // Payload lengths include the command byte. 482 is 6.02; 6.05, 6.06, 7.00
  // and current master use the larger tail. Unknown layouts are rejected.
  size_t typeIndex = 0, cellsIndex = 0, ahIndex = 0;
  if (len == 482) {
    typeIndex = len - 21;
    cellsIndex = len - 20;
    ahIndex = len - 19;
  } else if (len == 478 || len == 484 || len == 489 || len == 493) {
    typeIndex = len - 27;
    cellsIndex = len - 26;
    ahIndex = len - 25;
  } else {
    return false;
  }
  if (ahIndex + 4 > len) return false;
  const uint8_t type = payload[typeIndex];
  const uint8_t cells = payload[cellsIndex];
  size_t index = ahIndex;
  const float ah = vescReadFloat32Auto(payload, index);
  if (type > 2 || cells < 4 || cells > 32 || !isfinite(ah) || ah <= 0.0F || ah > 1000.0F)
    return false;
  out.batteryType = type;
  out.batteryCells = cells;
  out.batteryAh = ah;
  out.motorPoles = 0;
  out.gearRatio = 0.0F;
  out.wheelDiameterM = 0.0F;
  out.valid = true;
  return true;
}

bool vescParseMcconfTemp(const uint8_t *payload, size_t len, VescMcconfTemp &out) {
  // Command id plus the ten float32_auto fields the reply carries.
  const size_t kMcconfTempMinLen = 1u + 10u * 4u;
  if (!payload || len < kMcconfTempMinLen || payload[0] != VESC_COMM_GET_MCCONF_TEMP) return false;

  size_t i = 1;
  VescMcconfTemp v = {};
  v.currentMinScale = vescReadFloat32Auto(payload, i);
  v.currentMaxScale = vescReadFloat32Auto(payload, i);
  v.minErpm = vescReadFloat32Auto(payload, i);
  v.maxErpm = vescReadFloat32Auto(payload, i);
  v.minDuty = vescReadFloat32Auto(payload, i);
  v.maxDuty = vescReadFloat32Auto(payload, i);
  v.wattMin = vescReadFloat32Auto(payload, i);
  v.wattMax = vescReadFloat32Auto(payload, i);
  v.inCurrentMin = vescReadFloat32Auto(payload, i);
  v.inCurrentMax = vescReadFloat32Auto(payload, i);
  if (len >= 50) {
    v.motorPoles = payload[i++];
    v.gearRatio = vescReadFloat32Auto(payload, i);
    v.wheelDiameterM = vescReadFloat32Auto(payload, i);
    v.setupValid = v.motorPoles >= 2 && isfinite(v.gearRatio) && v.gearRatio > 0.01F &&
                   isfinite(v.wheelDiameterM) && v.wheelDiameterM >= 0.05F && v.wheelDiameterM <= 2.0F;
  }

  out = v;
  return true;
}
