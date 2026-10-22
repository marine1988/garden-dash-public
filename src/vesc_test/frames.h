// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Synthetic replies for GARDEN's current VESC decoder, not a claim of
// compatibility with every VESC firmware. Deliberately independent of
// src/lvgl_app/vesc_protocol.cpp: a sender that shared the receiver's encoder
// would agree with it even if both were wrong.
namespace fake_vesc {

enum : uint8_t { kCommandFwVersion = 0, kCommandGetValues = 4, kCommandForwardCan = 34, kCommandGetValuesSetup = 47,
                  kCommandGetMcconfTemp = 91, kCommandGetMcconf = 91, kCommandLispReplCmd = 138 };

// Instantaneous readings. Everything cumulative (charge, energy, distance,
// tachometer) lives in Totals, because a real controller counts those up and
// never replays them.
struct Sample {
  int16_t tempFetDeci, tempMotorDeci, voltageDeci;
  int32_t motorCurrentCenti, inputCurrentCenti, erpm, speedMmPerS;
  uint8_t fault;
};

struct Totals {
  int32_t ampHours1e4, ampHoursCharged1e4, wattHours1e4, wattHoursCharged1e4;
  int32_t tachometerAbs, tripMm, odometerMm;
};

// The controller's read-back profile, in the same field order as VescMcconfTemp.
// The fake sender keeps a shadow copy and updates it from the LispBM conf-set
// expressions the display sends, so COMM_GET_MCCONF_TEMP replies with the
// limits the display actually wrote.
struct McconfTemp {
  float currentMinScale, currentMaxScale, minErpm, maxErpm, minDuty, maxDuty, wattMin, wattMax, inCurrentMin, inCurrentMax;
  // Current VESC firmware appends the setup values used for road speed.
  uint8_t motorPoles;
  float gearRatio;
  float wheelDiameterM;
};
using Mcconf = McconfTemp;

// VESC's buffer_append_float32_auto, kept here so the sender stays independent
// of the firmware's protocol module.
inline void appendFloat32Auto(uint8_t *out, float value, size_t &index) {
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

// One request after the optional CAN wrapper is removed. COMM_FORWARD_CAN is
// `forward id | CAN id | inner command...`, so the inner payload starts at
// payload + 2 -- parsing the expression from payload + 1 would read the CAN id
// as the command and hand the REPL the wrapper instead of the expression.
struct RequestView {
  const uint8_t *payload;
  size_t length;
};

inline RequestView unwrapCanForward(const uint8_t *payload, size_t length) {
  if (payload && length >= 3 && payload[0] == kCommandForwardCan) {
    return {payload + 2, length - 2};
  }
  return {payload, length};
}

// A tiny LispBM evaluator for the forms the dashboard sends: a `{...}` block of
// (conf-set 'key value) forms for a ride profile, and (progn ...) or a single
// (app-adc-...) form for cruise. It is deliberately not a general interpreter:
// anything it does not recognise is an error, and every change is staged so a
// bad form leaves the controller exactly as it was.
inline bool isLispSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

inline void skipLispSpace(const char *&p) {
  while (isLispSpace(*p)) p++;
}

inline bool isLispSymbolChar(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
         c == '-' || c == '_' || c == '.' || c == '/' || c == '?' || c == '!' ||
         c == '+' || c == '*' || c == '<' || c == '>' || c == '=';
}

inline bool readLispSymbol(const char *&p, char *out, size_t capacity) {
  skipLispSpace(p);
  if (*p == '\'') p++;  // conf-set keys are quoted in the dashboard's forms
  const char *begin = p;
  while (isLispSymbolChar(*p)) p++;
  const size_t length = (size_t)(p - begin);
  if (length == 0 || length >= capacity) return false;
  for (size_t i = 0; i < length; i++) out[i] = begin[i];
  out[length] = '\0';
  return true;
}

inline bool readLispNumber(const char *&p, double &out) {
  skipLispSpace(p);
  char *end = NULL;
  const double value = strtod(p, &end);
  if (end == p || !isfinite(value)) return false;
  p = end;
  out = value;
  return true;
}

inline bool readLispInteger(const char *&p, long &out) {
  double value = 0.0;
  if (!readLispNumber(p, value)) return false;
  if (value < -2147483648.0 || value > 2147483647.0) return false;
  const long result = (long)value;
  if ((double)result != value) return false;  // exchange arguments are integers
  out = result;
  return true;
}

inline bool closeLispForm(const char *&p) {
  skipLispSpace(p);
  if (*p != ')') return false;
  p++;
  return true;
}

// The controller the fake VESC impersonates. It holds the six profile fields
// the dashboard writes in its own units, the COMM_GET_MCCONF_TEMP read-back,
// and the ADC cruise override. The dashboard's cruise expressions are:
//
//   (progn (app-adc-detach 2 1) (app-adc-override 3 1))   engage
//   (app-adc-override 3 0)                               release
//
// On engage the real ADC application captures the current eRPM and holds it, so
// the fake reports that eRPM in GET_VALUES and GET_VALUES_SETUP until release.
struct ControllerState {
  // The six profile fields, in the display's own names and units.
  float maxErpm = 0.0F;
  float currentMaxScale = 0.0F;
  float currentMinScale = 0.0F;
  float fieldWeakeningAmps = 0.0F;  // foc-fw-current-max
  float motorAmps = 0.0F;           // l-current-max
  float batteryAmps = 0.0F;         // l-in-current-max

  // The fields a real COMM_GET_MCCONF_TEMP reports back. The two absolute
  // currents above have no read-back field, which is why they are kept apart.
  McconfTemp mcconf = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 14, 1.0F, 0.254F};

  bool adcDetached = false;
  bool cruiseHeld = false;
  int32_t cruiseErpm = 0;
  int32_t cruiseSpeedMmPerS = 0;

  void reset() { *this = ControllerState(); }

  // Replaces the instantaneous sample with the held cruise setpoint. A released
  // cruise passes the sequence through untouched.
  Sample applyCruise(const Sample &sample) const {
    if (!cruiseHeld) return sample;
    Sample held = sample;
    held.erpm = cruiseErpm;
    held.speedMmPerS = cruiseSpeedMmPerS;
    return held;
  }

  // Applies one LispBM expression. `currentErpm` and `currentSpeedMmPerS` are
  // what the controller reads at the instant of the call, so engaging the ADC
  // override captures the speed the rider is actually doing. Returns the short
  // reply the fake puts on the wire: "OK" or "ERR".
  const char *applyLisp(const char *expression, int32_t currentErpm = 0,
                        int32_t currentSpeedMmPerS = 0) {
    if (!expression) return "ERR";
    ControllerState staged = *this;
    const char *cursor = expression;
    if (!staged.applyForm(cursor, currentErpm, currentSpeedMmPerS, 0)) return "ERR";
    skipLispSpace(cursor);
    if (*cursor != '\0') return "ERR";
    *this = staged;
    return "OK";
  }

 private:
  bool setProfileField(const char *key, double value) {
    const float number = (float)value;
    if (!isfinite(number)) return false;
    if (strcmp(key, "l-max-erpm") == 0) {
      maxErpm = number;
      mcconf.maxErpm = number;
      return true;
    }
    if (strcmp(key, "l-current-max-scale") == 0) {
      currentMaxScale = number;
      mcconf.currentMaxScale = number;
      return true;
    }
    if (strcmp(key, "l-current-min-scale") == 0) {
      currentMinScale = number;
      mcconf.currentMinScale = number;
      return true;
    }
    if (strcmp(key, "foc-fw-current-max") == 0) {
      fieldWeakeningAmps = number;
      return true;
    }
    if (strcmp(key, "l-current-max") == 0) {
      motorAmps = number;
      return true;
    }
    if (strcmp(key, "l-in-current-max") == 0) {
      batteryAmps = number;
      mcconf.inCurrentMax = number;
      return true;
    }
    return false;
  }

  bool applyForm(const char *&p, int32_t currentErpm, int32_t currentSpeedMmPerS, unsigned depth) {
    if (depth > 8) return false;  // a malformed expression must not exhaust the stack
    skipLispSpace(p);

    if (*p == '{') {
      p++;
      for (;;) {
        skipLispSpace(p);
        if (*p == '}') {
          p++;
          return true;
        }
        if (*p == '\0' || !applyForm(p, currentErpm, currentSpeedMmPerS, depth + 1)) return false;
      }
    }

    if (*p != '(') return false;
    p++;
    char head[24];
    if (!readLispSymbol(p, head, sizeof(head))) return false;

    if (strcmp(head, "progn") == 0) {
      for (;;) {
        skipLispSpace(p);
        if (*p == ')') {
          p++;
          return true;
        }
        if (*p == '\0' || !applyForm(p, currentErpm, currentSpeedMmPerS, depth + 1)) return false;
      }
    }

    if (strcmp(head, "conf-set") == 0) {
      char key[32];
      double value = 0.0;
      if (!readLispSymbol(p, key, sizeof(key)) || !readLispNumber(p, value) || !closeLispForm(p))
        return false;
      return setProfileField(key, value);
    }

    if (strcmp(head, "app-adc-detach") == 0) {
      long mode = 0, enable = 0;
      if (!readLispInteger(p, mode) || !readLispInteger(p, enable) || !closeLispForm(p)) return false;
      if (mode < 1 || mode > 3) return false;
      adcDetached = enable != 0;  // mode 2 detaches the buttons; the throttle stays
      return true;
    }

    if (strcmp(head, "app-adc-override") == 0) {
      long channel = 0, enable = 0;
      if (!readLispInteger(p, channel) || !readLispInteger(p, enable) || !closeLispForm(p)) return false;
      if (channel < 0 || channel > 3) return false;
      if (channel == 3) {
        if (enable != 0) {
          // Capture only on the transition into hold: re-sending engage must not
          // move the setpoint to a speed the fake is already holding.
          if (!cruiseHeld) {
            cruiseHeld = true;
            cruiseErpm = currentErpm;
            cruiseSpeedMmPerS = currentSpeedMmPerS;
          }
        } else {
          cruiseHeld = false;
          cruiseErpm = 0;
          cruiseSpeedMmPerS = 0;
        }
      }
      return true;
    }

    return false;  // unknown command
  }
};

constexpr Sample kStationary = {300, 350, 5200, 0, 0, 0, 0, 0};
constexpr uint32_t kRideDurationMs = 60000;

inline Sample rideSample(uint32_t elapsedMs) {
  struct Key { uint32_t ms; Sample value; };
  static const Key keys[] = {
      {0, kStationary}, {5000, kStationary},
      {15000, {340, 420, 5000, 2500, 1800, 4200, 6900, 0}},
      {25000, {420, 550, 4900, 6000, 4400, 8400, 13900, 0}},
      {35000, {450, 600, 5100, 1800, 1300, 6300, 10400, 0}},
      {40000, {420, 570, 5200, 0, 0, 5600, 9200, 0}},
      {50000, {370, 480, 5300, -1000, -700, 1400, 2300, 0}},
      {55000, kStationary}, {kRideDurationMs, kStationary},
  };
  const uint32_t t = elapsedMs % kRideDurationMs;
  unsigned i = 0;
  while (i + 1 < sizeof(keys) / sizeof(keys[0]) && t >= keys[i + 1].ms) i++;
  const Key &a = keys[i], &b = keys[i + 1];
  auto mix = [&](int32_t start, int32_t end) -> int32_t {
    return start + int32_t((int64_t(end - start) * int64_t(t - a.ms)) / int32_t(b.ms - a.ms));
  };
  Sample out = a.value;
  out.tempFetDeci = int16_t(mix(a.value.tempFetDeci, b.value.tempFetDeci));
  out.tempMotorDeci = int16_t(mix(a.value.tempMotorDeci, b.value.tempMotorDeci));
  out.voltageDeci = int16_t(mix(a.value.voltageDeci, b.value.voltageDeci));
  out.motorCurrentCenti = mix(a.value.motorCurrentCenti, b.value.motorCurrentCenti);
  out.inputCurrentCenti = mix(a.value.inputCurrentCenti, b.value.inputCurrentCenti);
  out.erpm = mix(a.value.erpm, b.value.erpm);
  out.speedMmPerS = mix(a.value.speedMmPerS, b.value.speedMmPerS);
  return out;
}

// CRC-16/XMODEM over the payload only: poly 0x1021, zero seed, no reflection.
inline uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc ^= uint16_t(uint16_t(data[i]) << 8);
    for (unsigned bit = 0; bit < 8; bit++) crc = (crc & 0x8000u) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

// VESC integers are big-endian on the wire.
inline void put16(uint8_t *out, size_t &i, int16_t value) {
  out[i++] = uint8_t(uint16_t(value) >> 8);
  out[i++] = uint8_t(uint16_t(value) & 0xFF);
}
inline void put32(uint8_t *out, size_t &i, int32_t value) {
  const uint32_t bits = uint32_t(value);
  out[i++] = uint8_t(bits >> 24);
  out[i++] = uint8_t(bits >> 16);
  out[i++] = uint8_t(bits >> 8);
  out[i++] = uint8_t(bits);
}

// Wraps a payload as 0x02 | length | payload | crc_hi | crc_lo | 0x03.
inline size_t frame(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length == 0 || length > 255 || capacity < length + 5) return 0;
  size_t i = 0;
  out[i++] = 2;
  out[i++] = uint8_t(length);
  for (size_t n = 0; n < length; n++) out[i++] = payload[n];
  const uint16_t crc = crc16(payload, length);
  out[i++] = uint8_t(crc >> 8);
  out[i++] = uint8_t(crc & 0xFF);
  out[i++] = 3;
  return i;
}

inline size_t makeValuesFrame(uint8_t *out, size_t capacity, const Sample &sample, const Totals &totals) {
  uint8_t payload[59];
  size_t i = 0;
  payload[i++] = kCommandGetValues;
  put16(payload, i, sample.tempFetDeci);
  put16(payload, i, sample.tempMotorDeci);
  put32(payload, i, sample.motorCurrentCenti);
  put32(payload, i, sample.inputCurrentCenti);
  put32(payload, i, 0);  // avg_id
  put32(payload, i, 0);  // avg_iq
  put16(payload, i, int16_t(sample.erpm / 100));  // duty x1000, unread but plausible
  put32(payload, i, sample.erpm);
  put16(payload, i, sample.voltageDeci);
  put32(payload, i, totals.ampHours1e4);
  put32(payload, i, totals.ampHoursCharged1e4);
  put32(payload, i, totals.wattHours1e4);
  put32(payload, i, totals.wattHoursCharged1e4);
  put32(payload, i, totals.tachometerAbs);  // signed count; the receiver skips it
  put32(payload, i, totals.tachometerAbs);
  payload[i++] = sample.fault;
  put32(payload, i, 0);  // pid position
  payload[i++] = 0;      // controller id
  return frame(payload, i, out, capacity);
}

inline size_t makeSetupFrame(uint8_t *out, size_t capacity, const Sample &sample, const Totals &totals) {
  uint8_t payload[58];
  size_t i = 0;
  payload[i++] = kCommandGetValuesSetup;
  put16(payload, i, sample.tempFetDeci);
  put16(payload, i, sample.tempMotorDeci);
  put32(payload, i, sample.motorCurrentCenti);
  put32(payload, i, sample.inputCurrentCenti);
  put16(payload, i, int16_t(sample.erpm / 100));
  put32(payload, i, sample.erpm);
  put32(payload, i, sample.speedMmPerS);  // x1000 gives m/s
  put16(payload, i, sample.voltageDeci);
  put16(payload, i, 750);  // battery level: 75% from the fake's battery settings
  put32(payload, i, totals.ampHours1e4);
  put32(payload, i, totals.ampHoursCharged1e4);
  put32(payload, i, totals.wattHours1e4);
  put32(payload, i, totals.wattHoursCharged1e4);
  put32(payload, i, totals.tripMm);
  put32(payload, i, totals.odometerMm);
  put32(payload, i, 0);  // pid position
  payload[i++] = sample.fault;
  payload[i++] = 0;  // controller id
  payload[i++] = 1;  // VESCs on the CAN bus
  return frame(payload, i, out, capacity);
}

inline size_t makeFwVersionFrame(uint8_t *out, size_t capacity, uint8_t major = 6, uint8_t minor = 2) {
  uint8_t payload[16];
  size_t i = 0;
  payload[i++] = kCommandFwVersion;
  payload[i++] = major;
  payload[i++] = minor;
  for (const char *name = "GARDEN-TEST"; *name; name++) payload[i++] = uint8_t(*name);
  payload[i++] = 0;
  return frame(payload, i, out, capacity);
}

// Simulates the LispBM REPL answer. The controller evaluates the expression and
// sends back a short reply: "OK" for a successful conf-set/progn block, or a
// short error string. The receiver uses this to confirm a profile write landed.
inline size_t makeLispReplyFrame(uint8_t *out, size_t capacity, const char *reply) {
  uint8_t payload[32];
  size_t i = 0;
  payload[i++] = kCommandLispReplCmd;
  for (const char *p = reply; *p; p++) payload[i++] = uint8_t(*p);
  payload[i++] = 0;
  return frame(payload, i, out, capacity);
}

inline size_t makeMcconfTempFrame(uint8_t *out, size_t capacity, const McconfTemp &mcconf = {}) {
  uint8_t payload[1 + 10 * 4 + 1 + 4 + 4];
  size_t i = 0;
  payload[i++] = kCommandGetMcconfTemp;
  appendFloat32Auto(payload, mcconf.currentMinScale, i);
  appendFloat32Auto(payload, mcconf.currentMaxScale, i);
  appendFloat32Auto(payload, mcconf.minErpm, i);
  appendFloat32Auto(payload, mcconf.maxErpm, i);
  appendFloat32Auto(payload, mcconf.minDuty, i);
  appendFloat32Auto(payload, mcconf.maxDuty, i);
  appendFloat32Auto(payload, mcconf.wattMin, i);
  appendFloat32Auto(payload, mcconf.wattMax, i);
  appendFloat32Auto(payload, mcconf.inCurrentMin, i);
  appendFloat32Auto(payload, mcconf.inCurrentMax, i);
  payload[i++] = mcconf.motorPoles;
  appendFloat32Auto(payload, mcconf.gearRatio, i);
  appendFloat32Auto(payload, mcconf.wheelDiameterM, i);
  return frame(payload, i, out, capacity);
}

// Convenience overload for callers that do not want to build the struct.
inline size_t makeMcconfTempFrame(uint8_t *out, size_t capacity,
                                  float currentMinScale, float currentMaxScale, float minErpm, float maxErpm,
                                  float minDuty, float maxDuty, float wattMin, float wattMax,
                                  float inCurrentMin, float inCurrentMax) {
  McconfTemp mcconf = {currentMinScale, currentMaxScale, minErpm, maxErpm, minDuty, maxDuty, wattMin, wattMax, inCurrentMin, inCurrentMax};
  return makeMcconfTempFrame(out, capacity, mcconf);
}

// Alias names, so the sender speaks whichever spelling a caller uses.
inline size_t makeMcconfFrame(uint8_t *out, size_t capacity, const McconfTemp &mcconf = {}) {
  return makeMcconfTempFrame(out, capacity, mcconf);
}
inline size_t makeMcconfFrame(uint8_t *out, size_t capacity,
                              float currentMinScale, float currentMaxScale, float minErpm, float maxErpm,
                              float minDuty, float maxDuty, float wattMin, float wattMax,
                              float inCurrentMin, float inCurrentMax) {
  return makeMcconfTempFrame(out, capacity, currentMinScale, currentMaxScale, minErpm, maxErpm,
                             minDuty, maxDuty, wattMin, wattMax, inCurrentMin, inCurrentMax);
}

}  // namespace fake_vesc
