#pragma once

// VESC UART wire protocol: framing, CRC, and the payload parsers this firmware
// needs. Deliberately free of Arduino and Stream dependencies so the same code
// compiles for the device and for the host parity/regression tests.
//
// Frame layout (payloads <= 255 bytes, the only form a VESC sends for the
// commands used here):
//
//   0x02 | length | payload[length] | crc_hi | crc_lo | 0x03
//
// The CRC is CRC-16/XMODEM over the payload only. Payload byte 0 is the
// command id; every parser below indexes from 1 for that reason.

#include <stddef.h>
#include <stdint.h>

enum : uint8_t {
  VESC_COMM_FW_VERSION = 0,
  VESC_COMM_GET_VALUES = 4,
  // Control commands. Each carries one signed 32-bit argument and is not
  // answered: the controller arms a command timeout as it accepts them and
  // releases the motor if they stop arriving, so a caller sends them on a
  // heartbeat rather than once.
  VESC_COMM_SET_DUTY = 5,
  VESC_COMM_SET_CURRENT = 6,
  VESC_COMM_SET_CURRENT_BRAKE = 7,
  VESC_COMM_SET_RPM = 8,
  VESC_COMM_FORWARD_CAN = 34,
  VESC_COMM_GET_VALUES_SETUP = 47,
  // Reads the controller's live profile numbers -- current scales, speed (ERPM),
  // duty and power limits -- to confirm a write and to show the rider what the
  // controller actually holds. Writing goes through LispBM instead, so the
  // display never touches the controller's flash.
  VESC_COMM_GET_MCCONF_TEMP = 91,
  // Runs one LispBM expression. The controller restarts LispBM if it is not
  // running, so this needs no script loaded. A profile is one expression of
  // (conf-set ...) forms: it writes only the fields named, applies in RAM, and
  // is the only route to field weakening, which has no GET/SET_MCCONF_TEMP field.
  // The firmware rate-limits it to one command per
  // 0.5 s, so callers must not send these back to back.
  VESC_COMM_LISP_REPL_CMD = 138,
};

// Longest frame this module builds: CAN-forward prefix, one command id, and the
// four-byte argument a SET_* command carries.
enum : size_t { VESC_MAX_COMMAND_FRAME = 16 };

// Longest frame vescSendPacket must carry: CAN-forward prefix (2) + command id
// (1) + Lisp payload (id + up to kVescProfileLispMaxLen expression + NUL) +
// framing overhead (5). A LispBM profile expression needs ~200 bytes, so the
// old frame[16] buffer silently dropped every profile write.
enum : size_t { VESC_MAX_FRAME = 265 };

// The COMM_GET_VALUES fields this firmware consumes. The packet carries more
// (avg_id, avg_iq, pid position, controller id); they are skipped
// in place rather than stored.
struct VescValues {
  float tempMosfet;
  float tempMotor;
  float avgMotorCurrent;
  float avgInputCurrent;
  float dutyCycle;  // signed modulation fraction, UART int16 / 1000
  float rpm;
  float inpVoltage;
  float ampHours;
  float ampHoursCharged;
  float wattHours;
  float wattHoursCharged;
  int32_t tachometerAbs;
  uint8_t faultCode;
};

// The COMM_GET_VALUES_SETUP fields used when the controller reports its own
// speed and distance, so wheel size and pole pairs are not needed.
struct VescSetupValues {
  float speedMs;
  float distanceAbsM;
  float voltage;
  float tempFet;
};

uint16_t vescCrc16(const uint8_t *buf, size_t len);

// Fundamental phase-to-neutral peak (amplitude) estimate for sinusoidal VESC FOC
// with foc_overmod_factor = 1. Not a measured phase voltage or switching waveform.
float vescEstimatePhasePeakVoltage(float batteryVoltage, float dutyCycle);

// Builds the payload for a command, optionally wrapped in a CAN forward.
// Returns the payload length, or 0 if `cap` is too small.
size_t vescBuildCommandPayload(uint8_t command, uint8_t canId, uint8_t *out, size_t cap);

// Builds the payload for a SET_* control command, which carries one signed
// 32-bit argument (duty x1e5, current in milli-amps, or eRPM), optionally
// wrapped in a CAN forward. Returns the payload length, or 0 if `cap` is too
// small. Deliberately separate from vescBuildCommandPayload: a control command
// takes an argument and gets no reply, so callers send the frame and move on
// instead of waiting for a response the controller never sends.
size_t vescBuildControlPayload(uint8_t command, uint8_t canId, int32_t value, uint8_t *out, size_t cap);

// The scaling the VESC firmware applies to each control argument (its own
// commands.c: duty / 1e5, current / 1e3 A). eRPM is sent unscaled. Both clamp
// their input to the command's range and return 0 for a non-finite value, so a
// caller cannot send a NaN as a motor current.
constexpr float VESC_DUTY_SCALE = 1.0e5F;
constexpr float VESC_CURRENT_SCALE = 1.0e3F;
int32_t vescEncodeDuty(float duty);
int32_t vescEncodeCurrent(float amps);

// Wraps a payload in start/length/CRC/stop. Returns the frame length, or 0 if
// the payload does not fit `cap`.
size_t vescBuildFrame(const uint8_t *payload, uint8_t payloadLen, uint8_t *out, size_t cap);

// Parsers. Each validates the command id and a minimum length before reading,
// and leaves `out` untouched when it returns false.
bool vescParseValues(const uint8_t *payload, size_t len, VescValues &out);
bool vescParseSetupValues(const uint8_t *payload, size_t len, VescSetupValues &out);
bool vescParseFwVersion(const uint8_t *payload, size_t len, uint8_t &major, uint8_t &minor);

// The COMM_GET_MCCONF_TEMP reply fields read back to confirm a profile took and
// to show the rider the controller's own numbers rather than the display's.
struct VescMcconfTemp {
  float currentMinScale;
  float currentMaxScale;
  float minErpm;
  float maxErpm;
  float minDuty;
  float maxDuty;
  float wattMin;
  float wattMax;
  float inCurrentMin;
  float inCurrentMax;
};

// VESC's compact float: a 24-bit significand with an 8-bit exponent folded into
// the top byte, written big-endian. Every field of the temp-config commands uses
// it, so it must round-trip exactly with the controller's own codec.
void vescAppendFloat32Auto(uint8_t *out, float value, size_t &index);
float vescReadFloat32Auto(const uint8_t *buf, size_t &index);

// Builds the COMM_LISP_REPL_CMD payload: command id, the expression text, and
// the trailing NUL the firmware's string read expects. Returns the payload
// length, or 0 for an empty expression or one that does not fit `cap`.
size_t vescBuildLispPayload(const char *expression, uint8_t *out, size_t cap);

// Parses a COMM_GET_MCCONF_TEMP reply. Validates the command id and a minimum
// length before reading, and leaves `out` untouched when it returns false.
bool vescParseMcconfTemp(const uint8_t *payload, size_t len, VescMcconfTemp &out);
