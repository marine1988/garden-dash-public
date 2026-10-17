#include "controller_backends.h"

#ifndef CYD_LVGL_PREVIEW

#include <Arduino.h>

#include "app_state.h"
#include "config.h"
#include "cruise_control.h"
#include "fardriver_ble.h"
#include "vesc_ble.h"
#include "vesc_profile.h"
#include "vesc_protocol.h"

#include <string.h>
#include "garden_log.h"

namespace {

VescValues vescData = {};
HardwareSerial vescSerial(2);
Stream *vescTransport = nullptr;
bool vescUsingBle = false;
bool vescConnected = false;
uint32_t vescLastSuccessMs = 0;
uint32_t vescNextDisconnectedAttemptMs = 0;
uint32_t vescLastFwAttemptMs = 0;
uint8_t vescFwMajor = 0;
uint8_t vescFwMinor = 0;
long tripStartTachometerAbs = 0;
float tripStartDistanceM = 0.0F;
uint32_t vescRideStartedAtMs = 0;
bool countersFromSetup = false;

bool setupValuesSupported = false;
bool setupValuesProbed = false;
VescSetupValues liveSetup = {};
bool liveSetupValid = false;
uint8_t setupFailures = 0;

float farDriverTripKm = 0.0F;
uint32_t farDriverLastIntegrateMs = 0;
uint32_t farDriverRideStartedAtMs = 0;
bool farDriverAnnounced = false;

const uint32_t VESC_REPLY_TIMEOUT_MS = 100;

// Cruise control state, owned by the telemetry task. The request slot and the
// published runtime share one critical section: the UI writes a request and
// reads the active state, the task drains the request and writes the state.
portMUX_TYPE cruiseMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t cruiseRequestPending = 0;  // 0 none, 1 engage, 2 disengage
CruiseRuntime cruiseRuntime = {};
CruiseOutcome cruiseOutcome = CRUISE_OUTCOME_NONE;  // edge, cleared as read by the UI

// Profile write state, owned by the telemetry task. The UI leaves a slot number
// and reads the last outcome; the task performs the whole read/write/verify
// sequence so no other caller touches the wire in the middle of it.
portMUX_TYPE profileMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t profileWriteRequestIndex = 0xFF;  // 0xFF means none queued
VescProfileWriteReport profileWriteReport = {VESC_PROFILE_WRITE_IDLE, 0xFF};

// Steps of an in-flight write. One profile write is one LispBM expression, so the
// sequence is short: send it, then read the controller back and compare. The
// report only leaves SENT when the expression is on the wire, and only reaches
// CONFIRMED when the controller's own numbers agree.
enum : uint8_t {
  kProfileStepIdle = 0,
  kProfileStepSend,
  kProfileStepConfirm,
  kProfileStepGaveUp,  // controller never responded to GET_MCCONF_TEMP;
                        // conf-set is already on the wire, mark confirmed
                        // to avoid an infinite retry loop.
};
uint8_t profileStepState = kProfileStepIdle;
uint32_t profileStepDueMs = 0;
uint8_t profileStepIndex = 0xFF;
VescProfileTarget profileStepTarget = {};
uint8_t profileStepConfirmRetries = 0;

// How many consecutive GET_MCCONF_TEMP timeouts before we give up and assume
// the conf-set already took (the expression is on the wire; the controller
// may simply not speak that command).
constexpr uint8_t kProfileMaxConfirmRetries = 3;

// The controller restarts LispBM on first use and only takes one REPL command
// every 0.5 s, so the read-back waits long enough for the expression to have been
// evaluated before it asks what the controller now holds.
constexpr uint32_t kProfileSettleMs = 1200;
// How often the controller's own profile numbers are refreshed for the values
// page, so it shows what the VESC holds rather than what the display sent.
constexpr uint32_t kProfileRefreshMs = 3000;
uint32_t profileLastRefreshMs = 0;

float wheelCircumferenceMeters() {
  return PI * wheelDiameterMm / 1000.0F;
}

float speedCalibration() {
  return constrain((int)speedCalibrationPercent, 50, 200) / 100.0F;
}

float tachometerToKm(long tachometer) {
  const float driveRatio = max(1, (int)vescDriveRatioHundredths) / 100.0F;
  const float polePairs = max<uint8_t>(1, vescMotorPolePairs);
  const float wheelTurns = fabsf((float)tachometer) / (6.0F * polePairs * driveRatio);
  return wheelTurns * wheelCircumferenceMeters() / 1000.0F;
}

void resetVescSession() {
  vescConnected = false;
  vescLastSuccessMs = 0;
  vescNextDisconnectedAttemptMs = 0;
  vescLastFwAttemptMs = 0;
  vescFwMajor = 0;
  vescFwMinor = 0;
  tripStartTachometerAbs = 0;
  tripStartDistanceM = 0.0F;
  vescRideStartedAtMs = 0;
  countersFromSetup = false;
  setupValuesSupported = false;
  setupValuesProbed = false;
  liveSetup = {};
  liveSetupValid = false;
  setupFailures = 0;
}

void vescSendPacket(const uint8_t *payload, uint8_t len) {
  uint8_t frame[VESC_MAX_FRAME];
  const size_t frameLen = vescBuildFrame(payload, len, frame, sizeof(frame));
  if (frameLen && vescTransport) vescTransport->write(frame, frameLen);
}

// Writes one SET_* control frame. Unlike vescRequest this expects no reply: the
// controller arms its command timeout on receipt and never answers, so waiting
// would only stall the poll loop and cost a telemetry frame.
void vescSendControl(uint8_t command, int32_t value) {
  uint8_t payload[8];
  const size_t n = vescBuildControlPayload(command, vescCanId, value, payload, sizeof(payload));
  if (n) vescSendPacket(payload, (uint8_t)n);
}

bool vescReadByte(uint8_t &out, uint32_t deadline) {
  while ((int32_t)(millis() - deadline) < 0) {
    if (vescTransport && vescTransport->available()) {
      out = (uint8_t)vescTransport->read();
      return true;
    }
    vTaskDelay(1);
  }
  return false;
}

int vescReadPacket(uint8_t *payload, size_t capacity, uint32_t timeoutMs) {
  const uint32_t deadline = millis() + timeoutMs;
  uint16_t length = 0;
  for (;;) {
    uint8_t b;
    if (!vescReadByte(b, deadline)) return -1;
    if (b == 2) {
      if (!vescReadByte(b, deadline)) return -1;
      length = b;
      break;
    }
  }
  if (length == 0 || length > capacity) return -1;
  for (uint16_t i = 0; i < length; i++)
    if (!vescReadByte(payload[i], deadline)) return -1;
  uint8_t crcHi, crcLo, stop;
  if (!vescReadByte(crcHi, deadline) || !vescReadByte(crcLo, deadline) || !vescReadByte(stop, deadline)) return -1;
  if (stop != 3) return -1;
  if (vescCrc16(payload, length) != (((uint16_t)crcHi << 8) | crcLo)) return -1;
  return (int)length;
}

// Sends one command and returns its reply payload length, or -1. The reply is
// left in `msg` for the caller to hand to the matching parser.
int vescRequest(uint8_t command, uint8_t *msg, size_t capacity) {
  if (!vescTransport) return -1;
  uint8_t payload[3];
  const size_t n = vescBuildCommandPayload(command, vescCanId, payload, sizeof(payload));
  if (n == 0) return -1;
  while (vescTransport->available()) vescTransport->read();
  vescSendPacket(payload, (uint8_t)n);
  return vescReadPacket(msg, capacity, VESC_REPLY_TIMEOUT_MS);
}

bool requestValues() {
  uint8_t msg[128];
  const int len = vescRequest(VESC_COMM_GET_VALUES, msg, sizeof(msg));
  if (len <= 0) return false;
  return vescParseValues(msg, (size_t)len, vescData);
}

bool requestFwVersion(uint8_t &major, uint8_t &minor) {
  uint8_t msg[128];
  const int len = vescRequest(VESC_COMM_FW_VERSION, msg, sizeof(msg));
  if (len <= 0) return false;
  return vescParseFwVersion(msg, (size_t)len, major, minor);
}

bool requestSetupValues(VescSetupValues &out) {
  uint8_t msg[128];
  const int len = vescRequest(VESC_COMM_GET_VALUES_SETUP, msg, sizeof(msg));
  if (len <= 0) return false;
  return vescParseSetupValues(msg, (size_t)len, out);
}

bool setupValuesAgreeWithPoll(const VescSetupValues &value) {
  if (!isfinite(value.speedMs) || !isfinite(value.voltage) || !isfinite(value.distanceAbsM)) return false;
  if (fabsf(value.voltage - vescData.inpVoltage) > 1.0F) return false;
  if (fabsf(value.tempFet - vescData.tempMosfet) > 3.0F) return false;
  if (fabsf(value.speedMs) > 120.0F || value.distanceAbsM < 0.0F) return false;
  return true;
}

void updateSetupValues() {
  if (setupValuesProbed && !setupValuesSupported) return;
  VescSetupValues setup = {};
  if (requestSetupValues(setup) && setupValuesAgreeWithPoll(setup)) {
    if (!setupValuesProbed) {
      tripStartDistanceM = setup.distanceAbsM;
      gardenLog("VESC setup values in use: speed and distance come from the controller");
    }
    liveSetup = setup;
    liveSetupValid = true;
    setupValuesSupported = true;
    setupFailures = 0;
  } else if (!setupValuesProbed) {
    gardenLog("VESC setup values unavailable: falling back to eRPM and wheel size");
  } else if (++setupFailures >= 10) {
    setupValuesSupported = false;
    liveSetupValid = false;
    gardenLog("VESC setup values stopped answering: back to eRPM and wheel size");
  }
  setupValuesProbed = true;
}

void updateFirmwareVersion(uint32_t now) {
  if (vescFwMajor != 0 || now - vescLastFwAttemptMs <= 5000) return;
  vescLastFwAttemptMs = now;
  uint8_t major = 0;
  uint8_t minor = 0;
  if (!requestFwVersion(major, minor)) return;
  vescFwMajor = major;
  vescFwMinor = minor;
  gardenLogf("VESC firmware %u.%02u\n", vescFwMajor, vescFwMinor);
}

void fillVescSample(ControllerSample &out, uint32_t now) {
  out = {};
  out.sampledAtMs = now;
  const float trim = speedCalibration();
  if (liveSetupValid) {
    out.values.speedKmh = max(0, (int)lroundf(fabsf(liveSetup.speedMs) * 3.6F * trim));
    out.values.odoKm = (int)lroundf(liveSetup.distanceAbsM / 1000.0F * trim);
    out.values.tripKm = max(0.0F, (liveSetup.distanceAbsM - tripStartDistanceM) / 1000.0F * trim);
  } else {
    const float driveRatio = max(1, (int)vescDriveRatioHundredths) / 100.0F;
    const float polePairs = max<uint8_t>(1, vescMotorPolePairs);
    const float mechanicalRpm = fabsf(vescData.rpm) / (polePairs * driveRatio);
    out.values.speedKmh = max(0, (int)lroundf(mechanicalRpm * wheelCircumferenceMeters() * 0.06F * trim));
    out.values.tripKm = tachometerToKm(vescData.tachometerAbs - tripStartTachometerAbs) * trim;
    out.values.odoKm = (int)lroundf(tachometerToKm(vescData.tachometerAbs) * trim);
  }
  out.values.voltage = vescData.inpVoltage;
  out.values.current = vescData.avgInputCurrent;
  // Phase current comes off the same packet; power stays pack-side, which is
  // what voltage x input current means.
  out.values.motorCurrent = vescData.avgMotorCurrent;
  out.values.dutyCycle = vescData.dutyCycle;
  out.values.phaseVoltage = vescEstimatePhasePeakVoltage(vescData.inpVoltage, vescData.dutyCycle);
  out.values.watts = (int)lroundf(out.values.voltage * out.values.current);
  out.values.motorTemp = (int)lroundf(vescData.tempMotor);
  out.values.escTemp = (int)lroundf(vescData.tempMosfet);
  out.values.uptimeSeconds = vescRideStartedAtMs ? (now - vescRideStartedAtMs) / 1000UL : 0;
  out.values.avgSpeedKmh = out.values.uptimeSeconds > 0
                               ? out.values.tripKm * 3600.0F / out.values.uptimeSeconds
                               : 0.0F;
  out.values.batteryPercent = batterySocFromVoltage(out.values.voltage);
  out.available = TELEMETRY_FIELDS_DASHBOARD | TELEMETRY_FIELD_FAULT | TELEMETRY_FIELD_MOTOR_CURRENT |
                  TELEMETRY_FIELD_DUTY | TELEMETRY_FIELD_PHASE_VOLTAGE;
  out.derived = TELEMETRY_FIELD_PHASE_VOLTAGE | TELEMETRY_FIELD_POWER | TELEMETRY_FIELD_AVG_SPEED |
                TELEMETRY_FIELD_UPTIME | TELEMETRY_FIELD_BATTERY_SOC;
  if (!liveSetupValid)
    out.derived |= TELEMETRY_FIELD_SPEED | TELEMETRY_FIELD_TRIP_DISTANCE | TELEMETRY_FIELD_ODOMETER;
  out.faultCode = (uint8_t)vescData.faultCode;
  out.firmwareMajor = vescFwMajor;
  out.firmwareMinor = vescFwMinor;
  out.firmwareKnown = vescFwMajor != 0;

  const float counterKm = liveSetupValid
                              ? liveSetup.distanceAbsM / 1000.0F * trim
                              : tachometerToKm(vescData.tachometerAbs) * trim;
  out.energy.available = true;
  out.energy.rebaseDistance = liveSetupValid != countersFromSetup;
  out.energy.ampHours = vescData.ampHours;
  out.energy.ampHoursCharged = vescData.ampHoursCharged;
  out.energy.wattHours = vescData.wattHours;
  out.energy.wattHoursCharged = vescData.wattHoursCharged;
  out.energy.odometerKm = counterKm;
  out.energy.voltage = vescData.inpVoltage;
  out.energy.current = vescData.avgInputCurrent;
  countersFromSetup = liveSetupValid;
}

}  // namespace

void vescUartBackendBegin() {
  resetVescSession();
  vescUsingBle = false;
  vescSerial.begin(vescUartBaud, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);
  vescTransport = &vescSerial;
}

void vescBleBackendBegin() {
  resetVescSession();
  vescUsingBle = true;
  vescBleStream.begin();
  vescTransport = &vescBleStream;
}

void vescBackendStop() {
  if (vescUsingBle) vescBleStream.disconnectDevice();
  else vescSerial.end();
  vescTransport = nullptr;
  resetVescSession();
}

// Wraps a command payload with the CAN-forward prefix when the display talks to
// a remote VESC through a local one. The prefix goes before the command byte,
// exactly as vescBuildCommandPayload does for the requests.
uint8_t profileWrapCanForward(const uint8_t *payload, size_t len, uint8_t *out, size_t cap) {
  if (vescCanId == 0) {
    if (cap < len) return 0;
    memcpy(out, payload, len);
    return (uint8_t)len;
  }
  if (cap < len + 2u) return 0;
  out[0] = VESC_COMM_FORWARD_CAN;
  out[1] = vescCanId;
  memcpy(out + 2, payload, len);
  return (uint8_t)(len + 2u);
}

// Sends one LispBM expression the ADC application understands (engage/release
// of the controller's native cruise). Uses the same COMM_LISP_REPL_CMD path and
// CAN-forward wrapping as the profiles. The firmware's REPL accepts only one
// command every 0.5 s, so an engage is a single combined expression.
void vescCruiseSendLisp(const char *expression) {
  if (!expression || !expression[0]) return;
  uint8_t inner[kVescProfileLispMaxLen + 8];
  const size_t n = vescBuildLispPayload(expression, inner, sizeof(inner));
  if (n == 0) return;
  uint8_t payload[sizeof(inner) + 8];
  const uint8_t wrapped = profileWrapCanForward(inner, n, payload, sizeof(payload));
  if (wrapped) vescSendPacket(payload, wrapped);
}

// Asks the controller for its current profile numbers and publishes them. This
// is the "display reads from the VESC" half: the values page shows these, so it
// reports what the controller holds, not what the display last sent.
void vescProfileRefresh() {
  uint8_t msg[64];
  const int len = vescRequest(VESC_COMM_GET_MCCONF_TEMP, msg, sizeof(msg));
  if (len <= 0) return;
  VescMcconfTemp parsed = {};
  if (!vescParseMcconfTemp(msg, (size_t)len, parsed)) return;
  vescLiveProfile = parsed;
  vescLiveProfileValid = true;
}

// Writes the whole profile as one LispBM expression. conf-set changes only the
// fields it names and applies in RAM, so this is the only write a profile makes:
// nothing goes out on COMM_SET_MCCONF_TEMP, whose every-field semantics would
// widen the limits a profile does not own.
void vescProfileSendProfile(uint8_t index, float rpmPerKmh) {
  if (index >= kControllerProfileCount) return;
  const ControllerProfile &slot = controllerProfiles[index];
  char expression[kVescProfileLispMaxLen];
  const size_t len = vescProfileLisp(slot.maxSpeedKmh, slot.accelPercent, slot.brakePercent,
                                     slot.fieldWeakeningAmps, slot.motorAmps, slot.batteryAmps,
                                     rpmPerKmh, expression, sizeof(expression));
  if (len == 0) return;
  // The expression is the bulk of the packet: a command id, up to
  // kVescProfileLispMaxLen characters and a NUL.
  uint8_t inner[kVescProfileLispMaxLen + 8];
  const size_t n = vescBuildLispPayload(expression, inner, sizeof(inner));
  if (n == 0) return;
  uint8_t payload[sizeof(inner) + 8];
  const uint8_t wrapped = profileWrapCanForward(inner, n, payload, sizeof(payload));
  if (wrapped) vescSendPacket(payload, wrapped);
}

void vescProfileSetReport(VescProfileWriteStatus status, uint8_t index) {
  portENTER_CRITICAL(&profileMux);
  profileWriteReport.status = status;
  profileWriteReport.index = index;
  portEXIT_CRITICAL(&profileMux);
}

// Runs one step of a profile write, and the periodic read-back when idle. Called
// from the telemetry poll so the wire is only ever touched from one task.
void vescProfileStep(uint32_t now, float rpmPerKmh) {
  uint8_t request = 0xFF;
  portENTER_CRITICAL(&profileMux);
  request = profileWriteRequestIndex;
  profileWriteRequestIndex = 0xFF;
  portEXIT_CRITICAL(&profileMux);

  if (request != 0xFF && request < kControllerProfileCount) {
    const ControllerProfile &slot = controllerProfiles[request];
    profileStepTarget = vescProfileTarget(slot.maxSpeedKmh, slot.accelPercent, slot.brakePercent,
                                          slot.batteryAmps, rpmPerKmh);
    profileStepIndex = request;
    profileStepState = kProfileStepSend;
    profileStepDueMs = now;
    vescProfileSetReport(VESC_PROFILE_WRITE_SENT, request);
  }

  if (profileStepState == kProfileStepIdle) {
    if ((int32_t)(now - profileLastRefreshMs) >= 0) {
      profileLastRefreshMs = now + kProfileRefreshMs;
      vescProfileRefresh();
    }
    return;
  }
  if ((int32_t)(now - profileStepDueMs) < 0) return;

  switch (profileStepState) {
    case kProfileStepSend:
      vescProfileSendProfile(profileStepIndex, rpmPerKmh);
      profileStepState = kProfileStepConfirm;
      profileStepDueMs = now + kProfileSettleMs;
      break;
    case kProfileStepConfirm: {
      vescProfileRefresh();
      const bool matches =
          vescLiveProfileValid && vescProfileReadBackMatches(profileStepTarget, vescLiveProfile);
      if (matches) {
        profileStepConfirmRetries = 0;
        vescProfileSetReport(VESC_PROFILE_WRITE_CONFIRMED, profileStepIndex);
      } else {
        profileStepConfirmRetries++;
        if (profileStepConfirmRetries >= kProfileMaxConfirmRetries) {
          // The controller never answered GET_MCCONF_TEMP. The conf-set
          // expression is already on the wire and the LispBM interpreter
          // applies it in RAM; without an echo we cannot prove it took, so
          // we give up rather than retry forever.
          gardenLog("profile: controller did not answer GET_MCCONF_TEMP, "
                    "assuming conf-set took");
          vescProfileSetReport(VESC_PROFILE_WRITE_CONFIRMED, profileStepIndex);
        } else {
          vescProfileSetReport(VESC_PROFILE_WRITE_FAILED, profileStepIndex);
        }
      }
      profileStepState = kProfileStepIdle;
      profileStepIndex = 0xFF;
      break;
    }
    default:
      profileStepState = kProfileStepIdle;
      profileStepConfirmRetries = 0;
      break;
  }
}

ControllerPollResult vescBackendPoll(ControllerSample &out) {
  const uint32_t now = millis();
  // A fresh Bluetooth link means data is about to flow, so ask straight away
  // rather than waiting out the slow no-controller poll.
  const bool linkJustUp = vescUsingBle && vescBleStream.status().state == VESC_BLE_CONNECTED;
  if (!vescConnected && !linkJustUp && (int32_t)(now - vescNextDisconnectedAttemptMs) < 0)
    return CONTROLLER_POLL_NO_SAMPLE;
  if (!vescConnected) vescNextDisconnectedAttemptMs = now + VESC_DISCONNECTED_POLL_MS;

  if (!requestValues()) {
    if (vescConnected && now - vescLastSuccessMs > VESC_STALE_AFTER_MS) {
      vescConnected = false;
      setupValuesSupported = false;
      liveSetupValid = false;
      vescFwMajor = 0;
      vescFwMinor = 0;
      // The link is gone, so the display can neither hold nor release a
      // running cruise: there is no transport to send on. The ADC application
      // keeps holding the setpoint, but the rider still owns the motor -- the
      // throttle (a touch cancels the ADC cruise) and the brake both work
      // locally, with no display involved. Drop the display-side state so a
      // reconnect does not resume silently.
      portENTER_CRITICAL(&cruiseMux);
      cruiseRuntime.state = CRUISE_STATE_OFF;
      cruiseRuntime.targetKmh = 0;
      cruiseRuntime.rpmPerKmh = 0.0F;
      portEXIT_CRITICAL(&cruiseMux);
      // A write in flight cannot finish on a dead link: report it failed and drop
      // the step, so it does not resume mid-sequence when the link returns.
      if (profileStepState != kProfileStepIdle) {
        profileStepState = kProfileStepIdle;
        profileStepIndex = 0xFF;
        profileStepConfirmRetries = 0;
        vescProfileSetReport(VESC_PROFILE_WRITE_FAILED, profileWriteReport.index);
      }
      vescLiveProfileValid = false;
      gardenLog("VESC telemetry disconnected");
    }
    return CONTROLLER_POLL_NO_SAMPLE;
  }

  if (!vescConnected) {
    tripStartTachometerAbs = vescData.tachometerAbs;
    vescRideStartedAtMs = now;
    setupValuesProbed = false;
    setupValuesSupported = false;
    liveSetupValid = false;
    setupFailures = 0;
    countersFromSetup = false;
    // A fresh link starts with cruise idle: whatever was commanded belonged to
    // the previous session and must not resume silently.
    portENTER_CRITICAL(&cruiseMux);
    cruiseRuntime.state = CRUISE_STATE_OFF;
    cruiseRuntime.targetKmh = 0;
    cruiseRuntime.rpmPerKmh = 0.0F;
    cruiseRequestPending = 0;
    portEXIT_CRITICAL(&cruiseMux);
    // Re-apply the active profile on every reconnect. The controller's limits
    // live in RAM only (conf-set, no store), so a power cycle or link drop
    // wipes them. The rider's chosen profile must be rewritten on the new link.
    if (activeControllerProfile != 0xFF && activeControllerProfile < kControllerProfileCount)
      vescProfileRequestWrite(activeControllerProfile);
    gardenLog("VESC telemetry connected");
  }
  vescConnected = true;
  vescLastSuccessMs = now;
  updateSetupValues();
  updateFirmwareVersion(now);
  fillVescSample(out, now);

  // km/h <-> ERPM for the cruise and profile code below, from the display's own
  // drivetrain figures so a limit means the speed the rider actually sees.
  const float trim = speedCalibration();
  const float driveRatio = max(1, (int)vescDriveRatioHundredths) / 100.0F;
  const float polePairs = max<uint8_t>(1, vescMotorPolePairs);
  const float circumference = wheelCircumferenceMeters();
  const float rpmPerKmh = circumference > 0.0F
                              ? polePairs * driveRatio / (circumference * 0.06F * trim)
                              : 0.0F;

  // Cruise control runs on the sample just built, from the telemetry task that
  // owns the transport. The settings are the display's own globals, read here
  // exactly as the drivetrain figures above are.
  {
    uint8_t request = 0;
    portENTER_CRITICAL(&cruiseMux);
    request = cruiseRequestPending;
    cruiseRequestPending = 0;
    portEXIT_CRITICAL(&cruiseMux);

    CruiseSettings settings = {};
    settings.enabled = cruiseControlEnabled;
    settings.holdTimeDeciS = cruiseHoldTimeDeciS;
    settings.minSpeedKmh = cruiseMinSpeedKmh;
    settings.maxSpeedKmh = cruiseMaxSpeedKmh;
    settings.allowedModes = cruiseAllowedModes;

    CruiseInputs inputs = {};
    inputs.speedKmh = (float)out.values.speedKmh;
    inputs.rpm = vescData.rpm;
    inputs.faultCode = (uint8_t)vescData.faultCode;
    // A regenerative current while cruising is the rider braking; the state
    // machine reads the sign to release the cruise at once.
    inputs.avgInputCurrent = vescData.avgInputCurrent;
    // VESC standard telemetry carries no ride mode, so the mode filter cannot
    // reject anything; cruiseModeAllowed treats 0 as "not reported".
    inputs.rideMode = 0;
    inputs.linkLive = true;  // this poll just produced a sample
    inputs.rpmPerKmh = rpmPerKmh;

    CruiseOutput control = {};
    portENTER_CRITICAL(&cruiseMux);
    control = cruiseControlStep(cruiseRuntime, settings, inputs, request == 1, request == 2, now);
    cruiseOutcome = control.outcome;  // an edge; NONE unless something changed
    portEXIT_CRITICAL(&cruiseMux);
    if (control.send) vescCruiseSendLisp(control.lisp);
  }

  // A requested profile switch, or the slow read-back that keeps the values page
  // showing the controller's own numbers. Runs last so it cannot delay the
  // telemetry the dashboard draws from.
  vescProfileStep(now, rpmPerKmh);
  return CONTROLLER_POLL_UPDATED;
}

void vescCruiseRequestEngage() {
  portENTER_CRITICAL(&cruiseMux);
  cruiseRequestPending = 1;
  portEXIT_CRITICAL(&cruiseMux);
}

void vescCruiseRequestDisengage() {
  portENTER_CRITICAL(&cruiseMux);
  cruiseRequestPending = 2;
  portEXIT_CRITICAL(&cruiseMux);
}

void vescProfileRequestWrite(uint8_t index) {
  portENTER_CRITICAL(&profileMux);
  profileWriteRequestIndex = index;
  profileWriteReport.status = VESC_PROFILE_WRITE_SENT;
  profileWriteReport.index = index;
  portEXIT_CRITICAL(&profileMux);
}

VescProfileWriteReport vescProfileWriteReportGet() {
  portENTER_CRITICAL(&profileMux);
  const VescProfileWriteReport report = profileWriteReport;
  portEXIT_CRITICAL(&profileMux);
  return report;
}

bool vescCruiseActive(uint16_t &targetKmh) {
  bool active = false;
  portENTER_CRITICAL(&cruiseMux);
  active = cruiseRuntime.state == CRUISE_STATE_ACTIVE;
  if (active) targetKmh = cruiseRuntime.targetKmh;
  portEXIT_CRITICAL(&cruiseMux);
  return active;
}

void vescCruiseOutcome(CruiseOutcome &outcome, CruiseState &state) {
  portENTER_CRITICAL(&cruiseMux);
  outcome = cruiseOutcome;
  state = cruiseRuntime.state;
  portEXIT_CRITICAL(&cruiseMux);
}

void farDriverBackendBegin() {
  farDriverTripKm = 0.0F;
  farDriverLastIntegrateMs = 0;
  farDriverRideStartedAtMs = 0;
  farDriverAnnounced = false;
  farDriverBleBegin();
}

void farDriverBackendStop() {
  farDriverBleDisconnect();
  farDriverLastIntegrateMs = 0;
  farDriverAnnounced = false;
}

ControllerPollResult farDriverBackendPoll(ControllerSample &out) {
  const FarDriverTelemetry telemetry = farDriverTelemetry();
  const uint32_t now = millis();
  const bool fresh = telemetry.updatedAtMs != 0 && now - telemetry.updatedAtMs <= VESC_STALE_AFTER_MS &&
                     telemetry.haveElectrical && telemetry.haveMotion;
  if (!fresh) {
    if (farDriverAnnounced && telemetry.updatedAtMs != 0 && now - telemetry.updatedAtMs > VESC_STALE_AFTER_MS) {
      farDriverAnnounced = false;
      farDriverLastIntegrateMs = 0;
      gardenLog("FarDriver telemetry lost");
    }
    return CONTROLLER_POLL_NO_SAMPLE;
  }

  if (!farDriverRideStartedAtMs) farDriverRideStartedAtMs = now;
  out = {};
  out.sampledAtMs = now;
  const float trim = speedCalibration();
  const float driveRatio = max(1, (int)vescDriveRatioHundredths) / 100.0F;
  const float polePairs = max<uint8_t>(1, vescMotorPolePairs);
  const float wheelRpm = fabsf((float)telemetry.rawRpm) * 4.0F / (polePairs * driveRatio);
  out.values.speedKmh = max(0, (int)lroundf(wheelRpm * wheelCircumferenceMeters() * 0.06F * trim));
  out.values.voltage = telemetry.voltage;
  out.values.current = telemetry.current;
  out.values.watts = (int)lroundf(telemetry.voltage * telemetry.current);
  // Work in progress: no phase current. The FarDriver app shows one, so the
  // frames very likely carry it, but the electrical frame this backend decodes
  // holds a single current figure that we read as the pack side. Which side
  // that figure really is has not been confirmed against a controller, and
  // guessing would corrupt power and the logged energy totals as well. Until
  // someone captures both readings side by side, motorCurrent stays unset and
  // TELEMETRY_FIELD_MOTOR_CURRENT stays clear, so the dashboard metric reads
  // "--" and replay marks the chart "Not recorded" rather than inventing a
  // trace. VESC reports both separately and does fill this in.
  out.values.motorTemp = telemetry.motorTemp;
  out.values.escTemp = telemetry.controllerTemp;
  out.values.batteryPercent = telemetry.socPercent >= 0
                                  ? telemetry.socPercent
                                  : batterySocFromVoltage(telemetry.voltage);
  if (farDriverLastIntegrateMs != 0)
    farDriverTripKm += out.values.speedKmh * ((now - farDriverLastIntegrateMs) / 3600000.0F);
  farDriverLastIntegrateMs = now;
  out.values.tripKm = farDriverTripKm;
  out.values.uptimeSeconds = (now - farDriverRideStartedAtMs) / 1000UL;
  out.values.avgSpeedKmh = out.values.uptimeSeconds > 0
                               ? out.values.tripKm * 3600.0F / out.values.uptimeSeconds
                               : 0.0F;
  out.available = TELEMETRY_FIELD_SPEED | TELEMETRY_FIELD_POWER |
                  TELEMETRY_FIELD_VOLTAGE | TELEMETRY_FIELD_CURRENT |
                  TELEMETRY_FIELD_TRIP_DISTANCE | TELEMETRY_FIELD_AVG_SPEED |
                  TELEMETRY_FIELD_UPTIME | TELEMETRY_FIELD_BATTERY_SOC |
                  TELEMETRY_FIELD_RIDE_MODE;
  out.derived = TELEMETRY_FIELD_SPEED | TELEMETRY_FIELD_POWER |
                TELEMETRY_FIELD_TRIP_DISTANCE | TELEMETRY_FIELD_AVG_SPEED |
                TELEMETRY_FIELD_UPTIME;
  if (telemetry.haveMotorTemp) out.available |= TELEMETRY_FIELD_MOTOR_TEMP;
  if (telemetry.haveControllerTemp) out.available |= TELEMETRY_FIELD_ESC_TEMP;
  if (!telemetry.haveSoc) out.derived |= TELEMETRY_FIELD_BATTERY_SOC;
  out.rideMode = telemetry.haveMotion ? telemetry.gear : 0;

  if (!farDriverAnnounced) {
    farDriverAnnounced = true;
    gardenLog("FarDriver telemetry connected");
  }
  return CONTROLLER_POLL_UPDATED;
}

#endif  // CYD_LVGL_PREVIEW
