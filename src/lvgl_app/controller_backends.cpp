#include "controller_backends.h"

#ifndef CYD_LVGL_PREVIEW

#include <Arduino.h>

#include "app_state.h"
#include "config.h"
#include "cruise_control.h"  // CruiseOutcome/CruiseState, for the existing UI API
#include "cruise_package.h"
#include "fardriver_ble.h"
#include "vesc_ble.h"
#include "vesc_profile.h"
#include "vesc_protocol.h"

#include <string.h>
#include "garden_log.h"

// Used by vescBackendStop before the definition further down.
void vescProfileSetReport(VescProfileWriteStatus status, uint8_t index);

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

// The VESC firmware's LispBM REPL accepts one command every 0.5 s. Profiles and
// cruise share that path, so every expression is spaced here rather than in the
// callers; 550 ms leaves a margin for the controller's own scheduler.
constexpr uint32_t kVescLispGapMs = 550;
// A reconnect re-applies the rider's profile, but only after the link has been
// stable for a moment: the first polls can race LispBM startup.
constexpr uint32_t kProfileReapplyDelayMs = 1500;
// GET_MCCONF_TEMP can take a few hundred milliseconds to answer on a busy
// controller. A timeout is retried before we give up, not treated as a
// mismatched profile.
constexpr uint32_t kProfileConfirmRetryMs = 300;

// Telemetry-task scratch space. This task is the only writer of the VESC
// transport, so these buffers are private to it in practice. They are allocated
// once on the heap instead of living in .bss or on the task stack: the firmware
// already sits at the edge of the static DRAM segment, while the stack cost of
// a profile expression (~1 KiB of locals plus the transport's call chain) was
// what rebooted the display when a persisted profile was re-applied.
struct LispScratch {
  char expression[kVescProfileLispMaxLen];
  uint8_t payload[kVescProfileLispMaxLen + 16];
  uint8_t frame[VESC_MAX_FRAME];
};
LispScratch *lispScratch = nullptr;

bool ensureLispScratch() {
  if (lispScratch) return true;
  lispScratch = (LispScratch *)malloc(sizeof(LispScratch));
  if (!lispScratch) gardenLog("VESC: could not allocate Lisp scratch buffers");
  return lispScratch != nullptr;
}

// Set after an expression goes out so the next one waits for the REPL window.
bool lispSent = false;
uint32_t lastLispSentMs = 0;

// Replies to COMM_LISP_REPL_CMD are asynchronous and are collected while a
// telemetry request waits for its own answer. Each reply is tagged with the
// purpose of the expression that was last sent, so a profile confirmation is
// never decided by a late cruise reply (or the reverse).
enum LispPurpose : uint8_t {
  kLispNone = 0,
  kLispProfile,
  kLispCruise,
};
LispPurpose sentLispPurpose = kLispNone;
bool lastLispReplyOk = false;
char lastLispReply[32] = {};
bool profileReplySeen = false;
bool profileReplyOk = false;

// Set when a cruise expression is on the wire so a REPL error can take the
// display out of the ACTIVE state instead of showing a cruise that never
// engaged. A controller that does not answer at all leaves the state alone: the
// ADC application may still have accepted the command.
bool cruiseLispAwaitingReply = false;
bool cruiseReplySeen = false;
bool cruiseReplyOk = false;

// A cruise expression that could not go out because the REPL was still busy.
// Only the latest rider action matters (release replaces a queued engage), so a
// single slot is enough and makes a quick cancel deterministic.
char pendingCruiseLisp[96] = {};
bool pendingCruiseLispActive = false;

uint32_t vescConnectedAtMs = 0;
bool profileReapplyPending = false;

// Cruise state comes from the vesc_cruise_control LispBM package, not from a
// display-side state machine. The package owns the throttle/brake pins and the
// ADC override; the display only queries and displays it.
portMUX_TYPE cruiseMux = portMUX_INITIALIZER_UNLOCKED;
CruisePackageSnapshot cruisePackage = {false, CRUISE_PACKAGE_UNAVAILABLE, "none", false,
                                       true, 50, 10, 5, 25, false, false, true};
uint16_t cruisePackageTargetKmh = 0;
uint32_t cruisePackageRevision = 0;
uint32_t cruisePackageLastLineMs = 0;
uint32_t cruisePackageLastQueryMs = 0;
bool cruisePackageSettingsSeen = false;
uint8_t cruisePackageQueryAttempts = 0;
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
};
uint8_t profileStepState = kProfileStepIdle;
uint32_t profileStepDueMs = 0;
uint8_t profileStepIndex = 0xFF;
VescProfileTarget profileStepTarget = {};
// The slot is copied when the request is consumed. Using the same snapshot for
// the target and the expression stops an edit made while the write is in flight
// from producing a read-back comparison against different numbers.
ControllerProfile profileStepSlot = {};
uint8_t profileStepConfirmRetries = 0;

// How many consecutive GET_MCCONF_TEMP timeouts before we stop asking. The
// write is then accepted only if the REPL acknowledged the expression; a silent
// controller is reported as failed rather than pretending it confirmed.
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
  lispSent = false;
  pendingCruiseLispActive = false;
  pendingCruiseLisp[0] = '\0';
  cruiseLispAwaitingReply = false;
  vescConnectedAtMs = 0;
  profileReapplyPending = false;
  profileStepState = kProfileStepIdle;
  profileStepIndex = 0xFF;
  profileStepDueMs = 0;
  profileStepConfirmRetries = 0;
  portENTER_CRITICAL(&cruiseMux);
  cruisePackage.available = false;
  cruisePackage.state = CRUISE_PACKAGE_UNAVAILABLE;
  cruisePackageTargetKmh = 0;
  cruisePackageRevision++;
  cruiseOutcome = CRUISE_OUTCOME_NONE;
  portEXIT_CRITICAL(&cruiseMux);
  cruisePackageSettingsSeen = false;
  cruisePackageLastLineMs = 0;
  cruisePackageLastQueryMs = 0;
  cruisePackageQueryAttempts = 0;
  // The values page must not keep showing numbers from the previous link.
  portENTER_CRITICAL(&profileMux);
  vescLiveProfileValid = false;
  portEXIT_CRITICAL(&profileMux);
}

bool vescSendPacket(const uint8_t *payload, size_t len) {
  // The VESC frame length is one byte, so a longer payload is not a frame this
  // protocol can carry. Check before narrowing: a CAN-wrapped profile plus a
  // maximal expression would otherwise truncate the length to a few bytes.
  if (!payload || len == 0 || len > 255 || !vescTransport || !ensureLispScratch()) return false;
  const size_t frameLen = vescBuildFrame(payload, (uint8_t)len, lispScratch->frame,
                                         sizeof(lispScratch->frame));
  if (!frameLen) return false;
  const size_t written = vescTransport->write(lispScratch->frame, frameLen);
  return written == frameLen;
}

// Writes one SET_* control frame. Unlike vescRequest this expects no reply: the
// controller arms its command timeout on receipt and never answers, so waiting
// would only stall the poll loop and cost a telemetry frame.
void vescSendControl(uint8_t command, int32_t value) {
  uint8_t payload[8];
  const size_t n = vescBuildControlPayload(command, vescCanId, value, payload, sizeof(payload));
  if (n) (void)vescSendPacket(payload, n);
}

bool vescLispReady(uint32_t now) {
  return !lispSent || (uint32_t)(now - lastLispSentMs) >= kVescLispGapMs;
}

// True when the text looks like a REPL error. Successful expressions may return
// a value, a number or "OK" depending on the firmware, so only obvious failure
// wording is treated as a rejection.
bool lispReplyLooksLikeError(const char *text) {
  char lower[32];
  size_t i = 0;
  for (; text && text[i] && i + 1 < sizeof(lower); i++) {
    char c = text[i];
    lower[i] = (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
  }
  lower[i] = '\0';
  return strstr(lower, "err") || strstr(lower, "fault") || strstr(lower, "invalid") ||
         strstr(lower, "not running") || strstr(lower, "unknown");
}

void vescRecordLispReply(const uint8_t *payload, size_t len) {
  if (!payload || len < 2) return;
  const char *text = (const char *)(payload + 1);
  size_t copy = 0;
  while (copy < len - 1 && copy < sizeof(lastLispReply) - 1 && text[copy]) copy++;
  memcpy(lastLispReply, text, copy);
  lastLispReply[copy] = '\0';
  lastLispReplyOk = !lispReplyLooksLikeError(lastLispReply);
  if (sentLispPurpose == kLispProfile) {
    profileReplySeen = true;
    profileReplyOk = lastLispReplyOk;
  } else if (sentLispPurpose == kLispCruise) {
    cruiseReplySeen = true;
    cruiseReplyOk = lastLispReplyOk;
  }
  gardenLogf("VESC Lisp reply: %s\n", lastLispReply);
}

CruiseOutcome cruisePackageCancelOutcome(const char *cancel) {
  if (!cancel) return CRUISE_OUTCOME_RELEASED_CANCEL;
  if (strcmp(cancel, "brake") == 0) return CRUISE_OUTCOME_RELEASED_BRAKE;
  if (strcmp(cancel, "speed_low") == 0) return CRUISE_OUTCOME_RELEASED_SPEED;
  if (strcmp(cancel, "overspeed") == 0) return CRUISE_OUTCOME_RELEASED_SPEED;
  if (strcmp(cancel, "script_restart") == 0) return CRUISE_OUTCOME_RELEASED_LINK;
  return CRUISE_OUTCOME_RELEASED_CANCEL;
}

bool sameCruisePackageSettings(const CruisePackageSnapshot &a, const CruisePackageSnapshot &b) {
  return a.enabled == b.enabled && a.holdDeciS == b.holdDeciS &&
         a.deadbandHundredths == b.deadbandHundredths && a.minSpeedKmh == b.minSpeedKmh &&
         a.maxSpeedKmh == b.maxSpeedKmh && a.legalEnabled == b.legalEnabled &&
         a.debugEnabled == b.debugEnabled && a.beepsEnabled == b.beepsEnabled;
}

// COMM_CUSTOM_APP_DATA from the installed cruise package. The package sends its
// state once per second and its settings when asked; both are plain text after
// the command byte. This is the only way the display learns the package's
// state, because the package owns the ADC override.
void vescHandleCustomAppData(const uint8_t *payload, size_t len) {
  if (!payload || len < 2) return;
  char line[128];
  size_t copy = 0;
  while (copy < len - 1 && copy + 1 < sizeof(line) && payload[copy + 1] != '\0') {
    line[copy] = (char)payload[copy + 1];
    copy++;
  }
  line[copy] = '\0';

  CruisePackageSnapshot current = {};
  portENTER_CRITICAL(&cruiseMux);
  current = cruisePackage;
  portEXIT_CRITICAL(&cruiseMux);

  CruisePackageSnapshot next = current;
  if (!cruisePackageParseLine(line, next)) return;
  next.available = true;
  if (strncmp(line, "cruise ", 7) == 0) cruisePackageSettingsSeen = true;

  portENTER_CRITICAL(&cruiseMux);
  const CruisePackageState previousState = cruisePackage.state;
  const bool changed = !cruisePackage.available || previousState != next.state ||
                       strcmp(cruisePackage.lastCancel, next.lastCancel) != 0 ||
                       cruisePackage.legalLocked != next.legalLocked ||
                       !sameCruisePackageSettings(cruisePackage, next);
  cruisePackage = next;
  if (changed) cruisePackageRevision++;
  if (previousState != CRUISE_PACKAGE_ON && next.state == CRUISE_PACKAGE_ON) {
    cruiseOutcome = CRUISE_OUTCOME_ENGAGED;
  } else if (previousState == CRUISE_PACKAGE_ON && next.state != CRUISE_PACKAGE_ON) {
    cruiseOutcome = cruisePackageCancelOutcome(next.lastCancel);
  }
  portEXIT_CRITICAL(&cruiseMux);
  cruisePackageLastLineMs = millis();
  cruisePackageQueryAttempts = 0;
}

// Sends one expression and starts the REPL gap. Returns false when the
// expression could not be built or handed to the transport. The CAN-forward
// prefix is written in front of the Lisp payload directly, so no second copy is
// needed for a forwarded controller.
bool vescSendLispExpression(const char *expression, uint32_t now, LispPurpose purpose) {
  if (!expression || !expression[0] || !vescTransport || !ensureLispScratch()) return false;
  const size_t prefix = vescCanId != 0 ? 2u : 0u;
  const size_t n = vescBuildLispPayload(expression, lispScratch->payload + prefix,
                                        sizeof(lispScratch->payload) - prefix);
  if (n == 0 || n + prefix > 255) {
    if (n != 0) gardenLog("VESC: Lisp expression does not fit one VESC frame");
    return false;
  }
  if (prefix) {
    lispScratch->payload[0] = VESC_COMM_FORWARD_CAN;
    lispScratch->payload[1] = vescCanId;
  }
  if (!vescSendPacket(lispScratch->payload, n + prefix)) return false;
  lispSent = true;
  lastLispSentMs = now;
  sentLispPurpose = purpose;
  return true;
}

void vescQueueCruiseLisp(const char *expression) {
  if (!expression || !expression[0]) return;
  const size_t len = strlen(expression);
  if (len >= sizeof(pendingCruiseLisp)) return;
  memcpy(pendingCruiseLisp, expression, len + 1);
  pendingCruiseLispActive = true;
}

void vescServiceCruiseLisp(uint32_t now) {
  if (!pendingCruiseLispActive) return;
  if (!vescTransport) {
    pendingCruiseLispActive = false;
    return;
  }
  if (!vescLispReady(now)) return;
  cruiseReplySeen = false;
  cruiseReplyOk = false;
  if (vescSendLispExpression(pendingCruiseLisp, now, kLispCruise)) {
    gardenLogf("VESC cruise command: %s\n", pendingCruiseLisp);
    cruiseLispAwaitingReply = true;
    pendingCruiseLispActive = false;
    pendingCruiseLisp[0] = '\0';
  }
}

// Takes the display out of the ACTIVE state when the REPL explicitly rejected
// the cruise expression. Without this the rider would see CRUISE ENGAGED after
// a command the controller never accepted. Silence is not treated as failure:
// some firmware builds evaluate the expression without printing a reply.
void vescCheckCruiseReply() {
  if (!cruiseLispAwaitingReply || !cruiseReplySeen) return;
  cruiseLispAwaitingReply = false;
  if (cruiseReplyOk) return;

  // The package did not understand the query. That means it is not installed,
  // not that a running cruise failed: the package owns the ADC override and
  // nothing in the display can cancel it.
  bool wasActive = false;
  bool wasAvailable = false;
  portENTER_CRITICAL(&cruiseMux);
  wasActive = cruisePackage.state == CRUISE_PACKAGE_ON;
  wasAvailable = cruisePackage.available;
  cruisePackage.available = false;
  cruisePackage.state = CRUISE_PACKAGE_UNAVAILABLE;
  cruisePackageRevision++;
  if (wasActive) cruiseOutcome = CRUISE_OUTCOME_FAILED;
  portEXIT_CRITICAL(&cruiseMux);
  if (wasAvailable) gardenLogf("cruise: package query rejected: %s\n", lastLispReply);
}

// Asks the installed package for its state/settings when it has not spoken for
// a while. The package normally sends state once per second on its own, so this
// is only a fallback and a way to discover a missing package.
void vescCruisePackageStep(uint32_t now) {
  if (!vescConnected) return;
  if (now - vescConnectedAtMs < 1000) return;
  const bool stateStale = cruisePackageLastLineMs == 0 || now - cruisePackageLastLineMs > 5000;
  const bool needSettings = !cruisePackageSettingsSeen;
  if (!stateStale && !needSettings) return;
  if (cruisePackageQueryAttempts >= 5) return;
  if (now - cruisePackageLastQueryMs < 2000) return;
  cruisePackageLastQueryMs = now;
  cruisePackageQueryAttempts++;
  vescQueueCruiseLisp(needSettings ? "(send-settings)" : "(send-state)");
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

// Sends one command and waits for the matching reply. A LispBM reply can be in
// the stream first (it is asynchronous), so it is recorded and skipped instead
// of being mistaken for the telemetry answer or discarded before the request.
// Stale replies to other commands are ignored the same way.
int vescRequest(uint8_t command, uint8_t *msg, size_t capacity) {
  if (!vescTransport) return -1;
  uint8_t payload[3];
  const size_t n = vescBuildCommandPayload(command, vescCanId, payload, sizeof(payload));
  if (n == 0) return -1;
  if (!vescSendPacket(payload, n)) return -1;

  const uint32_t deadline = millis() + VESC_REPLY_TIMEOUT_MS;
  for (;;) {
    const int32_t remaining = (int32_t)(deadline - millis());
    if (remaining <= 0) return -1;
    const int len = vescReadPacket(msg, capacity, (uint32_t)remaining);
    if (len <= 0) return -1;
    if (msg[0] == VESC_COMM_LISP_REPL_CMD) {
      vescRecordLispReply(msg, (size_t)len);
      continue;
    }
    if (msg[0] == VESC_COMM_CUSTOM_APP_DATA) {
      vescHandleCustomAppData(msg, (size_t)len);
      continue;
    }
    if (msg[0] == command) return len;
    // A reply for an older request: keep waiting for ours.
  }
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
  // Reserve the scratch space before Bluetooth and LVGL have fragmented the
  // heap. It is small, but every telemetry request uses the frame buffer.
  (void)ensureLispScratch();
  vescUsingBle = false;
  vescSerial.begin(vescUartBaud, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);
  vescTransport = &vescSerial;
}

void vescBleBackendBegin() {
  resetVescSession();
  (void)ensureLispScratch();
  vescUsingBle = true;
  vescBleStream.begin();
  vescTransport = &vescBleStream;
}

void vescBackendStop() {
  if (vescUsingBle) vescBleStream.disconnectDevice();
  else vescSerial.end();
  vescTransport = nullptr;
  // A write in flight cannot finish once the transport is gone. Report it
  // before the session state is cleared so the next backend does not inherit a
  // half-finished confirmation.
  if (profileStepState != kProfileStepIdle)
    vescProfileSetReport(VESC_PROFILE_WRITE_FAILED, profileStepIndex);
  resetVescSession();
}

// Queues one LispBM expression the ADC application understands (engage/release
// of the controller's native cruise). The service below sends it when the REPL
// window opens; a quick engage-then-cancel replaces the queued expression, so a
// release is never lost behind the rate limit.
void vescCruiseSendLisp(const char *expression) {
  vescQueueCruiseLisp(expression);
}

// Asks the controller for its current profile numbers and publishes them. This
// is the "display reads from the VESC" half: the values page shows these, so it
// reports what the controller holds, not what the display last sent. Returns
// false when the controller did not answer; a valid answer that differs from
// the target is a separate case the caller can distinguish.
bool vescProfileRefresh() {
  uint8_t msg[64];
  const int len = vescRequest(VESC_COMM_GET_MCCONF_TEMP, msg, sizeof(msg));
  if (len <= 0) return false;
  VescMcconfTemp parsed = {};
  if (!vescParseMcconfTemp(msg, (size_t)len, parsed)) return false;
  portENTER_CRITICAL(&profileMux);
  vescLiveProfile = parsed;
  vescLiveProfileValid = true;
  portEXIT_CRITICAL(&profileMux);
  return true;
}

// Writes the whole profile as one LispBM expression. conf-set changes only the
// fields it names and applies in RAM, so this is the only write a profile makes:
// nothing goes out on COMM_SET_MCCONF_TEMP, whose every-field semantics would
// widen the limits a profile does not own. The expression and wire buffers are
// heap-backed, not stack locals: the controller task used to carry about a
// kilobyte of arrays here on every reconnect.
bool vescProfileSendProfile(const ControllerProfile &slot, float rpmPerKmh, uint32_t now) {
  if (!ensureLispScratch()) return false;
  const size_t len = vescProfileLisp(slot.maxSpeedKmh, slot.accelPercent, slot.brakePercent,
                                     slot.fieldWeakeningAmps, slot.motorAmps, slot.batteryAmps,
                                     rpmPerKmh, lispScratch->expression,
                                     sizeof(lispScratch->expression));
  if (len == 0) return false;
  return vescSendLispExpression(lispScratch->expression, now, kLispProfile);
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
  // A queued cruise expression is the rider's latest action and gets the next
  // REPL slot. Release in particular must not wait behind a periodic profile
  // read-back or a write.
  vescServiceCruiseLisp(now);

  if (profileStepState == kProfileStepIdle) {
    uint8_t request = 0xFF;
    portENTER_CRITICAL(&profileMux);
    request = profileWriteRequestIndex;
    profileWriteRequestIndex = 0xFF;
    portEXIT_CRITICAL(&profileMux);

    if (request != 0xFF && request < kControllerProfileCount) {
      // Copy the slot now. The UI may edit another field while the settle and
      // read-back are in flight; the target and the expression must describe
      // the same numbers or the confirmation can disagree with itself.
      profileStepSlot = controllerProfiles[request];
      profileStepTarget = vescProfileTarget(profileStepSlot.maxSpeedKmh, profileStepSlot.accelPercent,
                                            profileStepSlot.brakePercent, profileStepSlot.batteryAmps,
                                            rpmPerKmh);
      profileStepIndex = request;
      profileStepState = kProfileStepSend;
      profileStepDueMs = now;
      profileStepConfirmRetries = 0;
      vescProfileSetReport(VESC_PROFILE_WRITE_SENT, request);
    } else {
      if ((int32_t)(now - profileLastRefreshMs) >= 0) {
        profileLastRefreshMs = now + kProfileRefreshMs;
        (void)vescProfileRefresh();
      }
      return;
    }
  }
  if ((int32_t)(now - profileStepDueMs) < 0) return;

  switch (profileStepState) {
    case kProfileStepSend:
      if (pendingCruiseLispActive) {
        // Do not let a profile write overtake a rider release that is waiting
        // for the REPL window. Try again after the cruise command goes out.
        profileStepDueMs = now + 50;
        return;
      }
      if (!vescLispReady(now)) {
        profileStepDueMs = lastLispSentMs + kVescLispGapMs;
        return;
      }
      profileReplySeen = false;
      profileReplyOk = false;
      if (!vescProfileSendProfile(profileStepSlot, rpmPerKmh, now)) {
        gardenLog("profile: could not build or send the Lisp expression");
        vescProfileSetReport(VESC_PROFILE_WRITE_FAILED, profileStepIndex);
        profileStepState = kProfileStepIdle;
        profileStepIndex = 0xFF;
        profileStepConfirmRetries = 0;
        return;
      }
      profileStepState = kProfileStepConfirm;
      profileStepDueMs = now + kProfileSettleMs;
      break;
    case kProfileStepConfirm: {
      // The REPL answer is asynchronous and may already have arrived while the
      // telemetry poll was waiting for GET_VALUES. An explicit error is final;
      // otherwise the controller's own numbers are still the confirmation.
      if (profileReplySeen && !profileReplyOk) {
        gardenLogf("profile: controller rejected the expression: %s\n", lastLispReply);
        vescProfileSetReport(VESC_PROFILE_WRITE_FAILED, profileStepIndex);
        profileStepState = kProfileStepIdle;
        profileStepIndex = 0xFF;
        profileStepConfirmRetries = 0;
        break;
      }

      if (vescProfileRefresh()) {
        VescMcconfTemp live = {};
        if (vescProfileReadBackCopy(live) && vescProfileReadBackMatches(profileStepTarget, live)) {
          vescProfileSetReport(VESC_PROFILE_WRITE_CONFIRMED, profileStepIndex);
        } else {
          gardenLog("profile: controller answered but the read-back does not match");
          vescProfileSetReport(VESC_PROFILE_WRITE_FAILED, profileStepIndex);
        }
        profileStepState = kProfileStepIdle;
        profileStepIndex = 0xFF;
        profileStepConfirmRetries = 0;
        break;
      }

      profileStepConfirmRetries++;
      if (profileStepConfirmRetries < kProfileMaxConfirmRetries) {
        // The controller can be busy with LispBM or motor work; ask again
        // instead of turning one missed reply into a failed profile.
        profileStepDueMs = now + kProfileConfirmRetryMs;
        break;
      }

      // No read-back after all retries. The expression is on the wire and
      // LispBM applies conf-set in RAM; accept it only when the REPL itself
      // acknowledged the command, otherwise report the failure instead of
      // claiming a confirmation the controller never gave.
      const bool acked = profileReplySeen && profileReplyOk;
      gardenLogf("profile: no GET_MCCONF_TEMP reply after %u tries; REPL %s\n",
                 (unsigned)kProfileMaxConfirmRetries, acked ? "acknowledged" : "silent");
      vescProfileSetReport(acked ? VESC_PROFILE_WRITE_CONFIRMED : VESC_PROFILE_WRITE_FAILED,
                           profileStepIndex);
      profileStepState = kProfileStepIdle;
      profileStepIndex = 0xFF;
      profileStepConfirmRetries = 0;
      break;
    }
    default:
      profileStepState = kProfileStepIdle;
      profileStepIndex = 0xFF;
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
    // A release is write-only and can still reach a controller whose telemetry
    // reply was lost, so do not let a failed poll strand it.
    vescServiceCruiseLisp(now);
    vescCheckCruiseReply();
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
      cruisePackage.available = false;
      cruisePackage.state = CRUISE_PACKAGE_UNAVAILABLE;
      cruisePackageTargetKmh = 0;
      cruisePackageRevision++;
      portEXIT_CRITICAL(&cruiseMux);
      cruisePackageSettingsSeen = false;
      cruisePackageLastLineMs = 0;
      cruisePackageLastQueryMs = 0;
      cruisePackageQueryAttempts = 0;
      // A write in flight cannot finish on a dead link: report it failed and drop
      // the step, so it does not resume mid-sequence when the link returns.
      if (profileStepState != kProfileStepIdle) {
        const uint8_t failedIndex = profileStepIndex;
        profileStepState = kProfileStepIdle;
        profileStepIndex = 0xFF;
        profileStepConfirmRetries = 0;
        vescProfileSetReport(VESC_PROFILE_WRITE_FAILED, failedIndex);
      }
      // Nothing queued for the old link may leak onto the next one.
      pendingCruiseLispActive = false;
      pendingCruiseLisp[0] = '\0';
      profileReapplyPending = false;
      lispSent = false;
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
    cruisePackage.available = false;
    cruisePackage.state = CRUISE_PACKAGE_UNAVAILABLE;
    cruisePackageTargetKmh = 0;
    cruisePackageRevision++;
    portEXIT_CRITICAL(&cruiseMux);
    cruisePackageSettingsSeen = false;
    cruisePackageLastLineMs = 0;
    cruisePackageLastQueryMs = 0;
    cruisePackageQueryAttempts = 0;
    // Re-apply the profile the rider actually applied on every reconnect. The
    // controller's limits live in RAM only (conf-set, no store), so a power
    // cycle or link drop wipes them. Selecting a slot in the menu is not an
    // apply; only a confirmed write is. Wait until the link has been stable for
    // a moment instead of writing in the same poll that discovered it: LispBM
    // may still be starting, and the first polls are the busiest.
    vescConnectedAtMs = now;
    profileReapplyPending =
        appliedControllerProfile != 0xFF && appliedControllerProfile < kControllerProfileCount;
    gardenLog("VESC telemetry connected");
  }
  vescConnected = true;
  vescLastSuccessMs = now;
  // requestValues() collects any asynchronous Lisp reply before it returns;
  // act on a cruise rejection before the next state-machine cycle.
  vescCheckCruiseReply();
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

  // The installed vesc_cruise_control package owns the cruise. The display
  // only mirrors its state and, while it is ON, keeps the speed it is holding
  // for the dashboard chip.
  portENTER_CRITICAL(&cruiseMux);
  if (cruisePackage.state == CRUISE_PACKAGE_ON) cruisePackageTargetKmh = (uint16_t)out.values.speedKmh;
  portEXIT_CRITICAL(&cruiseMux);
  vescCruisePackageStep(now);

  // A profile is re-applied only after the link has been stable for a moment,
  // and only when no other write is already in flight. This is deliberately
  // later than the first telemetry sample, so a reconnect cannot turn into a
  // bootloop before the dashboard has even drawn.
  if (profileReapplyPending && (int32_t)(now - vescConnectedAtMs) >= (int32_t)kProfileReapplyDelayMs) {
    profileReapplyPending = false;
    bool writeQueued = false;
    portENTER_CRITICAL(&profileMux);
    writeQueued = profileWriteRequestIndex != 0xFF;
    portEXIT_CRITICAL(&profileMux);
    if (!writeQueued && profileStepState == kProfileStepIdle &&
        appliedControllerProfile != 0xFF && appliedControllerProfile < kControllerProfileCount) {
      gardenLogf("profile: re-applying slot %u after reconnect\n", (unsigned)appliedControllerProfile);
      vescProfileRequestWrite(appliedControllerProfile);
    }
  }

  // A requested profile switch, or the slow read-back that keeps the values page
  // showing the controller's own numbers. Runs last so it cannot delay the
  // telemetry the dashboard draws from.
  vescProfileStep(now, rpmPerKmh);
  return CONTROLLER_POLL_UPDATED;
}

// The package engages and releases by itself; there is no display-side button.
// These stay as no-ops so the manager/UI API remains compatible.
void vescCruiseRequestEngage() {}
void vescCruiseRequestDisengage() {}

void vescProfileRequestWrite(uint8_t index) {
  // Only the slot is queued here. The report becomes SENT when the telemetry
  // task actually consumes it, so a request made while another write is still
  // being confirmed cannot claim a state the wire has not reached.
  portENTER_CRITICAL(&profileMux);
  if (index < kControllerProfileCount) profileWriteRequestIndex = index;
  portEXIT_CRITICAL(&profileMux);
}

VescProfileWriteReport vescProfileWriteReportGet() {
  portENTER_CRITICAL(&profileMux);
  const VescProfileWriteReport report = profileWriteReport;
  portEXIT_CRITICAL(&profileMux);
  return report;
}

bool vescProfileReadBackCopy(VescMcconfTemp &out) {
  portENTER_CRITICAL(&profileMux);
  const bool valid = vescLiveProfileValid;
  if (valid) out = vescLiveProfile;
  portEXIT_CRITICAL(&profileMux);
  return valid;
}

bool vescCruiseActive(uint16_t &targetKmh) {
  bool active = false;
  portENTER_CRITICAL(&cruiseMux);
  active = cruisePackage.available && cruisePackage.state == CRUISE_PACKAGE_ON;
  if (active) targetKmh = cruisePackageTargetKmh;
  portEXIT_CRITICAL(&cruiseMux);
  return active;
}

void vescCruiseOutcome(CruiseOutcome &outcome, CruiseState &state) {
  portENTER_CRITICAL(&cruiseMux);
  outcome = cruiseOutcome;
  // The outcome is an edge, not a level: the UI reads it once per tick. Leaving
  // it set made every tick re-deliver the same toast ("CRUISE ENGAGED" kept
  // appearing while the cruise ran) and hid the next real transition.
  cruiseOutcome = CRUISE_OUTCOME_NONE;
  state = cruisePackage.state == CRUISE_PACKAGE_ON ? CRUISE_STATE_ACTIVE : CRUISE_STATE_OFF;
  portEXIT_CRITICAL(&cruiseMux);
}

bool vescCruisePackageSnapshot(CruisePackageSnapshot &out) {
  portENTER_CRITICAL(&cruiseMux);
  out = cruisePackage;
  const bool available = cruisePackage.available;
  portEXIT_CRITICAL(&cruiseMux);
  return available;
}

uint32_t vescCruisePackageRevision() {
  portENTER_CRITICAL(&cruiseMux);
  const uint32_t revision = cruisePackageRevision;
  portEXIT_CRITICAL(&cruiseMux);
  return revision;
}

void vescCruiseRequestSettings() {
  // Queue a settings query for the telemetry task. The UI calls this when the
  // cruise page opens; the package answers on COMM_CUSTOM_APP_DATA.
  vescQueueCruiseLisp("(send-settings)");
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
