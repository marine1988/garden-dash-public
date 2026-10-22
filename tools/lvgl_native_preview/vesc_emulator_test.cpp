// Host test for the fake VESC controller added to src/vesc_test. It runs the
// dashboard's real profile builder against the emulator's LispBM subset, reads
// the result back with the firmware's COMM_GET_MCCONF_TEMP parser, and exercises
// the ADC cruise override the same way the controller does.
//
// The point is the round trip: vesc_profile.cpp builds the expression, the fake
// controller applies it over its wire framing, and vesc_protocol.cpp decodes the
// reply. A bug on either side of the link is visible here without a CYD.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "../../src/vesc_test/frames.h"
#include "cruise_control.h"
#include "vesc_profile.h"
#include "vesc_protocol.h"

namespace {

int failures = 0;

void expect(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    failures++;
  }
}

bool near(float actual, float wanted, float tolerance) {
  return std::fabs(actual - wanted) <= tolerance;
}

struct Profile {
  const char *name;
  uint16_t maxSpeedKmh;
  uint16_t accelPercent;
  uint16_t brakePercent;
  uint16_t fieldWeakeningAmps;
  uint16_t motorAmps;
  uint16_t batteryAmps;
};

// The defaults in app_logic.cpp: Eco, Drive and Sport.
constexpr Profile kProfiles[] = {
    {"Eco", 25, 40, 60, 0, 40, 20},
    {"Drive", 45, 70, 80, 5, 80, 35},
    {"Sport", 100, 100, 100, 15, 150, 60},
};
constexpr float kRpmPerKmh = 100.0F;

bool sameController(const fake_vesc::ControllerState &a, const fake_vesc::ControllerState &b) {
  return a.maxErpm == b.maxErpm && a.currentMaxScale == b.currentMaxScale &&
         a.currentMinScale == b.currentMinScale && a.fieldWeakeningAmps == b.fieldWeakeningAmps &&
         a.motorAmps == b.motorAmps && a.batteryAmps == b.batteryAmps &&
         a.mcconf.currentMinScale == b.mcconf.currentMinScale &&
         a.mcconf.currentMaxScale == b.mcconf.currentMaxScale &&
         a.mcconf.minErpm == b.mcconf.minErpm && a.mcconf.maxErpm == b.mcconf.maxErpm &&
         a.mcconf.minDuty == b.mcconf.minDuty && a.mcconf.maxDuty == b.mcconf.maxDuty &&
         a.mcconf.wattMin == b.mcconf.wattMin && a.mcconf.wattMax == b.mcconf.wattMax &&
         a.mcconf.inCurrentMin == b.mcconf.inCurrentMin &&
         a.mcconf.inCurrentMax == b.mcconf.inCurrentMax && a.adcDetached == b.adcDetached &&
         a.cruiseHeld == b.cruiseHeld && a.cruiseErpm == b.cruiseErpm &&
         a.cruiseSpeedMmPerS == b.cruiseSpeedMmPerS;
}

// Builds the profile, applies it from the bytes that would go on the wire, then
// reads it back through the same frame the fake sends on COMM_GET_MCCONF_TEMP.
void testProfile(const Profile &profile) {
  char expression[kVescProfileLispMaxLen];
  const size_t expressionLength =
      vescProfileLisp(profile.maxSpeedKmh, profile.accelPercent, profile.brakePercent,
                      profile.fieldWeakeningAmps, profile.motorAmps, profile.batteryAmps,
                      kRpmPerKmh, expression, sizeof(expression));
  if (expressionLength == 0) {
    std::printf("FAIL: %s profile expression not built\n", profile.name);
    failures++;
    return;
  }

  uint8_t lispPayload[kVescProfileLispMaxLen + 8];
  const size_t lispLength = vescBuildLispPayload(expression, lispPayload, sizeof(lispPayload));
  if (lispLength == 0 || lispLength > 255) {
    std::printf("FAIL: %s Lisp payload does not fit one packet\n", profile.name);
    failures++;
    return;
  }

  // The dashboard's frame buffer is VESC_MAX_FRAME. Build once into a larger
  // scratch buffer to learn the real size, then prove the profile fits the
  // production-sized one.
  uint8_t scratch[VESC_MAX_FRAME + 32];
  const size_t wireLength = vescBuildFrame(lispPayload, uint8_t(lispLength), scratch, sizeof(scratch));
  expect(wireLength > 0 && wireLength <= VESC_MAX_FRAME, "profile frame fits VESC_MAX_FRAME");
  uint8_t request[VESC_MAX_FRAME];
  expect(vescBuildFrame(lispPayload, uint8_t(lispLength), request, sizeof(request)) == wireLength,
         "profile frame builds in the production buffer");

  fake_vesc::ControllerState controller;
  expect(std::strcmp(controller.applyLisp((const char *)(request + 3), 0, 0), "OK") == 0,
         "profile expression is valid");

  expect(near(controller.maxErpm, float(vescProfileErpmForSpeed(profile.maxSpeedKmh, kRpmPerKmh)), 0.5F),
         "profile max eRPM applied");
  expect(near(controller.currentMaxScale, float(profile.accelPercent) / 100.0F, 1e-4F),
         "profile acceleration scale applied");
  expect(near(controller.currentMinScale, float(profile.brakePercent) / 100.0F, 1e-4F),
         "profile brake scale applied");
  expect(near(controller.fieldWeakeningAmps, float(profile.fieldWeakeningAmps), 1e-3F),
         "profile field weakening applied");
  expect(near(controller.motorAmps, float(profile.motorAmps), 1e-3F),
         "profile motor current applied");
  expect(near(controller.batteryAmps, float(profile.batteryAmps), 1e-3F),
         "profile battery current applied");

  uint8_t reply[VESC_MAX_FRAME];
  const size_t replyLength =
      fake_vesc::makeMcconfTempFrame(reply, sizeof(reply), controller.mcconf);
  VescMcconfTemp readBack = {};
  expect(replyLength > 0 && vescParseMcconfTemp(reply + 2, reply[1], readBack),
         "COMM_GET_MCCONF_TEMP reply parses");

  const VescProfileTarget target =
      vescProfileTarget(profile.maxSpeedKmh, profile.accelPercent, profile.brakePercent,
                        profile.batteryAmps, kRpmPerKmh);
  if (!vescProfileReadBackMatches(target, readBack)) {
    std::printf("FAIL: %s profile read-back disagrees (eRPM %.1f/%.1f, max %.3f, min %.3f, batt %.1f)\n",
                profile.name, readBack.maxErpm, target.maxErpm, readBack.currentMaxScale,
                readBack.currentMinScale, readBack.inCurrentMax);
    failures++;
  }
}

// A COMM_FORWARD_CAN request wraps the real command. The fake must unwrap it and
// apply the Lisp expression from the inner payload, not from the wrapper.
void testCanForwardedProfile() {
  const Profile &profile = kProfiles[1];  // Drive
  char expression[kVescProfileLispMaxLen];
  const size_t expressionLength =
      vescProfileLisp(profile.maxSpeedKmh, profile.accelPercent, profile.brakePercent,
                      profile.fieldWeakeningAmps, profile.motorAmps, profile.batteryAmps,
                      kRpmPerKmh, expression, sizeof(expression));
  expect(expressionLength > 0, "forwarded profile expression built");

  uint8_t inner[kVescProfileLispMaxLen + 8];
  const size_t innerLength = vescBuildLispPayload(expression, inner, sizeof(inner));
  expect(innerLength > 0, "forwarded Lisp payload built");

  uint8_t payload[sizeof(inner) + 2];
  payload[0] = fake_vesc::kCommandForwardCan;
  payload[1] = 7;  // CAN id of the remote controller
  std::memcpy(payload + 2, inner, innerLength);

  uint8_t request[VESC_MAX_FRAME];
  const size_t requestLength = vescBuildFrame(payload, uint8_t(innerLength + 2), request, sizeof(request));
  expect(requestLength > 0 && requestLength <= VESC_MAX_FRAME, "forwarded frame fits VESC_MAX_FRAME");

  const fake_vesc::RequestView view = fake_vesc::unwrapCanForward(request + 2, request[1]);
  expect(view.length == innerLength, "CAN wrapper removed exactly");
  expect(view.payload[0] == VESC_COMM_LISP_REPL_CMD, "inner command is the Lisp REPL");
  expect(view.payload == request + 4, "inner payload starts after the CAN id");

  fake_vesc::ControllerState controller;
  expect(std::strcmp(controller.applyLisp((const char *)(view.payload + 1), 0, 0), "OK") == 0,
         "forwarded profile expression is valid");
  expect(near(controller.maxErpm, float(vescProfileErpmForSpeed(profile.maxSpeedKmh, kRpmPerKmh)), 0.5F),
         "forwarded max eRPM applied");
  expect(near(controller.batteryAmps, float(profile.batteryAmps), 1e-3F),
         "forwarded battery limit applied");

  // The old path read the expression at payload + 1, i.e. at the CAN id. Keep
  // that mistake visible if the parser ever starts accepting arbitrary bytes.
  fake_vesc::ControllerState wrong;
  expect(std::strcmp(wrong.applyLisp((const char *)(request + 3), 0, 0), "ERR") == 0,
         "the CAN wrapper itself is not a valid expression");
}

// The largest expression a rider can generate, to pin the frame-size limit that
// the fake sender's request buffer and VESC_MAX_FRAME must both accommodate.
void testLongestProfileFits() {
  char expression[kVescProfileLispMaxLen];
  const size_t expressionLength =
      vescProfileLisp(200, 100, 100, kVescProfileMaxFieldWeakeningAmps, 500, 500, 10000.0F,
                      expression, sizeof(expression));
  expect(expressionLength > 0, "longest profile expression built");

  uint8_t payload[kVescProfileLispMaxLen + 8];
  const size_t payloadLength = vescBuildLispPayload(expression, payload, sizeof(payload));
  expect(payloadLength > 0 && payloadLength <= 255, "longest profile fits one VESC packet");

  uint8_t frame[VESC_MAX_FRAME];
  const size_t frameLength = vescBuildFrame(payload, uint8_t(payloadLength), frame, sizeof(frame));
  expect(frameLength > 0 && frameLength <= VESC_MAX_FRAME, "longest profile fits VESC_MAX_FRAME");
  // The old 64-byte request buffer could not hold this write; keep that visible.
  expect(frameLength > 64, "a profile write needs more than the old 64-byte buffer");

  fake_vesc::ControllerState controller;
  expect(std::strcmp(controller.applyLisp((const char *)(frame + 3), 0, 0), "OK") == 0,
         "longest profile expression is valid");
  expect(near(controller.maxErpm, 2000000.0F, 1.0F), "longest profile applies its eRPM limit");
  expect(near(controller.motorAmps, 500.0F, 1e-3F), "longest profile applies its motor limit");
}

void testCruise() {
  fake_vesc::ControllerState controller;
  controller.cruiseButtonEnabled = true;

  // 25 s into the ride sequence: the 8,400 eRPM peak. Engage must capture it.
  const fake_vesc::Sample engageSample = fake_vesc::rideSample(25000);
  expect(engageSample.erpm == 8400, "fixture rides at 8,400 eRPM");
  expect(std::strcmp(controller.applyLisp(kCruiseLispEngage, engageSample.erpm,
                                          engageSample.speedMmPerS),
                     "OK") == 0,
         "cruise engage expression is valid");
  expect(controller.adcDetached, "engage detaches the ADC buttons");
  expect(controller.cruiseHeld, "engage enables the override");
  expect(controller.cruiseErpm == engageSample.erpm, "engage captures the current eRPM");
  expect(controller.cruiseSpeedMmPerS == engageSample.speedMmPerS, "engage captures the current speed");

  // The sequence is stationary at 5 s, but the held controller keeps its own
  // reading in both telemetry replies until release.
  const fake_vesc::Sample later = fake_vesc::rideSample(5000);
  expect(later.erpm == 0, "fixture is stationary later in the sequence");
  const fake_vesc::Sample held = controller.applyCruise(later);
  expect(held.erpm == engageSample.erpm, "the held sample keeps the captured eRPM");
  expect(held.speedMmPerS == engageSample.speedMmPerS, "the held sample keeps the captured speed");

  uint8_t reply[VESC_MAX_FRAME];
  size_t replyLength = fake_vesc::makeValuesFrame(reply, sizeof(reply), held, {});
  VescValues values = {};
  expect(replyLength > 0 && vescParseValues(reply + 2, reply[1], values), "held GET_VALUES parses");
  expect(near(values.rpm, float(engageSample.erpm), 0.5F), "GET_VALUES reports the held eRPM");

  replyLength = fake_vesc::makeSetupFrame(reply, sizeof(reply), held, {});
  VescSetupValues setup = {};
  expect(replyLength > 0 && vescParseSetupValues(reply + 2, reply[1], setup),
         "held GET_VALUES_SETUP parses");
  expect(near(setup.speedMs, float(engageSample.speedMmPerS) / 1000.0F, 0.002F),
         "GET_VALUES_SETUP reports the held speed");

  // Re-sending engage while the hold is active must not move the setpoint to
  // whatever the sequence happens to be doing now.
  const fake_vesc::Sample other = fake_vesc::rideSample(35000);
  expect(std::strcmp(controller.applyLisp(kCruiseLispEngage, other.erpm, other.speedMmPerS), "OK") == 0,
         "repeat engage is accepted");
  expect(controller.cruiseErpm == engageSample.erpm, "repeat engage keeps the original setpoint");

  expect(std::strcmp(controller.applyLisp(kCruiseLispRelease), "OK") == 0, "cruise release is valid");
  expect(!controller.cruiseHeld, "release disables the override");
  const fake_vesc::Sample released = controller.applyCruise(later);
  expect(released.erpm == later.erpm, "release returns to the sequence eRPM");
  expect(released.speedMmPerS == later.speedMmPerS, "release returns to the sequence speed");
}

void testCruiseButtonDisabled() {
  fake_vesc::ControllerState controller;
  controller.cruiseButtonEnabled = false;
  const fake_vesc::Sample engage = fake_vesc::rideSample(25000);
  expect(std::strcmp(controller.applyLisp(kCruiseLispEngage, engage.erpm, engage.speedMmPerS), "OK") == 0,
         "disabled cruise button still acknowledges the Lisp command");
  expect(!controller.cruiseHeld, "disabled cruise button does not capture a hold");
  const fake_vesc::Sample later = fake_vesc::rideSample(35000);
  const fake_vesc::Sample held = controller.applyCruise(later);
  expect(held.erpm == later.erpm && held.speedMmPerS == later.speedMmPerS,
         "disabled cruise button lets the sequence continue");
}

void testInvalidExpressions() {
  fake_vesc::ControllerState controller;

  // Establish a known-good state, then make sure every bad form leaves it alone.
  char expression[kVescProfileLispMaxLen];
  const Profile &profile = kProfiles[0];
  expect(vescProfileLisp(profile.maxSpeedKmh, profile.accelPercent, profile.brakePercent,
                         profile.fieldWeakeningAmps, profile.motorAmps, profile.batteryAmps,
                         kRpmPerKmh, expression, sizeof(expression)) > 0,
         "invalid-expression fixture profile built");
  expect(std::strcmp(controller.applyLisp(expression, 0, 0), "OK") == 0,
         "invalid-expression fixture profile applied");
  const fake_vesc::ControllerState before = controller;

  const char *bad[] = {
      "",                                             // empty
      "(conf-set 'l-max-erpm 42",                     // unclosed form
      "(conf-set 'l-max-erpm nan)",                   // non-finite value
      "(conf-set 'l-unknown 1)",                      // unknown key
      "{(conf-set 'l-max-erpm 1000)(conf-set 'l-nope 1)}",  // unknown form in a valid block
      "(explode 1)",                                  // unknown command
      "(app-adc-override 3)",                         // missing argument
      "(app-adc-override 9 1)",                       // out-of-range channel
      "junk",                                         // not an expression
  };
  for (const char *source : bad) {
    if (std::strcmp(controller.applyLisp(source, 0, 0), "ERR") != 0) {
      std::printf("FAIL: invalid expression accepted: \"%s\"\n", source);
      failures++;
    }
    if (!sameController(controller, before)) {
      std::printf("FAIL: invalid expression changed the controller: \"%s\"\n", source);
      failures++;
      controller = before;  // keep later checks meaningful
    }
  }
}

void testSessionReset() {
  fake_vesc::ControllerState controller;
  char expression[kVescProfileLispMaxLen];
  const Profile &profile = kProfiles[2];
  expect(vescProfileLisp(profile.maxSpeedKmh, profile.accelPercent, profile.brakePercent,
                         profile.fieldWeakeningAmps, profile.motorAmps, profile.batteryAmps,
                         kRpmPerKmh, expression, sizeof(expression)) > 0,
         "reset fixture profile built");
  expect(std::strcmp(controller.applyLisp(expression, 0, 0), "OK") == 0, "reset fixture profile applied");
  expect(std::strcmp(controller.applyLisp(kCruiseLispEngage, 8400, 13900), "OK") == 0,
         "reset fixture cruise engaged");

  controller.reset();

  const fake_vesc::ControllerState empty;
  expect(sameController(controller, empty), "session reset clears profile and cruise");
  expect(controller.maxErpm == 0.0F && controller.currentMaxScale == 0.0F &&
             controller.currentMinScale == 0.0F && controller.fieldWeakeningAmps == 0.0F &&
             controller.motorAmps == 0.0F && controller.batteryAmps == 0.0F,
         "session reset zeroes the six profile fields");
  expect(!controller.cruiseHeld && !controller.adcDetached, "session reset releases the ADC state");
  expect(controller.applyCruise(fake_vesc::rideSample(25000)).erpm == 8400,
         "a reset controller passes the sequence through");
}

}  // namespace

int main() {
  for (const Profile &profile : kProfiles) testProfile(profile);
  testLongestProfileFits();
  testCanForwardedProfile();
  testCruise();
  testCruiseButtonDisabled();
  testInvalidExpressions();
  testSessionReset();

  if (failures) {
    std::printf("VESC emulator: %d failure(s)\n", failures);
    return 1;
  }
  std::puts("VESC emulator: profile round-trips, CAN forwarding, cruise and reset passed");
  return 0;
}
