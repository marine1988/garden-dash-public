// Host tests for the display-side cruise-control state machine.
//
// The module is pure logic, so these drive it directly: no controller, no
// radio, no LVGL. Each case fixes the settings and one cycle of inputs and
// checks what the dashboard would have sent, if anything. The output is a
// LispBM expression (the VESC ADC application holds the setpoint itself), not a
// periodic COMM_SET_RPM frame.

#include <cstdio>
#include <cstring>

#include "cruise_control.h"
#include "vesc_protocol.h"

namespace {

int failures = 0;

void expect(bool ok, const char *what) {
  if (!ok) {
    printf("FAIL: %s\n", what);
    failures++;
  }
}

CruiseSettings baseSettings() {
  CruiseSettings s = {};
  s.enabled = true;
  s.holdTimeDeciS = 50;
  s.minSpeedKmh = 5;
  s.maxSpeedKmh = 25;
  s.allowedModes = 0;  // any
  return s;
}

CruiseInputs baseInputs() {
  CruiseInputs in = {};
  in.speedKmh = 20.0F;
  in.rpm = 4000.0F;
  in.rpmPerKmh = 200.0F;  // 20 km/h -> 4000 eRPM
  in.avgInputCurrent = 0.0F;
  in.faultCode = 0;
  in.rideMode = 0;
  in.linkLive = true;
  return in;
}

CruiseRuntime fresh() {
  CruiseRuntime rt = {};
  return rt;
}

bool sentEngage(const CruiseOutput &out) {
  return out.send && out.lisp != nullptr && strcmp(out.lisp, kCruiseLispEngage) == 0;
}

bool sentRelease(const CruiseOutput &out) {
  return out.send && out.lisp != nullptr && strcmp(out.lisp, kCruiseLispRelease) == 0;
}

void testDisabledNeverSends() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.enabled = false;
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  expect(!out.send, "disabled: no command");
  expect(out.state == CRUISE_STATE_OFF, "disabled: stays off");
  expect(out.outcome == CRUISE_OUTCOME_NONE, "disabled: no outcome");
  expect(rt.targetKmh == 0, "disabled: no target captured");
}

void testEngageBelowMinIsBlocked() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  CruiseInputs in = baseInputs();
  in.speedKmh = 3.0F;  // under the 5 km/h floor
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(!out.send, "too slow: no command");
  expect(out.state == CRUISE_STATE_BLOCKED, "too slow: blocked");
  expect(out.outcome == CRUISE_OUTCOME_BLOCKED_SLOW, "too slow: outcome is BLOCKED_SLOW");
}

void testEngageCapturesTargetAndSends() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  expect(out.send, "engage: sends at once");
  expect(out.lisp != nullptr && strcmp(out.lisp, kCruiseLispEngage) == 0, "engage: uses the engage expression");
  expect(out.state == CRUISE_STATE_ACTIVE, "engage: active");
  expect(out.outcome == CRUISE_OUTCOME_ENGAGED, "engage: outcome is ENGAGED");
  expect(rt.targetKmh == 20, "engage: target is the current speed");
  expect(rt.rpmPerKmh == 200.0F, "engage: keeps the gearing figure");
}

void testEngageClampsTargetToMax() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.maxSpeedKmh = 15;
  CruiseInputs in = baseInputs();
  in.speedKmh = 22.0F;  // above the ceiling
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(out.state == CRUISE_STATE_ACTIVE, "over max: still engages");
  expect(rt.targetKmh == 15, "over max: target clamped to the ceiling");

  // A fractional speed must round down to the ceiling, never past it.
  CruiseRuntime rt2 = fresh();
  in.speedKmh = 15.4F;
  cruiseControlStep(rt2, s, in, true, false, 1000);
  expect(rt2.targetKmh == 15, "over max (fraction): target clamped to the ceiling");
}

void testEngageClampsTargetToMin() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.minSpeedKmh = 10;
  s.maxSpeedKmh = 25;
  CruiseInputs in = baseInputs();
  in.speedKmh = 10.4F;  // rounds down to the floor
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(out.state == CRUISE_STATE_ACTIVE && sentEngage(out), "at the floor: engages");
  expect(rt.targetKmh == 10, "at the floor: target never rounds under the minimum");

  // Under the floor is a refusal, not a clamp upwards.
  CruiseRuntime rt2 = fresh();
  in.speedKmh = 9.9F;
  const CruiseOutput blocked = cruiseControlStep(rt2, s, in, true, false, 1000);
  expect(!blocked.send && blocked.state == CRUISE_STATE_BLOCKED, "under the floor: blocked");
  expect(blocked.outcome == CRUISE_OUTCOME_BLOCKED_SLOW, "under the floor: BLOCKED_SLOW");
  expect(rt2.targetKmh == 0, "under the floor: no target captured");
}

void testEngageNoGearingIsBlocked() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  CruiseInputs in = baseInputs();
  in.rpmPerKmh = 0.0F;  // wheel size / gearing unset
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(!out.send && out.state == CRUISE_STATE_BLOCKED, "no gearing: blocked");
  expect(out.outcome == CRUISE_OUTCOME_BLOCKED_GEARING, "no gearing: outcome is BLOCKED_GEARING");

  CruiseRuntime rt2 = fresh();
  in.rpmPerKmh = -200.0F;  // a negative ratio is equally unusable
  const CruiseOutput negative = cruiseControlStep(rt2, s, in, true, false, 1000);
  expect(!negative.send && negative.state == CRUISE_STATE_BLOCKED, "negative gearing: blocked");
  expect(negative.outcome == CRUISE_OUTCOME_BLOCKED_GEARING, "negative gearing: BLOCKED_GEARING");
}

void testEngageModeBlocked() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.allowedModes = (uint8_t)(1u << (CRUISE_MODE_ECO - 1));  // Eco only
  CruiseInputs in = baseInputs();
  in.rideMode = CRUISE_MODE_SPORT;
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(!out.send && out.state == CRUISE_STATE_BLOCKED, "sport while Eco-only: blocked");
  expect(out.outcome == CRUISE_OUTCOME_BLOCKED_MODE, "sport while Eco-only: outcome is BLOCKED_MODE");
}

void testModeZeroAccepted() {
  const CruiseSettings s = baseSettings();  // allowedModes == 0

  // Mode 0 means the controller did not report a mode: it must not be filtered.
  CruiseRuntime rt = fresh();
  CruiseInputs in = baseInputs();
  in.rideMode = CRUISE_MODE_NONE;
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(out.state == CRUISE_STATE_ACTIVE && sentEngage(out), "mask 0 with mode 0: engages");
  expect(out.outcome == CRUISE_OUTCOME_ENGAGED, "mask 0 with mode 0: ENGAGED");

  // Out-of-range values are also "not reported", not a mode to compare.
  CruiseRuntime rt2 = fresh();
  in.rideMode = 200;
  const CruiseOutput unknown = cruiseControlStep(rt2, s, in, true, false, 1000);
  expect(unknown.state == CRUISE_STATE_ACTIVE && sentEngage(unknown), "mask 0 with unknown mode: engages");

  // But an empty mask still refuses a mode the controller does report.
  CruiseRuntime rt3 = fresh();
  in.rideMode = CRUISE_MODE_ECO;
  const CruiseOutput blocked = cruiseControlStep(rt3, s, in, true, false, 1000);
  expect(!blocked.send && blocked.state == CRUISE_STATE_BLOCKED, "mask 0 with reported mode: blocked");
  expect(blocked.outcome == CRUISE_OUTCOME_BLOCKED_MODE, "mask 0 with reported mode: BLOCKED_MODE");
}

void testEngageRefusedWhileBraking() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  CruiseInputs in = baseInputs();
  in.avgInputCurrent = -12.0F;  // regenerative braking current
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(!out.send, "engage while braking: no command");
  expect(out.state == CRUISE_STATE_OFF, "engage while braking: stays off");
  expect(out.outcome == CRUISE_OUTCOME_NONE, "engage while braking: nothing to report");
  expect(rt.targetKmh == 0, "engage while braking: no target captured");

  // The threshold itself is a coast: the brake test is strictly below -5 A.
  CruiseRuntime rt2 = fresh();
  CruiseInputs coast = baseInputs();
  coast.avgInputCurrent = kCruiseBrakeRegenAmps;
  const CruiseOutput engaged = cruiseControlStep(rt2, s, coast, true, false, 1000);
  expect(engaged.state == CRUISE_STATE_ACTIVE && sentEngage(engaged), "at the brake threshold: engages");
}

void testActiveSendsNothingMore() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);  // engages and sends
  // The VESC application holds the setpoint, so there is no heartbeat: a later
  // cycle stays silent while the conditions hold.
  const CruiseOutput later = cruiseControlStep(rt, s, baseInputs(), false, false, 1000 + 500);
  expect(!later.send, "active: no heartbeat frame");
  expect(later.outcome == CRUISE_OUTCOME_NONE, "active: no outcome on a quiet cycle");
  expect(later.state == CRUISE_STATE_ACTIVE, "active: still active");
}

void testBlockedEngageNeverActive() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  CruiseInputs in = baseInputs();
  in.speedKmh = 3.0F;
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(out.state != CRUISE_STATE_ACTIVE, "blocked engage: never active");
  expect(out.state == CRUISE_STATE_BLOCKED, "blocked engage: blocked");
  expect(!out.send, "blocked engage: no command");
  expect(rt.targetKmh == 0, "blocked engage: no target");

  // A refused attempt must not poison the next, valid one.
  const CruiseOutput retry = cruiseControlStep(rt, s, baseInputs(), true, false, 1100);
  expect(retry.state == CRUISE_STATE_ACTIVE && sentEngage(retry), "blocked then retry: engages");
  expect(rt.targetKmh == 20, "blocked then retry: target captured");
}

void testRepeatedEngageWhileActiveIsIgnored() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);

  // A second engage while the cruise is already running cannot retarget the
  // ADC application (it captures on the transition into cruise) and must not
  // drop the local hold either.
  CruiseInputs faster = baseInputs();
  faster.speedKmh = 22.0F;
  const CruiseOutput out = cruiseControlStep(rt, s, faster, true, false, 1100);
  expect(out.state == CRUISE_STATE_ACTIVE, "repeat engage: still active");
  expect(!out.send, "repeat engage: no second expression");
  expect(out.outcome == CRUISE_OUTCOME_NONE, "repeat engage: no outcome");
  expect(rt.targetKmh == 20, "repeat engage: captured target untouched");
}

void testActiveEngageUnderMinReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);

  // A repeated engage whose gates fail must release the live cruise rather
  // than leave the local state claiming nothing is held.
  CruiseInputs slow = baseInputs();
  slow.speedKmh = 2.0F;
  const CruiseOutput out = cruiseControlStep(rt, s, slow, true, false, 1100);
  expect(sentRelease(out), "active engage while slow: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_SPEED, "active engage while slow: RELEASED_SPEED");
  expect(out.state == CRUISE_STATE_OFF, "active engage while slow: off");
  expect(rt.targetKmh == 0, "active engage while slow: target cleared");
}

void testFaultReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.faultCode = 4;  // over-current
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send, "fault: sends a release");
  expect(out.lisp != nullptr && strcmp(out.lisp, kCruiseLispRelease) == 0, "fault: release expression");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_FAULT, "fault: outcome is RELEASED_FAULT");
  expect(out.state == CRUISE_STATE_OFF, "fault: off");
  // A persistent fault after the release must not release a second time.
  const CruiseOutput again = cruiseControlStep(rt, s, in, false, false, 1200);
  expect(!again.send && again.state == CRUISE_STATE_OFF, "fault: released exactly once");
}

void testLinkLossReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.linkLive = false;
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "link lost: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_LINK, "link lost: outcome is RELEASED_LINK");
  expect(out.state == CRUISE_STATE_OFF, "link lost: off");
  const CruiseOutput again = cruiseControlStep(rt, s, in, false, false, 1200);
  expect(!again.send && again.state == CRUISE_STATE_OFF, "link lost: released exactly once");
}

void testDisableReleases() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  s.enabled = false;
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), false, false, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "disabled while active: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_DISABLED, "disabled: outcome is RELEASED_DISABLED");
  expect(out.state == CRUISE_STATE_OFF, "disabled: off");
  const CruiseOutput again = cruiseControlStep(rt, s, baseInputs(), false, false, 1200);
  expect(!again.send && again.state == CRUISE_STATE_OFF, "disabled: released exactly once");

  // Re-enabling does not silently resume the released cruise.
  s.enabled = true;
  const CruiseOutput resumed = cruiseControlStep(rt, s, baseInputs(), false, false, 1300);
  expect(!resumed.send && resumed.state == CRUISE_STATE_OFF, "re-enabled: stays off until asked");
}

void testDisengageRequestReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), false, true, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "cancel: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_CANCEL, "cancel: outcome is RELEASED_CANCEL");
  expect(out.state == CRUISE_STATE_OFF, "cancel: off");
  const CruiseOutput again = cruiseControlStep(rt, s, baseInputs(), false, true, 1200);
  expect(!again.send && again.state == CRUISE_STATE_OFF, "cancel: released exactly once");

  // A fresh engage after cancelling works normally.
  const CruiseOutput reengaged = cruiseControlStep(rt, s, baseInputs(), true, false, 1300);
  expect(reengaged.state == CRUISE_STATE_ACTIVE && sentEngage(reengaged), "cancel then engage: re-engages");
}

void testDisengageBeatsEngage() {
  const CruiseSettings s = baseSettings();

  // While running, an explicit cancel wins over a simultaneous engage.
  CruiseRuntime rt = fresh();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), true, true, 1100);
  expect(sentRelease(out), "cancel + engage while active: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_CANCEL, "cancel + engage: outcome is RELEASED_CANCEL");
  expect(out.state == CRUISE_STATE_OFF, "cancel + engage while active: off");

  // While idle, the same pair must not start a cruise.
  CruiseRuntime idle = fresh();
  const CruiseOutput refused = cruiseControlStep(idle, s, baseInputs(), true, true, 1000);
  expect(!refused.send, "cancel + engage while idle: no command");
  expect(refused.state == CRUISE_STATE_OFF, "cancel + engage while idle: stays off");
}

void testSpeedDropReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.speedKmh = 2.0F;  // braked under the floor
  CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "braked below the floor: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_SPEED, "braked: outcome is RELEASED_SPEED");
  out = cruiseControlStep(rt, s, in, false, false, 1200);
  expect(!out.send && out.state == CRUISE_STATE_OFF, "braked: released exactly once");
}

void testBrakeReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.avgInputCurrent = -12.0F;  // regenerative braking current
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "brake: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_BRAKE, "brake: outcome is RELEASED_BRAKE");
  expect(out.state == CRUISE_STATE_OFF, "brake: off");
  const CruiseOutput again = cruiseControlStep(rt, s, in, false, false, 1200);
  expect(!again.send && again.state == CRUISE_STATE_OFF, "brake: released exactly once");

  // A light downhill regen stays under the threshold and must not release.
  CruiseRuntime rt2 = fresh();
  cruiseControlStep(rt2, s, baseInputs(), true, false, 1000);
  CruiseInputs coast = baseInputs();
  coast.avgInputCurrent = -1.0F;
  const CruiseOutput held = cruiseControlStep(rt2, s, coast, false, false, 1100);
  expect(!held.send && held.state == CRUISE_STATE_ACTIVE, "light regen: cruise holds");
}

void testAutomaticHoldEngages() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.holdTimeDeciS = 20;  // 2 s
  CruiseInputs in = baseInputs();
  in.avgInputCurrent = 10.0F;  // the rider is driving, not coasting

  CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1000);
  expect(out.state == CRUISE_STATE_HOLDING && !out.send, "auto: first steady tick starts HOLDING");
  expect(cruiseHoldProgress(rt, s, 2000) == 50, "auto: progress is 50% after half the hold time");

  out = cruiseControlStep(rt, s, in, false, false, 2000);
  expect(out.state == CRUISE_STATE_HOLDING && !out.send, "auto: still holding at half time");

  out = cruiseControlStep(rt, s, in, false, false, 3000);
  expect(out.state == CRUISE_STATE_ARMED && !out.send, "auto: arms after the hold time");
  in.avgInputCurrent = 0.0F;  // throttle released, ADC can capture the setpoint
  out = cruiseControlStep(rt, s, in, false, false, 3100);
  expect(out.state == CRUISE_STATE_ACTIVE && sentEngage(out), "auto: engages when throttle is released");
  expect(out.outcome == CRUISE_OUTCOME_ENGAGED, "auto: outcome ENGAGED");
  expect(rt.targetKmh == 20, "auto: captures the steady speed");

  // A speed change beyond the deadband cancels a running cruise, but only
  // after the transition grace and a sustained deviation.
  in.avgInputCurrent = 10.0F;
  in.speedKmh = 25.0F;
  out = cruiseControlStep(rt, s, in, false, false, 4000);  // inside engage grace
  expect(out.state == CRUISE_STATE_ACTIVE && !out.send, "auto: grace keeps a fresh hold");
  out = cruiseControlStep(rt, s, in, false, false, 4600);  // deviation starts
  expect(out.state == CRUISE_STATE_ACTIVE && !out.send, "auto: first moved tick still holds");
  out = cruiseControlStep(rt, s, in, false, false, 5600);  // 1 s outside
  expect(out.state == CRUISE_STATE_ACTIVE && !out.send, "auto: short deviation still holds");
  out = cruiseControlStep(rt, s, in, false, false, 6000);  // >1.2 s outside
  expect(out.state == CRUISE_STATE_OFF && sentRelease(out), "auto: sustained speed change releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_CANCEL, "auto: sustained speed change is a cancel");
}

void testAutomaticHoldIgnoresTransientDip() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.holdTimeDeciS = 20;
  CruiseInputs in = baseInputs();
  in.avgInputCurrent = 10.0F;
  cruiseControlStep(rt, s, in, false, false, 1000);
  cruiseControlStep(rt, s, in, false, false, 3000);  // armed
  in.avgInputCurrent = 0.0F;
  cruiseControlStep(rt, s, in, false, false, 3100);  // engage on release
  in.avgInputCurrent = 10.0F;

  in.speedKmh = 16.5F;  // a bump or short coast, outside the 3 km/h margin
  CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 5000);
  expect(out.state == CRUISE_STATE_ACTIVE && !out.send, "auto: transient dip keeps cruise");
  in.speedKmh = 20.0F;
  out = cruiseControlStep(rt, s, in, false, false, 5500);
  expect(out.state == CRUISE_STATE_ACTIVE && !out.send, "auto: recovering speed keeps cruise");
}

void testAutomaticHoldResetsWhenSpeedMoves() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.holdTimeDeciS = 20;
  CruiseInputs in = baseInputs();
  in.avgInputCurrent = 10.0F;

  cruiseControlStep(rt, s, in, false, false, 0);
  in.speedKmh = 23.0F;  // outside the 2 km/h deadband
  CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 500);
  expect(out.state == CRUISE_STATE_HOLDING && !out.send, "auto: moved speed restarts the hold");
  out = cruiseControlStep(rt, s, in, false, false, 1500);
  expect(out.state == CRUISE_STATE_HOLDING, "auto: restarted hold still waiting");
  out = cruiseControlStep(rt, s, in, false, false, 2500);
  expect(out.state == CRUISE_STATE_ARMED && !out.send, "auto: restarted window arms");
  in.avgInputCurrent = 0.0F;
  out = cruiseControlStep(rt, s, in, false, false, 2600);
  expect(out.state == CRUISE_STATE_ACTIVE && sentEngage(out), "auto: engages from the restarted window");
}

void testAutomaticHoldNeedsDriveCurrent() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  CruiseInputs in = baseInputs();
  in.avgInputCurrent = 0.2F;  // coasting/regen, not driving
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1000);
  expect(out.state == CRUISE_STATE_OFF && !out.send, "auto: coasting cannot start a hold");
}

void testNoReleaseWhenInactive() {
  const CruiseSettings s = baseSettings();
  const CruiseInputs base = baseInputs();

  {
    CruiseRuntime rt = fresh();
    CruiseInputs in = base;
    in.avgInputCurrent = -12.0F;
    const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1000);
    expect(!out.send && out.state == CRUISE_STATE_OFF, "idle brake: no release");
  }
  {
    CruiseRuntime rt = fresh();
    CruiseInputs in = base;
    in.faultCode = 7;
    const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1000);
    expect(!out.send && out.state == CRUISE_STATE_OFF, "idle fault: no release");
  }
  {
    CruiseRuntime rt = fresh();
    CruiseInputs in = base;
    in.linkLive = false;
    const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1000);
    expect(!out.send && out.state == CRUISE_STATE_OFF, "idle dead link: no release");
  }
  {
    CruiseRuntime rt = fresh();
    CruiseSettings off = baseSettings();
    off.enabled = false;
    const CruiseOutput out = cruiseControlStep(rt, off, base, false, false, 1000);
    expect(!out.send && out.state == CRUISE_STATE_OFF, "idle disabled: no release");
  }
  {
    CruiseRuntime rt = fresh();
    const CruiseOutput out = cruiseControlStep(rt, s, base, false, true, 1000);
    expect(!out.send && out.state == CRUISE_STATE_OFF, "idle cancel: no release");
  }
  {
    CruiseRuntime rt = fresh();
    CruiseInputs in = base;
    in.speedKmh = 1.0F;
    const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1000);
    expect(!out.send && out.state == CRUISE_STATE_OFF, "idle under the floor: no release");
  }
  {
    // A refused engage reports the refusal, never a release.
    CruiseRuntime rt = fresh();
    CruiseInputs in = base;
    in.speedKmh = 1.0F;
    const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
    expect(!out.send && out.state == CRUISE_STATE_BLOCKED, "refused engage: no release");
  }
}

void testModeAllowedWindow() {
  CruiseSettings s = baseSettings();
  s.allowedModes = (uint8_t)(1u << (CRUISE_MODE_DRIVE - 1));
  expect(cruiseModeAllowed(s, CRUISE_MODE_NONE), "unreported mode is never blocked");
  expect(cruiseModeAllowed(s, CRUISE_MODE_DRIVE), "allowed mode passes");
  expect(!cruiseModeAllowed(s, CRUISE_MODE_SPORT), "excluded mode is refused");
  expect(cruiseModeAllowed(s, 200), "unknown mode is treated as unreported");
  CruiseSettings none = baseSettings();
  none.allowedModes = 0;
  expect(!cruiseModeAllowed(none, CRUISE_MODE_ECO), "empty allow list refuses a reported mode");
  expect(cruiseModeAllowed(none, CRUISE_MODE_NONE), "empty allow list still admits an unreported mode");
}

void testEncodeHelpers() {
  expect(vescEncodeDuty(0.5F) == 50000, "duty x1e5");
  expect(vescEncodeDuty(1.5F) == 100000, "duty clamps to 1");
  expect(vescEncodeDuty(-2.0F) == -100000, "duty clamps to -1");
  expect(vescEncodeCurrent(-12.5F) == -12500, "current x1e3");
}

}  // namespace

int main() {
  testDisabledNeverSends();
  testEngageBelowMinIsBlocked();
  testEngageCapturesTargetAndSends();
  testEngageClampsTargetToMax();
  testEngageClampsTargetToMin();
  testEngageNoGearingIsBlocked();
  testEngageModeBlocked();
  testModeZeroAccepted();
  testEngageRefusedWhileBraking();
  testActiveSendsNothingMore();
  testBlockedEngageNeverActive();
  testRepeatedEngageWhileActiveIsIgnored();
  testActiveEngageUnderMinReleases();
  testFaultReleases();
  testLinkLossReleases();
  testDisableReleases();
  testDisengageRequestReleases();
  testDisengageBeatsEngage();
  testSpeedDropReleases();
  testBrakeReleases();
  testAutomaticHoldEngages();
  testAutomaticHoldIgnoresTransientDip();
  testAutomaticHoldResetsWhenSpeedMoves();
  testAutomaticHoldNeedsDriveCurrent();
  testNoReleaseWhenInactive();
  testModeAllowedWindow();
  testEncodeHelpers();
  if (failures == 0) printf("cruise_control: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
