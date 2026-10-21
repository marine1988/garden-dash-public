#include "cruise_control.h"

#include <math.h>

bool cruiseModeAllowed(const CruiseSettings &settings, uint8_t mode) {
  // A controller that reports no mode (VESC standard telemetry answers 0) must
  // not be locked out, or cruise would refuse to engage on the most common
  // link. Only a reported mode the rider excluded — or an empty allow list — is
  // refused.
  if (mode < CRUISE_MODE_ECO || mode > CRUISE_MODE_SPORT) return true;
  return (settings.allowedModes & (uint8_t)(1u << (mode - 1))) != 0;
}

uint8_t cruiseHoldProgress(const CruiseRuntime &rt, const CruiseSettings &settings, uint32_t nowMs) {
  if (rt.state != CRUISE_STATE_HOLDING || rt.holdSinceMs == 0) return 0;
  const uint32_t holdMs = (uint32_t)(settings.holdTimeDeciS ? settings.holdTimeDeciS : 1) * 100UL;
  const uint32_t elapsed = nowMs - rt.holdSinceMs;
  if (elapsed >= holdMs) return 100;
  return (uint8_t)(elapsed * 100UL / holdMs);
}

CruiseOutput cruiseControlStep(CruiseRuntime &rt, const CruiseSettings &settings, const CruiseInputs &in,
                               bool requestEngage, bool requestDisengage, uint32_t nowMs) {
  CruiseOutput out = {};
  out.state = rt.state;
  out.outcome = CRUISE_OUTCOME_NONE;

  const bool wasActive = rt.state == CRUISE_STATE_ACTIVE;
  const bool braking = in.avgInputCurrent < kCruiseBrakeRegenAmps;
  const auto resetHold = [&]() {
    rt.holdSinceMs = 0;
    rt.holdSpeedKmh = 0.0F;
  };

  // Hands a live cruise back explicitly: the override is cleared and the ADC
  // application takes the throttle over again, instead of the controller
  // holding the setpoint until its own command timeout expires.
  const auto release = [&](CruiseOutcome why) {
    out.send = true;
    out.lisp = kCruiseLispRelease;
    rt.state = CRUISE_STATE_OFF;
    rt.targetKmh = 0;
    rt.rpmPerKmh = 0.0F;
    resetHold();
    out.state = rt.state;
    out.outcome = why;
  };

  // A fault, a dead link, a disabled setting or the rider's own cancel always
  // wins, over a fresh hold just as much as over a running cruise.
  if (!settings.enabled || requestDisengage || !in.linkLive || in.faultCode != 0) {
    if (wasActive) {
      release(!settings.enabled ? CRUISE_OUTCOME_RELEASED_DISABLED
              : requestDisengage ? CRUISE_OUTCOME_RELEASED_CANCEL
              : !in.linkLive    ? CRUISE_OUTCOME_RELEASED_LINK
                                : CRUISE_OUTCOME_RELEASED_FAULT);
    } else {
      rt.state = CRUISE_STATE_OFF;
      rt.targetKmh = 0;
      rt.rpmPerKmh = 0.0F;
      resetHold();
      out.state = rt.state;
    }
    return out;
  }

  if (wasActive) {
    // The rider is on the brakes: a regenerative (negative battery-side)
    // current while cruising means braking, so give the motor back at once
    // rather than fighting the brake with the speed loop.
    if (braking) {
      release(CRUISE_OUTCOME_RELEASED_BRAKE);
      return out;
    }

    // Below the configured floor, or so far above the ceiling that the rider is
    // clearly asking for more than cruise, hand control back first. A stop is
    // "speed low", not just a moved throttle.
    if (in.speedKmh < (float)settings.minSpeedKmh ||
        in.speedKmh > (float)settings.maxSpeedKmh + kCruiseOverspeedMarginKmh) {
      release(CRUISE_OUTCOME_RELEASED_SPEED);
      return out;
    }

    // The rider has taken over: a real throttle pull or a coast moves speed
    // away from the captured target. The display cannot read the throttle pin,
    // so this is its equivalent of the package's "throttle moved".
    if (fabsf(in.speedKmh - (float)rt.targetKmh) > kCruiseOverspeedMarginKmh) {
      release(CRUISE_OUTCOME_RELEASED_CANCEL);
      return out;
    }

    // Nothing to send: the ADC application holds the setpoint every cycle.
    out.state = rt.state;
    return out;
  }

  // Manual engage remains available to host tests and older callers. The
  // display itself never uses it: the cruise page has no button.
  if (requestEngage) {
    if (braking) {
      rt.state = CRUISE_STATE_OFF;
      rt.targetKmh = 0;
      rt.rpmPerKmh = 0.0F;
      resetHold();
      out.state = rt.state;
      return out;
    }
    const bool fastEnough = in.speedKmh >= (float)settings.minSpeedKmh;
    const bool modeOk = cruiseModeAllowed(settings, in.rideMode);
    const bool convertible = in.rpmPerKmh > 0.0F;
    if (fastEnough && modeOk && convertible) {
      float target = in.speedKmh;
      if (target > (float)settings.maxSpeedKmh) target = (float)settings.maxSpeedKmh;
      if (target < (float)settings.minSpeedKmh) target = (float)settings.minSpeedKmh;
      rt.state = CRUISE_STATE_ACTIVE;
      rt.targetKmh = (uint16_t)lroundf(target);
      rt.rpmPerKmh = in.rpmPerKmh;
      resetHold();
      out.send = true;
      out.lisp = kCruiseLispEngage;
      out.state = rt.state;
      out.outcome = CRUISE_OUTCOME_ENGAGED;
      return out;
    }
    rt.state = CRUISE_STATE_BLOCKED;
    out.state = rt.state;
    out.outcome = !fastEnough ? CRUISE_OUTCOME_BLOCKED_SLOW
                  : !modeOk   ? CRUISE_OUTCOME_BLOCKED_MODE
                              : CRUISE_OUTCOME_BLOCKED_GEARING;
    return out;
  }

  // The scooter has to be moving inside the window and still driving. A coast
  // or regen produces no positive battery current, so it cannot start a hold.
  if (in.speedKmh < (float)settings.minSpeedKmh || in.speedKmh > (float)settings.maxSpeedKmh ||
      in.rpmPerKmh <= 0.0F || braking || in.avgInputCurrent < kCruiseMinDriveAmps) {
    rt.state = CRUISE_STATE_OFF;
    rt.targetKmh = 0;
    rt.rpmPerKmh = 0.0F;
    resetHold();
    out.state = rt.state;
    return out;
  }

  // Start a new window when speed was not already steady, or when the rider
  // moved outside the deadband. The window follows the current speed while it
  // is stable, so display rounding by one km/h does not restart it.
  if (rt.holdSinceMs == 0 || fabsf(in.speedKmh - rt.holdSpeedKmh) > kCruiseSteadyDeadbandKmh) {
    rt.holdSinceMs = nowMs;
    rt.holdSpeedKmh = in.speedKmh;
    rt.state = CRUISE_STATE_HOLDING;
    out.state = rt.state;
    return out;
  }

  const uint32_t holdMs = (uint32_t)(settings.holdTimeDeciS ? settings.holdTimeDeciS : 1) * 100UL;
  if ((uint32_t)(nowMs - rt.holdSinceMs) < holdMs) {
    rt.state = CRUISE_STATE_HOLDING;
    out.state = rt.state;
    return out;
  }

  float target = in.speedKmh;
  if (target > (float)settings.maxSpeedKmh) target = (float)settings.maxSpeedKmh;
  if (target < (float)settings.minSpeedKmh) target = (float)settings.minSpeedKmh;
  rt.state = CRUISE_STATE_ACTIVE;
  rt.targetKmh = (uint16_t)lroundf(target);
  rt.rpmPerKmh = in.rpmPerKmh;
  resetHold();
  out.send = true;
  out.lisp = kCruiseLispEngage;
  out.state = rt.state;
  out.outcome = CRUISE_OUTCOME_ENGAGED;
  return out;
}
