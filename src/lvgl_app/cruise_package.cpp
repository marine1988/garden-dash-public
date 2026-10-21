#include "cruise_package.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {

bool tokenIs(const char *token, const char *text) { return strcmp(token, text) == 0; }

uint16_t scaledNonNegative(float value, float scale) {
  if (!isfinite(value) || value <= 0.0F) return 0;
  const float scaled = value * scale;
  if (scaled >= 65535.0F) return 65535;
  return (uint16_t)lroundf(scaled);
}

void copyToken(char *out, size_t capacity, const char *token) {
  if (capacity == 0) return;
  if (!token) token = "";
  size_t length = strlen(token);
  if (length >= capacity) length = capacity - 1;
  memcpy(out, token, length);
  out[length] = '\0';
}

}  // namespace

bool cruisePackageParseSettings(const char *line, CruisePackageSnapshot &snapshot) {
  if (!line) return false;
  char enabled[8] = {};
  char legal[8] = {};
  char debug[8] = {};
  char beeps[8] = {};
  float hold = 0.0F;
  float deadband = 0.0F;
  float minSpeed = 0.0F;
  float maxSpeed = 0.0F;
  if (sscanf(line, "cruise %7s %f %f %f %f %7s %7s %7s", enabled, &hold, &deadband, &minSpeed,
             &maxSpeed, legal, debug, beeps) != 8)
    return false;
  snapshot.enabled = tokenIs(enabled, "true");
  snapshot.legalEnabled = tokenIs(legal, "true");
  snapshot.debugEnabled = tokenIs(debug, "true");
  snapshot.beepsEnabled = tokenIs(beeps, "true");
  snapshot.holdDeciS = scaledNonNegative(hold, 10.0F);
  snapshot.deadbandHundredths = scaledNonNegative(deadband, 100.0F);
  snapshot.minSpeedKmh = scaledNonNegative(minSpeed, 1.0F);
  snapshot.maxSpeedKmh = scaledNonNegative(maxSpeed, 1.0F);
  snapshot.available = true;
  return true;
}

bool cruisePackageParseState(const char *line, CruisePackageSnapshot &snapshot) {
  if (!line) return false;
  char state[16] = {};
  char cancel[20] = {};
  char legal[16] = {};
  if (sscanf(line, "state %15s %19s %15s", state, cancel, legal) != 3) return false;

  CruisePackageState parsed = CRUISE_PACKAGE_OFF;
  if (tokenIs(state, "off")) parsed = CRUISE_PACKAGE_OFF;
  else if (tokenIs(state, "engaging")) parsed = CRUISE_PACKAGE_ENGAGING;
  else if (tokenIs(state, "on")) parsed = CRUISE_PACKAGE_ON;
  else if (tokenIs(state, "cancelling")) parsed = CRUISE_PACKAGE_CANCELLING;
  else return false;

  snapshot.state = parsed;
  copyToken(snapshot.lastCancel, sizeof(snapshot.lastCancel), cancel);
  snapshot.legalLocked = tokenIs(legal, "locked");
  snapshot.available = true;
  return true;
}

bool cruisePackageParseLine(const char *line, CruisePackageSnapshot &snapshot) {
  if (!line) return false;
  if (strncmp(line, "cruise ", 7) == 0) return cruisePackageParseSettings(line, snapshot);
  if (strncmp(line, "state ", 6) == 0) return cruisePackageParseState(line, snapshot);
  return false;
}

const char *cruisePackageStateName(CruisePackageState state) {
  switch (state) {
    case CRUISE_PACKAGE_ENGAGING: return "ENGAGING";
    case CRUISE_PACKAGE_ON: return "ON";
    case CRUISE_PACKAGE_CANCELLING: return "CANCELLING";
    case CRUISE_PACKAGE_OFF: return "OFF";
    default: return "UNAVAILABLE";
  }
}

const char *cruisePackageCancelName(const CruisePackageSnapshot &snapshot) {
  return snapshot.lastCancel[0] ? snapshot.lastCancel : "none";
}
