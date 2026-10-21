// Host test for the vesc_cruise_control package line parser. The package owns
// the cruise; the display only mirrors these two lines.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "cruise_package.h"

namespace {

int failures = 0;

void expect(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    failures++;
  }
}

void testSettings() {
  CruisePackageSnapshot snapshot = {};
  expect(cruisePackageParseSettings("cruise true 5.5 0.12 7.5 42.0 false true true", snapshot),
         "settings line parses");
  expect(snapshot.available, "settings marks the package available");
  expect(snapshot.enabled, "enabled true");
  expect(snapshot.holdDeciS == 55, "hold 5.5 s -> 55 deci-seconds");
  expect(snapshot.deadbandHundredths == 12, "deadband 0.12 V -> 12 hundredths");
  expect(snapshot.minSpeedKmh == 8, "min 7.5 rounds to 8 km/h");
  expect(snapshot.maxSpeedKmh == 42, "max 42 km/h");
  expect(!snapshot.legalEnabled, "legal false");
  expect(snapshot.debugEnabled, "debug true");
  expect(snapshot.beepsEnabled, "beeps true");
}

void testState() {
  CruisePackageSnapshot snapshot = {};
  expect(cruisePackageParseState("state engaging none unlocked", snapshot), "engaging state parses");
  expect(snapshot.state == CRUISE_PACKAGE_ENGAGING, "engaging state mapped");
  expect(std::strcmp(snapshot.lastCancel, "none") == 0, "cancel none");
  expect(cruisePackageParseState("state on brake locked", snapshot), "on state parses");
  expect(snapshot.state == CRUISE_PACKAGE_ON, "on state mapped");
  expect(std::strcmp(snapshot.lastCancel, "brake") == 0, "cancel brake");
  expect(snapshot.legalLocked, "legal locked");
  expect(cruisePackageParseState("state cancelling throttle_moved unlocked", snapshot),
         "cancelling state parses");
  expect(snapshot.state == CRUISE_PACKAGE_CANCELLING, "cancelling state mapped");
  expect(cruisePackageParseState("state off overspeed unlocked", snapshot), "off state parses");
  expect(snapshot.state == CRUISE_PACKAGE_OFF, "off state mapped");
  expect(std::strcmp(snapshot.lastCancel, "overspeed") == 0, "cancel overspeed");
}

void testRejects() {
  CruisePackageSnapshot snapshot = {};
  snapshot.state = CRUISE_PACKAGE_ON;
  snapshot.enabled = true;
  expect(!cruisePackageParseState("state broken none unlocked", snapshot), "unknown state rejected");
  expect(!cruisePackageParseState("nonsense", snapshot), "nonsense rejected");
  expect(!cruisePackageParseSettings("cruise true 5.0", snapshot), "short settings rejected");
  expect(snapshot.state == CRUISE_PACKAGE_ON && snapshot.enabled, "rejected lines leave the snapshot untouched");
}

}  // namespace

int main() {
  testSettings();
  testState();
  testRejects();
  if (failures == 0) std::printf("cruise package: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
