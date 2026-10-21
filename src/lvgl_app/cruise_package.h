#pragma once

// Parser and snapshot for the settings/state lines emitted by the
// marine1988/vesc_cruise_control LispBM package.
//
// The package owns the cruise: it watches the throttle and brake pins, waits
// for the configured hold time, and drives the ADC application's override. The
// display only queries and displays it; it must never send its own
// app-adc-detach/app-adc-override commands while this package is installed.
//
// The package answers on COMM_CUSTOM_APP_DATA (36) with two line formats:
//
//   cruise <enabled> <hold-s> <deadband-v> <min-kmh> <max-kmh> <legal> <debug> <beeps>
//   state <off|engaging|on|cancelling> <last-cancel> <locked|unlocked>

#include <stddef.h>
#include <stdint.h>

enum CruisePackageState : uint8_t {
  CRUISE_PACKAGE_UNAVAILABLE = 0,
  CRUISE_PACKAGE_OFF,
  CRUISE_PACKAGE_ENGAGING,
  CRUISE_PACKAGE_ON,
  CRUISE_PACKAGE_CANCELLING,
};

struct CruisePackageSnapshot {
  bool available;
  CruisePackageState state;
  char lastCancel[20];
  bool legalLocked;

  bool enabled;
  uint16_t holdDeciS;           // 0.1 s units
  uint16_t deadbandHundredths;  // 0.01 V units
  uint16_t minSpeedKmh;
  uint16_t maxSpeedKmh;
  bool legalEnabled;
  bool debugEnabled;
  bool beepsEnabled;
};

// Parses one line. Returns true when the line matched and the snapshot was
// updated; unknown or malformed lines leave the snapshot untouched.
bool cruisePackageParseState(const char *line, CruisePackageSnapshot &snapshot);
bool cruisePackageParseSettings(const char *line, CruisePackageSnapshot &snapshot);
bool cruisePackageParseLine(const char *line, CruisePackageSnapshot &snapshot);

const char *cruisePackageStateName(CruisePackageState state);
const char *cruisePackageCancelName(const CruisePackageSnapshot &snapshot);
