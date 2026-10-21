# Controller backend interface

The display drives one controller family: VESC. It can reach it over a wired
UART or a Bluetooth LE UART bridge, so the controller boundary still has two
transport records behind one manager. The interface lives in
`controller_manager.h`, the hardware adapters in `controller_backends.cpp`, and
the registry and runtime manager in `controller_manager.cpp`.

This document is the contract for the boundary. It exists so a theme, a screen
or a settings page never has to know which transport owns telemetry.

## What already works

`src/lvgl_app/dashboards.cpp` contains twelve themes without a transport branch.
`screens.cpp` copies one `ControllerSnapshot` for a frame, then supplies its
values and field masks to the dashboard. Themes format missing data from the
semantic availability mask rather than knowing why a value is absent.

`main_lvgl.cpp` only starts the manager. It no longer selects a transport or
owns a telemetry task.

## Four things cross the boundary

- **telemetry** — what the vehicle is doing, in one canonical form, with
  availability;
- **capabilities** — what this link can do, so the UI can hide rather than
  invent;
- **link control** — device selection and connection state;
- **lifecycle** — start, stop, reset, and yielding the radio to a firmware
  update or companion session.

## Telemetry: backend samples, one published snapshot

The backend publishes one `ControllerSample`; the manager applies the shared
battery model, supplies staleness/link state, feeds the ride logger, and
atomically publishes the final `ControllerSnapshot`. Battery persistence,
logging policy and the snapshot lock stay out of every transport
implementation.

The masks describe semantic values, not struct members. `tripKm` and battery
state of charge occur in both structs but each gets one bit. A fault bit
distinguishes a genuine zero (no fault) from a link that cannot report the
field at all.

```c
enum TelemetryField : uint32_t {
  TELEMETRY_FIELD_SPEED              = 1UL << 0,
  TELEMETRY_FIELD_POWER              = 1UL << 1,
  TELEMETRY_FIELD_VOLTAGE            = 1UL << 2,
  TELEMETRY_FIELD_CURRENT            = 1UL << 3,
  TELEMETRY_FIELD_MOTOR_TEMP         = 1UL << 4,
  TELEMETRY_FIELD_ESC_TEMP           = 1UL << 5,
  TELEMETRY_FIELD_TRIP_DISTANCE      = 1UL << 6,
  TELEMETRY_FIELD_ODOMETER           = 1UL << 7,
  TELEMETRY_FIELD_AVG_SPEED          = 1UL << 8,
  TELEMETRY_FIELD_UPTIME             = 1UL << 9,
  TELEMETRY_FIELD_BATTERY_SOC        = 1UL << 10,
  TELEMETRY_FIELD_TRIP_ENERGY        = 1UL << 11,
  TELEMETRY_FIELD_REGEN_ENERGY       = 1UL << 12,
  TELEMETRY_FIELD_RIDE_EFFICIENCY    = 1UL << 13,
  TELEMETRY_FIELD_LIFETIME_ENERGY    = 1UL << 14,
  TELEMETRY_FIELD_LIFETIME_DISTANCE  = 1UL << 15,
  TELEMETRY_FIELD_LIFETIME_EFFICIENCY = 1UL << 16,
  TELEMETRY_FIELD_RANGE              = 1UL << 17,
  TELEMETRY_FIELD_EQUIVALENT_CYCLES  = 1UL << 18,
  TELEMETRY_FIELD_PACK_RESISTANCE    = 1UL << 19,
  TELEMETRY_FIELD_LEARNED_CAPACITY   = 1UL << 20,
  TELEMETRY_FIELD_LEARNED_SAMPLES    = 1UL << 21,
  TELEMETRY_FIELD_FAULT              = 1UL << 22,
  TELEMETRY_FIELD_RIDE_MODE          = 1UL << 23,
  TELEMETRY_FIELD_MOTOR_CURRENT      = 1UL << 24,
  TELEMETRY_FIELD_DUTY               = 1UL << 25,
  TELEMETRY_FIELD_PHASE_VOLTAGE      = 1UL << 26,
};

using TelemetryFieldMask = uint32_t;

inline bool telemetryHas(TelemetryFieldMask mask, TelemetryField field) {
  return (mask & static_cast<TelemetryFieldMask>(field)) != 0;
}
```

Optional absolute counters let the shared battery model bank deltas:

```c
struct ControllerEnergyCounters {
  bool available;
  bool rebaseDistance;  // conversion source changed; move, do not bank, baseline
  float ampHours;
  float ampHoursCharged;
  float wattHours;
  float wattHoursCharged;
  float odometerKm;
  float voltage;
  float current;
};

struct ControllerSample {
  uint32_t sampledAtMs;
  DashboardValues values;
  ControllerEnergyCounters energy;
  TelemetryFieldMask available;
  TelemetryFieldMask derived;  // always a subset of `available`
  uint8_t faultCode;
  uint8_t rideMode;
  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  bool firmwareKnown;
};

struct ControllerSnapshot {
  uint32_t sampledAtMs;
  TelemetryLink link;
  DashboardValues values;
  BatteryStats battery;
  TelemetryFieldMask available;
  TelemetryFieldMask derived;
  uint8_t faultCode;
  uint8_t rideMode;
  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  bool firmwareKnown;
};
```

The semantic set uses twenty-seven bits, so the masks are `uint32_t`. The
manager forces `derived & ~available == 0` before publishing.

### Why availability is not optional

`DashboardValues` by itself has no way to say a field is unavailable. Every
member is a bare number, so a controller that cannot report one has to publish
something, and whatever it publishes is indistinguishable from a real reading.
The VESC UART and BLE parsers fill only what the reply actually contained and
leave the matching bits clear; the dashboard then shows a dash or a "not
recorded" chart instead of a fabricated zero.

`BatteryStats` solved this in the same codebase, using negative values for
"cannot be known" and zero for "not yet", documented per field. The masks
generalise that rather than replacing it.

### Why `derived` is a separate mask

Some values are computed on the display rather than reported by the controller.
Trip distance and road speed can come from the controller's own counters, or be
integrated from eRPM and the configured wheel/pole/gearing figures. Both are
legitimate, both land in the same struct, and they are not the same quality of
number.

Whether the VESC reports its own speed is **negotiated at runtime**, not a
property of the backend: the setup-values probe is cross-checked against the
ordinary poll and disables itself for the session on disagreement, falling back
to eRPM. The `derived` bit lets the UI and the ride log know which happened.

## Capabilities: what the controller can do

```c
struct ControllerCapabilities {
  bool showsDeviceList;      // Bluetooth scan/pick UI
  bool usesWiredLink;        // UART baud field applies
  bool hasCanTarget;         // CAN id field applies
  bool reportsRideMode;      // standard VESC telemetry: false
  bool supportsCruiseControl;
  bool supportsProfileWrite;
};
```

`showsDeviceList` is about pairing, not bonding: the backends save an address
and reconnect to it, and the flag decides whether the wizard shows a device
list. The wired VESC record has no list, no scan and no connect work.

`supportsCruiseControl` and `supportsProfileWrite` are true for both VESC
transports. They are still capability fields so a future link cannot inherit a
command path it does not implement.

## Link control: shared types

Both transports fill the same `ControllerLinkStatus` and
`ControllerLinkDevice` structures. The Bluetooth backend supplies the device
list, saved address and link error; the UART backend reports connected/idle from
the telemetry snapshot and leaves the device list empty.

```c
constexpr uint8_t CONTROLLER_LINK_MAX_DEVICES = 12;
constexpr uint8_t CONTROLLER_ADDRESS_CHARS = 20;
constexpr uint8_t CONTROLLER_NAME_CHARS = 28;

enum ControllerLinkState : uint8_t {
  CONTROLLER_LINK_IDLE,
  CONTROLLER_LINK_SCANNING,
  CONTROLLER_LINK_CONNECTING,
  CONTROLLER_LINK_DISCOVERING,
  CONTROLLER_LINK_CONNECTED,
  CONTROLLER_LINK_FAILED,
};

enum ControllerLinkError : uint8_t {
  CONTROLLER_LINK_ERR_NONE,
  CONTROLLER_LINK_ERR_SCAN_FAILED,
  CONTROLLER_LINK_ERR_DEVICE_GONE,
  CONTROLLER_LINK_ERR_CONNECT_FAILED,
  CONTROLLER_LINK_ERR_SERVICE_MISSING,
  CONTROLLER_LINK_ERR_SUBSCRIBE_FAILED,
  CONTROLLER_LINK_ERR_WORKER_FAILED,
  CONTROLLER_LINK_ERR_BUSY,
};
```

## The backend record

The records are flat: VESC over UART and VESC over Bluetooth. A wired backend
needs no special case in the UI; `showsDeviceList` is false, its scan and
connect functions do nothing, and the setup page skips discovery.

```c
// Stable and persisted. Never renumber; never store a registry array index,
// which changes whenever a backend is added or reordered. Value 3 was the
// retired FarDriver BLE selection and is migrated to VESC BLE on load.
enum ControllerBackendId : uint8_t {
  CONTROLLER_ID_VESC_UART = 1,
  CONTROLLER_ID_VESC_BLE = 2,
};

enum ControllerTransport : uint8_t {
  CONTROLLER_TRANSPORT_UART,
  CONTROLLER_TRANSPORT_BLE,
};

enum ControllerPollResult : uint8_t {
  CONTROLLER_POLL_NO_SAMPLE,  // normal while disconnected or between frames
  CONTROLLER_POLL_UPDATED,    // `out` contains a complete coherent sample
  CONTROLLER_POLL_FAILED,     // backend cannot continue without restart
};

struct ControllerBackend {
  ControllerBackendId id;
  const char *name;            // "VESC"
  ControllerTransport transport;
  ControllerCapabilities caps;

  void (*begin)();
  void (*stop)();
  void (*resetSettings)();
  bool (*quiesceForFirmwareUpdate)();
  void (*startScan)();
  void (*connectDevice)(uint8_t index);
  void (*disconnect)();
  void (*reconnectSaved)();
  ControllerLinkStatus (*linkStatus)();
  ControllerPollResult (*poll)(ControllerSample &out);
  uint16_t pollIntervalMs;
  uint16_t taskStackBytes;
};
```

## Lifecycle and radio hand-off

`controllerManagerBegin()` starts one telemetry task. That task owns polling,
snapshot publication, reconnect attempts and backend switches.

A Bluetooth controller and the companion or firmware-update roles all need the
single NimBLE client slot. The manager asks the active backend to quiesce before
another role claims the radio, and resumes it after the role finishes. Setup
screens that scan or deliberately disconnect a controller set
`controllerManagerSetSetupOpen(true)` so the periodic reconnect does not fight
the wizard.

The selection is persisted as the backend id in the `app` settings namespace.
A stored value of 3 from the retired FarDriver selection is migrated to VESC
BLE at load; any other unknown value falls back to VESC UART.

## Ride logs carry the same masks

The record has carried the availability mask since V2, and the log format is
now at **V3**, which added phase current after pack current;
`docs/ride-log-format.md` is the authority on the layout. `kLogVersion`,
`kHeaderBytes` and `kRecordBytes` in `ride_replay_core.h` are shared by the
writer and the reader so the two cannot drift.

The record follows the snapshot rather than leading it: a field is logged
because a backend reports it, and its bit is the same `TelemetryField` bit the
dashboard reads. A ride recorded without phase current charts it as
"Not recorded" instead of as zero. A new field costs a bit, a record slot and a
format version, and nothing else.

## What this is worth

The boundary removes transport and controller branches from themes and screens.
It also gives one place to add staleness, logging, capability gating and radio
hand-off. The cost is one extra copy of the values per frame and a small amount
of pointer indirection in the manager.

When changing the boundary:

1. Keep the semantic masks authoritative. A zero without its bit is padding.
2. Do not add a backend without a stable id and a capability record.
3. Keep blocking work out of critical sections; the snapshot lock is short.
4. Hand the radio over through the manager, not from a screen.
5. Re-render the native previews and check the translated strings when a
   capability changes which controls are visible.
