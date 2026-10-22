#pragma once
#include <stdint.h>

// Display-side cruise control for a VESC controller.
//
// The VESC's ADC application owns the motor: its throttle thread rewrites the
// motor command hundreds of times a second, so a control frame the display sends
// at 10 Hz (COMM_SET_RPM, COMM_SET_CURRENT) is overwritten within one ADC cycle
// and the motor never follows it. The cruise that does hold is the ADC
// application's own: with its buttons detached and the cruise override set, the
// app itself holds the captured eRPM every cycle. The display commands exactly
// that over LispBM -- the same COMM_LISP_REPL_CMD path the profiles already use.
//
// So this module decides *when* cruise is on and hands the controller the right
// LispBM expression through a CruiseOutput; the controller's own application is
// the holder of the setpoint. Nothing here touches Arduino or LVGL, so the host
// tests drive the state machine directly.
//
// Engagement is automatic and time-based, like the scooter cruise packages:
// once the rider has held a steady speed for the configured hold time, the
// display sends the engage expression; there is no engage button. A brake, a
// speed change (throttle moved or coasting), falling below the minimum, a
// fault, a dead link or switching cruise off releases it. The display cannot
// read the throttle pin, so "steady" means speed within a small deadband while
// the controller is still drawing drive current.
//
// Engage sends one expression that detaches the ADC buttons (the throttle stays
// attached) and sets the cruise override; the app then holds the speed the
// rider had. Release clears the override and hands the throttle back untouched.
// The two steps must travel together: the firmware's LispBM REPL accepts only
// one command every 0.5 s, so two packets would drop the second.

// Ride modes the controller can report in telemetry, 1..3, matching the three
// configurable speed modes. 0 means the controller does not report one.
enum CruiseRideMode : uint8_t {
  CRUISE_MODE_NONE = 0,
  CRUISE_MODE_ECO = 1,
  CRUISE_MODE_DRIVE = 2,
  CRUISE_MODE_SPORT = 3,
};

struct CruiseSettings {
  bool enabled;
  uint16_t holdTimeDeciS;       // steady-speed time before automatic engage
  uint16_t minSpeedKmh;         // below this, cruise refuses to engage / releases
  uint16_t maxSpeedKmh;         // ceiling the captured target is clamped to
  uint8_t allowedModes;         // retained for the stored settings; unknown VESC modes are admitted
};

enum CruiseState : uint8_t {
  CRUISE_STATE_OFF,
  CRUISE_STATE_HOLDING,  // speed is steady, waiting for the hold time
  CRUISE_STATE_ARMED,    // hold time elapsed; waiting for the throttle to rest
  CRUISE_STATE_ACTIVE,
  CRUISE_STATE_BLOCKED,  // kept for the existing UI/outcome mapping
};

// The outcome of the cycle, for the UI to report. It is an edge: the step
// returns CRUISE_OUTCOME_NONE on every cycle where nothing changed, and one of
// the others on the cycle a request is granted, refused, or a running cruise is
// handed back.
enum CruiseOutcome : uint8_t {
  CRUISE_OUTCOME_NONE = 0,
  CRUISE_OUTCOME_ENGAGED,
  CRUISE_OUTCOME_FAILED,           // the controller rejected the LispBM command
  CRUISE_OUTCOME_BLOCKED_SLOW,     // refused: under the minimum speed
  CRUISE_OUTCOME_BLOCKED_MODE,     // refused: ride mode excluded by the profile
  CRUISE_OUTCOME_BLOCKED_GEARING,  // refused: wheel size/gearing not set
  CRUISE_OUTCOME_RELEASED_BRAKE,   // the rider braked (regenerative current)
  CRUISE_OUTCOME_RELEASED_SPEED,   // speed fell under the minimum
  CRUISE_OUTCOME_RELEASED_FAULT,   // a controller fault
  CRUISE_OUTCOME_RELEASED_LINK,    // the link went dead
  CRUISE_OUTCOME_RELEASED_CANCEL,  // the rider cancelled
  CRUISE_OUTCOME_RELEASED_DISABLED,
};

// What the caller should send to the controller this cycle.
struct CruiseOutput {
  bool send;         // true when a LispBM expression should go out
  const char *lisp;  // the expression to send (a static string), valid when send
  CruiseState state;
  CruiseOutcome outcome;
};

// Inputs the telemetry task already has for this cycle.
struct CruiseInputs {
  float speedKmh;         // display-side speed, the same figure the dashboard shows
  float rpm;              // controller eRPM, signed
  float rpmPerKmh;        // eRPM per km/h, from the drivetrain setup; <= 0 disables engage
  float avgInputCurrent;  // battery-side current; strongly negative means braking
  uint8_t faultCode;
  uint8_t rideMode;       // 1..3, 0 unknown
  bool linkLive;
};

// Runtime state, owned by the telemetry task.
struct CruiseRuntime {
  CruiseState state;
  uint16_t targetKmh;  // the speed capture shown on the UI; the app holds the eRPM
  float rpmPerKmh;     // kept for the record; not used to re-derive a setpoint
  uint32_t holdSinceMs;  // first tick of the current steady-speed window
  float holdSpeedKmh;    // speed the window is following
  uint32_t activeSinceMs;   // when the ADC hold was requested
  uint32_t speedOutSinceMs; // first tick speed left the target window, 0 if inside
};

// The two LispBM expressions the VESC ADC application understands. Engage keeps
// the throttle attached (mode 2 detaches the buttons only) so the rider can
// still brake with it after a release.
constexpr const char *kCruiseLispEngage = "(progn (app-adc-detach 2 1) (app-adc-override 3 1))";
// Clear the override and reattach the ADC buttons together. Leaving the buttons
// detached after a normal release would ignore a physical cruise button until
// the next engage.
constexpr const char *kCruiseLispRelease = "(progn (app-adc-override 3 0) (app-adc-detach 2 0))";
// If the engage expression is rejected after the detach form ran, give the
// controller's physical buttons back.
constexpr const char *kCruiseLispDetachUndo = "(app-adc-detach 2 0)";

// Battery-side current below this while cruising is read as the rider braking
// regeneratively, and releases the cruise at once. The value is deliberately
// conservative: a downhill coast can feed a little back, but a real brake pulls
// several amps.
constexpr float kCruiseBrakeRegenAmps = -5.0F;

// Speed may wander this much while the hold timer runs. The rider's hand and
// the controller's speed loop both move a little; a real throttle pull or a
// coast is larger and cancels a running cruise.
constexpr float kCruiseSteadyDeadbandKmh = 2.0F;
// Below this battery current the scooter is coasting or regenerating, not
// driving, so the hold window is not allowed to start or continue.
constexpr float kCruiseMinDriveAmps = 0.5F;
// A running cruise cancels if speed climbs this far above the captured target.
constexpr float kCruiseOverspeedMarginKmh = 3.0F;
// The VESC's ADC cruise captures its setpoint when the throttle returns to
// rest. Give that transition time to happen before judging a speed dip as a
// rider takeover; otherwise a normal release looks like a cancel.
constexpr uint32_t kCruiseEngageGraceMs = 1500;
// A real throttle pull or coast keeps speed outside the target window; a
// one-sample dip from a bump or a gear change should not release the cruise.
constexpr uint32_t kCruiseSpeedCancelMs = 1200;

// Applies one cycle and returns what, if anything, to send. Engagement is
// automatic: `requestEngage` is retained for API compatibility but no button
// uses it. `requestDisengage` still lets a caller cancel immediately. Called
// every telemetry poll.
CruiseOutput cruiseControlStep(CruiseRuntime &rt, const CruiseSettings &settings, const CruiseInputs &in,
                               bool requestEngage, bool requestDisengage, uint32_t nowMs);

// How far the current steady-speed window has progressed, 0..100. Zero when
// cruise is not holding. Used by the cruise page to show the countdown.
uint8_t cruiseHoldProgress(const CruiseRuntime &rt, const CruiseSettings &settings, uint32_t nowMs);

// True when the reported ride mode is allowed by the settings. An unknown mode
// (0 or any value outside 1..3) is admitted: the display only refuses a mode
// the rider explicitly excluded, and most controllers report no mode at all.
// An empty allow list admits no reported mode.
bool cruiseModeAllowed(const CruiseSettings &settings, uint8_t mode);
