# Cruise control: why it never holds on the scooter

Diagnosis for GD-3. The state machine in `cruise_control.cpp` is not the fault:
all 40 host tests pass, 14 of them on cruise. The cruise fails on the scooter
because of how the display talks to the VESC, not what it decides.

## Summary

The display engages cruise by sending `COMM_SET_RPM` and trusting the VESC's own
speed PID to hold the setpoint (`src/lvgl_app/cruise_control.h:6`). On a VESC
whose throttle is an ordinary ADC input — the normal arrangement on a scooter —
the VESC's ADC application thread rewrites the motor command about every 2 ms
and takes the motor back into current mode, so the display's `SET_RPM` is
overwritten almost as soon as it lands. The motor never follows the cruise
setpoint; it follows the throttle (which is at rest, so nothing happens).

The VESC does have a working cruise — but it lives in the ADC application, not
in `SET_RPM`, and it is switched on over LispBM.

## What the display does

- Engage: `cruiseHoldTimerCb` (`src/lvgl_app/screens.cpp:2934`) calls
  `controllerCruiseEngage()` after the hold time; that sets
  `cruiseRequestPending = 1` (`controller_backends.cpp:552`).
- The telemetry poll drains the request and calls `cruiseControlStep`
  (`controller_backends.cpp:540`), then `vescSendControl(VESC_COMM_SET_RPM, targetRpm)`
  (`controller_backends.cpp:542`).
- The heartbeat re-sends `SET_RPM` every `kCruiseCommandPeriodMs` = 50 ms, and
  the poll itself runs at `VESC_CONNECTED_POLL_MS` = 100 ms (`include/config.h:44`),
  so in practice it re-sends at ~10 Hz.

## What the VESC does with `SET_RPM`

`COMM_SET_RPM` (opcode 8, correct) reaches `mc_interface_set_pid_speed()`
(vedderb/bldc `motor/mc_interface.c`):

```c
void mc_interface_set_pid_speed(float rpm) {
    if (mc_interface_try_input()) {
        return;                     // dropped during detection, fault-ignore or motor lock
    }
    ...
    mcpwm_foc_set_pid_speed(DIR_MULT * rpm);   // sets control_mode = CONTROL_MODE_SPEED
}
```

So a single `SET_RPM` does switch the motor to `CONTROL_MODE_SPEED`. That is the
state the display relies on. The problem is that the display is not the only
writer of that state.

`mc_interface_try_input()` (`motor/mc_interface.c`) does **not** depend on the
throttle; it returns non-zero while the motor is detecting, while a fault-ignore
window is running, while the motor lock is enabled, or before the motor init
has finished. Any command sent in those windows is silently dropped.

## Why it is overwritten: the ADC application owns the motor

`applications/app_adc.c` runs its own thread at `update_rate_hz` (firmware
default 500 Hz, `appconf_default.h:219`; the VESC Tool commonly sets 1000 Hz).
Every iteration, unconditionally, it writes the motor command:

```c
// app_adc.c, end of the thread loop
if (current_mode) {                       // ADC_CTRL_TYPE_CURRENT* == the scooter case
    if (current_mode_brake) {
        mc_interface_set_brake_current_rel(current_rel);
    } else {
        ...
        mc_interface_set_current_rel(current_out);   // <- every ~2 ms
    }
}
```

Each `mc_interface_set_current_rel()` sets `control_mode = CONTROL_MODE_CURRENT`.
At 500 Hz that is a rewrite every ~2 ms, against a display heartbeat of 50 ms —
the display's `SET_RPM` survives for at most one ADC iteration. This is also why
sending `SET_CURRENT` from the display would not work either: the ADC thread
overwrites that too. Nothing the display sends at 10 Hz can out-run the app that
owns the motor.

This is the direct answer to "cruise does not work on the real scooter". It is
also consistent with the host tests passing: they drive the state machine in
isolation, with no ADC application competing for the motor.

## The VESC cruise that does work, and how to reach it

The ADC application has its own cruise: when its `cc_button` is held (throttle
at rest), the app stops commanding throttle current and instead holds the
captured RPM itself, every iteration (`app_adc.c:548`):

```c
if (current_mode && cc_button && fabsf(pwr) < 0.001) {
    static float pid_rpm = 0.0;
    if (!was_pid) { was_pid = true; pid_rpm = rpm_filtered; }
    mc_interface_set_pid_speed(pid_rpm);     // the app is the writer, so nothing overwrites it
    ...
    continue;
}
```

`cc_button` can be a physical button **or** an override driven over LispBM. The
LispBM extension `app-adc-override` (`lispBM/lispif_vesc_extensions.c:1822`,
registered as `"app-adc-override"` at line 6641) calls `app_adc_cc_override()`
(`app_adc.c:154`). Target `3` is cruise:

```
(app-adc-override 3 1)   ; engage the VESC's native cruise
(app-adc-override 3 0)   ; release it
```

Requirements, all read from `app_adc.c`:

- Buttons must be detached so the override is read instead of the pin:
  `(app-adc-detach 2 1)` sets `buttons_detached` and keeps the throttle ADC
  attached (`app_adc.c:1798-1814`, used at `app_adc.c:319`).
- The ADC configuration's cruise-button bit must be enabled, or
  `cc_button` is forced false (`app_adc.c:330-332`, bit 0 of
  `app_conf.adc.buttons`). This is a VESC Tool setting ("Cruise control
  button"), which must be on.
- The throttle must be at rest (`fabsf(pwr) < 0.001`, `app_adc.c:548`), and the
  controller must be in a current-mode ADC control type.
- The held RPM is captured once, at the moment cruise engages; changing the
  setpoint means releasing and re-engaging.

The display already has a working LispBM path: profiles are written as LispBM
expressions over `COMM_LISP_REPL_CMD` (138) in
`vescProfileSendProfile` (`controller_backends.cpp:353`), and GD-2 fixed the
frame buffer (`VESC_MAX_FRAME` = 265) so those expressions fit. The cruise
command above would go out the same way.

## Secondary findings in the display code

These do not cause the failure above, but they make it hard to diagnose in the
field and should be handled with the fix:

1. **`cruiseControlEnabled` is false by default** (`app_logic.cpp:86`) and the
   engage path does not check it: `cruiseHoldTimerCb` shows `CRUISE ENGAGED`
   even when cruise is disabled, while the state machine sends nothing. The
   rider sees a success toast for a no-op. There is no "cruise disabled" notice.
2. **A refused engage is silent.** When `cruiseControlStep` returns
   `CRUISE_STATE_BLOCKED` (below `minSpeedKmh`, or `rpmPerKmh <= 0` because the
   wheel/gearing is unset), no notice is raised and the button keeps reading
   "Ready". The rider has no way to tell "too slow" from "not wired".
3. **The deadband setting is dead code.** `settings.deadbandHundredths` is read
   from the globals (`controller_backends.cpp:524`) and never used in
   `cruiseControlStep`; the host tests do not exercise it either. The card asked
   for the deadband logic to be checked — there is none.
4. **The button does not reflect the active state.** The label is decided at
   screen build (`screens.cpp:6183`). The post-engage `queueRebuild` runs before
   the telemetry poll consumes the request, so the button rebuilds as
   "ENGAGE CRUISE / Ready"; it only changes on the next interaction. Worse, the
   `CLICKED` of the engaging press can arrive after that rebuild destroyed the
   button, leaving `cruiseHoldTriggered` set and swallowing the next tap.
5. **No brake input.** Release is inferred only from speed dropping below
   `minSpeedKmh` (`cruise_control.cpp:73`); the display never reads a brake. This
   is acceptable for a release-on-deceleration design, but it is not a "brake
   check" as the card describes.

## Options for the fix (GD-4)

- **A — Use the VESC's native cruise over LispBM (recommended).** Engage with
  `(app-adc-detach 2 1)` + `(app-adc-override 3 1)`, release with
  `(app-adc-override 3 0)`, sent over the existing `COMM_LISP_REPL_CMD` path.
  This works with the throttle as it is, reuses the proven GD-2 LispBM plumbing,
  and makes the ADC application itself the holder of the setpoint, so nothing
  overwrites it. Needs the cruise-button bit enabled in the VESC ADC config, and
  the display must gate the engage on the throttle being at rest.
- **B — Display-side speed PID over `SET_CURRENT`.** Does **not** work while the
  ADC application is running, because it overwrites `SET_CURRENT` every cycle.
  It would only work if the display detached the ADC entirely
  (`(app-adc-detach 3 1)`) and took over motor control, replicating ramps,
  limits, safe-start and braking. Large and risky.
- **C — Put the VESC in PID mode (`ADC_CTRL_TYPE_PID`).** The throttle then
  commands speed rather than current, but the display's `SET_RPM` is still
  overwritten by the ADC thread, so cruise still fails. Not a fix on its own.

## Verification

- `ctest` in `tools/lvgl_native_preview/build_test`: 100% of 40 tests pass,
  including the 14 cruise cases — the state machine is sound; the defect is in
  the integration with the VESC.
- Firmware behaviour read from vedderb/bldc at master: `motor/mc_interface.c`
  (`mc_interface_set_pid_speed`, `mc_interface_try_input`),
  `applications/app_adc.c` (the ADC thread and its cruise branch),
  `lispBM/lispif_vesc_extensions.c` (`app-adc-override`).

## Resolution (GD-4)

Option A is implemented. `cruise_control.cpp` still decides *when* cruise is on
(the same state machine and safety order), but its output is now the LispBM
expression the ADC application understands, not a periodic `SET_RPM`:

- **Engage** sends one expression, `(progn (app-adc-detach 2 1)
  (app-adc-override 3 1))`. `progn` is one REPL command, which matters because
  the firmware's LispBM REPL accepts only one command every 0.5 s
  (`lispif.c:287`) — two packets would drop the second. Mode 2 of
  `app-adc-detach` detaches the buttons but keeps the throttle attached
  (`lispif_vesc_extensions.c:1807`), and target 3 of `app-adc-override` is the
  cruise button (`app_adc.c:154`, `app_adc.c:319-332`). The ADC application then
  holds the captured eRPM itself (`app_adc.c:548-572`), so nothing overwrites it.
- **Release** sends `(app-adc-override 3 0)`: the throttle is handed back
  untouched.
- The two expressions travel over the existing `COMM_LISP_REPL_CMD` path in
  `controller_backends.cpp`, wrapping them for CAN-forward exactly as a profile
  write does.
- One REPL slot is shared with profile writes. A cruise expression is queued
  when the REPL is still inside its 0.5 s window, and the latest rider action
  replaces a queued one, so a quick cancel cannot be lost behind an engage.
- A repeated engage while cruise is running is ignored rather than re-evaluated.
  The ADC application captures its setpoint on the transition into cruise, so a
  second engage cannot retarget it; treating it as a fresh request could drop
  the local state while the controller still held the old setpoint.
- The dashboard shows the running cruise as a green `CRUISE <target>` chip over
  the title, so the state is visible after leaving the cruise page. It hides on
  release, brake, fault, or link loss.

Companion findings from the diagnosis are fixed:

1. `cruiseControlEnabled == false` no longer shows a false `CRUISE ENGAGED`; the
   engage path reports `CRUISE IS TURNED OFF` (`NOTICE_CRUISE_DISABLED`).
2. A refused engage is no longer silent. The state machine publishes a
   `CruiseOutcome`, and `observeCruiseState()` (the same edge pattern as
   `observeProfileWriteNotices`) raises `RIDE FASTER TO CRUISE`,
   `CRUISE OFF IN THIS MODE`, or `SET WHEEL SIZE FIRST` as appropriate.
3. The dead `deadbandHundredths` setting is removed from the UI, the settings
   store, and the runtime. It never had any effect, so dropping the stored key
   needs no migration.
4. The engage button reflects the live state: a state change rebuilds the cruise
   page on the dashboard tick (outside the press), and a cancel tap clears a
   stale hold flag so the `CLICKED` that follows is not swallowed.
5. Brake check: a regenerative (negative battery-side) current below
   `kCruiseBrakeRegenAmps` while cruising releases the cruise at once. A
   mechanical brake is still covered by the release when speed falls below the
   minimum, and by the tap-to-cancel.

### Requirements that stay on the VESC

The display cannot set these; they are VESC Tool settings:

- **App → ADC → "Cruise control button" must be enabled** (bit 0 of
  `app_conf.adc.buttons`). Its firmware default is 0 (`appconf_default.h:180`),
  and with it off the ADC application forces `cc_button` false
  (`app_adc.c:330`), so cruise never engages. There is no LispBM binding for the
  app ADC configuration, so this cannot be driven from the display.
- **The throttle must be at rest when engaging** (`fabsf(pwr) < 0.001`,
  `app_adc.c:548`). The display cannot read the throttle over standard telemetry,
  so it guides the rider ("Release the throttle, then hold to engage") and the
  controller refuses the engage safely if the throttle is applied.
