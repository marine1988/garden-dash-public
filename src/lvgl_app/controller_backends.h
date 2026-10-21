#pragma once

#include "controller_manager.h"

void vescUartBackendBegin();
void vescBleBackendBegin();
void vescBackendStop();
ControllerPollResult vescBackendPoll(ControllerSample &out);

// Cruise control, driven from the telemetry task because it owns the UART/BLE
// transport. The UI only leaves a request; the task applies it on its next poll
// and publishes the resulting state and outcome back for the UI to read.
// A request is consumed once, so calling engage twice engages once.
//
// Only the VESC backends implement these; the FarDriver link has no equivalent
// command, and controllerCapabilities().supportsCruiseControl is false there.
void vescCruiseRequestEngage();
void vescCruiseRequestDisengage();
// False when cruise is idle. `targetKmh` is written only while active.
bool vescCruiseActive(uint16_t &targetKmh);
// The last outcome the state machine produced, and the state it left cruise in.
// Reported as an edge: the UI compares against the value it saw before, so a
// message is shown once per transition. Writes both values under one lock.
void vescCruiseOutcome(CruiseOutcome &outcome, CruiseState &state);
// The current state, including HOLDING while the steady-speed timer runs.
void vescCruiseState(CruiseState &state);
uint8_t vescCruiseHoldProgress(uint32_t nowMs);

// Profile writes, driven from the telemetry task for the same reason. The UI
// leaves the slot number; the task snapshots it, writes the whole profile as
// one LispBM expression (conf-set applies in RAM, so nothing reaches the
// controller's flash), then reads the limits back and reports whether they
// match. The UI polls vescProfileWriteReportGet().
void vescProfileRequestWrite(uint8_t index);
VescProfileWriteReport vescProfileWriteReportGet();
// Copies the last controller read-back under the same lock the telemetry task
// writes it with. False when no valid reply has been seen for this link.
bool vescProfileReadBackCopy(VescMcconfTemp &out);

void farDriverBackendBegin();
void farDriverBackendStop();
ControllerPollResult farDriverBackendPoll(ControllerSample &out);

