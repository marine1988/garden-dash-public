# Fake VESC test sender

A minimal lab application that impersonates a VESC over Bluetooth, so the
dashboard's link handling can be exercised without a vehicle. Flash **one spare
CYD** with the sender, then use another CYD running the ordinary dashboard as
the receiver. It is a separate TFT_eSPI application, not an LVGL dashboard theme.

The sender answers requests; it never speaks first, matching a real VESC. Its
controls are about *what it answers* rather than how often it talks. It
exercises the protocol assumptions in the current firmware, not a real
controller's behaviour. Capture replay, malformed frames and fragmentation are
outside it.

## Flashing and restoring

| | |
| --- | --- |
| Upload | `scripts\upload_vesc_test.bat` |
| Build only | `--build-only`, or `pio run -e vesc_test` |
| Environment | `vesc_test` |

With one USB serial device the script selects that port; with several it asks
which is the spare CYD. To select explicitly, pass the port:
`scripts\upload_vesc_test.bat COM4`. Use the board's USB data connector and a
data cable. The script never builds the default dashboard environment, and
leaves the result visible until a keypress.

Restore the normal dashboard with `scripts\upload_firmware_usb.bat COM4`. The
sender uses the same partition table, does not erase the whole flash, and writes
only its own inversion preference.

## Controls

- **Mode** (bottom of screen): switch between **Ride Sequence** (default) and
  **Stationary**. Switching mode restarts the sequence. New connections reset
  its time.
- **Disconnect:** drop the current BLE client and resume advertising; reconnect
  from the receiver.
- **Invert Colors:** toggle panel inversion and save it in the sender's own
  settings namespace (`vesc-test`). Initial detection matches the dashboard's
  panel readback heuristic. This does not change the dashboard's stored panel
  profile.
- **BOOT:** short press/release toggles transmission; hold for 1.2 seconds and
  release to invert. Hold only after boot; holding during reset enters the
  bootloader.
- **Serial at 115200 baud:** `d` disconnects, `i` toggles inversion, `m`
  switches mode, `s` toggles transmission, `v` toggles setup values and `c`
  clears the shadow profile and cruise state. Connection and subscription events
  are logged.

Touch uses the existing dashboard calibration, read-only, if available;
otherwise the dashboard's factory mapping. BOOT and serial remain usable if
touch needs calibration. The screen is a fixed English 320x240 diagnostic
interface.

## Testing a VESC Bluetooth link

1. Power the sender. On the receiver choose VESC and Bluetooth in the controller
   setup, then scan. Select the `VESC TEST xxxx` entry.
2. The sender shows **CONNECTED / ANSWERING** once the receiver subscribes. The
   request counter should climb immediately: the dashboard polls continuously.
3. Tap **STOP ANSWERING** to make the controller go silent while the BLE link
   stays up. This is the "connected but not talking" case, and the dashboard
   should report the link as lost rather than freezing on stale values.
4. Tap **SETUP VALS: OFF** to test the eRPM fallback.

## Its own controls

- **Start/Stop answering:** answer or ignore every request. The BLE link stays
  connected either way, so this isolates a silent controller from a dropped
  link.
- **Setup values ON/OFF:** answer or ignore `COMM_GET_VALUES_SETUP`. Real VESC
  firmwares vary in whether they provide it, and the dashboard falls back to
  eRPM and the configured wheel size when it goes unanswered. Turning it off is
  how to test that fallback without finding an old VESC. The dashboard probes
  once and needs ten consecutive failures before it gives up, so allow a few
  seconds for the switch to take effect.
- **Serial `c`:** clear the shadow profile, the ADC detach flag and any cruise
  hold, as a fresh session or power cycle would. This is the way back to a known
  controller state without reconnecting.
- **Screen status:** below the live sample the sender shows `CRUISE ON <eRPM>`
  in green while the ADC hold is active, or `CRUISE OFF` in yellow. Two small
  lines follow with the profile the fake is applying: maximum eRPM, acceleration
  and brake scales in percent, field weakening in A, motor current in A and
  battery current in A. They show what the LispBM expressions actually changed,
  not what the dashboard asked for, so a dropped or rejected write stays
  visible.

Switching mode also resets the totals, but leaves the profile and cruise hold
in place so they can be exercised across a sequence restart. A new connection or
subscription clears everything (profile, ADC state and cruise), exactly like a
controller that rebooted.

## What the replies contain

Requests and replies use the VESC UART framing the dashboard already speaks:
`0x02 | length | payload | crc_hi | crc_lo | 0x03`, with CRC-16/XMODEM over the
payload. Replies are chunked into 20-byte notifications, so they arrive intact
at the smallest usable MTU; the receiver reassembles the byte stream exactly as
it would from a UART. `COMM_FORWARD_CAN` requests are unwrapped first: the inner
command starts at `payload + 2`, and only that inner payload is parsed, so a
CAN-wrapped profile write applies the same expression as a direct one.

Five commands are answered — `COMM_GET_VALUES` (4), `COMM_GET_VALUES_SETUP`
(47), `COMM_FW_VERSION` (0, reported as 6.2), `COMM_GET_MCCONF_TEMP` (91) and
`COMM_LISP_REPL_CMD` (138). Anything else is counted as ignored and left
unanswered, which is also what a real VESC does for commands it does not
implement.

### Profiles and cruise over LispBM

The dashboard writes a ride profile as one `{...}` block of
`(conf-set 'key value)` forms. The fake evaluates the six fields the display
owns — `l-max-erpm`, `l-current-max-scale`, `l-current-min-scale`,
`foc-fw-current-max`, `l-current-max` and `l-in-current-max` — keeps them in a
shadow store, and answers `OK`. An unknown key, an unknown command, a malformed
form or a non-finite number answers `ERR` and changes nothing, because the whole
expression is staged before it is committed. This matches the controller's
behaviour closely enough to catch a bad profile expression before a real VESC
sees it. A profile write is about 200 bytes, so the sender's request buffer is
512 bytes; garbage before a frame is still rejected and the parser resynchronises
on the next `0x02` instead of dropping a valid packet.

`COMM_GET_MCCONF_TEMP` reports the shadow back in the fields a real controller
has: current scales, eRPM limits, duty limits, watt limits and input-current
limits. `foc-fw-current-max` and `l-current-max` have no read-back field, which
is why they appear only on the sender's screen.

Cruise uses the ADC application's own hold, exactly as the dashboard sends it:

- `(progn (app-adc-detach 2 1) (app-adc-override 3 1))` engages. The fake notes
  the button detach, captures the eRPM and speed at that instant, and reports
  them from then on in both `COMM_GET_VALUES` and `COMM_GET_VALUES_SETUP`.
- `(app-adc-override 3 0)` releases. Both replies return to the ride-sequence
  values.

Re-sending engage while the hold is active keeps the original setpoint rather
than moving it, so a retried command cannot make the fake drift.

**Stationary** represents 52.0 V, 0 A, 0 eRPM, 30 C controller and 35 C motor
temperature, no fault. Zero eRPM is intentional.

**Ride Sequence** repeats every 60 seconds, interpolating between these points:

| Time | Phase | eRPM | Voltage | Input current | Speed | FET / motor |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 0–5 s | Idle | 0 | 52.0 V | 0 A | 0 km/h | 30 / 35 C |
| 15 s | Accelerating | 4,200 | 50.0 V | 18 A | 25 km/h | 34 / 42 C |
| 25 s | Peak | 8,400 | 49.0 V | 44 A | 50 km/h | 42 / 55 C |
| 35 s | Cruise | 6,300 | 51.0 V | 13 A | 37 km/h | 45 / 60 C |
| 40 s | Coast | 5,600 | 52.0 V | 0 A | 33 km/h | 42 / 57 C |
| 50 s | Regen | 1,400 | 53.0 V | -7 A | 8 km/h | 37 / 48 C |
| 55–60 s | Stopped | 0 | 52.0 V | 0 A | 0 km/h | 30 / 35 C |

Charge, energy, distance and the tachometer are **not** interpolated: they are
integrated from the current sample and count up the way a real controller's do,
so the receiver's trip and battery arithmetic sees a plausible series instead of
a value that jumps backwards every cycle. They reset on a new connection and on
a mode change.

The two replies are kept consistent with each other on purpose. The dashboard
cross-checks voltage and FET temperature between `GET_VALUES` and
`GET_VALUES_SETUP` before it will trust setup values, and abandons them for the
rest of the session if they disagree — so a sender that got this wrong would
silently test only the eRPM fallback.
`tools/lvgl_native_preview/vesc_sender_test.cpp` asserts the agreement, along
with decoding every frame of two full ride cycles with the firmware's own
parsers. `tools/lvgl_native_preview/vesc_emulator_test.cpp` covers the profile
and cruise path: it builds the Eco, Drive and Sport expressions with the
dashboard's own `vescProfileLisp`, applies them to the fake controller (directly
and through a `COMM_FORWARD_CAN` frame), reads `COMM_GET_MCCONF_TEMP` back with
`vescParseMcconfTemp`, and checks that the engage/release expressions hold and
release the captured eRPM.
