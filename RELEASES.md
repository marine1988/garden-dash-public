# Releases

Every published Garden-Dash firmware release, oldest first.

The version being worked on is the one in `include/config.h`. Option 5 in
`garden.bat` packages it: **T** makes a test package for your own boards, which
shows as `0.01-test` on the display and changes nothing here; **R** releases it.
A release adds its row to this table in the commit it tags (`v0.01`), then moves
`include/config.h` on to the next version. "Building a release" in
[docs/development.md](docs/development.md) has the details.

The SHA-256 is that of the firmware image, `firmware-v<code>.bin`, the same
value as the `sha256` in its signed manifest. It identifies a firmware file
later: `certutil -hashfile firmware-v1.bin SHA256` on Windows.

Keep the table at the end of this file: the release builder appends to it.

| Version | Code | Date | Firmware SHA-256 |
| --- | --- | --- | --- |
| 0.01 | 1 | 2026-10-07 | `d6375204728dc865df6c5901fb6d2feeb335b3fc27d88a64f673754ea4ad4373` |
| 0.02 | 2 | 2026-10-21 | `a755c280d777d3f20cae2b48f15f3cb4fab43c861a695a82c6756689a9c640b2` |

0.02 was tagged from the source checkout field-tested for profiles. The SHA above
is the default `garden` firmware image. Its signed OTA bundle is still pending
because the offline release key was not available on the build machine; the tag
itself is the source/firmware checkpoint.
