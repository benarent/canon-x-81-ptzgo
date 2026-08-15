# Changelog

Notable changes to this project. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Fixed

- **Safety stop on SIGINT/SIGTERM never fired.** The shutdown handler tried to
  tell a real signal apart from a normal exit via `context.Cause`, but Go's
  signal cause still matches `context.Canceled`, so the handler returned early
  and the camera was left moving on Ctrl-C or `kill`. The handler now uses a
  plain signal channel and always stops the head before exiting.
- **`on` now actually waits for the camera to boot.** Per Programmer's Manual
  §9.3 the power-off status flag clears as soon as Camera ON is accepted;
  readiness is signalled by the extended-status camera-busy flag clearing
  (~15 s). The command previously keyed the wait on an ACK byte the camera
  never sends and polled the wrong flag, so it returned immediately on real
  hardware.
- **Removed ACK/NAK (`06h`/`15h`) handling that is not part of the protocol.**
  Every camera answer is a full `FE 3030 <error> EF` frame; a stray `06h`
  noise byte on the line is no longer misread as a successful answer.
- **Ctrl-C in the interactive shell abandons the current line** and re-prompts
  (standard readline behavior) instead of quitting the shell. Ctrl-D,
  `quit`, or `exit` still leave the shell.
- **Config validation:** `baud_rate` ≤ 0 and `power_on_delay` ≤ 0 in
  `config.json` now fall back to their defaults (9600 bps / 15 s) instead of
  producing undefined behavior.
- **Zoom model gating:** `zoom` positions 129–238 (the digital-zoom range) are
  rejected client-side when the configured model is XU-80.
- On signal exit, the terminal is restored from raw mode before shell history
  is written.

### Changed

- Interactive shell history file renamed from `~/.mycamera_history` to
  `~/.ptz-control_history`. Existing history is not migrated.

### Internal

- Tests answer with genuine `FE`-frames instead of bare ACK bytes, matching
  the wire format of real hardware.
- The 500 ms response deadline is injectable per camera instance and the
  response read loop can no longer hot-spin, making timeout tests fast
  (~20 ms instead of 500 ms each).
