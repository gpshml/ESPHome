# SPD2010 touch port review (reference vs current ESPHome component)

## Executive summary

The current port carries over most low-level register transactions, but there are several integration mismatches that can prevent a successful port even when the touch controller itself is responsive.

Most critical blockers:

1. **Init is gated on `display_ready_`, but there is no code path that calls `notify_display_ready()`**.
2. **`update_touches()` reports touch data twice per loop and uses two different primary-selection strategies**.
3. **Reset/bring-up timing differs from the reference implementation and may be marginal for some boards**.

## What matches the reference implementation

- Register map and command values (`0x0200`, `0x0400`, `0x4600`, `0x5000`, `0x2000`, `0x0003`, `0xFC02`, `0x2600`) match between reference and port.
- HDP decode format (`id`, `x`, `y`, `weight`) and status-driven control flow are broadly aligned.

## Why the current port can fail

### 1) Initialization is permanently blocked in normal use

- In C++, `loop()` immediately returns until `display_ready_` is true.
- `display_ready_` only changes via `notify_display_ready()`.
- There is no caller in the repository that invokes `notify_display_ready()`.

**Impact:** touch init and polling never run, so the component appears dead even if I2C is correct.

### 2) Duplicate touch publishing introduces inconsistent pointer behavior

`update_touches()` currently:

- First selects a "stable by ID" primary and calls `add_raw_touch_position_()`.
- Then again selects a (different) primary by highest weight and calls `add_raw_touch_position_()` a second time before `send_touches_()`.

**Impact:** two conflicting updates in one cycle can cause jitter, phantom movement, or click/release instability in LVGL.

### 3) Reset pulse/timing deviates from known-good reference values

Reference reset sequence uses roughly 50 ms low + 50 ms high. The port uses 20 ms low + 80 ms high and adds display-ready ordering assumptions.

**Impact:** if SPD2010 startup is timing-sensitive on your hardware revision, shortened low pulse can produce intermittent init/probe failure.

### 4) Port introduces extra system-level assumptions not present in reference

The reference reads status every cycle and handles BIOS/CPU transitions directly. The port adds:

- deferred init scheduling,
- repeated ACK probing,
- periodic address scanning,
- optional IRQ path,
- display-ready dependency.

These can be useful, but each is another failure surface when wiring/config is incomplete.

## High-confidence actions to diagnose

1. **Confirm whether init is blocked by display-ready gate**
   - Boot with verbose logs.
   - Check whether you ever see logs from `try_init_()`/"touch initialised".
   - If not, temporarily bypass the gate or force `notify_display_ready()` at startup.

2. **Validate actual I2C responsiveness at 0x53 during runtime**
   - Enable and review probe scan logs.
   - Verify bus pull-ups, speed, and that no expander/reset line is holding controller in reset.

3. **Instrument status path with one-line snapshots**
   - Log `tic_in_bios`, `tic_in_cpu`, `cpu_run`, `pt_exist`, `gesture`, `read_len` every poll for first few seconds.
   - Compare observed state transitions against the reference state machine.

4. **Reduce touch reporting to a single publication per cycle**
   - Keep one deterministic primary selection method (prefer stable-by-ID).
   - Remove second `add_raw_touch_position_()` block.

5. **A/B reset timing against reference**
   - Test 50/50 ms pulse exactly as reference.
   - If needed, add a configurable reset pulse duration option in YAML for board variants.

6. **Add minimum integration test matrix**
   - No-touch idle for 60s (no spurious touches).
   - Single-finger drag (no pointer jumps).
   - Multi-finger contact while tracking one pointer (no oscillation).
   - Warm reboot / deep sleep wake (touch recovers without power-cycle).

## Suggested implementation order

1. Remove/blocker: ensure `notify_display_ready()` is called (or remove the gate).
2. Fix `update_touches()` double-publish bug.
3. Align reset timing with reference.
4. Re-run logs and touch behavior tests.
5. Optional: reintroduce advanced heuristics one-by-one only after stable baseline.
