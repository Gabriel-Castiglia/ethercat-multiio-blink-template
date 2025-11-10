# EtherCAT Multi-I/O Blink — Didactic Template

## What this is
A single-file, master-agnostic C template that shows how to (1) discover EtherCAT slaves, (2) safely map PDOs (even when not byte-aligned), and (3) drive a time-based output pattern while sampling inputs. It avoids proprietary headers and exposes a clear **Integration Layer (TODO)** you can wire to any master SDK.

## Who is this for
Engineers and students who know basic C and want a clean, beginner-friendly starting point for EtherCAT digital I/O (not motion). Comments follow a short **WHAT/WHY/HOW** style.

## Safety first
Digital outputs may actuate real hardware. Power off, verify wiring, and test off-line. Keep SDO writes **disabled** unless you know the device and the correct state (usually PRE-OP).

## File layout
- `ethercat_multiio_blink_template.c`: everything in one file (utils, mapping, integration stubs, `Init()`, `Run()`).

## Quick start
1. Open the file and skim the **User Config Knobs** section.
2. Implement the **Integration Layer (TODO)** using your master SDK (map PI pointers, read states, SDO R/W, sleep, log).
3. Build your app and call `Init()` once, then call `Run()` every cycle.
4. Verify states reach **OP**; confirm bit offsets/sizes match your ENI mapping.
5. Gradually enable optional SDO tweaks only if vendor docs say it's safe.

## Porting checklist
- [ ] Replace `il_get_num_slaves()`, `il_get_slave_eni()`.
- [ ] Map PI pointers in `il_map_output_ptr()` / `il_map_input_ptr()`.
- [ ] Return proper states in `il_get_state()` (PRE-OP/SAFE-OP/OP).
- [ ] Wire SDO helpers: `il_sdo_read()` / `il_sdo_write()`.
- [ ] Provide `il_sleep_ms()` and `il_log()`.
- [ ] Confirm PDO bit offsets/sizes from your ENI.
- [ ] Run with real hardware **before** enabling watchdog tweaks.

## Configuration knobs
- `BLINK_PERIOD_MS` — time-based toggle period (default 500 ms).
- `ENABLE_REFERENCE_BLOCK` — include/exclude the in-code “Reference:” notes.
- `ENABLE_WATCHDOG_TWEAKS` — guarded SDO writes to watchdog objects (default OFF).
- `ENABLE_ONE_SHOT_SDO_TEST` — guarded demo poke on a DO object (default OFF).
- `IL_ENABLE_SIM` — small built-in simulator for smoke-tests (no real EtherCAT).

## Signals explained
- **DO nibble (4 bits):** a 4-channel digital-output module often maps as a 4-bit field starting at any bit offset in the process image. The template writes `0xA`/`0x5` at the correct bit position, splitting across bytes if needed.
- **DI nibble (4 bits):** reads the low 4 bits and mirrors them into a local `flag` variable (bits `[11:8]`) for demo purposes.
- **OUT16/IN20 example:** shows writing a 16-bit output word and sampling an input byte to pack into `flag` (`[14:12]`).

## Troubleshooting
- Outputs don’t change:
  - Ensure slave state is **OP** (not SAFE-OP/PRE-OP).
  - Confirm ENI offsets/sizes and that outputs aren’t safety-masked by a PLC.
  - Verify the nibble doesn’t cross a byte without mapping the second byte.
- Inputs always zero:
  - Check that you mapped the correct input byte offset and requested enough bytes.
- Random bits toggling:
  - Misaligned writes (wrong shift) can clobber neighbors; recheck offsets.
- Watchdog trips:
  - Don’t enable SDO tweaks unless vendor docs allow them in the current state.
- Nothing is detected:
  - Replace the Integration Layer stubs; the template ships without a master.
- Timing looks jittery:
  - The blink uses a monotonic clock with catch-up to avoid freeze; the visible jitter likely comes from your logging/loop cadence.

## License
- MIT
