# Manual hold-to-run startup and handover

LCD direction press begins drive immediately; release/lost touch sends STOP.
STOP bypasses the 100 ms CAN schedule and remains pending on TX failure.
STM32 clears queued motion and startup assist, then applies its existing
voltage-aware deceleration. Actual stopping time depends on inertia.

A hold-to-run startup command floor is independent of cruise speed: 750
straight (30%) or 1500 pivot (60%), retaining cruise if it is already higher.
This command window is bounded to 600 ms and cancels on release/E-STOP.
These are command units (2500 = full scale), not measured wheel RPM.

Every hold-to-run phase uses CAN byte2=11 (1.1 s full-scale) and only the
STM32 PWM ramp. ESP32 passes the target without a second ramp. Startup
completion therefore cannot change an active curve duration from 1.1 s to
2 s or repeatedly retarget it. A lower cruise target uses STM32 deceleration.
Direction reversal retains the STM32 zero crossing and bridge settling.
Legacy tap byte2=10 remains separate; non-manual default is 20.

Estimated success requires two new samples 80..200 ms apart, after 200 ms:
current peak at least 1 A and 0.5 A above baseline, then a drop of at least
20% and 0.3 A; both RPMs must match command direction, reach 150 rpm, and
rise at least 100 rpm above baseline. Current timestamps must be unique,
newer than startup, and at most 150 ms old. Pre-start status must be fresh.
Success returns to cruise early. Missing evidence does not extend startup;
timeout returns to cruise and is logged TIMEOUT_UNCONFIRMED, never success.
ESTIMATED is a heuristic result, not proof of physical motion.

Validation: constexpr input, cancellation, evidence, and handover checks;
ESP-IDF build and flash hash verification. Physical startup/transition
behavior still requires vehicle testing on the actual surface.

## Automatic follow

Follow uses the same evidence estimator and a single STM32 ramp (byte2=11)
through startup/cruise. Straight/curved startup lasts at most 600 ms with a
30% command floor; opposite-sign pivot startup lasts at most 1000 ms with
60%. Scale both wheels equally to retain curvature and zero wheel commands.
Estimated success returns to the follow target early; expiry returns without
claiming success. There is no repeated boost while driving or auto-escalation.

A zero request sustained for 500 ms rearms the next start. A sign change,
near-target state, expired ranging geometry (>500 ms), stale CAN (>150 ms),
emergency, or detected LiDAR proximity cancels extra assistance. Require
clearance beyond stop distance of max(50 cm, configured braking distance).
Changing the output request invalidates current-peak evidence. The LiDAR
policy still monitors obstacles; this addition only excludes extra startup
output and is not an obstacle-stop implementation. UART M-only commands
without fresh D geometry cannot activate startup assist.

Near-target speed limiting and STM32 reversal/braking remain active. Follow
acceleration is now owned by STM32 rather than the previous ESP32 ramp.
This is a bounded initial field-test profile, not terrain-adaptive current
regulation or a guarantee of physical startup on loose ground.

## Sustained pivot correction

Passive COM4 observation showed +13%/-13% pivot commands, 4.5..5.8 A total
current, and one driver RPM stuck at 3600 while the user reported no vehicle
movement. In-place follow now removes turn/4 attenuation and uses a separate
rotation command: configured rotation command with a 750 (30%) floor, bounded
to 1500 (60%). This overrides a lower configured rotation command at heading
errors >=20 degrees. Between 20 and 10 degrees it tapers smoothly to zero;
alignment exits at 10 degrees. Translational speed no longer caps in-place
rotation. Straight/curved following is unchanged. The 60% startup boost and
its bounded time remain distinct from sustained pivot operation.

These are command levels, not actual wheel speed or verified ground torque.
Actual loaded pivot and heading overshoot need vehicle testing.

## Mode timing (supersedes fixed byte2=11 above)

Manual uses UI 15/16 driving acceleration/deceleration; follow uses UI 06/05
follow acceleration/deceleration. Startup and cruise share the selected timing.
See main/can/DRIVE_TIMING.md for the tagged, backward-compatible receiver format,
full-scale timing convention, and retained voltage protection.

## Pivot full-scale permission

Active hold-to-run and automatic-follow pivot startup now permit command
2500 (100%) instead of 1500 (60%). Startup durations, current/RPM evidence,
release cancellation, and approach guards are unchanged. The configured
acceleration ramp still applies; requesting 100% does not jump PWM to 100%.
Sustained follow pivot settings now accept 100..2500 through LCD and UART,
with matching simulator slider range. Existing saved settings are retained.
The STM32 overcurrent, thermal and DC-link protections were not modified.
Legacy bounded tap mode is unchanged and is not emitted by hold-to-run UI.

## Pivot feedback after startup

The reported issue is manual hold-to-run rotation. Tagged CAN pivot operation
now bypasses BEMF-based RPM PID on STM32, including pivot deceleration. This
prevents unreliable RPM from introducing asymmetric corrections after startup.
Straight, reverse, local, and legacy profiles retain existing PID behavior.
The shared pivot control also handles tagged follow rotation, but follow
command levels and saved rotation settings are unchanged.
Current, thermal, voltage, direction-change and stop protections remain active.
RPM remains available for telemetry and combined startup estimation. Manual
startup still returns to its configured cruise command; no new minimum cruise
command is imposed. Physical manual pivot continuity needs vehicle testing.
