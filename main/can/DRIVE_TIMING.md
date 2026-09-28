# Mode-specific drive timing

Manual motion selects UI index 6 (15 ?? ????) and 7 (16 ?? ????).
Follow selects index 5 (06 ?? ????) and 4 (05 ?? ????).
STOP keeps the preceding mode selection so deceleration does not switch profiles.
Startup assistance uses the same selected timing as cruise; legacy taps remain
separate. Follow approach geometry also continues to use its deceleration setting.

0x701 remains DLC 8. Tagged timing format:
- byte2: acceleration in 0.1 s units, 5..50 (0.5..5.0 s).
- byte3: 0x80 OR deceleration in 0.1 s units, 5..50.
- motor, toggle and emergency fields are unchanged.
Both values must be valid to select tagged timing. Legacy byte3 battery values
still select the old acceleration interpretation and default deceleration.
STM32 retains internal battery code 3 for valid tagged frames. A tagged 1.0 s
acceleration is not mistaken for the old byte2=10 tap-assist profile.

Times retain the existing full-scale PWM convention: 0..100% duration.
Smaller PWM changes take proportionally less time. These are commanded ramps,
not measured vehicle speed guarantees. STM32 snapshots acceleration duration
when a curve starts, so live setting changes cannot rewind an active curve.
Deceleration uses elapsed control time and retains DC-link voltage protection;
that protection can lengthen actual deceleration. No CAN frame length change.

Requires the matching STM32 receiver before deploying the ESP32 sender.
