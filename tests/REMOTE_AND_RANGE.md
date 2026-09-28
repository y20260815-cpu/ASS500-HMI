# Remote steering and intermittent ranging

- Direction bits: forward 1, reverse 2, left 4, right 8. Forward/reverse plus left/right keeps both wheels in the travel direction; the inner wheel uses 0% of the outer wheel command. Left/right means nose yaw, including reverse.
- Steering changes during the same travel direction do not rearm startup. Startup scales both wheel targets together to preserve steering. STM32 configured manual acceleration/deceleration shapes the output.
- Opposing buttons stop. Direction release stops; releasing only steering returns to straight travel. Repeated packets do not retrigger toggles or startup.
- Remote packet loss beyond 300 ms stops manual remote motion and requires direction release to rearm. Anchor loss alone does not stop manual operation while remote packets continue.
- Invalid or zero distances are excluded. Last measured distances remain displayed with validity LEDs off. A previous follow target is retained for at most 300 ms from the last valid pair; invalid packets cannot renew that deadline. Longer loss stops following; valid ranges permit resumption. This is bounded continuation, not position prediction.
- Tag ranges are no longer cleared to zero at every ranging cycle; response mask and 300 ms per-anchor freshness gate validity.

Validation: tests/remote_drive.cpp covers all four travel/steer combinations, pivots, conflicting keys, release, startup ratios and grace timeout boundaries. Physical driving remains to be tested. Follow direction geometry remains pending actual A0/A1 installation confirmation.

## Touch steering

The BSP forwards all GT911 contacts (up to configured five) each LVGL read, including release/error frames. Menu input retains the normal single pointer path. Manual button contacts are aggregated into the same direction mask used by the remote; primary contact ordering does not select the command. Both held buttons are highlighted. Stop/emergency contacts override movement and require all fingers to release. Settings overlay disables underlying drive buttons.

Physical acceptance: hold forward, add left, remove left (forward continues), add right, remove forward (right pivot), release all (stop). Repeat reverse plus left/right, and reverse finger press order. Verify stop with two drive fingers held. No autonomous movement tests are sent by the coding tools.

Follow stability: median-of-three ranges before the existing configurable low-pass filter; mirrored bearing candidate switches need a 20 degree advantage for 300 ms; front/rear hysteresis 80/100 degrees; rear turn latch removed; turn sign always follows the normalized selected bearing. This stabilizes an estimate; two ranges alone do not uniquely resolve mirror positions. LCD LiDAR horizontal projection, rays, sector lines and signed angle are mirrored consistently.

Test profile: translating follow turns set the slower same-direction wheel target to zero, preserving the outer target and existing approach envelope. Opposite-direction in-place pivot commands remain unchanged. Zero is a command; STM32 deceleration still determines physical stopping. Radar vehicle, TAG, LiDAR points/rays/sector are rotated 180 degrees for mounting; touch coordinates and motor direction are unchanged.

Shortest-turn correction: normalized selected bearing is always in [-180,180]; filter snaps when its sign differs from the new selected bearing, including rear seam and front zero crossing. Exact 180 degrees is an equal-angle tie. TAG marker horizontal projection matches mirrored LiDAR projection while retaining the 180 degree mounting rotation. This does not resolve the two-range mirror ambiguity.

STM32 inner-zero enforcement: for valid tagged CAN with exactly one requested wheel zero, clear that wheel's ramp state every 10 ms and clamp its final drive output to zero after PID. Both-zero stop retains normal deceleration. Confirmed direction is retained for reversal settling. Short-brake pulses and electrical protection remain active; zero drive is not a mechanical wheel lock. Existing STM32 $VCU UART fields target_pwm1/2 and drive_pwm1/2 distinguish commanded zero from final drive output. Physical measurement pending.
