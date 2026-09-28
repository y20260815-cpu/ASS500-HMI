# Front-start and range divergence trial

The operator must place TAG in front before starting FOLLOW. A new FOLLOW start
clears the old hemisphere and range history on the next range-processing cycle.
The simulator's default R0913-5 mode is range history, not mouse hemisphere input.

Only fresh, reconciled A0/A1 pairs update the divergence check. It compares the
two candidate center-distance changes against previous left/right speed commands.
This is a command-based hypothesis, not measured vehicle odometry.

Trial thresholds:

- At least 20 cm/s average commanded speed, with wheel-command difference no more
  than 25% of average speed; no stationary/pivot correction.
- Selected candidate must lie in commanded travel direction and exceed 100 cm.
- At least 1 second and 10 intervals, with at least 30 cm net range growth.
- Alternative candidate accumulated prediction error must be at least 15 cm lower.
- Gaps above 300 ms or distance jumps above 300 cm/s discard accumulated evidence.
- Corrected hemisphere sends zero targets for 300 ms while direction selection
  settles; the existing STM32 ramp still determines physical stopping.
- Three-second cooldown after correction. Evidence windows restart after 2 seconds.

Limits: A moving TAG, wheel slip or a stalled vehicle may invalidate commanded
motion assumptions. This heuristic cannot guarantee front/rear identity. Real
gyro/odometry can improve the prediction. No automatic probing motion is added.
Physical validation remains required; passing simulator tests is not proof of
real motion or robust localization.

Validation: `node tests/uwb_divergence.cjs`, `node tests/uwb_follow_model.cjs`,
`node tests/simulator_smoke.cjs <html>`, `node tests/browser_follow.cjs <html>`.
