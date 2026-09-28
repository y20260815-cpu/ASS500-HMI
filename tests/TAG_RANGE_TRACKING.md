# TAG range definitions and turn hysteresis (R0913-9)

- A0–A1: fixed anchor separation from vehicle-width setting 14.
- TAG–A0 and TAG–A1: the two time-varying measured ranges used by correction,
  filtering, geometry and front/rear history. Neither measures changing anchor separation.
- Both anchors are 60 cm ahead of the vehicle center.

Pivot control uses angle hysteresis and does not stop or latch FOLLOW:

- Enter pivot alignment at 45 degrees of relative approach-axis error.
- Stay in pivot alignment until the error falls below the distance-dependent
  exit threshold: 10 degrees near TAG to 20 degrees at 4 m.
- Resume translating steering after exit. A new large error may enter pivot again.
- Forward uses the 0-degree approach axis; reverse uses the 180-degree axis.
- Final arrival alignment is separate: settle inside 3 degrees and re-align
  outside 5 degrees while inside the arrival-distance hysteresis.

The former ineffective-turn stop/lock and timed pause guard were removed because
they could leave FOLLOW at zero output or interrupt continuous tracking. This
hysteresis prevents rapid switching between pivot and translation, but cannot
detect a physically stalled motor or solve two-range front/rear ambiguity.

Checks cover range geometry, front/rear divergence, final alignment, fixed and
moving mouse TAG trajectories, CSV behavior and headless Chrome execution.
Real vehicle validation and flashing remain separate operations.
