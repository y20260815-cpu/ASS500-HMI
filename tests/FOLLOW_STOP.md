# Final follow heading

The final error is relative to the selected approach axis: 0 degrees for forward
and 180 degrees for reverse. Automatic near-target stopping requires heading
alignment as well as distance. Manual stop, follow disable and emergency stop
continue to bypass this automatic arrival behavior.

At or inside the configured stop distance:

- Error above 3 degrees: command zero translation and a limited pivot.
- Error at most 3 degrees: finish alignment and send stop targets.
- Once settled, keep stopped up to 5 degrees and stop distance +15 cm, avoiding
  repeated acceleration due to small range fluctuations.
- Outside that band, re-evaluate alignment or resume approach.

Pivot target is 20 motor rpm per error degree, minimum 150 rpm, limited by the
configured pivot rpm clamped to 250..750 rpm. Existing STM32 drive ramps apply.
No startup torque boost is added at arrival; physical starting/stopping response
must be tested with the real motor driver. The measured UWB pose determines the
angle; incorrect UWB pose can still cause an incorrect physical heading.

Validation covers C++ static assertions, JS forward/reverse angle boundaries,
range-only fixed and moving target approaches, reference-mode rear/side arrival,
actual geometric arrival error within 5 degrees in those test cases, mouse
pin/unpin, CSV and Chrome execution. ESP32 flash is a separate operation.
