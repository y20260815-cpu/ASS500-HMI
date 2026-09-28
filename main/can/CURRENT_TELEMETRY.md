# CAN current telemetry extension

The existing standard 0x5B8 / DLC 8 status layout is unchanged.
Byte 3 still contains whole amperes for legacy receivers.

Standard ID 0x5B9, DLC 8, sent with each 0x5B8 status:

| Bytes | Meaning |
|---|---|
| 0..1 | Unsigned uint16, big endian, current in 0.1 A units |
| 2 | Protocol version: 1 |
| 3 | 1 = valid sample, 0 = invalid sample |
| 4..7 | Reserved, zero |

Example: 6.6 A -> `00 42 01 01 00 00 00 00`.
Conversion rounds to nearest 0.1 A and saturates at 6553.4 A.
65535 is reserved. NaN, infinity and negative measurements are invalid.
The ESP32 prefers a valid extension sample received within 500 ms,
otherwise uses the legacy whole-ampere field. An extension alone does
not refresh the base RPM/emergency/brake status.
LCD and CTRL/STATUS serial outputs continue using current_deciamps;
there is no additional multiplication by ten at the receiver.
This changes transport resolution, not sensor accuracy or sampling rate.
