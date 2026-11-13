# Demonstration CAN protocol

These are custom demonstration messages for the software simulation. They are
not the WP32 manufacturer's protocol or a J1939 implementation.

Both IDs fit in a standard 11-bit CAN identifier. The frame structure contains
an ID, payload length, and up to eight bytes. An in-memory queue carries frames
in each direction; physical CAN hardware and operating-system CAN interfaces
are not required.

## Frames

| Message | CAN ID | Length | Payload |
| --- | --- | --- | --- |
| Controller to pump: requested RPM | `0x200` | 2 bytes | Bytes 0 and 1: unsigned 16-bit RPM, low byte first |
| Pump to controller: actual RPM and fault status | `0x201` | 3 bytes | Bytes 0 and 1: unsigned 16-bit RPM, low byte first; byte 2: `0` healthy, `1` device fault |

For example, 3000 RPM is `0x0BB8`. A command frame is printed as:

```text
id=0x200 dlc=2 data=B8 0B
```

The codec accepts RPM values from 0 to 65535. The simulated pump rejects
commands above its configured maximum, 4600 RPM by default. Unknown IDs,
incorrect payload lengths, and unsupported status bits are rejected by the
relevant decoder.

## Timing and faults

- At the default 100 ms control step, the controller sends a command every step
  with ignition ON, and the pump sends status every step while powered.
- With a coarser step, feedback is sent on the first step after the nominal
  100 ms interval has elapsed. It is not emitted more than once per step.
- A valid status message is required for communication to be considered healthy.
  A freshly powered pump sends its first status before control is evaluated.
- The feedback watchdog expires when the time since the last valid message is
  greater than the configured timeout, 0.5 s by default. A 1e-12 s tolerance
  avoids false expiry due to floating-point rounding at an exact boundary.
- Malformed feedback and out-of-order receive timestamps do not refresh the
  watchdog. A stopped pump can still have healthy communication.
- A separate local pump command watchdog requests a stop after missing commands.
  This is a simulation assumption that needs confirmation for real hardware.
- Every receive queue has a fixed capacity, 64 frames by default. Full queues
  drop new frames and count the drops.
- The `pump-timeout` scenario drops frames in both directions from 40 s until
  55 s. The controller detects the timeout at 40.5 s with default settings and
  resumes normal control when valid feedback returns at 55 s.

CAN trace lines report `QUEUED` when a frame enters the destination queue and
`DROPPED` when it is discarded. `QUEUED` does not confirm actuator movement.
Actual pump speed is calculated separately and is returned in later feedback.

This simulation does not model bus arbitration, bit timing, bit stuffing,
physical wiring, CAN CRC errors, or a manufacturer's message addressing scheme.
