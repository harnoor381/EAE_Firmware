# Validation

The project was built and run on Linux with GCC 14.2.0, CMake 4.4.4, and
GoogleTest 1.17.0. Compiler warnings are treated as errors for project sources.

## Automated checks

The 92 GoogleTest tests pass. They cover:

- Ignition priority, immediate hot-start protection, fault ordering, and recovery.
- Exact warning and critical hysteresis boundaries.
- Invalid temperatures, sensor status failures, and preservation of thermal history.
- Normal pump bands and calibration limits.
- PID direction, integration time, output saturation, anti-windup, derivative
  filtering, setpoint changes, reset after a safety override, large integral
  steps, and numerical overflow handling.
- CAN byte order, round-trip encoding, malformed frames, queue ordering,
  bounded queues, dropped messages, and command/feedback timeouts.
- Pump speed response, command limits, device faults, and ignition restart.
- Closed-loop temperature regulation and increased fan output after a heat step.
- All five injected equipment faults and subsequent recovery.
- Runtime argument parsing, invalid values, and conflicting setpoints.

Reproduce with:

```bash
./test.sh
```

All 92 tests also pass with AddressSanitizer and UndefinedBehaviorSanitizer.
LeakSanitizer could not run in this execution environment, so that run used
`ASAN_OPTIONS=detect_leaks=0`. No leak-check result is claimed.

Command-line checks also confirmed valid CAN trace output, CSV paths containing
spaces, a 72 C alternate target, and exit status 2 for invalid setpoints,
numerical overflow, and unwritable CSV destinations. The alternate nominal run
with `Kp=6`, `Ki=0.3`, and `Kd=1` finished at approximately 72.00 C.

## Default 240-second demonstrations

Each case uses a 75 C target, an initial temperature of 70 C, and a 0.1 s step.
Final values below are rounded from the CSV sample at 239.9 s. The console
summary also advances the final partial interval to the end of the run.

| Scenario | Peak temperature (C) | Final sampled temperature (C) | Observed fault |
| --- | --- | --- | --- |
| nominal | 78.47 | 75.04 | None |
| heat-load | 78.54 | 74.58 | None |
| pump-timeout | 78.57 | 75.09 | PUMP_COMMUNICATION |
| low-coolant | 78.76 | 75.09 | LOW_COOLANT |
| sensor-fault | 78.47 | 75.21 | TEMP_SENSOR |
| overtemperature | 97.00, injected | 75.05 | OVER_TEMPERATURE |
| pump-fault | 78.62 | 75.09 | PUMP_DEVICE |

The default heat-load demonstration stays below the 90 C warning threshold.
It is not a zero-overshoot design; temperature initially rises above 75 C
while the integral term develops the required steady fan output.

## Observed transitions

- All scenarios: OFF at 0 s, then NORMAL at 1 s.
- CAN loss: FAULT at 40.5 s, NORMAL at 55 s.
- Low coolant, sensor fault, and pump device fault: FAULT at 40 s, NORMAL at 55 s.
- Injected overtemperature: FAULT at 40 s, WARNING at 44.6 s, NORMAL at 47.3 s.

The fault scenarios request load shutdown while leaving cooling outputs under
the state machine's control. The simulated load obeys that shutdown request.

## Scope of these results

These tests verify this program and its stated simulation assumptions. They
do not validate actual PLC execution, WP32 CAN compatibility, SPAL electrical
PWM mapping, hydraulic performance, or inverter/DC-DC derating prevention.
The GitHub Actions workflow is included for reproducible remote builds; local
test results do not imply that a GitHub workflow has already run.
