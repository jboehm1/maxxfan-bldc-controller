# Dynamic q-current feature

Status: **implemented on `feature/dynamic-current`, software-tested, hardware validation pending**.

## Why this branch exists

In SimpleFOC `velocity_openloop + foc_current`, the open-loop velocity path uses the configured current limit as the q-current request. That is useful for commissioning, but it means a fixed value such as 0.60 A is requested at low speed as well as high speed.

This branch separates two concepts:

- **Hard current ceiling** (`currentLimitA`): absolute software ceiling for the branch, default 0.60 A.
- **Dynamic q-current request**: feed-forward current command selected from commanded electrical speed, always clamped below the hard ceiling.

The current ADCs on GPIO39/GPIO36 remain the physical feedback used by `foc_current`. No Hall signal is read in OPEN mode.

## Default profile

Defaults:

| Parameter | Default | Meaning |
|---|---:|---|
| Dynamic current | enabled | Enable adaptive Iq in OPEN mode |
| Hard current ceiling | 0.60 A | Maximum q-current request |
| Minimum run current | 0.22 A | Low-speed feed-forward current near `minRpm` |
| Curve exponent | 2.0 | Current rise versus commanded RPM |
| Startup boost ceiling | 0.60 A | Temporary startup target, still hard-clamped |
| Startup boost fade | 650 ms | Smooth fade from boost toward steady profile |

The steady request is approximately:

```text
Iq(rpm) = low_speed_floor(rpm)
        + (I_hard - I_min) * (abs(rpm) / maxRpm)^exponent
```

`low_speed_floor` uses a quintic smootherstep from 0 A at zero commanded speed to `I_min` by `minRpm`. The total is always clamped to the hard current ceiling.

The startup boost is also gated by commanded field speed, so it does **not** apply a static full-current kick at zero electrical speed. It fades smoothly over the configured startup interval.

## Important limitation

This is **feed-forward**, not real load-aware velocity control. In OPEN mode there is still no measured rotor RPM. The current sensors regulate the requested phase/q current accurately, but they do not by themselves prove that the rotor follows the commanded speed.

A blocked, overloaded, or desynchronised rotor can therefore still consume current while failing to follow the synthetic open-loop angle. Do not treat this branch as stall-safe until a real rotor observer/Hall feedback is active.

## GUI / NVS

The GUI exposes:

- dynamic current enable;
- minimum run current;
- curve exponent;
- startup boost current;
- startup boost duration;
- requested Iq versus measured Iq.

The dynamic-current settings use independent NVS keys (`dynen`, `dynmin`, `dynexp`, `dynboost`, `dynbms`). The existing `CONFIG_VERSION=6` blob is not changed, preserving compatibility with the known firmware configuration.

## Validation before merge

1. Keep `currentLimitA = 0.60 A` and `voltage_limit <= 3.0 V` for first tests.
2. Test 200/400/600/800/1000/1200 rpm with no fan obstruction.
3. Verify startup catches the rotor reliably from rest in both directions.
4. Compare requested Iq and measured Iq in Serial diagnostics.
5. Check motor/driver temperature after sustained operation.
6. Test direction changes during ramp-up and at stable speed.
7. Only tune `minRunCurrentA`, exponent or startup boost after the baseline test.

Do not merge to `develop` or `main` until these hardware tests pass.
