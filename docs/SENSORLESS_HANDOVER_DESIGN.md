# Sensorless handover design

This document defines the intended closed-loop architecture after the passive observer probe has been validated. It is **not enabled in the current firmware**.

## State machine

```text
STOPPED
  |
  | non-zero speed command
  v
OPEN_START
  |
  | rotating field established
  v
OBSERVE_OPEN
  |
  | observer quality criteria satisfied continuously
  v
ANGLE_BLEND
  |
  | electrical phase error blended to observer without a step
  v
SENSORLESS_CLOSED
  |
  | loss of observer quality
  v
FAULT / CONTROLLED_STOP
```

## 1. OPEN_START

Use the existing Hall-independent `velocity_openloop + foc_current` startup path. The observer runs in parallel but has no control authority.

Do not attempt observer takeover at standstill. Flux/BEMF observer information is weakest there.

## 2. OBSERVE_OPEN

Collect, at minimum:

- observer electrical angle;
- observer electrical-angle derivative / estimated mechanical RPM;
- commanded synthetic electrical angle;
- commanded RPM;
- flux-vector magnitude;
- alpha/beta current;
- observer sample interval / maximum loop gap.

The takeover threshold must be selected from actual measurements, not guessed in code before the passive tests.

## 3. Observer quality gate

A future `observerLocked` condition should require **all** of the following for a continuous dwell time:

1. Estimated RPM sign agrees with commanded direction.
2. Estimated RPM is finite and within a hardware-derived tolerance of the expected speed.
3. Electrical angle evolves continuously without repeated unwrap jumps/spikes.
4. Flux magnitude stays in an experimentally established valid band.
5. Observer sample interval remains within the qualified timing envelope.
6. Current readings remain finite and below the hard current ceiling.

No single instantaneous sample may trigger takeover.

## 4. ANGLE_BLEND

Never switch from synthetic angle to observer angle with a step.

At the beginning of handover:

```text
phase_error = wrapPi(observer_electrical_angle - openloop_electrical_angle)
```

Then reduce this error smoothly over a short blend interval while continuing torque/current control. The final blend duration should be established on hardware. A discontinuous electrical-angle jump can create a torque impulse and audible/mechanical click.

An alternative is to seed the observer-linked sensor offset so the observer initially reports exactly the current synthetic angle, then fade the offset to zero.

## 5. SENSORLESS_CLOSED

Once the observer owns angle feedback:

- use measured/estimated velocity for the velocity PID;
- let the velocity loop request q-current only as required by load;
- keep `currentLimitA` as a true ceiling;
- retain the 3 V commissioning voltage ceiling until sensorless operation is stable;
- keep GUI/network activity outside the timing-critical control path.

This is the stage where sensorless FOC gives the behaviour originally expected from “the motor only takes the current it needs”: velocity error drives the requested torque/current instead of OPEN mode blindly requesting a fixed/feed-forward Iq.

## 6. Loss-of-lock policy

Do not automatically continue blind high-speed operation after observer loss.

Initial safe policy:

1. flag loss of observer lock;
2. ramp q-current down;
3. ramp commanded speed toward zero if angle is still credible, otherwise disable torque promptly;
4. require an explicit restart or a separately validated re-acquisition sequence.

Automatic seamless fallback to open-loop can be investigated later, but it should not be the first sensorless implementation.

## 7. Reverse direction

A closed-loop reverse should be handled as a controlled deceleration toward low speed. Since the observer becomes weak near zero speed, the architecture should expect to leave sensorless closed-loop before the zero crossing, perform the low-speed/zero crossing with the open-loop startup state machine, then re-acquire the observer in the opposite direction.

Trying to remain purely flux-observer closed-loop through 0 rpm is not the first target for this fan application.

## 8. Parameters that must come from testing

Do not hard-code these before the passive observer recordings:

- minimum observer takeover RPM;
- allowed RPM estimation error;
- flux magnitude valid band;
- lock dwell time;
- angle blend duration;
- low-speed release RPM during deceleration/reverse;
- R/L/KV compensation versus motor temperature.

The branch should move to a closed-loop prototype only after these values can be justified from captured data.
