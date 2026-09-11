# Sensorless FOC on the MKS ESP32 FOC Mega

Status: **feasible in principle; passive observer probe implemented; closed-loop handover not yet authorised**.

## Conclusion

The existing board already has the minimum signals needed to try a flux-observer sensorless FOC architecture without Hall sensors:

- two real phase-current measurements on GPIO39/GPIO36;
- the voltage vector commanded by SimpleFOC (`Ualpha/Ubeta`);
- known starting motor parameters R, L, KV/back-EMF and pole-pair count.

An additional three-channel phase-voltage ADC is therefore **not mandatory** for the MXLEMMING-style flux observer. Direct phase-voltage sensing could later improve the voltage model, but it is not required for the first experiment.

SimpleFOCDrivers v1.0.9 officially includes `MXLEMMINGObserverSensor`. Its documentation explicitly describes sensorless operation using phase currents and motor parameters. The implementation integrates the RL motor model using measured alpha/beta current and the motor's commanded `Ualpha/Ubeta` voltage vector.

## Board and motor starting parameters

MKS motor-0 current-sense mapping:

- shunt: 0.01 ohm;
- amplifier gain: 50 V/V;
- current ADCs: GPIO39 and GPIO36.

57BYA54-12-01 published values:

- pole pairs: 2 (4 poles);
- phase resistance: 0.27 ohm +/-10%;
- phase inductance: 0.57 mH +/-20%;
- back-EMF: 2.2 V/krpm +/-5%;
- initial KV model estimate: `1000 / 2.2 ~= 454.5 rpm/V`.

The datasheet tolerances and voltage-convention details mean these are **starting model values**, not calibration truth. Sensorless quality depends strongly on the effective R/L/flux linkage used by the observer.

## Why current alone is not a direct rotor sensor

The two shunts do not directly encode rotor angle. The observer combines them with the applied voltage model:

```text
psi_alpha += (U_alpha - R * I_alpha) * dt - L * dI_alpha
psi_beta  += (U_beta  - R * I_beta ) * dt - L * dI_beta
angle_e    = atan2(psi_beta, psi_alpha)
```

This estimates rotor flux angle. Differentiating the unwrapped electrical angle and dividing by pole pairs gives an estimated mechanical speed.

## Low-speed limitation

A flux/BEMF observer is weak at standstill and low speed because the informative back-EMF/flux dynamics become small relative to ADC offsets, model error and inverter non-idealities. The official observer documentation explicitly says it is intended for applications where tracking at low speed or while not driving is not important.

The practical architecture should therefore be **hybrid**:

```text
STOP
  -> open-loop startup
  -> observer runs passively in parallel
  -> observer confidence + speed become stable
  -> phase/angle handover
  -> closed-loop sensorless FOC
```

On loss of observer confidence, the safe response should be torque reduction/stop, not blind continued closed-loop operation.

## Stage 1 implemented: passive observer probe

`experiments/sensorless_probe/MaxxFan_Sensorless_Probe.ino`

The probe:

- requires only the existing SimpleFOC library;
- never initialises or reads Hall pins;
- drives the motor with the already-known `velocity_openloop + foc_current` path;
- runs an independent RL flux observer in parallel;
- reports commanded RPM, estimated RPM, electrical angle, alpha/beta current, alpha/beta commanded voltage and flux magnitude;
- allows R/L/KV model tuning from Serial;
- **never feeds estimated angle back into commutation**.

This is deliberately safer than immediately linking an unvalidated observer as the motor sensor.

Serial examples:

```text
s 600
s 800
s 1000
s 1200
s -600
status
kv 454.5
r 0.27
l 0.57
resetobs
stop
```

## Hardware test plan

### Stage 0 — current path

Confirm the normal current-sense alignment succeeds and that measured phase currents are sane. No Hall wiring is needed.

### Stage 1 — passive observer

At 600, 800, 1000 and 1200 commanded RPM:

1. let the motor stabilise;
2. compare sign and approximate value of observer RPM to command;
3. watch for angle discontinuities and large RPM spikes;
4. record flux ratio and loop/sample timing;
5. repeat both directions.

The observer does not need to equal the command perfectly in this stage; it needs to be continuous, directionally correct, and reasonably stable.

### Stage 2 — model tuning

Tune effective R, L and KV/flux linkage using the probe. Do this before any closed-loop handover. Temperature changes phase resistance, so a solution that only works with one exact cold R value is not robust enough.

### Stage 3 — handover experiment

Only after Stage 1/2 pass, implement a guarded open-loop-to-observer transition above a measured stable observer speed. Handover must match electrical phase before the observer is allowed to command FOC.

### Stage 4 — closed-loop sensorless velocity

Add a real velocity controller using observer RPM, current ceiling, loss-of-lock detection and fallback/stop logic. Keep Wi-Fi/GUI telemetry out of the critical motor task until timing is qualified.

## Why phase-voltage ADC remains optional, not useless

The current probe uses the commanded `Ualpha/Ubeta` vector as its voltage estimate, like the official SimpleFOC observer. Real bridge output differs because of bus-voltage variation, transistor drops, PWM/dead-time effects and switching non-idealities. Direct phase/bus voltage sensing can improve a later observer, but adding it before testing the existing current-based observer would add hardware without first proving it is needed.

## Dependency option later

Once the hardware probe is validated, there are two reasonable closed-loop implementations:

1. **Use SimpleFOCDrivers `MXLEMMINGObserverSensor`** — less custom code, but adds one library dependency.
2. **Keep a small project-local observer** — no new install, easier to instrument for this exact board, but we own validation and maintenance.

The research branch currently chooses option 2 only for the passive diagnostic probe. It does **not** claim production sensorless control.
