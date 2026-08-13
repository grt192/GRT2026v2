# Project notes

## Deferred work

### Simulation fidelity — two improvements borrowed from the Cheesy Poofs (not yet done)

Reference: `/Users/daniel/Developer/robotics/meetingCode/cheesyPoofs_2025`,
`src/main/java/com/team254/lib/subsystems/SimTalonFXIO.java`.
Context: `docs/subsystem-library-design.md` §8.

Our sim IOs (`*IOTalonFXSim`, and later `MotorIOTalonFXSim`) are less faithful than they should
be in two specific ways. Both are additive and touch only the sim IO. **Do these later — do not
bundle them into the subsystem-template migration**, since each changes sim behavior and needs
its own before/after comparison.

**1. Step the plant on a 5 ms `Notifier`, not once per `updateInputs` at 20 ms.**

Every sim IO currently does `sim.update(LOOP_PERIOD_SECONDS)` with `LOOP_PERIOD_SECONDS = 0.02`,
inside `updateInputs`. The Poofs instead run:

```java
simNotifier = new Notifier(() -> updateSimState());
simNotifier.startPeriodic(0.005);
```

and measure the real elapsed time (`sim.update(now - lastUpdateTimestamp)`) rather than assuming
a fixed period. This matters because our closed loops run **onboard the TalonFX at ~1 kHz**, so a
20 ms plant step makes simulated gains behave nothing like real gains. That is plausibly why all
six mechanism subsystems currently need a separate `SIM_P` / `SIM_KV` gain set at all.

**2. Model static friction in the sim plant.**

```java
protected double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
        motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
        motorVoltage -= frictionVoltage;
    } else {
        motorVoltage += frictionVoltage;
    }
    return motorVoltage;
}
```

They apply it with `frictionVoltage = 0.25`. Without it, `kS` is meaningless in simulation — the
hood's real `kS` is `120` while its sim gains omit `kS` entirely.

Suggested shape: an optional `frictionVolts` field on `MechanismConfig.Sim.Arm` /
`Sim.RotatingMass`, defaulting to `0.0` so nothing changes until a mechanism opts in.
