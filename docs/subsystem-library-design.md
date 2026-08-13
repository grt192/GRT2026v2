# Subsystem Template Library — Design

Status: **proposal, revision 3.** No implementation code exists yet.

Target package: `frc.robot.util.subsystems`

Environment this is designed against:

| Component | Version |
| --- | --- |
| GradleRIO / WPILib | 2026.2.1 |
| AdvantageKit | 26.0.0 |
| Phoenix 6 | 26.1.1 |
| Java | 17 (`sourceCompatibility`/`targetCompatibility`) |

Java 17 matters for two specific design choices: records and sealed interfaces are available,
but **pattern-matching `switch` is not** (it standardized in Java 21). Anywhere the design
dispatches on a sealed type it uses `instanceof` chains rather than switch patterns.

### Decisions

| Question | Decision |
| --- | --- |
| §7.1 Variant handling | **Two subsystem classes**, optional features in the IO layer |
| §7.2 Where the closed loop lives | **Onboard the motor controller**, high-level IO |
| §7.3 Config surface | **`TalonFXConfiguration` is the source of truth**, translated per vendor |
| §7.4 Hardware/mode selection | **One factory**; `RobotContainer` reads the same in all modes |
| §7.5 Units | **Raw doubles**, rotations and rot/s |
| §7.6 SysId | **Helper class**, not a `Command` on the base |
| §7.7 Gain slots | **Adopted** — slot ↔ control mode ↔ output type |
| Command factories | **Setters only** on the template; domain commands stay on the subsystem |
| Log keys | **Hierarchical**, one-time break (`Shooter/Hood`, `Intake/Pivot`) |
| Motion Magic | **Carried forward unchanged** |
| Homing | **Out of scope**, seam left open |
| Tower reporting ratio | **Explicit 1:1** (§5.4) |
| Migration order | **Intake first** — roller then pivot, as a unit |

### Correction from revision 2

Revision 2 claimed `shooter.tower` had a ratio bug — that its real-robot velocity was 4× its sim
velocity. **That was wrong.** Re-deriving it (§5.4) shows sim and real agree; the tower simply
*reports rotor units* where the other five report mechanism units. Both are self-consistent.

Working through it did surface a genuine flaw in revision 2's own design: it used a single
`gearRatio` for two different quantities — the sim plant's mechanical reduction, and the ratio
the Talon applies to produce reported units. For five of six mechanisms those numbers are equal,
which is why it looked fine. For the tower they are 4.0 and 1.0. §3.2 now separates them.

---

## 1. Scope

### 1.1 In scope

Six mechanism subsystems, all TalonFX, all on the `mechCAN` CANivore, all built as an
`IO` / `IOTalonFX` / `IOTalonFXSim` / `Subsystem` quadruple:

| Subsystem | Kind | Motors | External encoder | Closed-loop request in use |
| --- | --- | --- | --- | --- |
| `intake.pivot` | servo | 1 | CANcoder (remote feedback) | `PositionVoltage` (FOC) |
| `shooter.hood` | servo | 1 | CANcoder (remote feedback) | `PositionTorqueCurrentFOC` |
| `intake.roller` | roller | 1 | — | `VelocityVoltage` (FOC) |
| `hopper` | roller | 1 | — | `VelocityVoltage` (FOC) |
| `shooter.tower` | roller | 1 | — | `VelocityVoltage` (FOC) |
| `shooter.flywheel` | roller | 2 (1 follower, opposed) | — | `VelocityVoltage` (FOC) |

**Only three of the eight logical variants have any instance.** Servo-with-followers: zero.
Encoder-with-followers: zero. Servo-with-followers-and-encoder: zero. This is the single most
important fact for the variant-handling question in §7.1.

### 1.2 Out of scope

**Swerve** — `SwerveSubsystem`, `ModuleIO`/`ModuleIOTalonFX`, `KrakenSwerveModule`,
`DriveMotor`, `SteerMotor`, `GyroIO`, `PhoenixOdometryThread`. It is a different problem:
drive and steer are coupled inside one module IO, there is a 250 Hz odometry thread feeding
`odometryDrivePositionsRads`/`odometrySteerPositions` arrays through the Inputs class, and
`SteerMotor`/`DriveMotor` extend `SubsystemBase` while publishing raw NetworkTables entries
instead of going through AdvantageKit at all. Forcing swerve into a servo/roller taxonomy
would mean either splitting the module into two subsystems (losing the synchronized
`BaseStatusSignal.refreshAll` and the coupled optimization) or bloating the template with
odometry concepts nothing else uses.

Swerve will be touched twice by this work, in small mechanical ways: the enum move in §3.1 and
the `PhoenixUtil` split in §3.6.

**No-motor subsystems** — `VisionSubsystem`, `FuelDetectionSubsystem`,
`FieldManagementSubsystem`, `AimSubsystem`. Nothing to share.

### 1.3 Third category: a homing seam, not a homing feature

`climbSubsystemCommandOverview(inline).txt` describes a planned climber whose winch and arm both
"wait to reach the hardstop with timeout, and if timed out, back into the hardstop". That is a
capability with **zero instances in the current code** — see §2.3.

Homing stays out of v1. What the design does instead is keep the two hooks homing needs
reachable from day one, as no-op defaults on `MotorIO`:

```java
default void setCurrentPositionRot(double positionRot) {}
default void setSoftLimitsEnabled(boolean forward, boolean reverse) {}
```

Adding homing later means writing a `HomingRoutine` against those two methods plus
`inputs.statorCurrentAmps`/`inputs.velocityRps`. It does not require reopening `ServoSubsystem`
or `MechanismConfig`.

---

## 2. Duplication inventory

### 2.1 Measured overlap

Line counts after normalizing away identifiers, numeric literals and whitespace — that is,
lines that are *structurally* the same code:

| Pair | Structurally identical lines |
| --- | --- |
| `HopperIOTalonFX` vs `TowerIOTalonFX` | **172 of 178 / 175** |
| `PivotIOTalonFX` vs `HoodIOTalonFX` | **182 of 226 / 223** |
| `HopperIOTalonFX` vs `RollerIOTalonFX` | 140 of 178 / 160 |
| `HopperIOTalonFX` vs `FlywheelIOTalonFX` | 112 of 178 / 220 |
| `HopperSubsystem` vs `TowerSubsystem` | 129 of 168 / 149 |
| `PivotSubsystem` vs `HoodSubsystem` | 126 of 181 / 161 |
| `RollerSubsystem` vs `FlywheelSubsystem` | 109 of 127 / 130 |
| `HopperIO` vs `TowerIO` | 30 of 32 / 32 |
| `PivotIO` vs `HoodIO` | 33 of 35 / 36 |
| `TowerIOTalonFXSim` vs `RollerIOTalonFXSim` | identical except names and one gear ratio |

Total across the six subsystems: **3,155 lines** (271 IO interfaces + 1,395 `IOTalonFX` +
374 `IOTalonFXSim` + 1,115 `Subsystem`).

### 2.2 What is repeated, and where it diverges

| # | Repeated code | Present in | Approx. size | Divergence |
| --- | --- | --- | --- | --- |
| 1 | `StatusSignal` field declarations, `List.of(...)`, `setUpdateFrequencyForAll(100.0, …)`, `optimizeBusUtilization(0, 1.0)`, `PhoenixUtil.registerSignals`, initial `refreshAll` | 6/6 | ~60 lines each | Hood adds `getAcceleration`. Servos add a second signal list for the CANcoder. Flywheel adds a five-signal follower list. Frequencies and optimize args identical everywhere. |
| 2 | `updateInputs` copy-out (`signal.getValueAsDouble()` → `inputs.field`) | 6/6 | ~20 lines each | Field-for-field identical except the extra signals in #1. |
| 3 | `Alert`/`GatedAlert` scaffold: `MOTOR_ALERT_PREFIX`, disconnect alert, 4–6 gated alerts, `refreshXAlerts(boolean)` | 6/6 | ~15 lines each | Servos duplicate the whole block again for the CANcoder; flywheel duplicates it for the follower. Message strings already follow one format. |
| 4 | `@AutoLog` Inputs class | 6/6 | 12 common fields | **Naming split**: rollers call it `connected`, servos call it `motorConnected`. Hood adds `accelerationRPSPerSec`. Servos add 3 encoder fields. Flywheel adds 6 follower fields. |
| 5 | Six `LoggedTunableNumber` gain fields + `Watcher` + `pidWatcher.ifChanged(() -> io.updatePID(...))` | 6/6 | ~20 lines each | Pivot's `updatePID` signature carries `kG`; the other five omit it. Purely accidental drift. |
| 6 | `io.getDefaultPID()` seeding, and a sim-specific override of `getDefaultPID()` in every `*IOTalonFXSim` | 6/6 | ~10 lines each | None structurally; only which gains. |
| 7 | `setDutyCycle` (clamp ±1) / `setVoltage` (clamp ±12) / `setVoltage(Voltage)` bridge / `stop()`, each paired with `setpointTracker.updateSetpoint(...)` | 6/6 | ~25 lines each | Identical. |
| 8 | `periodic()` shape: `updateInputs` → `Logger.processInputs` → disabled check → `setpointTracker.logAll()` → `recordOutput(at*Setpoint)` → mechanism viz → `pidWatcher.ifChanged` → `LoggedTracer.record` | 6/6 | ~18 lines each | Identical ordering in all six. |
| 9 | `SysIdRoutine` construction + `runSysID()` four-phase command | 6/6 | ~18 lines each | Only three numbers differ: servos `(0.5 V/s, 1 V, 5 s)`, rollers `(1 V/s, 7 V, 10 s)`. |
| 10 | Sim IO body: `setSupplyVoltage(getBatteryVoltage())` → `sim.setInputVoltage(simState.getMotorVoltage())` → `sim.update(0.02)` → write rotor position/velocity → `super.updateInputs` | 6/6 | ~20 lines each | Plant differs (`SingleJointedArmSim` ×2, `DCMotorSim` ×4). Servos additionally drive a `CANcoderSimState`. |
| 11 | MOI-from-`kA` derivation (`kA/2π`, `kT`, `R`, `moi = kA·ratio·kT/R`) | 3 (hopper, tower, intake roller) | 4 lines each | Roller omits the reduction because it is direct drive. |
| 12 | `ChassisReference.Orientation` derived from the inversion constant | 4 (rollers) | 3 lines each | Servos hardcode the orientation instead of deriving it — and **hood's hardcoded value contradicts the naive derivation**, see §3.5. |
| 13 | Current limits | 5/6 | 3–6 lines | **Genuinely inconsistent** — see §2.3. |
| 14 | Motion Magic config + 3 tunables + `updateMotionMagicConfig` | 3 (hopper, tower, flywheel) | ~15 lines each | **Configured but never used** — see §2.3. |
| 15 | Soft limit switch config | 2 (both servos) | 5 lines each | Identical shape. |
| 16 | Setpoint clamp in the subsystem (`MathUtil.clamp` to the same limits already on the Talon) | 2 (both servos) | 5 lines each | Identical shape. |
| 17 | `Follower` control + applying the leader's full config to the follower | 1 (flywheel) | ~8 lines | Only one instance. |
| 18 | CANcoder config: `refresh` → set `SensorDirection` + `AbsoluteSensorDiscontinuityPoint` (computed as `((mid+0.5)%1+1)%1` from the soft limits) → `apply` | 2 (both servos) | ~10 lines each | Byte-identical apart from the constants feeding it. |

Items 1–10 are the bulk: roughly **2,100 of the 3,155 lines** are mechanical repetition with no
behavioral intent behind the differences.

### 2.3 Things you assumed were duplicated that are not

**Zeroing / homing does not exist.** Grep turns up exactly one trace: a commented-out
`// pivotIntake.zeroEncoder();` inside `RobotContainer.onAutonInit()`.

**Current limiting is not a shared pattern.** It is inconsistent in a way that looks
unintentional:

| Subsystem | Stator | Supply | Neutral mode |
| --- | --- | --- | --- |
| `shooter.hood` | 50 A, enabled | 40 A, enabled | Brake |
| `intake.pivot` | 40 A, enabled | — | Brake |
| `intake.roller` | 120 A, enabled | — | Brake |
| `hopper` | 120 A, **disabled** | — | Coast |
| `shooter.tower` | 120 A, **disabled** | — | Coast |
| `shooter.flywheel` | **none configured** | **none** | Coast |

Hopper and tower define `SUPPLY_CURRENT_LIMIT_AMPS = 80.0` in `Constants` and never apply it.

**Motion Magic is configured and never used.** `HopperIOTalonFX`, `TowerIOTalonFX` and
`FlywheelIOTalonFX` build a `MotionMagicConfigs`, apply it, expose `updateMotionMagicConfig`,
and back three `LoggedTunableNumber`s each — but all three drive `VelocityVoltage`, not
`MotionMagicVelocityVoltage`. Carried forward verbatim per your decision, but it is dead weight.

**Followers are a single instance**, and **external encoders are two instances** — both
CANcoders wired as `RemoteCANcoder` feedback on the Talon. Decisive for §7.1.

**Mechanism2d is not duplicated as written**, but that is a statement about the current code:
§3.5 shows an arm and an N-spoke spinner cover five of six once dimensions move into config.

**`Hood.MAGNET_OFFSET = -0.05688` is dead.** `HoodIOTalonFX` refreshes the CANcoder config and
then sets only `SensorDirection` and `AbsoluteSensorDiscontinuityPoint`, so the offset comes from
whatever is burned into the device. The constant is never read. The design preserves the
refresh-then-modify behavior so this is not silently changed.

### 2.4 Divergences that look deliberate and must survive

| Divergence | Where | How the design carries it |
| --- | --- | --- |
| `PositionTorqueCurrentFOC` vs `PositionVoltage` | hood vs pivot | `SlotSpec.output` |
| Sign flip: `RotorToSensorRatio = −244.41`, `SensorToMechanismRatio = −1` | hood only | Written directly in `fx.Feedback` |
| Reports rotor units, not mechanism units | tower only | Explicit `SensorToMechanismRatio = 1.0` (§5.4) |
| Mechanism2d driven by `encoderAbsolutePositionRot` | pivot only | `Visualizer.Source` |
| Follower spins opposed | flywheel only | `Follower.opposeLeader` |
| Sim gains differ from real gains | 6/6 | `simOverrides` on the config |

---

## 3. Package and type structure

```
frc/robot/util/
    Hardware.java                 ← renamed from ComponentStatus; our enums (§3.1)
    CanBuses.java                 lazy LoggedCanivore registry (§4.2)
    PhoenixUtil.java              slimmed: tryUntilOk + signal registry only (§3.6)
    SysIdFactory.java             builds a SysIdRoutine + four-phase command (§3.7)

frc/robot/util/subsystems/
    MechanismConfig.java          the config surface (§3.2)
    MotorIO.java                  interface + @AutoLog MotorInputs (§3.3)
    EncoderIO.java                interface + @AutoLog EncoderInputs (§3.3)
    MechanismState.java           record(positionRot, velocityRps) — sim linkage (§4.3)
    MechanismSubsystem.java       abstract base (§3.4)
    ServoSubsystem.java
    RollerSubsystem.java
    MechanismVisualizer.java      default Mechanism2d shapes (§3.5)
    MechanismHardware.java        the single real/sim/replay + vendor decision (§4)
    talonfx/
        MotorIOTalonFX.java
        MotorIOTalonFXSim.java
        EncoderIOCancoder.java
        EncoderIOCancoderSim.java
        TalonFXTranslator.java    Phoenix status enums -> ours, + slot plumbing (§3.6)
```

### 3.1 `Hardware` — one home for our enums

`frc/robot/util/ComponentStatus.java` is renamed to `frc/robot/util/Hardware.java`. Because
§7.3 puts `TalonFXConfiguration` in the config, Phoenix's own `InvertedValue`,
`NeutralModeValue` and `GravityTypeValue` are used directly and need no neutral twin. What
remains ours is the vocabulary that appears in **logs** (which must stay vendor-free for replay)
and in **control-request selection**:

```java
package frc.robot.util;

/** Vocabulary shared by IO layers, logging and configs. */
public final class Hardware {

    private Hardware() {}

    // --- existing, moved verbatim ---

    public enum MotorControlMode {
        Disabled, Follower, DutyCycle, Voltage, TorqueCurrent, Position, Velocity
    }

    public enum EncoderHealth { Good, Marginal, Bad, Unknown }

    // --- new: which control request a closed-loop setpoint should use ---

    public enum ClosedLoopOutput { Voltage, TorqueCurrent, DutyCycle }
}
```

`ClosedLoopOutput` cannot come from Phoenix because it is not a configuration value at all — it
is a choice of *control request type* (`PositionVoltage` vs `PositionTorqueCurrentFOC`), which
lives in code rather than in `TalonFXConfiguration`. §7.7 attaches it to the gain slot.

Notes:

- The two existing enum **names are unchanged** so the rename is a one-line import change at
  each use site.
- Constant naming follows the existing house style — `PascalCase` (`ClosedLoopOutput.Voltage`),
  matching `MotorControlMode.DutyCycle`.
- Blast radius outside the six migrating subsystems is four files: `LoggedSetpointTracker`,
  `PhoenixUtil`, `swerve/ModuleIO`, `swerve/ModuleIOTalonFX`. Import-only changes.

### 3.2 `MechanismConfig`

`TalonFXConfiguration` carries every device setting. The record adds only what Phoenix has no
field for: identity, sim plant, visualization, tolerance, slot semantics, and the sim overlay.

```java
package frc.robot.util.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.CANType;
import frc.robot.util.Hardware.ClosedLoopOutput;
import frc.robot.util.Hardware.MotorControlMode;

public record MechanismConfig(
        String name,                              // log key root, e.g. "Shooter/Hood"
        CANType bus,
        Controller controller,
        int canId,
        TalonFXConfiguration fx,                  // ← the source of truth for the device
        Consumer<TalonFXConfiguration> simOverrides,
        Follower[] followers,                     // empty when none
        Encoder encoder,                          // null when none
        SlotSpec[] slots,
        double tolerance,                         // rotations (servo) or rot/s (roller)
        Visualizer visualizer,
        Sim sim,
        SysIdSpec sysId) {

    public enum Controller { TALON_FX, NONE }

    public record Follower(int canId, boolean opposeLeader) {}

    public record Encoder(int canId, CANcoderConfiguration cc) {}

    /**
     * Which Phoenix gain slot serves which control mode, and what control request to issue.
     * The gains themselves live in {@code fx.Slot0} / {@code Slot1} / {@code Slot2}.
     */
    public record SlotSpec(int slot, MotorControlMode mode, ClosedLoopOutput output) {

        public static SlotSpec position() {
            return new SlotSpec(0, MotorControlMode.Position, ClosedLoopOutput.Voltage);
        }

        public static SlotSpec velocity() {
            return new SlotSpec(0, MotorControlMode.Velocity, ClosedLoopOutput.Voltage);
        }

        public SlotSpec withOutput(ClosedLoopOutput output) { /* ... */ }
        public SlotSpec withSlot(int slot) { /* ... */ }
    }

    public record SysIdSpec(double rampVoltsPerSec, double stepVolts, double timeoutSec) {
        public static final SysIdSpec SERVO = new SysIdSpec(0.5, 1.0, 5.0);
        public static final SysIdSpec ROLLER = new SysIdSpec(1.0, 7.0, 10.0);
    }

    public record Visualizer(
            Shape shape,
            double canvasWidthM, double canvasHeightM,
            double rootX, double rootY,
            double lengthM,          // arm length, or spinner radius
            int spokes,              // spinner only; 1 for a plain roller, 4 for the hopper
            double thicknessPx,
            Color8Bit color,
            Source source) {         // which input drives the angle

        public enum Shape { NONE, ARM, SPINNER }
        public enum Source { MECHANISM_POSITION, ENCODER_ABSOLUTE }

        public static Visualizer arm(double lengthM) { /* sensible canvas + color */ }
        public static Visualizer spinner(double radiusM) { /* spokes = 1 */ }
        public static Visualizer spinner(double radiusM, int spokes) { /* ... */ }
        public static final Visualizer NONE = /* Shape.NONE */;

        public Visualizer withCanvas(double widthM, double heightM) { /* ... */ }
        public Visualizer withRoot(double x, double y) { /* ... */ }
        public Visualizer withColor(Color8Bit color) { /* ... */ }
        public Visualizer withSource(Source source) { /* ... */ }
    }

    /**
     * Sim plant. {@code mechanicalReduction} is rotor rotations per rotation of the physical
     * output shaft — a property of the gearbox, NOT of how the Talon reports position. Those
     * are the same number for five of six mechanisms and differ for the tower (§5.4).
     */
    public sealed interface Sim permits Sim.None, Sim.Arm, Sim.RotatingMass {

        double mechanicalReduction();

        record None() implements Sim {
            @Override public double mechanicalReduction() { return 1.0; }
        }

        record Arm(
                DCMotor gearbox, double mechanicalReduction,
                double moiKgM2, double comLengthM,
                double minAngleRot, double maxAngleRot,
                boolean simulateGravity, double startAngleRot) implements Sim {}

        record RotatingMass(
                DCMotor gearbox, double mechanicalReduction, double moiKgM2) implements Sim {

            /**
             * Derives plant inertia from a measured kA, replacing the formula duplicated in
             * HopperIOTalonFXSim, TowerIOTalonFXSim and RollerIOTalonFXSim.
             */
            public static RotatingMass fromKa(
                    DCMotor gearbox, double mechanicalReduction, double kA) {
                double kaRadPerSecSq = kA / (2.0 * Math.PI);
                double kT = gearbox.stallTorqueNewtonMeters / gearbox.stallCurrentAmps;
                double resistance = gearbox.nominalVoltageVolts / gearbox.stallCurrentAmps;
                return new RotatingMass(gearbox, mechanicalReduction,
                    (kaRadPerSecSq * mechanicalReduction * kT) / resistance);
            }
        }
    }

    // ---- derived ----

    /**
     * Rotor rotations per reported unit, as the Talon computes it. Distinct from
     * {@code sim.mechanicalReduction()} — see §5.4.
     */
    public double rotorToReportedRatio() {
        return fx.Feedback.RotorToSensorRatio * fx.Feedback.SensorToMechanismRatio;
    }

    /**
     * Sim rotor orientation. Naively this is just {@code fx.MotorOutput.Inverted}, but a
     * negative RotorToSensorRatio flips it again — which is why HoodIOTalonFXSim hardcodes
     * Clockwise_Positive despite the hood being configured CounterClockwise_Positive.
     */
    public ChassisReference simOrientation() {
        boolean clockwise = fx.MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
        boolean sensorFlipped = fx.Feedback.RotorToSensorRatio < 0.0;
        return (clockwise ^ sensorFlipped)
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    }

    // ---- construction ----

    public static MechanismConfig of(String name, CANType bus, int canId) { /* ... */ }

    public MechanismConfig withFx(Consumer<TalonFXConfiguration> mutator) { /* ... */ }
    public MechanismConfig withSimOverrides(Consumer<TalonFXConfiguration> mutator) { /* ... */ }
    public MechanismConfig withFollowers(Follower... followers) { /* ... */ }
    public MechanismConfig withEncoder(int canId, Consumer<CANcoderConfiguration> mutator) { … }
    public MechanismConfig withSlots(SlotSpec... slots) { /* ... */ }
    public MechanismConfig withToleranceRot(double toleranceRot) { /* ... */ }
    public MechanismConfig withToleranceRps(double toleranceRps) { /* ... */ }
    public MechanismConfig withVisualizer(Visualizer visualizer) { /* ... */ }
    public MechanismConfig withSim(Sim sim) { /* ... */ }
    public MechanismConfig withSysId(SysIdSpec sysId) { /* ... */ }
    public MechanismConfig disabled() { /* controller = NONE */ }

    /** "Shooter/Hood" -> "Shooter Hood", for alert message prefixes. */
    public String displayName() { return name.replace('/', ' '); }
}
```

Three things this buys over revision 2's neutral records:

- **`gearRatio`, `Motor`, `CurrentLimits`, `SoftLimits`, `MotionMagic`, `GainSet` and the
  `Inversion`/`NeutralMode`/`GravityType` enums all disappear** — Phoenix already has them, and
  `rotorToReportedRatio()` derives what the sim needs from `fx.Feedback`.
- **The escape hatch disappears.** A Phoenix-only setting is just a field on `fx`.
- **`simOverrides` replaces the six `*IOTalonFXSim.getDefaultPID()` overrides**, and also lets
  sim switch off current limits and ramp rates — which the Poofs do deliberately
  (`TalonFXIO`: *"Current limits and ramp rates do not perform well in sim"*) and we currently
  do not.

`withFx(Consumer<TalonFXConfiguration>)` rather than exposing a mutable field keeps
`MechanismConfig` a value: each wither clones `fx` before mutating, so two configs can never
alias the same `TalonFXConfiguration`.

**The guardrail.** The cost of this decision is that a template class *could* read
`config.fx()`. That is not a promise to keep, it is a rule to enforce — see §7.3.

### 3.3 `MotorIO` and `EncoderIO` — split, and vendor-free

The config is now Phoenix-typed; the **IO boundary is not**, because it is the replay boundary.
Everything crossing it is a `double`, a `boolean`, or an enum from `Hardware`.

```java
package frc.robot.util.subsystems;

public interface MotorIO {

    @AutoLog
    class MotorInputs {
        public double positionRot = 0.0;
        public double velocityRps = 0.0;
        public double appliedVoltage = 0.0;
        public double appliedDutyCycle = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempC = 0.0;
        public boolean tempFault = false;
        public boolean connected = false;

        public MotorControlMode controlMode = MotorControlMode.Disabled;
        public double closedLoopSetpoint = 0.0;
        public double closedLoopOutput = 0.0;
    }

    default void updateInputs(MotorInputs inputs) {}

    /** Gains currently on the device for the slot serving {@code mode}. */
    default PIDConstants defaultGains(MotorControlMode mode) { return PIDConstants.ZERO; }

    default void setGains(MotorControlMode mode, PIDConstants gains) {}

    default void setMotionMagic(double cruiseRps, double accelRps2, double jerkRps3) {}

    default void setDutyCycle(double dutyCycle) {}
    default void setVoltage(double volts) {}
    default void setPositionRot(double positionRot) {}
    default void setVelocityRps(double velocityRps) {}
    default void stop() {}

    // ---- homing seam, unused today (§1.3) ----
    default void setCurrentPositionRot(double positionRot) {}
    default void setSoftLimitsEnabled(boolean forward, boolean reverse) {}
}
```

```java
public interface EncoderIO {

    @AutoLog
    class EncoderInputs {
        public double absolutePositionRot = 0.0;
        public double velocityRps = 0.0;
        public EncoderHealth health = EncoderHealth.Unknown;
        public boolean connected = false;
    }

    default void updateInputs(EncoderInputs inputs) {}
}
```

Three consequences of the split:

1. **No dead fields.** The four subsystems without an encoder have no `EncoderIO` and no
   `<name>/Encoder` log branch, instead of three zeroed fields.
2. **Followers are just more `MotorIO`s**, reusing `MotorInputs` — which is what tells you a
   follower is slipping or dead, not just its current draw. Logged under `<name>/Follower0`, …
3. **Encoder hardware is swappable independently of the motor controller.**

The CANcoder remains the Talon's *feedback source*, configured through `fx.Feedback` on the
motor side. `EncoderIO` is purely telemetry — which is all the current code uses it for.

`setGains(mode, …)` rather than `setGains(…)` is §7.7. `MotorIOTalonFX` resolves the mode to a
slot number and applies a `SlotConfigs` with `SlotNumber` set — verified present in Phoenix
26.1.1, along with `TalonFXConfigurator.apply(SlotConfigs)` and
`SlotConfigs.from(Slot0Configs)`.

### 3.4 `MechanismSubsystem`

```java
public abstract class MechanismSubsystem extends SubsystemBase {

    protected final MechanismConfig config;
    protected final MotorIO motor;
    protected final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

    private final MotorIO[] followers;
    private final MotorInputsAutoLogged[] followerInputs;

    private final EncoderIO encoder;                       // no-op instance when absent
    private final EncoderInputsAutoLogged encoderInputs = new EncoderInputsAutoLogged();

    protected final LoggedSetpointTracker setpointTracker;

    private final GainTuner gainTuner;                     // LoggedTunableNumbers, per slot
    private final MotionMagicTuner motionMagicTuner;       // null when no Motion Magic
    private final MechanismVisualizer visualizer;          // null when Shape.NONE

    protected MechanismSubsystem(
            MechanismConfig config, MechanismIOBundle io, MotorControlMode... trackedModes) {
        super(config.name());
        // ...
    }

    // ---- setters (no Command factories; template is setters-only) ----

    public final void setDutyCycle(double dutyCycle) {
        double clamped = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        motor.setDutyCycle(clamped);
        setpointTracker.updateSetpoint(clamped, MotorControlMode.DutyCycle);
    }

    public final void setVoltage(double volts) { /* clamp ±12, track */ }

    public final void stop() {
        motor.stop();
        setpointTracker.setControlMode(MotorControlMode.Disabled);
    }

    // ---- reads ----

    public final double getPositionRot()                 { return inputs.positionRot; }
    public final double getVelocityRps()                 { return inputs.velocityRps; }
    public final boolean isConnected()                   { return inputs.connected; }
    public final double getEncoderAbsolutePositionRot()  { return encoderInputs.absolutePositionRot; }

    // ---- loop ----

    @Override
    public final void periodic() {
        motor.updateInputs(inputs);
        Logger.processInputs(config.name(), inputs);

        for (int i = 0; i < followers.length; i++) {
            followers[i].updateInputs(followerInputs[i]);
            Logger.processInputs(config.name() + "/Follower" + i, followerInputs[i]);
        }
        if (config.encoder() != null) {
            encoder.updateInputs(encoderInputs);
            Logger.processInputs(config.name() + "/Encoder", encoderInputs);
        }

        if (DriverStation.isDisabled()) {
            setpointTracker.setControlMode(MotorControlMode.Disabled);
        }

        setpointTracker.logAll();
        logAtSetpoint();

        if (visualizer != null) {
            visualizer.update(config.visualizer().source() == Source.ENCODER_ABSOLUTE
                ? encoderInputs.absolutePositionRot
                : inputs.positionRot);
        }

        onPeriodic();

        gainTuner.applyIfChanged(motor);
        if (motionMagicTuner != null) {
            motionMagicTuner.applyIfChanged(motor);
        }

        LoggedTracer.record(config.name());
    }

    /** Mechanism-specific per-loop work: extra visualization, derived state. */
    protected void onPeriodic() {}

    /** Implemented by ServoSubsystem / RollerSubsystem. */
    protected abstract void logAtSetpoint();

    /** Exposed so a subsystem can append bespoke ligaments (see PivotSubsystem). */
    protected final MechanismVisualizer visualizer() { return visualizer; }
}
```

`periodic()` is `final` with an `onPeriodic()` hook so the fixed ordering — inputs before
`processInputs` before setpoint logging before `LoggedTracer.record` — cannot be broken by a
subclass forgetting `super.periodic()`.

`MechanismIOBundle` is a record — `(MotorIO motor, MotorIO[] followers, EncoderIO encoder)` — so
the constructor takes one argument instead of three.

```java
public abstract class ServoSubsystem extends MechanismSubsystem {

    private final double reverseLimitRot;
    private final double forwardLimitRot;

    protected ServoSubsystem(MechanismConfig config, MechanismIOBundle io) {
        super(config, io,
            MotorControlMode.DutyCycle, MotorControlMode.Voltage, MotorControlMode.Position);
        // Derived accessors, so no template class ever touches config.fx() (§7.3).
        clampToSoftLimits = config.hasSoftLimits();
        reverseLimitRot = config.reverseSoftLimitRot();
        forwardLimitRot = config.forwardSoftLimitRot();
    }

    public final void setPositionRot(double positionRot) {
        double clamped = MathUtil.clamp(positionRot, reverseLimitRot, forwardLimitRot);
        motor.setPositionRot(clamped);
        setpointTracker.updateSetpoint(clamped, MotorControlMode.Position);
    }

    public final boolean atPositionSetpoint() {
        return setpointTracker.atSetpoint(
            MotorControlMode.Position, inputs.positionRot, config.tolerance());
    }

    @Override
    protected void logAtSetpoint() {
        Logger.recordOutput(config.name() + "/atPositionSetpoint", atPositionSetpoint());
    }
}
```

`RollerSubsystem` is the same shape with `Velocity` in place of `Position` and no clamp.

Revision 2 had this constructor reach into `fx.SoftwareLimitSwitch` directly and carved it out of
the §7.3 guardrail. Reading it through derived accessors on `MechanismConfig` instead means **no
template class reads `config.fx()` at all**, and the same trick covers the Motion Magic defaults
(`motionMagicCruiseRps()` and friends). The guardrail's allowlist got smaller as a result.

The double clamp — subsystem-side `MathUtil.clamp` *and* Talon soft limits — is preserved: the
clamp makes the *logged* setpoint match what the mechanism will actually chase.

### 3.5 `MechanismVisualizer` — sizes in config, defaults per kind

With dimensions in config, the two shapes cover five of six mechanisms:

| Subsystem | Shape | Config | Override needed |
| --- | --- | --- | --- |
| `shooter.hood` | `ARM` | `arm(0.4)` | none |
| `intake.pivot` | `ARM` | `arm(0.333).withCanvas(0.673, 0.381).withRoot(0.229, 0.038).withSource(ENCODER_ABSOLUTE)` | wall ligament |
| `intake.roller` | `SPINNER` | `spinner(0.2)` | none |
| `shooter.tower` | `SPINNER` | `spinner(0.3)` | none |
| `shooter.flywheel` | `SPINNER` | `spinner(0.4)` | none |
| `hopper` | `SPINNER` | `spinner(0.4, 4)` | none |

The hopper's four vanes fall out of `spokes = 4` rather than a bespoke loop — `RollerMechanism2D`
already builds one spike plus an N-sided polygon, so generalizing it to N spikes is a small
change to code that already exists.

```java
public class MechanismVisualizer {

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d[] driven;   // arm: 1; spinner: `spokes`
    private final double spokeSpacingDeg;

    /** Returns null for Shape.NONE. */
    static MechanismVisualizer create(MechanismConfig.Visualizer spec) { /* ... */ }

    public LoggedMechanism2d mechanism2d() { return mechanism; }
    public LoggedMechanismRoot2d root()    { return root; }

    void update(double positionRot) {
        double deg = Units.rotationsToDegrees(positionRot);
        for (int i = 0; i < driven.length; i++) {
            driven[i].setAngle(deg + i * spokeSpacingDeg);
        }
    }
}
```

**Log key mechanics — resolved.** The `LoggedMechanism2d` lives on `MechanismSubsystem`, so
`@AutoLogOutput(key = "...")` cannot carry a per-subsystem literal, and revision 2 left this as
an open question with a `Logger.recordOutput` fallback.

Neither was needed. AdvantageKit's `AutoLogOutputManager.makeKey` substitutes `{fieldName}`
placeholders in a key from a field on the annotated field's **declaring** class (verified by
disassembling `akit-java` 26.0.0: it tracks a `FieldAndDeclaringClass` per field and resolves
placeholders with `getDeclaredField`). So the base class carries:

```java
/** Backs the {@code {logKey}} substitution in the AutoLogOutput key below. */
private final String logKey;

@AutoLogOutput(key = "{logKey}/Mechanism2d")
private final LoggedMechanism2d mechanism2d;
```

with `logKey = config.name()`, giving `Shooter/Hood/Mechanism2d`. The annotation stays on the
field, as the house convention requires, and there is no per-loop `recordOutput` call.

**Sim orientation is not what it looks like.** Four sim IOs derive `ChassisReference` from the
inversion constant; `HoodIOTalonFXSim` hardcodes `Clockwise_Positive` even though the hood is
configured `CounterClockwise_Positive`. Naively centralizing "orientation = f(Inverted)" would
therefore break the hood. `MechanismConfig.simOrientation()` (§3.2) accounts for the second sign
flip from a negative `RotorToSensorRatio`, and reproduces all six current values:

| Subsystem | `Inverted` | `RotorToSensorRatio` | Derived | Current code |
| --- | --- | --- | --- | --- |
| `shooter.hood` | CCW+ | −244.411765 | Clockwise+ | Clockwise+ ✔ |
| `intake.pivot` | CCW+ (default) | +20.0 | CounterClockwise+ | CounterClockwise+ ✔ |
| `hopper` | CCW+ | +1.0 (default) | CounterClockwise+ | derived from `HOPPER_INVERTED` ✔ |
| `shooter.tower` | CW+ | +1.0 (default) | Clockwise+ | derived from `HOPPER_INVERTED` ✔ |
| `shooter.flywheel` | CW+ | +1.0 (default) | Clockwise+ | derived from `F_INVERTED_VALUE` ✔ |
| `intake.roller` | CCW+ | +1.0 (default) | CounterClockwise+ | derived from `ROLLER_INVERTED` ✔ |

Pivot's bespoke wall is the only visualization override left:

```java
public class PivotSubsystem extends ServoSubsystem {

    private final LoggedMechanismRoot2d wallRoot;

    public PivotSubsystem(MechanismIOBundle io) {
        super(IntakePivotConfig.CONFIG, io);
        wallRoot = visualizer().mechanism2d().getRoot("WallRoot", ROOT_X, ROOT_Y);
        wallRoot.append(new LoggedMechanismLigament2d(
            "Wall", WALL_HEIGHT_M, 90.0, 6.0, new Color8Bit(Color.kDarkRed)));
    }

    @Override
    protected void onPeriodic() {
        wallRoot.setPosition(
            ROOT_X + Units.inchesToMeters(getWallDistanceFromPivotRoot(
                Units.rotationsToRadians(getEncoderAbsolutePositionRot()))),
            ROOT_Y);
    }
}
```

### 3.6 `PhoenixUtil` split, and `TalonFXTranslator`

`PhoenixUtil.toMotorControlMode(ControlModeValue)` and `toEncoderHealth(MagnetHealthValue)` are
Phoenix→ours translation, so they move into `TalonFXTranslator`. What is left in `PhoenixUtil`
is bus-level utility with no translation in it:

```java
// frc/robot/util/PhoenixUtil.java — after
public class PhoenixUtil {
    public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) { … }
    public static StatusCode tryUntilOk(int n, Supplier<StatusCode> cmd, Alert alert) { … }
    public static void registerSignals(CANType canType, BaseStatusSignal... signals) { … }
    public static void registerSignals(CANType canType, List<BaseStatusSignal> signals) { … }
    public static void refreshAllStatusSignals() { … }
}
```

```java
// frc/robot/util/subsystems/talonfx/TalonFXTranslator.java
public final class TalonFXTranslator {

    // inbound: Phoenix status -> our vocabulary (moved from PhoenixUtil, unchanged bodies)
    public static Hardware.MotorControlMode toMotorControlMode(ControlModeValue value) { … }
    public static Hardware.EncoderHealth toEncoderHealth(MagnetHealthValue value) { … }

    // slot plumbing (§7.7)
    static SlotConfigs slotConfigs(TalonFXConfiguration fx, int slot) { … }
    static PIDConstants toPidConstants(SlotConfigs slot) { … }

    // control requests
    static ControlRequest positionRequest(Hardware.ClosedLoopOutput output) { … }
    static ControlRequest velocityRequest(Hardware.ClosedLoopOutput output) { … }
}
```

Under §7.3 there is no outbound *config* translation left — `fx` goes straight to
`configurator.apply(...)`. What remains is status translation, slot plumbing and request
selection.

The two inbound methods are `public` (swerve's `ModuleIOTalonFX` calls `toMotorControlMode`);
everything else is package-private.

### 3.7 `SysIdFactory`

SysId is a helper, not a `Command` on the base class, keeping the template setters-only:

```java
package frc.robot.util;

public final class SysIdFactory {

    private SysIdFactory() {}

    public static SysIdRoutine routine(
            Subsystem subsystem, String logKey,
            MechanismConfig.SysIdSpec spec, DoubleConsumer setVoltage) { … }

    /** quasistatic fwd → quasistatic rev → dynamic fwd → dynamic rev. */
    public static Command fullSweep(SysIdRoutine routine) { … }
}
```

Two lines per subsystem instead of eighteen.

---

## 4. Hardware and mode selection

### 4.1 The problem being solved

Today, `RobotContainer` contains this:

```java
private final LoggedCanivore swerveCan = new LoggedCanivore(CANType.SWERVE);
private final LoggedCanivore mechCan = new LoggedCanivore(CANType.MECH);
// ...
switch (Constants.CURRENT_MODE) {
    case REAL:
        pivot    = new PivotSubsystem(new PivotIOTalonFX(mechCan));
        roller   = new RollerSubsystem(new RollerIOTalonFX(mechCan));
        hopper   = new HopperSubsystem(new HopperIOTalonFX(mechCan));
        tower    = new TowerSubsystem(new TowerIOTalonFX(mechCan));
        flywheel = new FlywheelSubsystem(new FlywheelIOTalonFX(mechCan));
        hood     = new HoodSubsystem(new HoodIOTalonFX(mechCan));
        break;
    case SIM:   /* the same six lines with IOTalonFXSim */   break;
    case REPLAY:
    default:    /* the same six lines with `new XIO() {}` */ break;
}
```

Two separable things are tangled: **the mode decision**, written once per subsystem per mode
(18 lines for 6 mechanisms), and **the bus handles**, which are fields of `RobotContainer` purely
because every `IOTalonFX` constructor needs one.

### 4.2 `CanBuses`

`LoggedCanivore` extends Phoenix's `CANBus`, spawns a daemon polling thread in its constructor,
self-registers into a static list for `LoggedCanivore.updateCanivoreStatuses()`, and throws for
`CANType.RIO`. There must be exactly one instance per physical bus — today enforced only by
"they happen to be fields of `RobotContainer`".

```java
package frc.robot.util;

public final class CanBuses {

    private static final EnumMap<CANType, LoggedCanivore> BUSES = new EnumMap<>(CANType.class);

    private CanBuses() {}

    /** One LoggedCanivore per bus, created on first use. */
    public static synchronized LoggedCanivore of(CANType type) {
        return BUSES.computeIfAbsent(type, LoggedCanivore::new);
    }
}
```

That is the whole class. It is *lazy* on purpose: in REPLAY no IO ever asks for a bus, so no
CANivore is constructed and no polling thread starts.

### 4.3 `MechanismHardware`

```java
package frc.robot.util.subsystems;

/** The one place the real/sim/replay and vendor decisions are made. */
public final class MechanismHardware {

    private MechanismHardware() {}

    public static MechanismIOBundle create(MechanismConfig config) {

        // 1. Replay, or a deliberately disabled mechanism: nothing is constructed.
        if (Constants.CURRENT_MODE == Mode.REPLAY || config.controller() == Controller.NONE) {
            return MechanismIOBundle.noOp(config);
        }

        boolean sim = Constants.CURRENT_MODE == Mode.SIM;

        switch (config.controller()) {
            case TALON_FX: {
                LoggedCanivore bus = CanBuses.of(config.bus());

                // 2. Leader.
                MotorIOTalonFX leader = sim
                    ? new MotorIOTalonFXSim(config, config.canId(), bus)
                    : new MotorIOTalonFX(config, config.canId(), bus);

                // 3. Followers — same class, follower CAN ID, told to follow the leader.
                MotorIO[] followers = new MotorIO[config.followers().length];
                for (int i = 0; i < followers.length; i++) { … }

                // 4. Encoder. In sim it is driven by the leader's plant state — the one
                //    piece of wiring that genuinely needs a central place.
                EncoderIO encoder = config.encoder() == null
                    ? new EncoderIO() {}
                    : sim
                        ? new EncoderIOCancoderSim(
                              config, bus, ((MotorIOTalonFXSim) leader)::mechanismState)
                        : new EncoderIOCancoder(config, bus);

                return new MechanismIOBundle(leader, followers, encoder);
            }
            default:
                throw new IllegalStateException("No IO for controller " + config.controller());
        }
    }
}
```

**How the real/sim switch works.** It is the same decision `RobotContainer` makes today, taken
once per mechanism instead of once per mechanism per mode:

- `Constants.CURRENT_MODE` is already `RobotBase.isReal() ? Mode.REAL : SIM_MODE`. Nothing about
  how the mode is *determined* changes.
- **REAL** → `MotorIOTalonFX`: constructs a `TalonFX`, applies `config.fx()`, registers status
  signals.
- **SIM** → `MotorIOTalonFXSim extends MotorIOTalonFX`: same Talon, same `fx` with
  `config.simOverrides()` applied on top, plus a WPILib plant (`SingleJointedArmSim` or
  `DCMotorSim`, chosen from `config.sim()`) driven through `TalonFXSimState`. This is exactly
  what the six `*IOTalonFXSim` classes do today; the subclass relationship is unchanged.
- **REPLAY** → `MechanismIOBundle.noOp(config)`: `MotorIO`/`EncoderIO` instances with every
  method defaulted. `updateInputs` writes nothing; AdvantageKit replays recorded values into the
  Inputs objects instead. Identical to today's `new PivotIO() {}`.

**As built, the vendor wiring moved one level down.** `MechanismHardware` makes only the
mode-and-vendor decision and hands off to a single public entry point per vendor package:

```java
switch (config.controller()) {
    case TALON_FX:
        return TalonFXHardware.create(config, CanBuses.of(config.bus()), simulated);
    ...
}
```

`TalonFXHardware.create` does the leader/follower/encoder assembly shown below. The payoff is
that **every other class in `util/subsystems/talonfx` is package-private** — `MotorIOTalonFX`,
`MotorIOTalonFXSim`, `EncoderIOCancoder`, `EncoderIOCancoderSim` and the translator are all
invisible outside the vendor layer, and adding a vendor means adding a sibling package plus one
`case`. It also softens §4.4's first objection: with this shape, even a
`RobotContainer`-hosted factory would not need the IO classes public.

**Step 4 is the part that could not live in the IO classes themselves.** In sim the CANcoder's
simulated position must come from the motor's plant — today `HoodIOTalonFXSim` owns both sim
states and writes `cancoderSimState.setRawPosition(...)` itself. With the IOs split, something
has to hand the encoder a live view of the plant:

```java
public record MechanismState(double positionRot, double velocityRps) {}
```

`MotorIOTalonFXSim.mechanismState()` returns the current plant state (output-shaft side);
`EncoderIOCancoderSim` calls it each `updateInputs` and writes the `CANcoderSimState`, scaling by
`fx.Feedback.SensorToMechanismRatio`. Same technique as the Poofs' `getSupplierForCancoder`.

### 4.4 Why not just leave this in `RobotContainer`?

It can. `MechanismHardware.create` is a pure static function of a config, so it works equally
well as a private static method:

```java
public class RobotContainer {

    private static MechanismIOBundle io(MechanismConfig config) {
        /* the exact body of MechanismHardware.create */
    }

    private final IntakeRollerSubsystem roller = new IntakeRollerSubsystem(io(IntakeRollerConfig.CONFIG));
    private final PivotSubsystem       pivot  = new PivotSubsystem(io(IntakePivotConfig.CONFIG));
    // ...
}
```

That satisfies your original non-goal — the six declarations read identically in REAL, SIM and
REPLAY, and the branching exists once rather than eighteen times. Two costs, both concrete:

**1. Every vendor IO class becomes public API.** With the factory in `util/subsystems`,
`MotorIOTalonFX`, `MotorIOTalonFXSim`, `EncoderIOCancoder` and `EncoderIOCancoderSim` can be
package-private in `util/subsystems/talonfx/` — the only thing outside that package that
constructs them is `MechanismHardware`, its sibling. Move the factory to `frc.robot` and all four
must be public, `RobotContainer` imports `frc.robot.util.subsystems.talonfx.*`, and the cast
`((MotorIOTalonFXSim) leader)::mechanismState` — sim-wiring trivia — sits in the file that is
supposed to describe the robot.

**2. Subsystems lose their no-arg constructor.** `new HoodSubsystem()` requires the subsystem to
call the factory itself, which means `frc.robot.subsystems.shooter.hood` imports
`frc.robot.RobotContainer` — a subsystem depending on the container that owns it. So with the
factory in `RobotContainer`, every declaration must pass the bundle explicitly, as above.

Neither is fatal. **Recommendation: keep the separate class** — it is ~30 lines, it keeps the
vendor package's internals private, and it lets `new HoodSubsystem()` work in tests and one-off
tools without constructing `RobotContainer` (which would build controllers, vision, swerve and
the auto chooser as a side effect). But if you would rather not add a class, the
`RobotContainer`-hosted version is a legitimate implementation of the same design and nothing
else in this document changes.

### 4.5 What `RobotContainer` looks like afterwards

```java
// identical text in REAL, SIM and REPLAY
private final IntakeRollerSubsystem roller   = new IntakeRollerSubsystem();
private final PivotSubsystem        pivot    = new PivotSubsystem();
private final HopperSubsystem       hopper   = new HopperSubsystem();
private final TowerSubsystem        tower    = new TowerSubsystem();
private final FlywheelSubsystem     flywheel = new FlywheelSubsystem();
private final HoodSubsystem         hood     = new HoodSubsystem();
```

The 27-line `switch` and both `LoggedCanivore` fields are gone. Each subsystem carries a
two-constructor pattern:

```java
public HoodSubsystem() {
    this(MechanismHardware.create(HoodConfig.CONFIG));
}

/** Injection point for tests and for a hand-built IO. */
public HoodSubsystem(MechanismIOBundle io) {
    super(HoodConfig.CONFIG, io);
}
```

**Adding a vendor** costs one `Controller` enum constant, one `case`, and the new IO classes.
Java's exhaustive `switch` makes a forgotten case a compile error. (A self-registering
`Map<Controller, Factory>` was considered and rejected: it depends on class-initialization order,
defeats the exhaustiveness check, and buys nothing at our scale.)

---

## 5. Worked example — `shooter.hood` (servo, encoder, no followers)

The hardest of the six: torque-current closed loop, an inverted encoder, a 244:1 reduction, soft
limits, both current limits, and an arm sim plant.

### 5.1 `subsystems/shooter/hood/HoodConfig.java`

```java
package frc.robot.subsystems.shooter.hood;

/** Every device setting and mechanism constant for the shooter hood. */
public final class HoodConfig {

    private HoodConfig() {}

    /** Compile-time constant so log keys have exactly one source of truth. */
    public static final String LOG_KEY = "Shooter/Hood";

    private static final int MOTOR_ID = 16;
    private static final int ENCODER_ID = 18;
    private static final double GEAR_RATIO = 244.411765;

    public static final double LOWER_LIMIT_ROT = 0.0;
    public static final double UPPER_LIMIT_ROT = 0.1;
    public static final double INIT_ANGLE_ROT = UPPER_LIMIT_ROT;
    public static final double TOLERANCE_ROT = 0.01;

    private static final double MOI_KG_M2 = 598.456909 * Constants.LB_IN2_TO_KG_M2;
    private static final double COM_LENGTH_M =
        Units.inchesToMeters(Math.hypot(0.121549, 9.035458));

    public static final MechanismConfig CONFIG =
        MechanismConfig.of(LOG_KEY, CANType.MECH, MOTOR_ID)
            .withFx(fx -> {
                fx.MotorOutput
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake);

                fx.CurrentLimits
                    .withStatorCurrentLimit(50.0).withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40.0).withSupplyCurrentLimitEnable(true);

                // Encoder counts run opposite the mechanism convention: negate the rotor->
                // sensor ratio and the sensor->mechanism ratio so mechanism position comes
                // out in the desired direction.
                fx.Feedback
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withFeedbackRemoteSensorID(ENCODER_ID)
                    .withRotorToSensorRatio(-GEAR_RATIO)
                    .withSensorToMechanismRatio(-1.0);

                fx.SoftwareLimitSwitch
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(UPPER_LIMIT_ROT)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(LOWER_LIMIT_ROT);

                fx.Slot0.withKP(2000).withKD(60).withKS(120);
            })
            .withSimOverrides(fx -> fx.Slot0.withKP(60).withKD(2).withKS(0))
            .withEncoder(ENCODER_ID, cc -> cc.MagnetSensor
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive))
            .withSlots(SlotSpec.position().withOutput(ClosedLoopOutput.TorqueCurrent))
            .withToleranceRot(TOLERANCE_ROT)
            .withVisualizer(Visualizer.arm(0.4)
                .withColor(new Color8Bit(Color.kCornflowerBlue)))
            .withSim(new Sim.Arm(
                DCMotor.getKrakenX60Foc(1), GEAR_RATIO, MOI_KG_M2, COM_LENGTH_M,
                LOWER_LIMIT_ROT, UPPER_LIMIT_ROT, false, INIT_ANGLE_ROT))
            .withSysId(SysIdSpec.SERVO);
}
```

Note what is *not* here: no `AbsoluteSensorDiscontinuityPoint` (derived from the soft limits by
`EncoderIOCancoder`, as today), and no `ChassisReference` (derived by
`MechanismConfig.simOrientation()`, §3.5).

Roughly 55 lines, replacing `ShooterConstants.Hood` plus 265 lines of `HoodIOTalonFX` plus 70 of
`HoodIOTalonFXSim` plus 47 of `HoodIO`. Every device line is a verbatim copy of what
`HoodIOTalonFX` writes today, which makes the migration diff reviewable field by field.

### 5.2 `subsystems/shooter/hood/HoodSubsystem.java`

```java
public class HoodSubsystem extends ServoSubsystem {

    public HoodSubsystem() {
        this(MechanismHardware.create(HoodConfig.CONFIG));
    }

    public HoodSubsystem(MechanismIOBundle io) {
        super(HoodConfig.CONFIG, io);
    }

    public Command goToPosition(double positionRot) {
        return this.runOnce(() -> setPositionRot(positionRot));
    }

    public Command holdPosition(double positionRot) {
        return this.startEnd(() -> setPositionRot(positionRot), this::stop);
    }

    public Command holdDownHood() {
        return this.run(() -> setPositionRot(HoodConfig.LOWER_LIMIT_ROT));
    }

    public Command hideHood() {
        return this.runOnce(() -> setPositionRot(HoodConfig.LOWER_LIMIT_ROT))
            .andThen(Commands.waitUntil(this::atPositionSetpoint));
    }

    public Command jiggleHood() { /* unchanged */ }

    public Command setHoodManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command stopHood() {
        return this.runOnce(this::stop);
    }

    public Command runSysId() {
        return SysIdFactory.fullSweep(SysIdFactory.routine(
            this, HoodConfig.LOG_KEY, HoodConfig.CONFIG.sysId(), this::setVoltage));
    }
}
```

199 lines becomes about 50, with no Mechanism2d code at all — the arm comes from
`Visualizer.arm(0.4)`.

### 5.3 The TalonFX side

There is no hood-specific IO file, and under §7.3 there is no config translation either:

```java
class MotorIOTalonFX implements MotorIO {

    protected final TalonFX talon;
    protected final MechanismConfig config;
    private final EnumMap<MotorControlMode, SlotSpec> slotByMode;
    private final ControlRequest positionRequest;
    private final ControlRequest velocityRequest;

    MotorIOTalonFX(MechanismConfig config, int canId, LoggedCanivore bus) {
        this.config = config;
        this.talon = new TalonFX(canId, bus);

        TalonFXConfiguration fx = config.fx();            // already a defensive copy
        if (Constants.CURRENT_MODE == Mode.SIM) {
            config.simOverrides().accept(fx);
        }
        tryUntilOk(5, () -> talon.getConfigurator().apply(fx), failedToConfigureAlert);

        slotByMode = /* index config.slots() by mode, validating 0..2 and no duplicates */;
        positionRequest = TalonFXTranslator.positionRequest(
            slotByMode.get(MotorControlMode.Position).output());
        // ... signals, alerts, registerSignals — as today
    }

    @Override
    public void setPositionRot(double positionRot) {
        talon.setControl(withPosition(positionRequest, positionRot)
            .withSlot(slotByMode.get(MotorControlMode.Position).slot()));
    }

    @Override
    public void setGains(MotorControlMode mode, PIDConstants gains) {
        int slot = slotByMode.get(mode).slot();
        SlotConfigs slotConfigs = TalonFXTranslator.slotConfigs(config.fx(), slot);
        slotConfigs.SlotNumber = slot;
        // apply kP..kA from `gains`
        tryUntilOk(5, () -> talon.getConfigurator().apply(slotConfigs), gainsNotSetAlert);
    }
}
```

`SlotConfigs` with a public `SlotNumber` field, `SlotConfigs.from(Slot0Configs)` and
`TalonFXConfigurator.apply(SlotConfigs)` are all present in Phoenix 26.1.1 (verified against the
vendor jar), so slot-generic gain tuning is one code path rather than a three-way switch.

The CANcoder is configured exactly as today — refresh first so the device-stored magnet offset
survives, then set direction and the discontinuity point derived from the soft limits:

```java
double mid = (fx.SoftwareLimitSwitch.ReverseSoftLimitThreshold
            + fx.SoftwareLimitSwitch.ForwardSoftLimitThreshold) / 2.0;
double discontinuity = ((mid + 0.5) % 1.0 + 1.0) % 1.0;
```

Signal frequency (100 Hz) and `optimizeBusUtilization(0, 1.0)` are constants in the IO — all six
subsystems use identical values.

### 5.4 Reporting ratio vs mechanical reduction — and the tower

**Correcting revision 2.** It claimed `shooter.tower` had a bug making real-robot velocity 4× sim
velocity. Re-deriving:

- **Real:** `TowerIOTalonFX` never calls `withFeedback`, so `SensorToMechanismRatio` is 1.0 and
  `motor.getVelocity()` returns **rotor** RPS.
- **Sim:** `DCMotorSim(createDCMotorSystem(gearbox, moi, 4.0), gearbox)` models the **output
  shaft**, and `TowerIOTalonFXSim` then writes `setRotorVelocity(mechanismRps * 4.0)`. The Talon
  reports that unchanged (ratio 1.0) — **also rotor RPS.**

Sim and real agree. The tower is self-consistent; it simply reports in rotor units where the
other five report in mechanism units. The real consequence is a readability wart:
`TowerConstants.TARGET_VELO_RPS = 30.0` and `VELOCITY_TOLERANCE_RPS = 7.58` are rotor RPS, while
`HopperConstants.TARGET_RPS` is mechanism RPS, and the two `_rps` log keys mean different things.

**Per your decision, the tower's config states 1:1 explicitly:**

```java
fx.Feedback.withSensorToMechanismRatio(1.0);   // reports rotor rotations, as today
// ...
.withSim(Sim.RotatingMass.fromKa(DCMotor.getKrakenX60Foc(1), 4.0, TowerConstants.kA))
//                                                           ^ mechanical reduction, unchanged
```

Real-robot behavior is byte-identical to today; the intent is now written down instead of being
a defaulted field. `TowerConstants.GEAR_REDUCTION = 4.0` survives only as the sim plant's
reduction.

**This is what revealed the design flaw in revision 2.** It used one `gearRatio` field for two
different quantities:

| Subsystem | Mechanical reduction (sim plant) | Rotor-to-reported ratio (`fx.Feedback`) |
| --- | --- | --- |
| `shooter.hood` | 244.411765 | 244.411765 (= −244.411765 × −1.0) |
| `intake.pivot` | 20.0 | 20.0 |
| `hopper` | 4.0 | 4.0 |
| `intake.roller` | 1.0 | 1.0 |
| `shooter.flywheel` | 1.0 | 1.0 |
| **`shooter.tower`** | **4.0** | **1.0** |

Equal for five of six, which is why one field looked sufficient. §3.2 now puts mechanical
reduction on the `Sim` record — where it belongs, since it is a property of the gearbox — and
derives the reporting ratio from `fx.Feedback` via `rotorToReportedRatio()`.

---

## 6. Worked example — `shooter.flywheel` (roller with a follower)

### 6.1 `subsystems/shooter/flywheel/FlywheelConfig.java`

```java
public final class FlywheelConfig {

    private FlywheelConfig() {}

    public static final String LOG_KEY = "Shooter/Flywheel";

    private static final int LEADER_ID = 17;
    private static final int FOLLOWER_ID = 25;

    public static final double MAX_SPEED_RPS = 120.0;
    public static final double TOLERANCE_RPS = 2.0;

    /** Flywheel kA is not characterized yet; this is a hand-picked plant inertia. */
    private static final double MOI_KG_M2 = 0.025;

    public static final MechanismConfig CONFIG =
        MechanismConfig.of(LOG_KEY, CANType.MECH, LEADER_ID)
            .withFx(fx -> {
                fx.MotorOutput
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast);

                // No current limits configured today — see §2.3, §11 item 1.

                fx.Feedback.withSensorToMechanismRatio(1.0);

                fx.Slot0.withKP(10).withKV(0.12);

                fx.MotionMagic                       // configured but unused, as today
                    .withMotionMagicCruiseVelocity(500.0)
                    .withMotionMagicAcceleration(100.0)
                    .withMotionMagicJerk(150.0);
            })
            .withSimOverrides(fx -> fx.Slot0.withKP(0.8).withKV(0))
            .withFollowers(new Follower(FOLLOWER_ID, true))   // opposed, as today
            .withSlots(SlotSpec.velocity())
            .withToleranceRps(TOLERANCE_RPS)
            .withVisualizer(Visualizer.spinner(0.4))
            .withSim(new Sim.RotatingMass(DCMotor.getKrakenX60Foc(2), 1.0, MOI_KG_M2))
            .withSysId(SysIdSpec.ROLLER);
}
```

The follower is one line. `MechanismHardware` builds a second `MotorIOTalonFX` on the follower's
CAN ID, applies the leader's `fx` to it (preserving today's behavior), and issues
`setControl(new Follower(leaderId, MotorAlignmentValue.Opposed))`. Its telemetry lands under
`Shooter/Flywheel/Follower0` using the full `MotorInputs`, so a slipping or dead follower shows
up in position and velocity, not just current draw.

`hopper` and `shooter.tower` use `Sim.RotatingMass.fromKa(...)` — the shared helper replacing the
four-line MOI derivation duplicated in three sim IOs today.

### 6.2 `subsystems/shooter/flywheel/FlywheelSubsystem.java`

```java
public class FlywheelSubsystem extends RollerSubsystem {

    public FlywheelSubsystem() {
        this(MechanismHardware.create(FlywheelConfig.CONFIG));
    }

    public FlywheelSubsystem(MechanismIOBundle io) {
        super(FlywheelConfig.CONFIG, io);
    }

    public Command rampToVelocity(DoubleSupplier rpsSupplier) {
        return this.run(() -> setVelocityRps(rpsSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command setFlywheelManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command stopFlywheel() {
        return this.runOnce(this::stop);
    }
}
```

164 lines becomes about 30. Call sites change only in method names: `setVelocity` →
`setVelocityRps`, `atSetpoint` → `atVelocitySetpoint`.

---

## 7. Design questions

### 7.1 Variant handling — **decided: Option D**

#### Option A — eight concrete base classes

Every combination is a named type; no optional fields. But five of eight would have zero
instances, the class count doubles when a third axis appears, and shared behavior must be
duplicated across siblings or hoisted into a base the leaves partially override.

The Poofs took a partial version of this route and it already broke. They have
`ServoMotorSubsystem`, `ServoMotorSubsystemWithFollowers` and `ServoMotorSubsystemWithCanCoder`
and **no class combining the last two** — so an elevator with followers *and* an absolute encoder
is unrepresentable in their hierarchy. That is the explosion arriving after two axes, in a
codebase written by people who are very good at this.

#### Option B — generics over the IO type

```java
public class ServoSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO>
        extends MechanismSubsystem<T, U> {
    protected U io;
    protected T inputs;
}

public class HoodSubsystem extends ServoSubsystem<HoodInputsAutoLogged, HoodIO> { … }
```

The premise: some mechanism will need Inputs fields or IO methods the template does not know
about, and generics let it have them without casting.

**Test the premise against our six subsystems.** Go through every field the current Inputs
classes have beyond the common twelve:

| Extra field | Subsystem | Where it goes in this design |
| --- | --- | --- |
| `accelerationRPSPerSec` | hood | **Deleted** — nothing reads it (§10 Phase 0) |
| `encoderAbsolutePositionRot`, `encoderHealth`, `encoderConnected` | pivot, hood | `EncoderInputs` (§3.3) |
| `followerAppliedVoltage`, `followerSupplyCurrentAmps`, `followerTorqueCurrentAmps`, `followerStatorCurrentAmps`, `followerTempC`, `followerConnected` | flywheel | `MotorInputs` on a second `MotorIO` (§3.3) |

**After the split, zero of our six need a specialized Inputs type.** The generic parameter would
be `<MotorInputsAutoLogged, MotorIO>` at all six use sites — exactly the outcome in the Poofs'
codebase, where every instantiation is the base type.

Now check the other direction — the state our subsystems *do* carry beyond the template:

```java
// PivotSubsystem — empirical Desmos fit, pure math, no hardware
static double getWallDistanceFromPivotRoot(double theta) { … }

// RollerSubsystem — dashboard-tunable domain values
private final LoggedTunableNumber inSpeed  = new LoggedTunableNumber("Roller/InSpeed_rps", …);
private final LoggedTunableNumber outSpeed = new LoggedTunableNumber("Roller/OutSpeed_rps", …);

// HoodSubsystem's caller in RobotContainer
private double desiredHoodSpeed = 0;
```

None of it is hardware state. That is the structural reason, not an accident: **the extra state a
mechanism needs is almost never IO state.** When a mechanism genuinely needs another *device* — a
beam break, a limit switch — the honest answer is a second IO interface, which is exactly the
pattern §3.3 establishes with `EncoderIO`, and it needs no generics.

**Cost if adopted:** ~8 lines of angle brackets per template class; signatures like
`ServoMotorSubsystemWithCanCoder<MotorInputsAutoLogged, MotorIO, CanCoderInputsAutoLogged,
CanCoderIO>` (100 characters before the class name, four times over in their codebase); a
barrier to a student adding a subsystem. **Benefit:** none demonstrable.

#### Option C — composition of behavior objects

```java
public class MechanismSubsystem extends SubsystemBase {
    private final List<MechanismFeature> features;
    @Override public void periodic() {
        motor.updateInputs(inputs);
        features.forEach(f -> f.update(inputs));
    }
}
```

**Where it is right, and this design already uses it.** `GainTuner`, `MotionMagicTuner`,
`MechanismVisualizer` and `SysIdFactory` are exactly this pattern — owned and delegated to, not
inherited. Where a feature is RIO-side bookkeeping, composition is correct and the design takes
it.

**Where it fails — from our code.** The two features that motivated the eight-variant question
are firmware capabilities, not RIO-side ones.

A follower, in `FlywheelIOTalonFX`:

```java
// constructor — issued once, and never again anywhere in the file
tryUntilOk(5,
    () -> follower.setControl(
        new Follower(ShooterConstants.Flywheel.UPPER_MOTOR_ID, MotorAlignmentValue.Opposed)),
    failedToSetFollowerAlert);
```

Grep the rest of `FlywheelIOTalonFX`: `follower` appears only in config application and signal
reads. A RIO-side `FollowerGroup.update()` would do nothing but copy telemetry — a class with no
behavior.

An encoder, in `HoodIOTalonFX`:

```java
config.withFeedback(new FeedbackConfigs()
    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
    .withFeedbackRemoteSensorID(cancoder.getDeviceID())
    .withRotorToSensorRatio(-1.0 * ShooterConstants.Hood.GEAR_RATIO)
    .withSensorToMechanismRatio(-1.0));
// ...
inputs.positionRot = position.getValueAsDouble();   // ← this IS the CANcoder, via the Talon
```

An `AbsoluteEncoder` feature "providing position" would compete with a value that already arrives
through the motor's own inputs — two sources of truth for one number, differing only in whether
the Talon's ratio math has been applied. (`HoodSubsystem` uses `inputs.positionRot`;
`PivotSubsystem` uses `inputs.encoderAbsolutePositionRot` — for its *visualization only*. That
divergence is preserved by `Visualizer.Source`, not by a feature object.)

Two further costs: `Logger.processInputs` needs a stable key per Inputs object, so each feature
would own its key-building and a reordered list would silently reshuffle the log tree; and the
ordering guarantees in `periodic()` would become emergent from list order rather than written
down.

**Verdict:** composition for RIO-side concerns (already done); not for hardware topology.

#### Option D (chosen) — two subsystem classes, optional features in the IO layer

The observation Option C stumbles on is the one that makes this work: **followers and external
encoders are invisible above the IO boundary.** So both axes leave the type system, and the
2 × 2 × 2 explosion becomes:

```
MechanismSubsystem   (abstract)   inputs, logging, gains, setpoints, viz, disabled handling
 ├── ServoSubsystem                position setpoint, soft-limit clamp, atPositionSetpoint
 └── RollerSubsystem               velocity setpoint, atVelocitySetpoint
```

**Concretely, using our own code.** Here is what happens to each variant-specific block.

*The follower block* — `FlywheelIOTalonFX` lines 96–97, 122–129, 144–148, 164–169, 205–213:
declaring `follower`, applying config, `setControl(new Follower(...))`, five follower
`StatusSignal`s, a `followerSignals` list, and six `inputs.follower*` copy-outs. All of it
becomes one line of config plus a loop in `MechanismHardware`:

```java
.withFollowers(new Follower(FOLLOWER_ID, true))
```

`FlywheelSubsystem` never mentions a follower before or after. Compare its class declaration
today and under this design:

```java
// today
public class FlywheelSubsystem extends SubsystemBase          // 164 lines
// under this design
public class FlywheelSubsystem extends RollerSubsystem        // ~30 lines
```

— structurally identical to `TowerSubsystem`, which has no follower. **The variant has vanished
above the IO boundary.**

*The encoder block* — `HoodIOTalonFX` lines 49, 98–120, 217–219, plus five CANcoder `GatedAlert`s
and `refreshEncoderAlerts`: a `CANcoder` field, config refresh/modify/apply, two
`StatusSignal`s, a `cancoderSignals` list, frequency and bus-optimization calls, and three
`inputs.encoder*` copy-outs. All of it becomes:

```java
.withEncoder(ENCODER_ID, cc -> cc.MagnetSensor
    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive))
```

plus the `fx.Feedback` lines that were already in the config. `HoodSubsystem` becomes
structurally identical to a hypothetical encoder-less servo — the only trace is
`PivotSubsystem`'s `Visualizer.Source.ENCODER_ABSOLUTE`, one enum value.

*The result across all eight variants:*

| Variant | Types needed | Config |
| --- | --- | --- |
| servo | `ServoSubsystem` | — |
| servo + followers | `ServoSubsystem` | `.withFollowers(…)` |
| servo + encoder | `ServoSubsystem` | `.withEncoder(…)` |
| servo + followers + encoder | `ServoSubsystem` | both |
| roller (×4, same pattern) | `RollerSubsystem` | — |

Two classes, no generics, no unused code, and the four variants we do not have cost nothing.
Adding homing later adds an optional config block and a `HomingRoutine` object (Option C's
pattern, correctly applied) — not a fourth axis of subclasses.

**Why servo and roller stay separate** rather than merging as the Poofs do: the difference is
load-bearing. Different control mode registered on `LoggedSetpointTracker`; different unit for
`tolerance`; different `at*Setpoint` semantics; different soft-limit behavior; different default
visualizer shape. Merging would hand `HopperSubsystem` a `setPositionRot(...)` that clamps
against soft limits it does not have — which is how their `IntakeRollerSubsystem` ends up
inheriting `positionSetpointCommand` and `withoutLimitsTemporarily`.

**Naming collision to resolve:** `frc.robot.subsystems.intake.roller.RollerSubsystem` clashes by
simple name with the template. Rename the concrete class to `IntakeRollerSubsystem` during its
migration step — it is ambiguous today anyway, since `hopper`, `shooter.tower` and
`shooter.flywheel` are all rollers.

### 7.2 Where the closed loop lives — **decided: onboard**

**Option A — low-level IO (`setVoltage` only), loop on the RIO.** Fully replayable and
unit-testable, and sim exercises identical control code. But 50 Hz instead of the Talon's 1 kHz,
loses Phoenix's integrated feedforward and its interaction with on-device soft limits, and
invalidates every gain in `Constants` — six mechanisms needing a fresh characterization pass
mid-season to get back to where they already are.

**Option B (chosen) — high-level IO, loop onboard, controller state logged as inputs.**

The replay objection is weaker than it looks: all six subsystems already log the controller's
state as *inputs* (`controlMode`, `closedLoopSetpoint`, `closedLoopOutput`). A replay answers
what was commanded, what the controller targeted, what it output, and what the mechanism did.
What it cannot re-derive is the controller's internal arithmetic — also true of every other
Phoenix-based AdvantageKit codebase, and not where mechanism bugs live.

Sim fidelity is *better* under Option B: the sim IOs drive `TalonFXSimState`, so the real Phoenix
closed loop runs against a WPILib plant. A RIO-side loop would make sim exercise different code
than the robot.

Servo and roller do not differ; neither needs profile shaping the controller cannot do, nor
fusion of a sensor it cannot read. Both keep `setVoltage`/`setDutyCycle` regardless — SysId and
manual control need them.

### 7.3 The config surface — **decided: `TalonFXConfiguration`**

Your argument: *if I'm rebuilding the config for TalonFX anyway, and any other motor controller
has to deal with a foreign config either way — mine or Talon's — why not just use
`TalonFXConfiguration` and translate that?*

**Adopted.** The reasons, and the counter-arguments that lost, are recorded here because the
guardrail below only makes sense against them.

**For:**

- **Zero translation cost for the vendor we actually use** — 100% of our motors are TalonFX.
- **Complete and maintained by CTRE.** No hand-written schema that lags. Concretely: while
  verifying `SlotConfigs` against the 26.1.1 jar for this revision, it turned out to carry
  `GravityArmPositionOffset`, `StaticFeedforwardSign` and `GainSchedBehavior` — three gain-slot
  settings revision 2's neutral `GainSet` did not model and I did not know existed. That is the
  lag arriving before a line of code was written.
- **Field-for-field parity with Tuner X**, so code and tuning tool read the same.
- **Non-goal 4.** A second vendor is exactly the speculative requirement that rule targets.
- The escape hatch that revision 2 needed in order to stay honest (a
  `Consumer<TalonFXConfiguration>` per subsystem) is an admission that the neutral layer was
  incomplete. Deleting the neutral layer deletes the escape hatch too.

**Against, and how it is mitigated:**

The strongest objection is that `TalonFXConfiguration` is an *encoding* while a neutral config is
the *intent*. The hood:

```
// Talon encoding                              // intent
RotorToSensorRatio     = -244.411765           gearRatio = 244.411765
SensorToMechanismRatio = -1.0                  encoder inverted = true
```

A future SparkMax translator must *infer* that two independently-negative ratios mean "the
encoder reads backwards" rather than "the gearbox reverses" or "someone made a sign error" — the
three are indistinguishable in the encoding. So the second vendor writes a decoder **and** an
encoder, and the decode step is guesswork.

That cost is real and it is being accepted, deliberately, on the grounds that a second vendor is
speculative and a lagging neutral schema is not. If a SparkMax ever appears, the neutral layer
gets extracted then — informed by that vendor's real requirements rather than by guesses, and by
a codebase where every subsystem's intent is already written down in a `*Config.java` file.
Notably, `MechanismConfig` keeps the fields Phoenix has no answer for (`Sim`, `Visualizer`,
`SlotSpec`, `tolerance`, `name`), so the extraction would be additive.

The second objection has teeth today, so it gets a mechanism rather than a promise:

> **Once the vendor config is in the shared object, template classes start reading vendor
> fields.** In the Poofs' `ServoMotorSubsystem` — a *template class* — `withoutLimitsTemporarily()`
> reads `conf.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable` directly. Their template
> cannot compile without Phoenix.

**The guardrail.** As built, `config.fx()` is read in exactly three places:

1. `frc/robot/util/subsystems/talonfx/**` — the vendor IO layer.
2. `**/*Config.java` — where each mechanism's device settings are authored.
3. `src/test/**` — the parity test has to read the configuration it pins.

No template class reads it at all. Everything they need — soft limits, Motion Magic defaults,
the rotor-to-reported ratio, the simulated rotor orientation — comes from derived accessors on
`MechanismConfig`, which is the only file in the shared layer that touches the Phoenix object.

Enforce it with Checkstyle, which the build already gates on at `maxWarnings = 0`:

```xml
<!-- config/checkstyle/google_checks.xml -->
<module name="RegexpSinglelineJava">
  <property name="format" value="\bconfig\.fx\(\)|\bconf\.fx\(\)"/>
  <property name="message"
            value="config.fx() is Phoenix-specific: read it only in util/subsystems/talonfx or in a *Config class. See docs/subsystem-library-design.md 7.3."/>
</module>
```

```xml
<!-- config/checkstyle/checkstyle-suppressions.xml -->
<suppress id="NoVendorConfigOutsideVendorLayer"
          files="[\\/]util[\\/]subsystems[\\/]talonfx[\\/].*\.java"/>
<suppress id="NoVendorConfigOutsideVendorLayer" files=".*Config\.java"/>
<suppress id="NoVendorConfigOutsideVendorLayer" files="[\\/]src[\\/]test[\\/].*\.java"/>
```

This is the difference between "we promise not to" and "the build fails" — and it was checked
that way: adding a `config.fx()` read to `RollerSubsystem` fails `./gradlew build` with this
message, and removing it passes again.

**Option C — layered (a generic config *plus* a vendor config supplied alongside)** was also
considered and rejected: in practice the vendor half becomes the primary, because whoever is
tuning reaches for the file that actually controls the hardware. It is Option A with extra steps.

### 7.4 Hardware / mode selection — **decided**

See §4, including §4.4 on why this is a separate class and what it would cost to keep it in
`RobotContainer` instead.

### 7.5 Units — **decided: raw doubles, rotations**

**Option A — `Rotation2d`.** Rejected: it normalizes to a single revolution, wrong for a 244:1
hood, wrong for a multi-turn climber winch, meaningless for velocity.

**Option B — WPILib units.** Compile-time dimensional safety, but immutable `Measure` allocates
on every read in a 20 ms loop (the `MutableMeasure` workaround is awkward inside an `@AutoLog`
Inputs class), every gain in `Constants` is a bare `double`, and this team already tried it —
commit `5135ac83` is *"Remove WPILib units library usage across robot code"*.

**Option C (chosen) — raw doubles, unit suffixes on every name.** Already the convention and
already enforced: `LoggedSetpointTracker.unitSuffix()` appends `_rot`, `_rps`, `_v`, `_amps` to
log keys, and every Inputs field carries its unit.

Replay favors it (`double` is AdvantageKit's native log type — bit-exact, no struct schema to
version). Sim favors it (`SingleJointedArmSim` and `DCMotorSim` speak radians; the sim IO does
one explicit conversion at the boundary rather than threading `Measure` objects through a plant).

Rotations, not radians — Phoenix is natively rotations, every gain was characterized in
rotations, and the log history is in rotations.

**But "rotations of what" is now an explicit question**, not an assumption. Five subsystems report
mechanism rotations and the tower reports rotor rotations (§5.4). The library does not force
these to agree; it makes each config say which it is via `fx.Feedback`, and keeps the sim plant's
mechanical reduction as a separate number.

### 7.6 SysId — **decided: helper class**

`SysIdFactory` (§3.7). Two lines per subsystem instead of eighteen.

### 7.7 Gain slots keyed by control mode — **decided: adopted**

Today every subsystem writes one `Slot0Configs` and every closed-loop request implicitly uses
slot 0. Phoenix offers three slots.

**Option A — status quo, one gain set, always slot 0.** Smallest surface. Breaks the moment one
mechanism wants both position and velocity closed loop — the planned climber winch plausibly
wants position for setpoints and velocity for a controlled descent, and those need different
gains. Handling it later means changing `MotorIO`'s signatures.

**Option B (chosen) — `SlotSpec(slot, mode, output)`; gains live in `fx.SlotN`.**

```java
public record SlotSpec(int slot, MotorControlMode mode, ClosedLoopOutput output) {}
```

`MotorIOTalonFX` builds an `EnumMap<MotorControlMode, SlotSpec>` once at construction, then each
setter resolves its slot and its control request (§5.3). Live gain tuning goes through
`SlotConfigs` with `SlotNumber` set — one code path for all three slots, verified available in
Phoenix 26.1.1.

Three things fall out that are worth having independently of the multi-slot case:

- **`ClosedLoopOutput` lands where it belongs.** Revision 2 made it a per-mechanism choice, which
  is wrong: Phoenix picks the output type per *request*, and you would only pair torque-current
  requests with torque-current-tuned gains. Attaching it to the slot models the real
  relationship.
- **Gravity settings are per-slot in Phoenix** (`Slot0Configs.GravityType`,
  `GravityArmPositionOffset`), not per-mechanism. Under §7.3 they simply live in `fx.Slot0`,
  which is now automatically correct.
- **Tunable keys become self-describing**: `Shooter/Hood/Gains/Position/kP` rather than `Hood/kP`.

**Cost:** ~15 lines in `MotorIOTalonFX` and ~5 in the config. **All six subsystems will have
exactly one `SlotSpec`** — this buys nothing today, and that is worth saying plainly. Adopted
because the incremental cost is near zero, it fixes a modelling error that exists regardless, and
it removes a future `MotorIO` signature change.

**Rules to enforce at construction:** slot in 0–2; at most one `SlotSpec` per control mode, so a
setter never has to guess. A mechanism wanting two position slots (loaded vs unloaded) would need
an explicit `setPositionRot(double, int slot)` overload — not designed now, since nothing needs
it.

---

## 8. Deviations from the Poofs

I read `com.team254.lib.subsystems` in full (17 files) plus their `Constants` config blocks,
`RobotContainer` wiring and four concrete subsystems.

| # | They do | We do | Why |
| --- | --- | --- | --- |
| 1 | `TalonFXConfiguration fxConfig` in the shared config object | **Same** — but the template layer is fenced off from it by a Checkstyle rule | §7.3. We are following them here, having argued the other side and lost on the merits |
| 2 | The *base class* `ServoMotorSubsystem.withoutLimitsTemporarily()` reads `conf.fxConfig.SoftwareLimitSwitch` | Template classes may not read `config.fx()`; `ServoSubsystem` caches soft limits once in its constructor, and that is the sole carve-out | The failure mode their code demonstrates is the one the guardrail prevents |
| 3 | Nine `buildXSubsystem()` methods on `RobotContainer`, each an `if (RobotBase.isSimulation())` | One `MechanismHardware.create(config)` | Your non-goal 2 (§4) |
| 4 | `ServoMotorSubsystem<T, U>` and `ServoMotorSubsystemWithCanCoder<T, U, V, W>` | No generics | Every one of their instantiations is the base type; after our IO split, none of our six would specialize either (§7.1 Option B) |
| 5 | `WithFollowers` and `WithCanCoder` as sibling subclasses — **with no class combining them** | Followers and encoders are config fields plus extra IO instances | Their explosion already bit them (§7.1 Option D) |
| 6 | `ServoMotorSubsystem` used for rollers too | Separate `ServoSubsystem` / `RollerSubsystem` | Different tolerance units, tracked control mode, soft-limit semantics, default visualizer |
| 7 | Split `MotorIO` / `CanCoderIO` | **Same** (§3.3) | We are following them here; revision 2 was wrong to merge them |
| 8 | `SimCanCoderIO` driven by `simTalon.getSupplierForCancoder(...)` | **Same** (§4.3) | Adopted wholesale |
| 9 | Sim disables current limits and ramp rates (`TalonFXIO`: *"do not perform well in sim"*) | **Same**, via `simOverrides` (§3.2) | Adopted; we currently do not do this at all |
| 10 | `MotorInputs` logs six fields | Thirteen: adds torque current, temperature, temp fault, connectivity, control mode, closed-loop reference and output | Ours drive `GatedAlert` disconnect gating and the §7.2 replay story |
| 11 | Gains baked into `fxConfig.Slot0` at construction; no live tuning in the base | `LoggedTunableNumber` gains in `MechanismSubsystem`, re-applied only on change | We already have this in all six subsystems and use it |
| 12 | Generic command factories on the base plus domain factories in `factories/*.java` | Setters only; domain commands stay on the concrete subsystem | Your call |
| 13 | Follower position/velocity **averaged into** `getCurrentPosition()`/`getCurrentVelocity()` | Leader only; follower telemetry logged separately | Averaging a mechanically-coupled follower hides a slipping or dead follower behind a plausible number |
| 14 | `clampPosition` in `TalonFXIO` | Clamp in `ServoSubsystem`, plus Talon soft limits | Clamping in the subsystem makes the *logged* setpoint match what the mechanism will chase |
| 15 | `unitToRotorRatio` folds gearing and unit conversion into one number, in radians | `fx.Feedback` for reporting, `Sim.mechanicalReduction` for the plant, both in rotations | Theirs makes every gain's units depend on a composite constant; and it conflates the two ratios that §5.4 shows are different for the tower |

**Still deferred** (recorded in `CLAUDE.md`): their 5 ms `Notifier` sim plant step, and their
`addFriction` static-friction model. Both are additive to `MotorIOTalonFXSim`.

---

## 9. Decisions made without asking

1. **`frc.robot.subsystems.intake.roller.RollerSubsystem` is renamed `IntakeRollerSubsystem`**
   to free the name for the template class (§7.1).
2. **`TalonFXTranslator`'s two inbound status methods are public**, the rest package-private
   (§3.6), so swerve can keep calling `toMotorControlMode`.
3. **`MechanismConfig.simOrientation()` derives the sim `ChassisReference` from both the
   inversion and the sign of `RotorToSensorRatio`** (§3.5), rather than from inversion alone.
   Inversion alone reproduces four of six and would silently break the hood.

---

## 10. Phased migration plan

**Status: all seven phases are implemented.** `./gradlew build` passes — compile, Checkstyle on
main and test at `maxWarnings = 0`, and nine tests. What that does *not* cover is anything
requiring the robot or a running simulation, so the per-phase checkpoints below still stand as a
bring-up checklist. In particular nobody has yet watched a mechanism move.

Two things were confirmed mechanically rather than by eye, because they were the checkpoints most
likely to be fudged:

- **Device configuration parity.** `MechanismConfigParityTest` pins every mechanism's
  `TalonFXConfiguration` — inversion, neutral mode, current limits, feedback ratios, soft limits,
  gains, Motion Magic, follower ID and direction — to the values its hand-written
  `*IOTalonFX` applied before the migration. That is the transcription risk, and it is now a
  build failure rather than a field discovery.
- **Simulated rotor orientation.** Also pinned, because the naive derivation reproduces four of
  six mechanisms and silently inverts the hood (§3.5).

The library is purely additive: Phase 0 touches no existing subsystem, and each later phase
converts one mechanism. The robot builds and runs at every checkpoint.

**The intake goes first, as a unit** — roller then pivot — so a whole mechanism can be
field-tested rather than half of one. That front-loads the first servo conversion into Phase 2,
the riskiest ordering choice here; Checkpoint 1 exists to catch design problems before that risk
is taken.

Each conversion carries its own log-key rename, so during migration the log contains a legitimate
mix (`Intake/Roller/...` alongside `Hopper/...`) that shows at a glance what has been converted.

### Phase 0 — build the library

- Create the files in §3.
- Rename `ComponentStatus` → `Hardware`, add `ClosedLoopOutput` (§3.1). Update the four
  non-migrating references: `LoggedSetpointTracker`, `PhoenixUtil`, `swerve/ModuleIO`,
  `swerve/ModuleIOTalonFX`.
- Move `toMotorControlMode` / `toEncoderHealth` into `TalonFXTranslator` (§3.6).
- Generalize `RollerMechanism2D` to N spokes (§3.5).
- Drop `accelerationRPSPerSec` from the Inputs vocabulary and `getAcceleration()` from the hood's
  signal list — hood is the only user, nothing reads it, and it costs CAN bandwidth.
- **Add the Checkstyle `config.fx()` rule and its suppressions** (§7.3).

**Checkpoint 0:** `./gradlew build` passes including Checkstyle at `maxWarnings = 0`. The six
subsystems still compile against their own IO classes; runtime behavior is unchanged.

### Phase 1 — `intake.roller` (proof of design)

Simplest target: direct drive, no Motion Magic, no follower, no encoder, default spinner
visualizer, three call sites (`RobotContainer`, `PivotAndRollerIntakeCommand`,
`ShootAndLeaveAuton`).

```java
public static final MechanismConfig CONFIG =
    MechanismConfig.of("Intake/Roller", CANType.MECH, 14)
        .withFx(fx -> {
            fx.MotorOutput
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
            fx.CurrentLimits
                .withStatorCurrentLimit(120.0).withStatorCurrentLimitEnable(true);
            fx.Feedback.withSensorToMechanismRatio(1.0);
            fx.Slot0.withKP(0.0930).withKS(0.866).withKV(0.100).withKA(0.0116);
        })
        .withSimOverrides(fx -> fx.Slot0.withKP(0.9).withKS(0).withKV(0.12).withKA(0))
        .withSlots(SlotSpec.velocity())
        .withToleranceRps(5.0)
        .withVisualizer(Visualizer.spinner(0.2))
        .withSim(Sim.RotatingMass.fromKa(DCMotor.getKrakenX60Foc(1), 1.0, 0.0116))
        .withSysId(SysIdSpec.ROLLER);
```

Also: rename to `IntakeRollerSubsystem`; delete `RollerIO`, `RollerIOTalonFX`,
`RollerIOTalonFXSim`; move the `inSpeed`/`outSpeed` tunables onto the concrete subsystem (they
are domain values, not device settings).

**Checkpoint 1 — the design proof. Do not proceed until all pass:**

- `./gradlew build` and Checkstyle clean, **including the new `config.fx()` rule**.
- **Confirm the `@AutoLogOutput` key for the base-class `LoggedMechanism2d` resolves to
  `Intake/Roller/…`** (§3.5). If not, apply the `Logger.recordOutput` fallback now, before five
  more subsystems depend on it.
- Sim: roller spins in and out; `Intake/Roller/*` keys match the old `Roller/*` values from a
  pre-migration sim log.
- `Intake/Roller/atVelocitySetpoint`, `controlMode` and `VelocitySetpoint_rps` behave as before.
- Replay a pre-migration log: the unconverted five still replay; the converted one produces the
  no-op IO without throwing.
- On the robot: rollers run, and disconnecting the motor raises
  `"Intake Roller Motor (ID 14): Disconnected"`.

If any of this is awkward, **stop and revise the design** — that is what this phase is for.

### Phase 2 — `intake.pivot` (completes the intake)

First servo, first `EncoderIO`, first soft limits, first visualization override (the wall).
Exercises `Visualizer.Source.ENCODER_ABSOLUTE`, the sim CANcoder linkage of §4.3, and
`GravityTypeValue.Arm_Cosine` — pivot is the only mechanism with a `kG`.

**Checkpoint 2:** build clean; `Intake/Pivot/Encoder/*` keys present; pivot deploys and retracts
to the same physical positions; soft limits still stop it; `Intake/Pivot/atPositionSetpoint`
matches observed behavior; arm and moving wall animate correctly. **Field-test the whole
intake** — this is the payoff for the ordering choice.

### Phase 3 — `hopper`

Adds Motion Magic plumbing (carried forward unchanged) and the four-vane visualizer via
`Visualizer.spinner(0.4, 4)`, proving both `Sim.RotatingMass.fromKa` and the generalized spinner.

**Checkpoint 3:** build clean; sim hopper spins and four vanes animate; the three Motion Magic
tunables reappear under the new keys and still write to the device.

### Phase 4 — `shooter.tower`

Sets `fx.Feedback.SensorToMechanismRatio = 1.0` explicitly and keeps the sim plant's mechanical
reduction at 4.0 (§5.4). Real-robot behavior is unchanged; the config now states what was
previously an unwritten default.

**Checkpoint 4:** build clean; tower velocity for a given commanded RPS is unchanged on the
robot; sim velocity for the same command is unchanged too (both were already rotor RPS).

### Phase 5 — `shooter.flywheel`

First follower. Proves the follower path end to end.

**Checkpoint 5:** build clean; `Shooter/Flywheel/Follower0/*` keys present and populated;
unplugging the follower raises its own alert while the leader keeps running; flywheel reaches
commanded RPS with unchanged spin-up time.

### Phase 6 — `shooter.hood`

Last, because it is the hardest: `ClosedLoopOutput.TorqueCurrent`, the negative ratio pair, a
244:1 reduction, both current limits, and the sim orientation flip of §3.5.

**Checkpoint 6:** build clean; **read back from Tuner X** that `RotorToSensorRatio` is
`-244.411765` and `SensorToMechanismRatio` is `-1.0` — do not trust the config on inspection
alone; **verify the composed sim sign** (rotor orientation × ratio sign) rather than assuming
`simOrientation()` generalizes; hood travels its full range in the correct direction and holds
position under load as before.

### Phase 7 — cleanup

- Delete the now-unused device blocks from `Constants.java`. Game-strategy constants like
  `PIVOT_OUT_POS_ROT` and `TARGET_VELO_RPS` move to the respective `*Config` classes.
- Remove the mode `switch` and both `LoggedCanivore` fields from `RobotContainer`.
- Route `Constants.MECH_ENABLED` through `MechanismConfig.disabled()` instead of null checks.

**Checkpoint 7:** full build, full sim pass, one full teleop and one full auton run on the robot,
and a replay of a fresh log confirming every mechanism reproduces.

---

## 11. Open items

Nothing here blocks bring-up; all three are decisions rather than defects.

1. **Flywheel current limits** (§2.3) — it runs on Phoenix's 120 A stator / 70 A supply defaults
   because the code has never configured them. Deliberate, or worth setting explicitly?
2. **Tower reported units** (§5.4) — resolved for the migration (explicit 1:1, behavior
   unchanged, pinned by test), but the underlying wart stands: `TARGET_VELO_RPS = 30.0` is rotor
   rot/s on the tower and mechanism rot/s on the hopper, and the two `_rps` log keys mean
   different things. Worth a separate decision now that it is written down.
3. **`Constants.MECH_ENABLED`** — Phase 7 proposed routing it through
   `MechanismConfig.disabled()`. **Not done**, deliberately: today the flag only suppresses
   button bindings, and the motors are still configured. Routing it through `disabled()` would
   also stop configuring them, which is a real behavior change and does not belong in a
   migration whose contract is that nothing changes. The seam is there when you want it —
   `.disabled()` on any config yields no-op IO.
4. **Deferred, recorded in `CLAUDE.md`** — the 5 ms `Notifier` sim step and the static-friction
   model (§8).
