package frc.robot.util.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.CANType;
import frc.robot.util.Hardware.ClosedLoopOutput;
import frc.robot.util.Hardware.MotorControlMode;

/**
 * Everything needed to build and run one mechanism.
 *
 * <p>
 * Device settings live in a {@link TalonFXConfiguration}: it is complete, maintained by CTRE,
 * and matches Tuner X field for field, so there is no hand-written schema to keep in sync. What
 * this record adds is the things Phoenix has no field for — identity and log key, the simulation
 * plant, the dashboard visualization, the setpoint tolerance, which gain slot serves which
 * control mode, and the gain overlay applied in simulation.
 *
 * <p>
 * <b>{@link #fx()} is Phoenix-specific and must not be read by the template classes.</b> Only
 * the {@code util.subsystems.talonfx} package, per-subsystem {@code *Config} classes, and the
 * derived accessors on this record may touch it; a Checkstyle rule enforces that. Template code
 * that needs a value out of it goes through {@link #reverseSoftLimitRot()} and friends.
 *
 * <p>
 * Built with {@code withX} witherers in the style of {@link frc.robot.util.PIDConstants}:
 *
 * <pre>
 * MechanismConfig.of("Intake/Roller", CANType.MECH, 14)
 *     .withFx(fx -&gt; fx.MotorOutput.withNeutralMode(NeutralModeValue.Brake))
 *     .withSlots(SlotSpec.velocity())
 *     .withToleranceRps(5.0)
 * </pre>
 */
public record MechanismConfig(
    String name,
    CANType bus,
    Controller controller,
    int canId,
    TalonFXConfiguration fx,
    Consumer<TalonFXConfiguration> simOverrides,
    Follower[] followers,
    Encoder encoder,
    SlotSpec[] slots,
    double tolerance,
    Visualizer visualizer,
    Sim sim,
    SysIdSpec sysId) {

    /** Which IO implementation to build. {@code NONE} yields no-op IO for a disabled mechanism. */
    public enum Controller {
        TALON_FX,
        NONE
    }

    /** A motor that mirrors the leader. Never commanded after construction. */
    public record Follower(int canId, boolean opposeLeader) {}

    /** An absolute encoder used for telemetry, and as the Talon's remote feedback source. */
    public record Encoder(int canId, CANcoderConfiguration cc) {}

    /**
     * Which Phoenix gain slot serves which control mode, and what control request to issue for
     * it. The gains themselves live in {@code fx.Slot0} / {@code Slot1} / {@code Slot2}.
     */
    public record SlotSpec(int slot, MotorControlMode mode, ClosedLoopOutput output) {

        public static SlotSpec position() {
            return new SlotSpec(0, MotorControlMode.Position, ClosedLoopOutput.Voltage);
        }

        public static SlotSpec velocity() {
            return new SlotSpec(0, MotorControlMode.Velocity, ClosedLoopOutput.Voltage);
        }

        public SlotSpec withOutput(ClosedLoopOutput output) {
            return new SlotSpec(slot, mode, output);
        }

        public SlotSpec withSlot(int slot) {
            return new SlotSpec(slot, mode, output);
        }
    }

    /** SysId sweep shape. Servos ramp slowly over a short range; rollers ramp hard and long. */
    public record SysIdSpec(double rampVoltsPerSec, double stepVolts, double timeoutSec) {
        public static final SysIdSpec SERVO = new SysIdSpec(0.5, 1.0, 5.0);
        public static final SysIdSpec ROLLER = new SysIdSpec(1.0, 7.0, 10.0);
    }

    /** Geometry for the dashboard {@code Mechanism2d}. See {@link MechanismVisualizer}. */
    public record Visualizer(
        Shape shape,
        double canvasWidthM,
        double canvasHeightM,
        double rootX,
        double rootY,
        double lengthM,
        int spokes,
        double thicknessPx,
        Color8Bit color,
        Source source) {

        /** Which default shape to draw. */
        public enum Shape {
            NONE,
            ARM,
            SPINNER
        }

        /** Which logged input drives the drawn angle. */
        public enum Source {
            MECHANISM_POSITION,
            ENCODER_ABSOLUTE
        }

        public static final Visualizer NONE = new Visualizer(
            Shape.NONE, 1.0, 1.0, 0.5, 0.5, 0.0, 0, 6.0,
            new Color8Bit(Color.kBlueViolet), Source.MECHANISM_POSITION);

        /** A single ligament pinned at the root, angled to the mechanism position. */
        public static Visualizer arm(double lengthM) {
            return new Visualizer(
                Shape.ARM, 1.0, 1.0, 0.5, 0.2, lengthM, 1, 6.0,
                new Color8Bit(Color.kCornflowerBlue), Source.MECHANISM_POSITION);
        }

        /** A spinning polygon with one spoke. */
        public static Visualizer spinner(double radiusM) {
            return spinner(radiusM, 1);
        }

        /** A spinning polygon with {@code spokes} evenly spaced spokes. */
        public static Visualizer spinner(double radiusM, int spokes) {
            return new Visualizer(
                Shape.SPINNER, 1.0, 1.0, 0.5, 0.5, radiusM, spokes, 6.0,
                new Color8Bit(Color.kBlueViolet), Source.MECHANISM_POSITION);
        }

        public Visualizer withCanvas(double widthM, double heightM) {
            return new Visualizer(
                shape, widthM, heightM, rootX, rootY, lengthM, spokes, thicknessPx, color, source);
        }

        public Visualizer withRoot(double x, double y) {
            return new Visualizer(
                shape, canvasWidthM, canvasHeightM, x, y, lengthM, spokes, thicknessPx, color,
                source);
        }

        public Visualizer withColor(Color8Bit color) {
            return new Visualizer(
                shape, canvasWidthM, canvasHeightM, rootX, rootY, lengthM, spokes, thicknessPx,
                color, source);
        }

        public Visualizer withSource(Source source) {
            return new Visualizer(
                shape, canvasWidthM, canvasHeightM, rootX, rootY, lengthM, spokes, thicknessPx,
                color, source);
        }
    }

    /**
     * The simulation plant.
     *
     * <p>
     * {@code mechanicalReduction} is rotor rotations per rotation of the physical output shaft —
     * a property of the gearbox, <b>not</b> of how the Talon reports position. Those are the same
     * number for most mechanisms and differ wherever the feedback config does not fold the
     * gearing in (the tower reports rotor rotations while turning a 4:1 reduction).
     */
    public sealed interface Sim permits Sim.None, Sim.Arm, Sim.RotatingMass {

        double mechanicalReduction();

        /** No plant: the mechanism does not move in simulation. */
        record None() implements Sim {
            @Override
            public double mechanicalReduction() {
                return 1.0;
            }
        }

        /** A gravity-loaded arm with hard travel limits. */
        record Arm(
            DCMotor gearbox,
            double mechanicalReduction,
            double moiKgM2,
            double comLengthM,
            double minAngleRot,
            double maxAngleRot,
            boolean simulateGravity,
            double startAngleRot) implements Sim {}

        /** A freely spinning mass. */
        record RotatingMass(
            DCMotor gearbox,
            double mechanicalReduction,
            double moiKgM2) implements Sim {

            /**
             * Derives plant inertia from a measured kA, so a characterized roller does not need a
             * separately guessed moment of inertia.
             */
            public static RotatingMass fromKa(
                DCMotor gearbox, double mechanicalReduction, double kA) {
                double kaRadPerSecSq = kA / (2.0 * Math.PI);
                double kt = gearbox.stallTorqueNewtonMeters / gearbox.stallCurrentAmps;
                double resistance = gearbox.nominalVoltageVolts / gearbox.stallCurrentAmps;
                return new RotatingMass(
                    gearbox, mechanicalReduction, (kaRadPerSecSq * mechanicalReduction * kt) / resistance);
            }
        }
    }

    /** A minimal config: one TalonFX, no encoder, no followers, no closed loop. */
    public static MechanismConfig of(String name, CANType bus, int canId) {
        return new MechanismConfig(
            name,
            bus,
            Controller.TALON_FX,
            canId,
            new TalonFXConfiguration(),
            fx -> {
            },
            new Follower[0],
            null,
            new SlotSpec[0],
            0.0,
            Visualizer.NONE,
            new Sim.None(),
            SysIdSpec.ROLLER);
    }

    /**
     * Mutates a copy of the device configuration. Every call clones first, so two configs can
     * never alias the same {@link TalonFXConfiguration}.
     */
    public MechanismConfig withFx(Consumer<TalonFXConfiguration> mutator) {
        TalonFXConfiguration copy = fx.clone();
        mutator.accept(copy);
        return new MechanismConfig(name, bus, controller, canId, copy, simOverrides, followers,
            encoder, slots, tolerance, visualizer, sim, sysId);
    }

    /**
     * Applied on top of {@link #fx()} in simulation only. Use for sim-specific gains, and to
     * switch off current limits and ramp rates, which do not behave well against a WPILib plant.
     */
    public MechanismConfig withSimOverrides(Consumer<TalonFXConfiguration> mutator) {
        return new MechanismConfig(name, bus, controller, canId, fx, mutator, followers,
            encoder, slots, tolerance, visualizer, sim, sysId);
    }

    public MechanismConfig withFollowers(Follower... followers) {
        return new MechanismConfig(name, bus, controller, canId, fx, simOverrides, followers,
            encoder, slots, tolerance, visualizer, sim, sysId);
    }

    /** Declares an absolute encoder. Its feedback wiring still goes in {@link #withFx}. */
    public MechanismConfig withEncoder(int canId, Consumer<CANcoderConfiguration> mutator) {
        CANcoderConfiguration cc = new CANcoderConfiguration();
        mutator.accept(cc);
        return new MechanismConfig(name, bus, controller, this.canId, fx, simOverrides, followers,
            new Encoder(canId, cc), slots, tolerance, visualizer, sim, sysId);
    }

    public MechanismConfig withSlots(SlotSpec... slots) {
        return new MechanismConfig(name, bus, controller, canId, fx, simOverrides, followers,
            encoder, slots, tolerance, visualizer, sim, sysId);
    }

    /** Position tolerance, in mechanism rotations. Servos only. */
    public MechanismConfig withToleranceRot(double toleranceRot) {
        return new MechanismConfig(name, bus, controller, canId, fx, simOverrides, followers,
            encoder, slots, toleranceRot, visualizer, sim, sysId);
    }

    /** Velocity tolerance, in rotations per second. Rollers only. */
    public MechanismConfig withToleranceRps(double toleranceRps) {
        return new MechanismConfig(name, bus, controller, canId, fx, simOverrides, followers,
            encoder, slots, toleranceRps, visualizer, sim, sysId);
    }

    public MechanismConfig withVisualizer(Visualizer visualizer) {
        return new MechanismConfig(name, bus, controller, canId, fx, simOverrides, followers,
            encoder, slots, tolerance, visualizer, sim, sysId);
    }

    public MechanismConfig withSim(Sim sim) {
        return new MechanismConfig(name, bus, controller, canId, fx, simOverrides, followers,
            encoder, slots, tolerance, visualizer, sim, sysId);
    }

    public MechanismConfig withSysId(SysIdSpec sysId) {
        return new MechanismConfig(name, bus, controller, canId, fx, simOverrides, followers,
            encoder, slots, tolerance, visualizer, sim, sysId);
    }

    /** Builds no hardware at all — for a mechanism switched off by a feature flag. */
    public MechanismConfig disabled() {
        return new MechanismConfig(name, bus, Controller.NONE, canId, fx, simOverrides, followers,
            encoder, slots, tolerance, visualizer, sim, sysId);
    }

    // ---- derived: the only sanctioned way for template code to read the device config ----

    /** "Shooter/Hood" to "Shooter Hood", for alert message prefixes. */
    public String displayName() {
        return name.replace('/', ' ');
    }

    /**
     * Rotor rotations per reported unit, as the Talon computes it. Distinct from
     * {@code sim().mechanicalReduction()}, which is the physical gearbox.
     */
    public double rotorToReportedRatio() {
        return fx.Feedback.RotorToSensorRatio * fx.Feedback.SensorToMechanismRatio;
    }

    public double reverseSoftLimitRot() {
        return fx.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
    }

    public double forwardSoftLimitRot() {
        return fx.SoftwareLimitSwitch.ForwardSoftLimitThreshold;
    }

    public boolean hasSoftLimits() {
        return fx.SoftwareLimitSwitch.ForwardSoftLimitEnable
            || fx.SoftwareLimitSwitch.ReverseSoftLimitEnable;
    }

    /** Midpoint of the travel range, used to place the CANcoder's discontinuity point. */
    public double softLimitMidpointRot() {
        return (reverseSoftLimitRot() + forwardSoftLimitRot()) / 2.0;
    }

    public double motionMagicCruiseRps() {
        return fx.MotionMagic.MotionMagicCruiseVelocity;
    }

    public double motionMagicAccelRps2() {
        return fx.MotionMagic.MotionMagicAcceleration;
    }

    public double motionMagicJerkRps3() {
        return fx.MotionMagic.MotionMagicJerk;
    }

    public boolean hasMotionMagic() {
        return motionMagicCruiseRps() != 0.0 || motionMagicAccelRps2() != 0.0;
    }

    /**
     * Rotor orientation for the simulated Talon.
     *
     * <p>
     * Naively this is just the configured inversion, but a negative {@code RotorToSensorRatio}
     * flips it a second time — which is why a mechanism configured counter-clockwise-positive can
     * need a clockwise-positive sim rotor.
     */
    public ChassisReference simOrientation() {
        boolean clockwise = fx.MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
        boolean sensorFlipped = fx.Feedback.RotorToSensorRatio < 0.0;
        return (clockwise ^ sensorFlipped)
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    }
}
