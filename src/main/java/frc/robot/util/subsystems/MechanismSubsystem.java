package frc.robot.util.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Hardware.MotorControlMode;
import frc.robot.util.LoggedSetpointTracker;
import frc.robot.util.LoggedTracer;
import frc.robot.util.subsystems.MechanismConfig.Visualizer.Source;

/**
 * Everything a motor-driven mechanism does that is not specific to which mechanism it is:
 * reading and logging inputs, tracking setpoints, tunable gains, the dashboard visualization,
 * and neutralizing on disable.
 *
 * <p>
 * Followers and absolute encoders are optional {@linkplain MechanismConfig config} features
 * rather than subclasses, because neither is visible above the IO boundary — a Phoenix follower
 * is told once to follow and never commanded again, and a remote encoder's reading already
 * arrives as the motor's own position. That is what keeps this a two-class hierarchy instead of
 * one class per combination of features.
 *
 * <p>
 * Deliberately setter-only: {@code Command} factories belong on the concrete subsystem, where
 * they can be named after what the mechanism actually does.
 */
public abstract class MechanismSubsystem extends SubsystemBase {

    protected final MechanismConfig config;
    protected final MotorIO motor;
    protected final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

    private final MotorIO[] followers;
    private final MotorInputsAutoLogged[] followerInputs;

    private final EncoderIO encoder;
    private final EncoderInputsAutoLogged encoderInputs = new EncoderInputsAutoLogged();

    protected final LoggedSetpointTracker setpointTracker;

    private final GainTuner gainTuner;
    private final MotionMagicTuner motionMagicTuner;

    private final MechanismVisualizer visualizer;

    /** Backs the {@code {logKey}} substitution in the {@link AutoLogOutput} key below. */
    private final String logKey;

    @AutoLogOutput(key = "{logKey}/Mechanism2d")
    private final LoggedMechanism2d mechanism2d;

    protected MechanismSubsystem(
        MechanismConfig config, MechanismIOBundle io, MotorControlMode... trackedModes) {
        super(config.name());

        this.config = config;
        this.logKey = config.name();
        this.motor = io.motor();
        this.followers = io.followers();
        this.encoder = io.encoder();

        followerInputs = new MotorInputsAutoLogged[followers.length];
        for (int i = 0; i < followers.length; i++) {
            followerInputs[i] = new MotorInputsAutoLogged();
        }

        setpointTracker = new LoggedSetpointTracker(config.name(), trackedModes);

        gainTuner = new GainTuner(config, motor);
        motionMagicTuner = config.hasMotionMagic() ? new MotionMagicTuner(config) : null;

        visualizer = new MechanismVisualizer(config.name(), config.visualizer());
        mechanism2d = visualizer.mechanism2d();
    }

    // ---- open-loop setters, shared by every mechanism ----

    public final void setDutyCycle(double dutyCycle) {
        double clamped = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        motor.setDutyCycle(clamped);
        setpointTracker.updateSetpoint(clamped, MotorControlMode.DutyCycle);
    }

    public final void setVoltage(double volts) {
        double clamped = MathUtil.clamp(volts, -12.0, 12.0);
        motor.setVoltage(clamped);
        setpointTracker.updateSetpoint(clamped, MotorControlMode.Voltage);
    }

    public final void stop() {
        motor.stop();
        setpointTracker.setControlMode(MotorControlMode.Disabled);
    }

    // ---- reads ----

    public final double getPositionRot() {
        return inputs.positionRot;
    }

    public final double getVelocityRps() {
        return inputs.velocityRps;
    }

    public final boolean isConnected() {
        return inputs.connected;
    }

    /** Raw absolute-encoder position, unscaled by any ratio. Zero when there is no encoder. */
    public final double getEncoderAbsolutePositionRot() {
        return encoderInputs.absolutePositionRot;
    }

    // ---- loop ----

    /**
     * Final so the ordering cannot be broken by a subclass forgetting {@code super.periodic()}:
     * inputs are read before they are logged, setpoint state is logged after it can still change
     * this loop, and the tracer epoch closes last. Mechanism-specific work goes in
     * {@link #onPeriodic()}.
     */
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

        visualizer.update(config.visualizer().source() == Source.ENCODER_ABSOLUTE
            ? encoderInputs.absolutePositionRot
            : inputs.positionRot);

        onPeriodic();

        gainTuner.applyIfChanged(motor);
        if (motionMagicTuner != null) {
            motionMagicTuner.applyIfChanged(motor);
        }

        LoggedTracer.record(config.name());
    }

    /** Per-loop work specific to one mechanism: extra visualization, derived state. */
    protected void onPeriodic() {}

    /** Implemented by {@link ServoSubsystem} / {@link RollerSubsystem}. */
    protected abstract void logAtSetpoint();

    /** Exposed so a subsystem can append geometry to the default visualization. */
    protected final MechanismVisualizer visualizer() {
        return visualizer;
    }
}
