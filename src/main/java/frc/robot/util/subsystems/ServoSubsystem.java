package frc.robot.util.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.Hardware.MotorControlMode;

/**
 * A mechanism commanded to a position: a pivot, a hood, an arm.
 *
 * <p>
 * Kept separate from {@link RollerSubsystem} rather than merged into one class because the
 * differences are load-bearing — a different control mode is tracked, {@code tolerance} means
 * rotations rather than rotations per second, and soft limits apply. Merging them would hand
 * every roller a {@code setPositionRot} that clamps against limits it does not have.
 */
public abstract class ServoSubsystem extends MechanismSubsystem {

    private final double reverseLimitRot;
    private final double forwardLimitRot;
    private final boolean clampToSoftLimits;

    protected ServoSubsystem(MechanismConfig config, MechanismIOBundle io) {
        super(config, io,
            MotorControlMode.DutyCycle, MotorControlMode.Voltage, MotorControlMode.Position);

        clampToSoftLimits = config.hasSoftLimits();
        reverseLimitRot = config.reverseSoftLimitRot();
        forwardLimitRot = config.forwardSoftLimitRot();
    }

    /**
     * Commands a position, clamped to the soft limits.
     *
     * <p>
     * The motor controller enforces the same limits on device. Clamping here as well is what
     * makes the <i>logged</i> setpoint match the position the mechanism will actually chase,
     * rather than the out-of-range value someone asked for.
     */
    public final void setPositionRot(double positionRot) {
        double clamped = clampToSoftLimits
            ? MathUtil.clamp(positionRot, reverseLimitRot, forwardLimitRot)
            : positionRot;
        motor.setPositionRot(clamped);
        setpointTracker.updateSetpoint(clamped, MotorControlMode.Position);
    }

    /** False unless position control is the active mode and the mechanism is within tolerance. */
    public final boolean atPositionSetpoint() {
        return setpointTracker.atSetpoint(
            MotorControlMode.Position, inputs.positionRot, config.tolerance());
    }

    @Override
    protected final void logAtSetpoint() {
        Logger.recordOutput(config.name() + "/atPositionSetpoint", atPositionSetpoint());
    }
}
