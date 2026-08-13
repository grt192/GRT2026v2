package frc.robot.util.subsystems;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.Hardware.MotorControlMode;

/**
 * A mechanism commanded to a speed: an intake roller, a hopper, a tower, a flywheel.
 *
 * <p>
 * Position is still read and logged — it is what drives the spinning visualization — but it is
 * never a setpoint, and {@code tolerance} is in rotations per second.
 */
public abstract class RollerSubsystem extends MechanismSubsystem {

    protected RollerSubsystem(MechanismConfig config, MechanismIOBundle io) {
        super(config, io,
            MotorControlMode.DutyCycle, MotorControlMode.Voltage, MotorControlMode.Velocity);
    }

    public final void setVelocityRps(double velocityRps) {
        motor.setVelocityRps(velocityRps);
        setpointTracker.updateSetpoint(velocityRps, MotorControlMode.Velocity);
    }

    /** False unless velocity control is the active mode and the mechanism is within tolerance. */
    public final boolean atVelocitySetpoint() {
        return setpointTracker.atSetpoint(
            MotorControlMode.Velocity, inputs.velocityRps, config.tolerance());
    }

    @Override
    protected final void logAtSetpoint() {
        Logger.recordOutput(config.name() + "/atVelocitySetpoint", atVelocitySetpoint());
    }
}
