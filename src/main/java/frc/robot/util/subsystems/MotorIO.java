package frc.robot.util.subsystems;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.Hardware.MotorControlMode;
import frc.robot.util.PIDConstants;

/**
 * One motor, behind the AdvantageKit IO boundary.
 *
 * <p>
 * The closed loop runs on the motor controller, so the setpoint methods are high level. What
 * the controller then did with the setpoint comes back through {@link MotorInputs#controlMode},
 * {@link MotorInputs#closedLoopSetpoint} and {@link MotorInputs#closedLoopOutput}, which is what
 * makes a replayed log answer "what was commanded, what did the controller target, what did the
 * mechanism do" without the loop itself being replayable.
 *
 * <p>
 * Nothing crossing this interface is vendor-typed, even though {@link MechanismConfig} is —
 * this is the boundary a replay has to reconstruct.
 *
 * <p>
 * A follower is just another {@code MotorIO}: it is told once to follow and never commanded
 * again, but its inputs are logged in full so a slipping or dead follower shows up in position
 * and velocity rather than only in current draw.
 */
public interface MotorIO {

    /** Everything read back from one motor each loop. */
    @AutoLog
    public static class MotorInputs {
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

    /**
     * Gains this implementation starts with for the slot serving {@code mode} — the configured
     * gains on real hardware, the simulation overlay in sim. Used to seed the tuning dashboard.
     */
    default PIDConstants defaultGains(MotorControlMode mode) {
        return PIDConstants.ZERO;
    }

    default void setGains(MotorControlMode mode, PIDConstants gains) {}

    default void setMotionMagic(double cruiseRps, double accelRps2, double jerkRps3) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setVoltage(double volts) {}

    default void setPositionRot(double positionRot) {}

    default void setVelocityRps(double velocityRps) {}

    default void stop() {}

    // ---- seam for homing, which no mechanism does yet ----

    default void setCurrentPositionRot(double positionRot) {}

    default void setSoftLimitsEnabled(boolean forward, boolean reverse) {}
}
