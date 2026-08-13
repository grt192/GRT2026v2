package frc.robot.util;

import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.Hardware.MotorControlMode;

public class LoggedSetpointTracker {
    private final Map<MotorControlMode, Double> setpoints = new EnumMap<>(MotorControlMode.class);
    private MotorControlMode currentControlMode = MotorControlMode.Disabled;

    private final String logPath;

    public LoggedSetpointTracker(String logPath, MotorControlMode... registeredControlModes) {
        this.logPath = logPath.endsWith("/") ? logPath : logPath + "/";

        for (MotorControlMode controlMode : registeredControlModes) {
            registerControlMode(controlMode);
        }
    }

    public void registerControlMode(MotorControlMode mode) {
        if (isStaticControlMode(mode)) {
            throw new IllegalArgumentException("Cannot track a setpoint for " + mode);
        }
        setpoints.put(mode, 0.0);
    }

    public boolean isStaticControlMode(MotorControlMode mode) {
        return (mode == MotorControlMode.Disabled || mode == MotorControlMode.Follower);
    }

    private void requireRegistered(MotorControlMode mode, boolean allowStaticMode) {
        if (isStaticControlMode(mode) && allowStaticMode) {
            return;
        } else if (!setpoints.containsKey(mode)) {
            throw new IllegalStateException(mode + " is not a registered setpoint mode for " + logPath);
        }
    }

    private void requireRegistered(MotorControlMode mode) {
        requireRegistered(mode, false);
    }

    public MotorControlMode getControlMode() {
        return currentControlMode;
    }

    public void setControlMode(MotorControlMode controlMode) {
        requireRegistered(controlMode, true);
        currentControlMode = controlMode;
    }

    public double getSetpoint(MotorControlMode mode) {
        requireRegistered(mode);
        return setpoints.get(mode);
    }

    public boolean atSetpoint(MotorControlMode controlMode, double curState, double tolerance) {
        requireRegistered(controlMode, true);
        if (controlMode != getControlMode() || isStaticControlMode(controlMode)) {
            return false;
        }
        return Math.abs(curState - getSetpoint(controlMode)) <= tolerance;
    }

    public void updateSetpoint(double setpoint, MotorControlMode mode, boolean setControlMode) {
        requireRegistered(mode);
        setpoints.put(mode, setpoint);

        if (setControlMode) {
            setControlMode(mode);
        }
    }

    public void updateSetpoint(double setpoint, MotorControlMode mode) {
        updateSetpoint(setpoint, mode, true);
    }

    private static String unitSuffix(MotorControlMode mode) {
        return switch (mode) {
            case DutyCycle -> "";
            case Voltage -> "_v";
            case Velocity -> "_rps";
            case Position -> "_rot";
            case TorqueCurrent -> "_amps";
            default -> throw new IllegalArgumentException("No unit defined for " + mode);
        };
    }

    public void logControlMode() {
        Logger.recordOutput(logPath + "controlMode", currentControlMode);
    }

    public void logSetpoint(MotorControlMode mode) {
        requireRegistered(mode);
        double value = (mode == currentControlMode) ? setpoints.get(mode) : 0.0;
        Logger.recordOutput(logPath + mode + "Setpoint" + unitSuffix(mode), value);
    }

    public void logAll() {
        logControlMode();
        for (MotorControlMode mode : setpoints.keySet()) {
            logSetpoint(mode);
        }
    }
}
