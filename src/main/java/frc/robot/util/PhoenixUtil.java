// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants.CANType;
import java.util.EnumMap;
import java.util.List;
import java.util.function.Supplier;
import frc.robot.util.Hardware.EncoderHealth;
import frc.robot.util.Hardware.MotorControlMode;

/**
 * Class to refresh all phoenix status signals
 * Taken from 6328's 2025 code
 */
public class PhoenixUtil {
    /** Attempts to run the command until no error is produced. Returns error code */
    public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        StatusCode latestError = StatusCode.OK;
        for (int i = 0; i < maxAttempts; i++) {
            latestError = command.get();
            if (latestError.isOK()) {
                return latestError;
            }
        }
        return latestError;
    }

    /** Attempts to run the command until no error is produced. Returns error code */
    public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command, Alert failureAlert) {
        StatusCode result = tryUntilOk(maxAttempts, command);
        failureAlert.set(!result.isOK());

        return result;
    }

    /** Signals for synchronized refresh, keyed by bus. */
    private static EnumMap<CANType, BaseStatusSignal[]> signalsByBus = new EnumMap<>(CANType.class);

    static {
        for (CANType type : CANType.values()) {
            signalsByBus.put(type, new BaseStatusSignal[0]);
        }
    }

    /** Registers a set of signals for synchronized refresh. */
    public static void registerSignals(CANType canType, BaseStatusSignal... signals) {
        BaseStatusSignal[] current = signalsByBus.get(canType);
        BaseStatusSignal[] combined = new BaseStatusSignal[current.length + signals.length];
        System.arraycopy(current, 0, combined, 0, current.length);
        System.arraycopy(signals, 0, combined, current.length, signals.length);
        signalsByBus.put(canType, combined);
    }

    public static void registerSignals(CANType canType, List<BaseStatusSignal> signals) {
        registerSignals(canType, signals.toArray(new BaseStatusSignal[0]));
    }

    /** Refresh all registered signals. */
    public static void refreshAllStatusSignals() {
        for (BaseStatusSignal[] signals : signalsByBus.values()) {
            if (signals.length > 0) {
                BaseStatusSignal.refreshAll(signals);
            }
        }
    }

    /** Maps Phoenix 6's hardware-specific {@link ControlModeValue} to the hardware-agnostic {@link MotorControlMode}. */
    public static MotorControlMode toMotorControlMode(ControlModeValue value) {
        switch (value) {
            case DisabledOutput:
            case NeutralOut:
            case StaticBrake:
            case CoastOut:
                return MotorControlMode.Disabled;
            case Follower:
                return MotorControlMode.Follower;
            case DutyCycleOut:
            case DutyCycleFOC:
                return MotorControlMode.DutyCycle;
            case VoltageOut:
            case VoltageFOC:
                return MotorControlMode.Voltage;
            case TorqueCurrentFOC:
                return MotorControlMode.TorqueCurrent;
            case PositionDutyCycle:
            case PositionDutyCycleFOC:
            case PositionVoltage:
            case PositionVoltageFOC:
            case PositionTorqueCurrentFOC:
            case MotionMagicDutyCycle:
            case MotionMagicDutyCycleFOC:
            case MotionMagicVoltage:
            case MotionMagicVoltageFOC:
            case MotionMagicTorqueCurrentFOC:
            case MotionMagicExpoDutyCycle:
            case MotionMagicExpoDutyCycleFOC:
            case MotionMagicExpoVoltage:
            case MotionMagicExpoVoltageFOC:
            case MotionMagicExpoTorqueCurrentFOC:
                return MotorControlMode.Position;
            case VelocityDutyCycle:
            case VelocityDutyCycleFOC:
            case VelocityVoltage:
            case VelocityVoltageFOC:
            case VelocityTorqueCurrentFOC:
            case MotionMagicVelocityDutyCycle:
            case MotionMagicVelocityDutyCycleFOC:
            case MotionMagicVelocityVoltage:
            case MotionMagicVelocityVoltageFOC:
            case MotionMagicVelocityTorqueCurrentFOC:
                return MotorControlMode.Velocity;
            default:
                return MotorControlMode.Disabled;
        }
    }

    public static EncoderHealth toEncoderHealth(MagnetHealthValue value) {
        switch (value) {
            case Magnet_Green:
                return EncoderHealth.Good;
            case Magnet_Orange:
                return EncoderHealth.Marginal;
            case Magnet_Red:
                return EncoderHealth.Bad;
            case Magnet_Invalid:
            default:
                return EncoderHealth.Unknown;
        }
    }
}
