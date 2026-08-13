package frc.robot.util.subsystems.talonfx;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;

import frc.robot.util.Hardware.ClosedLoopOutput;
import frc.robot.util.Hardware.EncoderHealth;
import frc.robot.util.Hardware.MotorControlMode;
import frc.robot.util.PIDConstants;

/**
 * The Phoenix-to-vendor-neutral boundary: status enums on the way in, gain slots and control
 * requests on the way out.
 *
 * <p>
 * There is no configuration translation here, because device settings are authored directly as
 * a {@code TalonFXConfiguration}. What is left is the vocabulary that has to cross the IO
 * boundary into replayable logs, and the request plumbing that has no configuration equivalent.
 */
public final class TalonFXTranslator {

    private TalonFXTranslator() {}

    /** Builds a control request for one setpoint and gain slot. */
    @FunctionalInterface
    interface SetpointRequest {
        ControlRequest with(double setpoint, int slot);
    }

    /**
     * Maps Phoenix's hardware-specific {@link ControlModeValue} to the hardware-agnostic
     * {@link MotorControlMode}.
     */
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

    /**
     * Reads one gain slot generically. {@code SlotConfigs} carries its own slot number, so a
     * mechanism using more than one slot needs no per-slot branching downstream.
     */
    static SlotConfigs slotConfigs(TalonFXConfiguration fx, int slot) {
        SlotConfigs configs;
        switch (slot) {
            case 1:
                configs = SlotConfigs.from(fx.Slot1);
                break;
            case 2:
                configs = SlotConfigs.from(fx.Slot2);
                break;
            default:
                configs = SlotConfigs.from(fx.Slot0);
                break;
        }
        configs.SlotNumber = slot;
        return configs;
    }

    static PIDConstants toPidConstants(SlotConfigs slot) {
        return new PIDConstants(
            slot.kP, slot.kI, slot.kD, slot.kS, slot.kG, slot.kV, slot.kA);
    }

    /** Copies gains onto a slot, leaving gravity type and feedforward sign untouched. */
    static SlotConfigs withGains(SlotConfigs slot, PIDConstants gains) {
        return slot
            .withKP(gains.kP())
            .withKI(gains.kI())
            .withKD(gains.kD())
            .withKS(gains.kS())
            .withKG(gains.kG())
            .withKV(gains.kV())
            .withKA(gains.kA());
    }

    /**
     * FOC is enabled everywhere it is optional; torque-current requests are FOC by nature.
     *
     * <p>
     * The request object is built once and mutated per call, matching how Phoenix expects
     * control requests to be reused.
     */
    static SetpointRequest positionRequest(ClosedLoopOutput output) {
        if (output == ClosedLoopOutput.TorqueCurrent) {
            PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(0.0);
            return (setpoint, slot) -> request.withPosition(setpoint).withSlot(slot);
        }
        if (output == ClosedLoopOutput.DutyCycle) {
            PositionDutyCycle request = new PositionDutyCycle(0.0).withEnableFOC(true);
            return (setpoint, slot) -> request.withPosition(setpoint).withSlot(slot);
        }
        PositionVoltage request = new PositionVoltage(0.0).withEnableFOC(true);
        return (setpoint, slot) -> request.withPosition(setpoint).withSlot(slot);
    }

    static SetpointRequest velocityRequest(ClosedLoopOutput output) {
        if (output == ClosedLoopOutput.TorqueCurrent) {
            VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0.0);
            return (setpoint, slot) -> request.withVelocity(setpoint).withSlot(slot);
        }
        if (output == ClosedLoopOutput.DutyCycle) {
            VelocityDutyCycle request = new VelocityDutyCycle(0.0).withEnableFOC(true);
            return (setpoint, slot) -> request.withVelocity(setpoint).withSlot(slot);
        }
        VelocityVoltage request = new VelocityVoltage(0.0).withEnableFOC(true);
        return (setpoint, slot) -> request.withVelocity(setpoint).withSlot(slot);
    }
}
