package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.subsystems.MechanismConfig.SysIdSpec;

/**
 * Builds the characterization routine every mechanism used to hand-roll identically.
 *
 * <p>
 * A helper rather than a method on the subsystem template, which stays setter-only so that every
 * {@code Command} a mechanism exposes is named after something the mechanism actually does.
 */
public final class SysIdFactory {

    private SysIdFactory() {}

    public static SysIdRoutine routine(
        Subsystem subsystem, String logKey, SysIdSpec spec, DoubleConsumer setVoltage) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(spec.rampVoltsPerSec()).per(Seconds),
                Volts.of(spec.stepVolts()),
                Seconds.of(spec.timeoutSec()),
                state -> Logger.recordOutput(logKey + "/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> setVoltage.accept(volts.in(Volts)),
                null,
                subsystem));
    }

    /** Quasistatic forward and reverse, then dynamic forward and reverse. */
    public static Command fullSweep(SysIdRoutine routine) {
        return routine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse));
    }
}
