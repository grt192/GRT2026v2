package frc.robot.subsystems.intake.roller;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SysIdFactory;
import frc.robot.util.subsystems.MechanismHardware;
import frc.robot.util.subsystems.MechanismIOBundle;
import frc.robot.util.subsystems.RollerSubsystem;

/**
 * The intake roller: pulls game pieces in, spits them back out.
 *
 * <p>
 * Named {@code IntakeRollerSubsystem} rather than {@code RollerSubsystem} both because the
 * template class owns that name now and because it was always ambiguous — the hopper, tower and
 * flywheel are rollers too.
 */
public class IntakeRollerSubsystem extends RollerSubsystem {

    private final LoggedTunableNumber inSpeed = new LoggedTunableNumber(
        IntakeRollerConfig.LOG_KEY + "/InSpeed_rps", IntakeRollerConfig.IN_SPEED_RPS);
    private final LoggedTunableNumber outSpeed = new LoggedTunableNumber(
        IntakeRollerConfig.LOG_KEY + "/OutSpeed_rps", IntakeRollerConfig.OUT_SPEED_RPS);

    public IntakeRollerSubsystem() {
        this(MechanismHardware.create(IntakeRollerConfig.CONFIG));
    }

    /** Injection point for tests and for a hand-built IO. */
    public IntakeRollerSubsystem(MechanismIOBundle io) {
        super(IntakeRollerConfig.CONFIG, io);
    }

    public Command runRollerIn() {
        return this.runEnd(() -> setVelocityRps(Math.abs(inSpeed.get())), this::stop);
    }

    public Command runRollerOut() {
        return this.runEnd(() -> setVelocityRps(-Math.abs(outSpeed.get())), this::stop);
    }

    public Command setRollerManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command stopRoller() {
        return this.runOnce(this::stop);
    }

    public Command runSysId() {
        return SysIdFactory.fullSweep(SysIdFactory.routine(
            this, IntakeRollerConfig.LOG_KEY, IntakeRollerConfig.CONFIG.sysId(), this::setVoltage));
    }
}
