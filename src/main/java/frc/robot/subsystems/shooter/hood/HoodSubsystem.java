package frc.robot.subsystems.shooter.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.SysIdFactory;
import frc.robot.util.subsystems.MechanismHardware;
import frc.robot.util.subsystems.MechanismIOBundle;
import frc.robot.util.subsystems.ServoSubsystem;

/** The shooter hood: sets the launch angle. */
public class HoodSubsystem extends ServoSubsystem {

    public HoodSubsystem() {
        this(MechanismHardware.create(HoodConfig.CONFIG));
    }

    /** Injection point for tests and for a hand-built IO. */
    public HoodSubsystem(MechanismIOBundle io) {
        super(HoodConfig.CONFIG, io);
    }

    public Command goToPosition(double positionRot) {
        return this.runOnce(() -> setPositionRot(positionRot));
    }

    public Command holdPosition(double positionRot) {
        return this.startEnd(() -> setPositionRot(positionRot), this::stop);
    }

    public Command holdDownHood() {
        return this.run(() -> setPositionRot(HoodConfig.LOWER_LIMIT_ROT));
    }

    public Command hideHood() {
        return this.runOnce(() -> setPositionRot(HoodConfig.LOWER_LIMIT_ROT))
            .andThen(Commands.waitUntil(this::atPositionSetpoint));
    }

    public Command jiggleHood() {
        // Jiggle within the middle of the hood's travel range.
        double range = HoodConfig.UPPER_LIMIT_ROT - HoodConfig.LOWER_LIMIT_ROT;
        double lowPos = HoodConfig.LOWER_LIMIT_ROT + range * 0.25;
        double highPos = HoodConfig.LOWER_LIMIT_ROT + range * 0.25;

        Command jiggle = Commands.sequence(
            this.runOnce(() -> setPositionRot(highPos)),
            Commands.waitSeconds(0.5),
            this.runOnce(() -> setPositionRot(lowPos)),
            Commands.waitSeconds(0.5)).repeatedly();
        jiggle.addRequirements(this);

        return jiggle;
    }

    public Command setHoodManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command stopHood() {
        return this.runOnce(this::stop);
    }

    public Command runSysId() {
        return SysIdFactory.fullSweep(SysIdFactory.routine(
            this, HoodConfig.LOG_KEY, HoodConfig.CONFIG.sysId(), this::setVoltage));
    }
}
