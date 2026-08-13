package frc.robot.subsystems.shooter.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SysIdFactory;
import frc.robot.util.subsystems.MechanismHardware;
import frc.robot.util.subsystems.MechanismIOBundle;
import frc.robot.util.subsystems.RollerSubsystem;

/**
 * The shooter flywheel: two opposed wheels that throw the ball.
 *
 * <p>
 * The second motor is a follower, declared in {@link FlywheelConfig} and handled entirely
 * inside the IO layer — nothing here is aware of it, but its telemetry is logged in full under
 * {@code Shooter/Flywheel/Follower0} so a dead or slipping wheel is visible.
 */
public class FlywheelSubsystem extends RollerSubsystem {

    public FlywheelSubsystem() {
        this(MechanismHardware.create(FlywheelConfig.CONFIG));
    }

    /** Injection point for tests and for a hand-built IO. */
    public FlywheelSubsystem(MechanismIOBundle io) {
        super(FlywheelConfig.CONFIG, io);
    }

    public Command rampToVelocity(DoubleSupplier rpsSupplier) {
        return this.run(() -> setVelocityRps(rpsSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command setFlywheelManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command stopFlywheel() {
        return this.runOnce(this::stop);
    }

    public Command runSysId() {
        return SysIdFactory.fullSweep(SysIdFactory.routine(
            this, FlywheelConfig.LOG_KEY, FlywheelConfig.CONFIG.sysId(), this::setVoltage));
    }
}
