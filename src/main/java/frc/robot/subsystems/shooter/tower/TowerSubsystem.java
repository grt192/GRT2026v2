package frc.robot.subsystems.shooter.tower;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.tower.TowerConfig.TowerIntake;
import frc.robot.util.SysIdFactory;
import frc.robot.util.subsystems.MechanismHardware;
import frc.robot.util.subsystems.MechanismIOBundle;
import frc.robot.util.subsystems.RollerSubsystem;

/** The shooter tower: lifts balls from the hopper up into the flywheel. */
public class TowerSubsystem extends RollerSubsystem {

    public TowerSubsystem() {
        this(MechanismHardware.create(TowerConfig.CONFIG));
    }

    /** Injection point for tests and for a hand-built IO. */
    public TowerSubsystem(MechanismIOBundle io) {
        super(TowerConfig.CONFIG, io);
    }

    public void setTower(TowerIntake state) {
        switch (state) {
            case BALL_UP:
                setVelocityRps(TowerConfig.TARGET_VELO_RPS);
                break;
            case BALL_DOWN:
                setVelocityRps(-TowerConfig.TARGET_VELO_RPS);
                break;
            default:
                stop();
                break;
        }
    }

    public Command runTowerInput() {
        return this.runEnd(() -> setTower(TowerIntake.BALL_UP), this::stop);
    }

    public Command runTowerOutput() {
        return this.runEnd(() -> setTower(TowerIntake.BALL_DOWN), this::stop);
    }

    public Command setTowerManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command stopTower() {
        return this.runOnce(this::stop);
    }

    public Command runSysId() {
        return SysIdFactory.fullSweep(SysIdFactory.routine(
            this, TowerConfig.LOG_KEY, TowerConfig.CONFIG.sysId(), this::setVoltage));
    }
}
