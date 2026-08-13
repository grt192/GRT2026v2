package frc.robot.subsystems.hopper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.HopperConfig.HopperIntake;
import frc.robot.util.SysIdFactory;
import frc.robot.util.subsystems.MechanismHardware;
import frc.robot.util.subsystems.MechanismIOBundle;
import frc.robot.util.subsystems.RollerSubsystem;

/** The hopper spindexer: rotates to feed balls toward the tower, or back out. */
public class HopperSubsystem extends RollerSubsystem {

    public HopperSubsystem() {
        this(MechanismHardware.create(HopperConfig.CONFIG));
    }

    /** Injection point for tests and for a hand-built IO. */
    public HopperSubsystem(MechanismIOBundle io) {
        super(HopperConfig.CONFIG, io);
    }

    public void setHopperState(HopperIntake state) {
        switch (state) {
            case BALL_IN:
                setVelocityRps(HopperConfig.TARGET_RPS);
                break;
            case BALL_OUT:
                setVelocityRps(-HopperConfig.TARGET_RPS);
                break;
            default:
                stop();
                break;
        }
    }

    public Command runHopperState(HopperIntake state) {
        return this.runEnd(() -> setHopperState(state), this::stop);
    }

    public Command runHopperIn() {
        return runHopperState(HopperIntake.BALL_IN);
    }

    public Command runHopperOut() {
        return runHopperState(HopperIntake.BALL_OUT);
    }

    public Command setHopperManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command stopHopper() {
        return this.runOnce(this::stop);
    }

    public Command runSysId() {
        return SysIdFactory.fullSweep(SysIdFactory.routine(
            this, HopperConfig.LOG_KEY, HopperConfig.CONFIG.sysId(), this::setVoltage));
    }
}
