package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CycleShooterConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.shooter.hood.HoodConfig;

/**
 * Manual shooter sequence - no auto-aim.
 * Hood position and FlywheelSubsystem RPS are passed in via the constructor so the same
 * sequence can be reused for different shot types (smash, cycle, etc.). Pivot
 * timing comes from SmashAndShootConstants.
 *
 * The shooterLearner offsets are applied on every loop, so live tuning via the
 * dashboard or operator buttons takes effect mid-shot.
 */
public class CycleShot extends Command {

    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final TowerSubsystem tower;
    private final HopperSubsystem hopper;

    private final DoubleSupplier flywheelVeloRPS;

    public CycleShot(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        DoubleSupplier flyWheelVeloRPSSupplier) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.tower = tower;
        this.hopper = hopper;
        this.flywheelVeloRPS = flyWheelVeloRPSSupplier;

        addRequirements(flywheel, hood, tower, hopper);
    }

    @Override
    public void initialize() {
        // Start ramping FlywheelSubsystem and moving hood to position
        flywheel.setVelocityRps(flywheelVeloRPS.getAsDouble());
        hood.setPositionRot(CycleShooterConstants.HOOD_POSITION_ROT);
    }

    @Override
    public void execute() {
        // Keep commanding FlywheelSubsystem and hood targets (with live operator offsets)
        flywheel.setVelocityRps(flywheelVeloRPS.getAsDouble());
        hood.setPositionRot(CycleShooterConstants.HOOD_POSITION_ROT);

        // Only feed balls when FlywheelSubsystem is at speed AND hood is at position
        if (/* fly.wantedVel() && hd.wantedAngl() */ true) {
            tower.setDutyCycle(SmashAndShootConstants.TOWER_DUTY_CYCLE);
            hopper.setDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        } else {
            tower.stop();
            hopper.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        hood.setPositionRot(HoodConfig.LOWER_LIMIT_ROT);
        tower.stop();
        hopper.stop();
    }
}
