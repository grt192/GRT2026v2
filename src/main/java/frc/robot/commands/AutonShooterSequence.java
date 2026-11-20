package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

/**
 * Auton shooter sequence - no auto-aim.
 * Uses fixed hood position and FlywheelSubsystem RPS from SmashAndShootConstants,
 * and keeps the pivot retracted while feeding. Runs until interrupted, so callers
 * bound it with a timeout.
 */
public class AutonShooterSequence extends ParallelCommandGroup {

    public AutonShooterSequence(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        PivotSubsystem pivot) {
        super(
            flywheel.setFlywheelVelocity(SmashAndShootConstants.FLYWHEEL_VELO_RPS),
            hood.holdPositionThenHide(SmashAndShootConstants.HOOD_POSITION_ROT),
            tower.runTowerDutyCycle(SmashAndShootConstants.TOWER_DUTY_CYCLE),
            hopper.runHopperDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE),
            pivot.retractPivot());
    }
}
