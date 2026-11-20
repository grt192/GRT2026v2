package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

/**
 * Manual shooter sequence - no auto-aim.
 * Hood position and FlywheelSubsystem RPS are passed in via the constructor so the same
 * sequence can be reused for different shot types (smash, cycle, etc.). Pivot
 * timing comes from SmashAndShootConstants.
 *
 * Runs until interrupted; each subsystem's own command stops it and returns it to
 * its resting position on the way out.
 */
public class ManualShooterSequence extends ParallelCommandGroup {

    public ManualShooterSequence(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        PivotSubsystem pivot,
        double hoodPositionRot,
        double flywheelVeloRPS) {
        super(
            flywheel.setFlywheelVelocity(flywheelVeloRPS),
            hood.holdPositionThenHide(hoodPositionRot),
            tower.runTowerDutyCycle(SmashAndShootConstants.TOWER_DUTY_CYCLE),
            hopper.runHopperDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE),
            pivot.cyclePivotMid(
                SmashAndShootConstants.INITIAL_DELAY_SECONDS,
                SmashAndShootConstants.TOGGLE_INTERVAL_SECONDS));
    }
}
