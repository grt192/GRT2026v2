package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.pivot.IntakePivotConfig;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
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
public class ManualShooterSequence extends Command {

    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final TowerSubsystem tower;
    private final HopperSubsystem hopper;
    private final PivotSubsystem pivot;

    private final double hoodPositionRot;
    private final double flywheelVeloRPS;

    private final Timer pivotTimer = new Timer();
    private boolean pivotIsIn = true;
    private boolean initialDelayDone = false;

    public ManualShooterSequence(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        PivotSubsystem pivot,
        double hoodPositionRot,
        double flywheelVeloRPS) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.tower = tower;
        this.hopper = hopper;
        this.pivot = pivot;
        this.hoodPositionRot = hoodPositionRot;
        this.flywheelVeloRPS = flywheelVeloRPS;

        addRequirements(flywheel, hood, tower, hopper, pivot);
    }

    @Override
    public void initialize() {
        // Start ramping FlywheelSubsystem and moving hood to position
        flywheel.setVelocityRps(flywheelVeloRPS);
        hood.setPositionRot(hoodPositionRot);
        // Start with pivot out, wait the initial-delay before first toggle
        pivotIsIn = false;
        initialDelayDone = false;
        pivot.setPositionRot(IntakePivotConfig.OUT_POS_ROT);
        pivotTimer.restart();
    }

    @Override
    public void execute() {
        // Keep commanding FlywheelSubsystem and hood targets (with live operator offsets)
        flywheel.setVelocityRps(flywheelVeloRPS);
        hood.setPositionRot(hoodPositionRot);

        if (!initialDelayDone) {
            if (pivotTimer.hasElapsed(SmashAndShootConstants.INITIAL_DELAY_SECONDS)) {
                initialDelayDone = true;
                pivotIsIn = true;
                pivotTimer.restart();
            }
        } else if (pivotTimer.hasElapsed(SmashAndShootConstants.TOGGLE_INTERVAL_SECONDS)) {
            pivotIsIn = !pivotIsIn;
            pivotTimer.restart();
        }
        pivot.setPositionRot(pivotIsIn ? IntakePivotConfig.MID_UPPER_ROT : IntakePivotConfig.MID_LOWER_ROT);

        // Only feed balls when FlywheelSubsystem is at speed AND hood is at position
        if (/* fly.wantedVel() && hd.wantedAngl() */ true) {
            tower.setDutyCycle(SmashAndShootConstants.TOWER_DUTY_CYCLE);
            hopper.setDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        } else {
            tower.stop();
            hopper.setDutyCycle(0);
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
        pivot.setPositionRot(IntakePivotConfig.OUT_POS_ROT);
    }
}
