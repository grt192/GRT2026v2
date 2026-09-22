package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.CycleShooterConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Cycle shot - no auto-aim. Both the FlywheelSubsystem RPS and the hood position are
 * supplied so live tuning via the dashboard or operator buttons takes effect mid-shot.
 * The pivot overload also agitates the pivot between its mid positions while feeding.
 */
public class CycleShot extends ParallelCommandGroup {

    /** Cycle shot at the CycleShooterConstants hood position, with no pivot movement. */
    public CycleShot(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        DoubleSupplier flyWheelVeloRPSSupplier) {
        this(flywheel, hood, tower, hopper, flyWheelVeloRPSSupplier, () -> CycleShooterConstants.HOOD_POSITION_ROT);
    }

    /** Cycle shot with a live hood position, with no pivot movement. */
    public CycleShot(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        DoubleSupplier flyWheelVeloRPSSupplier,
        DoubleSupplier hoodPosRotSupplier) {
        super(
            flywheel.setFlywheelVelocity(flyWheelVeloRPSSupplier),
            hood.holdPositionThenHide(hoodPosRotSupplier),
            tower.runTowerDutyCycle(SmashAndShootConstants.TOWER_DUTY_CYCLE),
            hopper.runHopperDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE));
    }

    /** Cycle shot with a live hood position that agitates the pivot while feeding. */
    public CycleShot(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        PivotSubsystem pivot,
        DoubleSupplier flyWheelVeloRPSSupplier,
        DoubleSupplier hoodPosRotSupplier) {
        this(flywheel, hood, tower, hopper, flyWheelVeloRPSSupplier, hoodPosRotSupplier);
        addCommands(pivot.cyclePivotMid(
            SmashAndShootConstants.INITIAL_DELAY_SECONDS,
            SmashAndShootConstants.TOGGLE_INTERVAL_SECONDS));
    }
}
