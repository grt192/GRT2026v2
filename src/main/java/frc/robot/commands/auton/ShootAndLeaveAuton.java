package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class ShootAndLeaveAuton extends SequentialCommandGroup {
    // Link to dimensions https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public ShootAndLeaveAuton(
        DriveSubsystem swerveSubsystem,
        FlywheelSubsystem flySubsystem,
        HoodSubsystem hoodSubsystem,
        HopperSubsystem hopperSubsystem,
        TowerSubsystem towerSubsystem,
        PivotSubsystem pivot,
        RollerSubsystem roller) {
        addCommands(
            new AutonShooterSequence(
                flySubsystem,
                hoodSubsystem,
                towerSubsystem,
                hopperSubsystem,
                pivot).withTimeout(3.0));
    }
}
