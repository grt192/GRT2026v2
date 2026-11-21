package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class Turn90AutonPP extends SequentialCommandGroup {

    public Turn90AutonPP(DriveSubsystem swerve) {
        PathPlannerPath path;

        try {
            path = PathPlannerPath.fromPathFile("90deg");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        addCommands(
            AutoBuilder.followPath(path));

        addRequirements(swerve);
    }
}
