package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.allign.RotateToFieldAngleCommand;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class Turnto90Auton extends SequentialCommandGroup {

    public Turnto90Auton(DriveSubsystem swerveSubsystem) {
        double currentAngle = swerveSubsystem.getRobotPosition().getRotation().getDegrees();
        double targetAngle = currentAngle + 90;

        addCommands(
            new RotateToFieldAngleCommand(swerveSubsystem, targetAngle, () -> false));
    }
}
