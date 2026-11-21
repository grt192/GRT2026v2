package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class DriveBackwardsCommand extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final ChassisSpeeds speeds;

    public DriveBackwardsCommand(DriveSubsystem swerveSubsystem, ChassisSpeeds speeds) {
        this.swerveSubsystem = swerveSubsystem;
        this.speeds = speeds;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.setDrivePowers(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond);
    }

}
