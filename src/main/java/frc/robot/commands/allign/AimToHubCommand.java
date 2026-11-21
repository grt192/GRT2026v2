package frc.robot.commands.allign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

public class AimToHubCommand extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final FieldManagementSubsystem fmsSubsystem;
    // Shooter offset relative to robot center (x: forward/back, y: left/right in meters)
    private static final Translation2d SHOOTER_OFFSET = new Translation2d(-0.08, 0.073);

    // Link to dimensions https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public AimToHubCommand(DriveSubsystem swerveSubsystem, FieldManagementSubsystem fms) {
        this.swerveSubsystem = swerveSubsystem;
        this.fmsSubsystem = fms;
        addRequirements(swerveSubsystem, fms);
    }

    public double calculateTargetAngle() {
        Pose2d robotPose = swerveSubsystem.getRobotPosition();

        // Get the hub position based on alliance
        Translation2d hubTrans;
        if (fmsSubsystem.isRedAlliance()) {
            hubTrans = AlignConstants.RED_HUB_TRANS;
        } else {
            hubTrans = AlignConstants.BLUE_HUB_TRANS;
        }

        // Apply shooter offset to robot pose to get shooter position
        // Rotate the offset by the robot's heading, then add to robot position
        Translation2d shooterPosition = robotPose.getTranslation()
            .plus(SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));

        // Calculate vector from shooter to hub
        Translation2d shooterToHub = hubTrans.minus(shooterPosition);

        // Get the angle we need to point at
        Rotation2d targetAngle = shooterToHub.getAngle();

        // Add 90° because shooter is on the right side of the robot, not the front
        // So we need to rotate the robot 90° from where the shooter needs to point
        return targetAngle.getDegrees() + 90;
    }

    public Command createAimCommand(BooleanSupplier cancelCondition) {
        double targetAngle = calculateTargetAngle();
        return new RotateToFieldAngleCommand(swerveSubsystem, targetAngle, cancelCondition);
    }

    public double getDistanceToHub() {
        Translation2d robotPosition = swerveSubsystem.getRobotPosition().getTranslation();

        // Get the hub position based on alliance
        Translation2d hubTrans;
        if (fmsSubsystem.isRedAlliance()) {
            hubTrans = AlignConstants.RED_HUB_TRANS;
        } else {
            hubTrans = AlignConstants.BLUE_HUB_TRANS;
        }

        return robotPosition.getDistance(hubTrans);
    }
}

