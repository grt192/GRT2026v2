package frc.robot.commands.allign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class AimWhileDrivingCommand extends Command {
    private final DriveSubsystem swerve;
    private final FieldManagementSubsystem fms;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    // Shooter offset from robot center
    private static final Translation2d SHOOTER_OFFSET = new Translation2d(-0.08, 0.073);

    private final PIDController rotationPID = new PIDController(5.0, 0.0, 0.2);

    public AimWhileDrivingCommand(
        DriveSubsystem swerve,
        FieldManagementSubsystem fms,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier) {
        this.swerve = swerve;
        this.fms = fms;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    /**
     * Select the correct aim point depending on robot location.
     */
    private Translation2d getTargetPoint() {
        boolean redTeam = fms.isRedAlliance();
        Pose2d pose = swerve.getRobotPosition();

        if (redTeam) {
            if (pose.getX() > AlignConstants.RED_WALL_X) {
                return AlignConstants.RED_HUB_TRANS;
            }

            if (pose.getY() > AlignConstants.HUB_Y) {
                return AlignConstants.RED_AIM_TOP;
            }

            return AlignConstants.RED_AIM_BOTTOM;
        } else {
            if (pose.getX() < AlignConstants.BLUE_WALL_X) {
                return AlignConstants.BLUE_HUB_TRANS;
            }

            if (pose.getY() > AlignConstants.HUB_Y) {
                return AlignConstants.BLUE_AIM_TOP;
            }

            return AlignConstants.BLUE_AIM_BOTTOM;
        }
    }

    /**
     * Compute the angle the robot should face to aim the shooter.
     */
    private Rotation2d getTargetRotation() {
        Pose2d robotPose = swerve.getRobotPosition();
        Translation2d target = getTargetPoint();

        Translation2d shooterPosition = robotPose.getTranslation().plus(
            SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));

        Translation2d toTarget = target.minus(shooterPosition);

        // Shooter points sideways → add 90°
        return toTarget.getAngle().plus(Rotation2d.fromDegrees(90));
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();

        Rotation2d currentHeading = swerve.getRobotPosition().getRotation();
        Rotation2d targetHeading = getTargetRotation();

        double omega = rotationPID.calculate(
            currentHeading.getRadians(),
            targetHeading.getRadians());

        swerve.setDrivePowers(x, y, omega);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setDrivePowers(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
