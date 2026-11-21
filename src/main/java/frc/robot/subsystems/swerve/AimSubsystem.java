package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.fms.FieldManagementSubsystem;

/**
 * Computes and continuously publishes aiming-related values (currently just
 * distance to the hub) so they can be watched live on the dashboard while
 * tuning the shooter interpolation table.
 *
 * Link to dimensions:
 * https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
 */
public class AimSubsystem extends SubsystemBase {

    private final DriveSubsystem swerve;
    private final FieldManagementSubsystem fms;

    private final DoublePublisher distancePub;

    public AimSubsystem(DriveSubsystem swerve, FieldManagementSubsystem fms) {
        this.swerve = swerve;
        this.fms = fms;
        distancePub = NetworkTableInstance.getDefault()
            .getTable("Aim")
            .getDoubleTopic("distanceToHub")
            .publish();
    }

    public double getDistanceToHub() {
        Translation2d robot = swerve.getRobotPosition().getTranslation();
        Translation2d hub = fms.isRedAlliance()
            ? AlignConstants.RED_HUB_TRANS
            : AlignConstants.BLUE_HUB_TRANS;
        return robot.getDistance(hub);
    }

    @Override
    public void periodic() {
        distancePub.set(getDistanceToHub());
    }
}
