package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TagObservation;

public class VisionCamera {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final CameraConfig camConfig;
    private final String logKey;

    private final Alert disconnectedAlert;

    private double lastFrameTime = Double.NEGATIVE_INFINITY;
    private List<TimestampedVisionUpdate> visionEstimates = List.of();

    public VisionCamera(VisionIO io, CameraConfig camConfig) {
        this.io = io;
        this.camConfig = camConfig;
        logKey = "Vision/Camera" + camConfig.getCameraName();

        disconnectedAlert = new Alert("Vision Camera " + camConfig.getCameraName() + ": Disconnected", AlertType.kError);
    }

    public static boolean isOnField(Pose3d pose) {
        return (pose.getX() > VisionConstants.MIN_X_M) && (pose.getX() < VisionConstants.MAX_X_M)
            && (pose.getY() > VisionConstants.MIN_Y_M) && (pose.getY() < VisionConstants.MAX_Y_M);
    }

    public void updateIO() {
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
        disconnectedAlert.set(!inputs.connected);

        updateVisionEstimates();
    }

    /** Vision estimates from the last {@link #updateIO()} call. */
    public List<TimestampedVisionUpdate> getVisionEstimates() {
        return visionEstimates;
    }

    private void updateVisionEstimates() {
        List<TimestampedVisionUpdate> predictions = new ArrayList<>();

        boolean hasFrames = inputs.tagObservations.length > 0;
        if (hasFrames) {
            lastFrameTime = Timer.getTimestamp();
        }

        for (PoseObservation obs : inputs.poseObservations) {
            // CHANGE Z LOGIC IF FIELD HAS DIFFERENT ALTITUDES
            if (!isOnField(obs.pose()) || Math.abs(obs.pose().getZ()) > VisionConstants.Z_TOLERANCE_M
                || obs.tagCount() == 0) {
                continue;
            }

            // Tags from the same frame share the observation's timestamp
            List<Pose3d> tagPoses = new ArrayList<>();
            for (TagObservation tag : inputs.tagObservations) {
                if (tag.timestamp() != obs.timestamp()) {
                    continue;
                }
                VisionConstants.FIELD_LAYOUT.getTagPose(tag.tagId()).ifPresent(tagPoses::add);
            }

            // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/3ea1eb036b2dc06e4ecb14d98bba7f602a1cd62a/src/main/java/org/littletonrobotics/frc2025/subsystems/vision/Vision.java#L212-L234
            double scale = (Math.pow(obs.avgTagDist(), VisionConstants.STD_DIST_PWR) / (obs.tagCount() * obs.tagCount())) * camConfig.getStdDevFactor();
            double xyStdDev = (obs.isMultiTag() ? VisionConstants.XY_COEFF_MULTI_TAG : VisionConstants.XY_COEFF_SINGLE_TAG) * scale;
            double thetaStdDev = obs.isMultiTag() ? VisionConstants.THETA_COEFF * scale : Double.POSITIVE_INFINITY;

            predictions.add(new TimestampedVisionUpdate(obs.timestamp(), obs.pose(), obs.isMultiTag(), VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                thetaStdDev)));

            Logger.recordOutput(logKey + "/LatencySecs", Timer.getTimestamp() - obs.timestamp());
            Logger.recordOutput(logKey + "/RobotPose", obs.pose().toPose2d());
            Logger.recordOutput(logKey + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
        }

        // If no recent frames from this camera, clear tag poses
        if (Timer.getTimestamp() - lastFrameTime > VisionConstants.TARGET_LOG_TIME_SECS) {
            Logger.recordOutput(logKey + "/TagPoses", new Pose3d[] {});
        }

        visionEstimates = predictions;
    }

    public void updateHeading(double timestamp, Rotation2d heading) {
        io.updateHeading(timestamp, heading);
    }

    public void resetHeading(double timestamp, Rotation2d heading) {
        io.resetHeading(timestamp, heading);
    }
}
