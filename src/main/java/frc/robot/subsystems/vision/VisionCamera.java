package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionCamera {

    private PhotonPoseEstimator poseEstimator;
    private PhotonCamera cam;
    private CameraConfig camConfig;

    public VisionCamera(CameraConfig camConfig) {
        this.camConfig = camConfig;
        cam = new PhotonCamera(camConfig.getCameraName());

        poseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, camConfig.getCameraPose());
    }

    public static boolean isOnField(Pose3d pose) {
        return (pose.getX() > VisionConstants.MIN_X_M) && (pose.getX() < VisionConstants.MAX_X_M)
            && (pose.getY() > VisionConstants.MIN_Y_M) && (pose.getY() < VisionConstants.MAX_Y_M);
    }

    public List<TimestampedVisionUpdate> getVisionEstimates() {
        List<TimestampedVisionUpdate> predictions = new ArrayList<>();

        for (PhotonPipelineResult res : cam.getAllUnreadResults()) {
            if (!res.hasTargets()) {
                continue;
            }
            Optional<EstimatedRobotPose> possiblePoseEstimate = poseEstimator.estimateCoprocMultiTagPose(res);
            boolean isMultiTag = possiblePoseEstimate.isPresent();

            if (!isMultiTag) {
                possiblePoseEstimate = poseEstimator.estimatePnpDistanceTrigSolvePose(res);
            }
            if (possiblePoseEstimate.isEmpty()) { // No Detections
                continue;
            }

            EstimatedRobotPose poseEstimate = possiblePoseEstimate.get();
            double poseZValue = poseEstimate.estimatedPose.getZ();

            // CHANGE Z LOGIC IF FIELD HAS DIFFERENT ALTITUDE
            if (!isOnField(poseEstimate.estimatedPose) || Math.abs(poseZValue) > VisionConstants.Z_TOLERANCE_M) {
                continue;
            }

            int tagCount;
            double avgDist;
            if (isMultiTag) {
                tagCount = poseEstimate.targetsUsed.size();
                avgDist = poseEstimate.targetsUsed.stream()
                    .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                    .average()
                    .orElse(Double.POSITIVE_INFINITY);
            } else {
                tagCount = 1;
                avgDist = res.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            }
            if (tagCount == 0) {
                continue;
            }

            // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/3ea1eb036b2dc06e4ecb14d98bba7f602a1cd62a/src/main/java/org/littletonrobotics/frc2025/subsystems/vision/Vision.java#L212-L234
            double scale = (Math.pow(avgDist, VisionConstants.STD_DIST_PWR) / (tagCount * tagCount)) * camConfig.getStdDevFactor();
            double xyStdDev = (isMultiTag ? VisionConstants.XY_COEFF_MULTI_TAG : VisionConstants.XY_COEFF_SINGLE_TAG) * scale;
            double thetaStdDev = isMultiTag ? VisionConstants.THETA_COEFF * scale : Double.POSITIVE_INFINITY;

            predictions.add(new TimestampedVisionUpdate(poseEstimate.timestampSeconds, poseEstimate.estimatedPose, isMultiTag, VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                thetaStdDev)));
        }

        return predictions;
    }

    public void updateHeading(double timestamp, Rotation2d heading) {
        poseEstimator.addHeadingData(timestamp, heading);
    }

    public void resetHeading(double timestamp, Rotation2d heading) {
        poseEstimator.resetHeadingData(timestamp, heading);
    }
}
