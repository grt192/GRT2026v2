package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements VisionIO {
    private final PhotonCamera cam;
    private final PhotonPoseEstimator poseEstimator;

    public VisionIOPhoton(CameraConfig camConfig) {
        cam = new PhotonCamera(camConfig.getCameraName());
        poseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, camConfig.getCameraPose());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = cam.isConnected();

        List<PoseObservation> observations = new ArrayList<>();
        List<TagObservation> tagObservations = new ArrayList<>();
        for (PhotonPipelineResult res : cam.getAllUnreadResults()) {
            if (!res.hasTargets()) {
                continue;
            }
            for (PhotonTrackedTarget target : res.getTargets()) {
                tagObservations.add(new TagObservation(
                    res.getTimestampSeconds(),
                    target.getFiducialId(),
                    target.getBestCameraToTarget(),
                    target.getAlternateCameraToTarget(),
                    target.getPoseAmbiguity()));
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

            observations.add(new PoseObservation(
                poseEstimate.timestampSeconds,
                poseEstimate.estimatedPose,
                isMultiTag,
                tagCount,
                avgDist));
        }

        inputs.poseObservations = observations.toArray(new PoseObservation[0]);
        inputs.tagObservations = tagObservations.toArray(new TagObservation[0]);
    }

    @Override
    public void updateHeading(double timestamp, Rotation2d heading) {
        poseEstimator.addHeadingData(timestamp, heading);
    }

    @Override
    public void resetHeading(double timestamp, Rotation2d heading) {
        poseEstimator.resetHeadingData(timestamp, heading);
    }
}
