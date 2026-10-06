package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.PolynomialRegression;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private static AprilTagFieldLayout aprilTagFieldLayout;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final CameraConfig cameraConfig;
    private final String camID;

    private NetworkTableInstance ntInstance;
    private NetworkTable visionStatsTable;
    private StructPublisher<Pose2d> visionPosePublisher;
    private DoublePublisher visionDistPublisher;
    private StructPublisher<Pose3d> cameraPosePublisher;
    private DoubleArrayPublisher tagDistancePublisher;

    private Consumer<TimestampedVisionUpdate> visionConsumer = (x) -> {
    };


    private boolean connected;
    private Transform3d latestTransform3d = new Transform3d();

    public VisionSubsystem(CameraConfig cameraConfig) {
        // Initialize the camera with its name
        this.camera = new PhotonCamera(cameraConfig.getCameraName());
        this.camID = cameraConfig.getCameraName();
        this.cameraConfig = cameraConfig;
        // Load AprilTag field layout
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2026-rebuilt-welded.json");
        } catch (Exception e) {
            throw new RuntimeException("Failed to load field layout", e);
        }

        // Create pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, cameraConfig.getCameraPose());
        // photonPoseEstimator = new PhotonPoseEstimator(
        // aprilTagFieldLayout,
        // cameraConfig.getPoseStrategy(),
        // cameraConfig.getCameraPose()
        // );

        initNt(cameraConfig);
    }

    @Override
    public void periodic() {
        connected = camera.isConnected();

        // Get all unread results in the queue from the camera
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        // Loops through all unread results
        for (PhotonPipelineResult result : results) {

            // checks if the camera detected any apriltags
            if (!result.hasTargets()) {
                continue;
            }

            double minDistance = Double.MAX_VALUE;
            long[] tagIDs = new long[result.getTargets().size()];
            double[] tagDistances = new double[result.getTargets().size()];
            // loops through all detected targets from the camera
            for (int i = 0; i < result.getTargets().size(); i++) {
                PhotonTrackedTarget target = result.getTargets().get(i);

                Translation3d translation = target.getBestCameraToTarget().getTranslation();
                latestTransform3d = target.getBestCameraToTarget();

                double distance = Math.sqrt(
                    Math.pow(translation.getX(), 2)
                        + Math.pow(translation.getY(), 2)
                        + Math.pow(translation.getZ(), 2));
                if (distance < minDistance) {
                    minDistance = distance;
                }
                tagIDs[i] = target.getFiducialId();
                tagDistances[i] = distance;
            }

            visionDistPublisher.set(minDistance);
            tagDistancePublisher.set(tagDistances);

            Logger.recordOutput(cameraConfig.getCameraName() + " Tag Distances", tagDistances);
            Logger.recordOutput(cameraConfig.getCameraName() + " Tag IDs", tagIDs);

            // Don't use vision measurement if tags are too far
            if (minDistance > 4) {
                continue;
            }

            Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.estimateAverageBestTargetsPose(result);

            if (!estimatedPose.isPresent()) {
                continue;
            }
            Pose3d estimatedPose3d = estimatedPose.get().estimatedPose;
            Pose2d estimatedPose2d = estimatedPose3d.toPose2d();
            double xyStdDev = VisionConstants.XY_COEFF_SINGLE_TAG * Math.pow(minDistance, VisionConstants.STD_DIST_PWR) * cameraConfig.getStdDevFactor();

            // double x = estimatedPose2d.getTranslation().getX();
            // double y = estimatedPose2d.getTranslation().getY();
            // if (x - VisionConstants.ROBOT_RADIUS < 0 ||
            // x + VisionConstants.ROBOT_RADIUS > VisionConstants.FIELD_X ||
            // y - VisionConstants.ROBOT_RADIUS < 0 ||
            // y + VisionConstants.ROBOT_RADIUS > VisionConstants.FIELD_Y
            // ){
            // continue;
            // }

            visionConsumer.accept(
                new TimestampedVisionUpdate(
                    result.getTimestampSeconds(),
                    estimatedPose3d,
                    false,
                    VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY)));
            visionPosePublisher.set(estimatedPose2d);
            Logger.recordOutput(cameraConfig.getCameraName() + " Estimated Pose", estimatedPose2d);
        }

    }

    /**
     * Sets up interfaces between swerve subsystem and vision subsystem
     * 
     * @param consumer consumer to receive vision updates
     */
    public void setInterface(Consumer<TimestampedVisionUpdate> consumer) {
        visionConsumer = consumer; // thiing for vision to interface with the swerve subsystem
    }

    /**
     * Initializes Networktables.
     */
    private void initNt(CameraConfig cameraConfig) {
        ntInstance = NetworkTableInstance.getDefault();
        visionStatsTable = ntInstance.getTable("Vision Debug " + cameraConfig.getCameraName());
        visionPosePublisher = visionStatsTable.getStructTopic("estimated pose", Pose2d.struct).publish();
        visionDistPublisher = visionStatsTable.getDoubleTopic("dist").publish();
        cameraPosePublisher = visionStatsTable.getStructTopic("camera pose", Pose3d.struct).publish();
        tagDistancePublisher = visionStatsTable.getDoubleArrayTopic("Tag Distances").publish();
        cameraPosePublisher.set(new Pose3d().transformBy(cameraConfig.getCameraPose()));
    }

    public void snapShot() { // download image from
        camera.takeOutputSnapshot();
    }

    public Transform3d cameraToApriltag() {
        return latestTransform3d;
    }

    public String getCamID() {
        return camID;
    }
}
