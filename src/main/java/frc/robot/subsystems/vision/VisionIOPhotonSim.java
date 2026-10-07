package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.LoggedTunableNumber;

/**
 * Real PhotonVision IO fed by PhotonVision's camera simulation: every loop it renders the AprilTag
 * this camera would see from the robot's true pose, then reads results like on the robot.
 * The sim field has one tag (see {@link VisionConstants#SIM_TAG_POSE}) whose height is tunable.
 * Based on the AdvantageKit vision template.
 */
public class VisionIOPhotonSim extends VisionIOPhoton {
    private final VisionSystemSim visionSim;
    private final Supplier<Pose3d> truePoseSupplier;
    private final String tagPoseKey;

    private final LoggedTunableNumber tagHeight;
    private final LoggedTunableNumber.Watcher tagHeightWatcher;
    private Pose3d tagPose;

    /**
     * @param truePoseSupplier where the robot actually is (the physics sim's pose, not odometry). 3D so
     *        the cameras tilt and rise with the robot on the BUMPs
     */
    public VisionIOPhotonSim(CameraConfig camConfig, Supplier<Pose3d> truePoseSupplier) {
        super(camConfig);
        this.truePoseSupplier = truePoseSupplier;
        tagPoseKey = "Vision/" + camConfig.cameraName() + "/SimTagPose";

        tagHeight = new LoggedTunableNumber("Vision/SimTagHeight_m", VisionConstants.SIM_TAG_DEFAULT_HEIGHT_M);
        tagHeightWatcher = tagHeight.watcher();
        AprilTagFieldLayout tagLayout = createTagLayout(tagHeight.get());

        SimCameraProperties properties = new SimCameraProperties()
            .setCalibration(
                VisionConstants.SIM_RESOLUTION_WIDTH_PX,
                VisionConstants.SIM_RESOLUTION_HEIGHT_PX,
                Rotation2d.fromDegrees(VisionConstants.SIM_DIAGONAL_FOV_DEG))
            .setCalibError(VisionConstants.SIM_AVG_PIXEL_ERROR, VisionConstants.SIM_PIXEL_ERROR_STD_DEV)
            .setFPS(VisionConstants.SIM_FPS)
            .setAvgLatencyMs(VisionConstants.SIM_AVG_LATENCY_MS)
            .setLatencyStdDevMs(VisionConstants.SIM_LATENCY_STD_DEV_MS);

        // The layout here is only used for multi-tag estimates, which one tag never makes, so it's
        // fine that it keeps the starting height
        PhotonCameraSim cameraSim = new PhotonCameraSim(cam, properties, tagLayout);
        // Rendering the video streams is the expensive part of camera sim and nothing reads them
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);

        // One sim per camera, so each camera is rendered once per loop
        visionSim = new VisionSystemSim(camConfig.cameraName());
        visionSim.addAprilTags(tagLayout);
        visionSim.addCamera(cameraSim, camConfig.robotToCamera());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        tagHeightWatcher.ifChanged(() -> {
            visionSim.clearAprilTags();
            visionSim.addAprilTags(createTagLayout(tagHeight.get()));
        });
        Logger.recordOutput(tagPoseKey, tagPose);

        visionSim.update(truePoseSupplier.get());
        super.updateInputs(inputs);
    }

    private AprilTagFieldLayout createTagLayout(double heightMeters) {
        tagPose = new Pose3d(
            VisionConstants.SIM_TAG_POSE.getX(),
            VisionConstants.SIM_TAG_POSE.getY(),
            heightMeters,
            new Rotation3d(VisionConstants.SIM_TAG_POSE.getRotation()));
        return new AprilTagFieldLayout(
            List.of(new AprilTag(VisionConstants.SIM_TAG_ID, tagPose)),
            VisionConstants.SIM_FIELD.getFieldLength(),
            VisionConstants.SIM_FIELD.getFieldWidth());
    }
}
