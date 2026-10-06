package frc.robot.subsystems.vision;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Real PhotonVision IO fed by PhotonVision's camera simulation: every loop it renders the AprilTags
 * this camera would see from the robot's true pose, then reads results like on the robot.
 * Based on the AdvantageKit vision template.
 */
public class VisionIOPhotonSim extends VisionIOPhoton {
    private final VisionSystemSim visionSim;
    private final Supplier<Pose3d> truePoseSupplier;

    /**
     * @param truePoseSupplier where the robot actually is (the physics sim's pose, not odometry). 3D so
     *        the cameras tilt and rise with the robot on the BUMPs
     */
    public VisionIOPhotonSim(CameraConfig camConfig, Supplier<Pose3d> truePoseSupplier) {
        super(camConfig);
        this.truePoseSupplier = truePoseSupplier;

        SimCameraProperties properties = new SimCameraProperties()
            .setCalibration(
                VisionConstants.SIM_RESOLUTION_WIDTH_PX,
                VisionConstants.SIM_RESOLUTION_HEIGHT_PX,
                Rotation2d.fromDegrees(VisionConstants.SIM_DIAGONAL_FOV_DEG))
            .setCalibError(VisionConstants.SIM_AVG_PIXEL_ERROR, VisionConstants.SIM_PIXEL_ERROR_STD_DEV)
            .setFPS(VisionConstants.SIM_FPS)
            .setAvgLatencyMs(VisionConstants.SIM_AVG_LATENCY_MS)
            .setLatencyStdDevMs(VisionConstants.SIM_LATENCY_STD_DEV_MS);

        PhotonCameraSim cameraSim = new PhotonCameraSim(cam, properties, VisionConstants.FIELD_LAYOUT);
        // Rendering the video streams is the expensive part of camera sim and nothing reads them
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);

        // One sim per camera, so each camera is rendered once per loop
        visionSim = new VisionSystemSim(camConfig.cameraName());
        visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
        visionSim.addCamera(cameraSim, camConfig.robotToCamera());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(truePoseSupplier.get());
        super.updateInputs(inputs);
    }
}
