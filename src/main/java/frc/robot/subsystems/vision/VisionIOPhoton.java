package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements VisionIO {
    protected final PhotonCamera cam;

    public VisionIOPhoton(CameraConfig camConfig) {
        cam = new PhotonCamera(camConfig.cameraName());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = cam.isConnected();

        List<TagObservation> tagObservations = new ArrayList<>();
        for (PhotonPipelineResult res : cam.getAllUnreadResults()) {
            for (PhotonTrackedTarget target : res.getTargets()) {
                tagObservations.add(new TagObservation(
                    res.getTimestampSeconds(),
                    target.getFiducialId(),
                    target.getBestCameraToTarget(),
                    target.getPoseAmbiguity()));
            }
        }

        inputs.tagObservations = tagObservations.toArray(new TagObservation[0]);
    }
}
