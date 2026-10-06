package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.vision.VisionIO.TagObservation;

public class VisionCamera {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final CameraConfig camConfig;
    private final String logKey;

    private final Alert disconnectedAlert;

    private List<TagSighting> sightings = List.of();

    public VisionCamera(VisionIO io, CameraConfig camConfig) {
        this.io = io;
        this.camConfig = camConfig;
        logKey = "Vision/Camera" + camConfig.cameraName();

        disconnectedAlert = new Alert("Vision Camera " + camConfig.cameraName() + ": Disconnected", AlertType.kError);
    }

    /** @param robotPose where the robot is now, only used to place the logged tags on the field */
    public void updateIO(Pose3d robotPose) {
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
        disconnectedAlert.set(!inputs.connected);

        List<TagSighting> newSightings = new ArrayList<>();
        for (TagObservation obs : inputs.tagObservations) {
            // Ambiguous single-tag solves can flip the tag around, which would send the robot the wrong way
            if (obs.ambiguity() > VisionConstants.MAX_AMBIGUITY) {
                continue;
            }
            newSightings.add(new TagSighting(
                camConfig,
                obs.tagId(),
                obs.timestamp(),
                camConfig.robotToCamera().plus(obs.cameraToTag())));
        }
        sightings = newSightings;

        // Placed off the robot's pose so they line up with the real tags in AdvantageScope
        Logger.recordOutput(logKey + "/TagPoses", sightings.stream()
            .map(sighting -> robotPose.transformBy(sighting.robotToTag()))
            .toArray(Pose3d[]::new));
    }

    /** Tags seen since the previous {@link #updateIO(Pose3d)} call. */
    public List<TagSighting> getSightings() {
        return sightings;
    }
}
