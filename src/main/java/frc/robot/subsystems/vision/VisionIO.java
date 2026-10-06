package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public TagObservation[] tagObservations = new TagObservation[0];
    }

    public static record TagObservation(
        double timestamp,
        int tagId,
        Transform3d cameraToTag,
        double ambiguity) {}

    default void updateInputs(VisionIOInputs inputs) {}
}
