package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public TagObservation[] tagObservations = new TagObservation[0];
    }

    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        boolean isMultiTag,
        int tagCount,
        double avgTagDist) {}

    public static record TagObservation(
        double timestamp,
        int tagId,
        Transform3d cameraToTag,
        Transform3d altCameraToTag,
        double ambiguity) {}

    default void updateInputs(VisionIOInputs inputs) {}

    default void updateHeading(double timestamp, Rotation3d heading) {}

    default void resetHeading(double timestamp, Rotation3d heading) {}
}
