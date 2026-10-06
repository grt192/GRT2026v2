package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final Supplier<Pose3d> robotPoseSupplier;
    private final VisionCamera[] cameras;

    private List<TagSighting> sightings = List.of();

    /**
     * @param robotPoseSupplier the robot's pose, only used to place the logged tags on the field
     * @param cameras cameras to read from
     */
    public VisionSubsystem(Supplier<Pose3d> robotPoseSupplier, VisionCamera... cameras) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.cameras = cameras;
    }

    @Override
    public void periodic() {
        Pose3d robotPose = robotPoseSupplier.get();
        List<TagSighting> newSightings = new ArrayList<>();
        for (VisionCamera camera : cameras) {
            camera.updateIO(robotPose);
            newSightings.addAll(camera.getSightings());
        }
        sightings = newSightings;
    }

    /** Every tag any camera saw since the previous loop. */
    public List<TagSighting> getSightings() {
        return sightings;
    }
}
