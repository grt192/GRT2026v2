package frc.robot.subsystems.vision;

import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final Consumer<TimestampedVisionUpdate> visionConsumer;
    private final Supplier<Rotation2d> headingSupplier;
    private final VisionCamera[] cameras;

    /**
     * @param visionConsumer receives every accepted vision update (e.g. swerve's addVisionMeasurements)
     * @param headingSupplier field-relative robot heading, used for single-tag trig solving
     * @param cameras cameras to read from
     */
    public VisionSubsystem(
        Consumer<TimestampedVisionUpdate> visionConsumer,
        Supplier<Rotation2d> headingSupplier,
        VisionCamera... cameras) {
        this.visionConsumer = visionConsumer;
        this.headingSupplier = headingSupplier;
        this.cameras = cameras;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getFPGATimestamp();
        Rotation2d heading = headingSupplier.get();

        for (VisionCamera camera : cameras) {
            camera.updateHeading(timestamp, heading);
            camera.updateIO();

            for (TimestampedVisionUpdate update : camera.getVisionEstimates()) {
                visionConsumer.accept(update);
            }
        }
    }

    public void resetHeading(double timestamp, Rotation2d heading) {
        for (VisionCamera camera : cameras) {
            camera.resetHeading(timestamp, heading);
        }
    }
}
