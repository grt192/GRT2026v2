package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    // Only the sim uses this, to know where tags are. The robot aligns to whatever tag it sees
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Pose ambiguity above this is ignored (0 = certain, 1 = coin flip between two solutions)
    public static final double MAX_AMBIGUITY = 0.2;

    // Camera simulation (typical AprilTag camera; swap in the real calibration when known)
    public static final int SIM_RESOLUTION_WIDTH_PX = 1280;
    public static final int SIM_RESOLUTION_HEIGHT_PX = 800;
    public static final double SIM_DIAGONAL_FOV_DEG = 80.0;
    public static final double SIM_AVG_PIXEL_ERROR = 0.25;
    public static final double SIM_PIXEL_ERROR_STD_DEV = 0.08;
    public static final double SIM_FPS = 30.0;
    public static final double SIM_AVG_LATENCY_MS = 35.0;
    public static final double SIM_LATENCY_STD_DEV_MS = 5.0;

    // On the front edge of the frame, centered, facing forward and tilted up 30 degrees (negative
    // pitch is up). Name and height are placeholders: match the name to PhotonVision and measure
    // the height from the floor to the lens
    public static final CameraConfig CAMERA_CONFIG = new CameraConfig(
        "front",
        new Transform3d(
            Units.inchesToMeters(27.5 / 2),
            0.0,
            Units.inchesToMeters(8),
            new Rotation3d(0.0, -Math.toRadians(30), 0.0)));
}
