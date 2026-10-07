package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    // The sim has a single tag at the center of the field, facing the red side (+X, where the robot
    // spawns). Its height (floor to tag center) is tunable. The robot aligns to whatever tag it sees
    public static final AprilTagFieldLayout SIM_FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final int SIM_TAG_ID = 1;
    public static final Pose2d SIM_TAG_POSE =
        new Pose2d(SIM_FIELD.getFieldLength() / 2, SIM_FIELD.getFieldWidth() / 2, Rotation2d.kZero);
    public static final double SIM_TAG_DEFAULT_HEIGHT_M = Units.inchesToMeters(44.25);

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

    // On the front of the frame, centered side to side, 8 in off the floor. CAD gives the tilt as
    // 61.386926 degrees with 90 = level, so it's 90 - 61.386926 = 28.613074 degrees up (negative
    // pitch is up). The name is a placeholder: match it to PhotonVision
    public static final double CAMERA_TILT_UP_DEG = 90.0 - 61.386926;
    public static final CameraConfig CAMERA_CONFIG = new CameraConfig(
        "front",
        new Transform3d(
            Units.inchesToMeters(27.5 / 2), // forward from robot center: placeholder, measure it
            0.0,
            Units.inchesToMeters(8),
            new Rotation3d(0.0, -Math.toRadians(CAMERA_TILT_UP_DEG), 0.0)));
}
