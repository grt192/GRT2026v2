package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ZyzToXyzEulerConverter;

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

    // the big 3 cameras
    public static final CameraConfig CAMERA_CONFIG_1 = new CameraConfig(// climb camera
        "1",
        new Transform3d(
            Units.inchesToMeters(27.5 / 2 - 16.75),
            Units.inchesToMeters(27.5 / 2 - 2.75),
            Units.inchesToMeters(11.5),
            new Rotation3d(Math.PI, -Math.toRadians(30), -Math.PI / 2.0)));

    // intake
    public static final CameraConfig CAMERA_CONFIG_2 = new CameraConfig(// shooter camera
        "2",
        new Transform3d(
            Units.inchesToMeters(-(27.5 / 2 - 1.5)),
            Units.inchesToMeters(-(27.5 / 2 - 6.5)),
            Units.inchesToMeters(18.75),
            ZyzToXyzEulerConverter.zyxToXyz(-Math.PI / 2, -Math.toRadians(21), 0)));

    // hopper
    public static final CameraConfig CAMERA_CONFIG_3 = new CameraConfig(// auxillary camera
        "3",
        new Transform3d(
            Units.inchesToMeters(-(27.5 / 2 - 2.5)),
            Units.inchesToMeters(-(27.5 / 2 - 10.5)),
            Units.inchesToMeters(15),
            ZyzToXyzEulerConverter.zyxToXyz(Math.PI, -Math.toRadians(11), 0)));
}
