package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ZyzToXyzEulerConverter;

public final class VisionConstants {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double FIELD_X_M = FIELD_LAYOUT.getFieldLength();
    public static final double FIELD_Y_M = FIELD_LAYOUT.getFieldWidth();

    public static final double ROBOT_RADIUS_M = 0.4191;
    public static final double XY_TOLERANCE_M = 0.5;
    public static final double Z_TOLERANCE_M = 0.25;

    public static final double MIN_X_M = ROBOT_RADIUS_M - XY_TOLERANCE_M;
    public static final double MAX_X_M = FIELD_X_M - (ROBOT_RADIUS_M - XY_TOLERANCE_M);
    public static final double MIN_Y_M = ROBOT_RADIUS_M - XY_TOLERANCE_M;
    public static final double MAX_Y_M = FIELD_Y_M - (ROBOT_RADIUS_M - XY_TOLERANCE_M);

    public static final double STD_DIST_PWR = 1.2;
    public static final double XY_COEFF_MULTI_TAG = 0.01;
    public static final double XY_COEFF_SINGLE_TAG = 0.01;
    public static final double THETA_COEFF = 0.03;

    // How long a tag stays in the logged TagPoses after it was last seen
    public static final double TARGET_LOG_TIME_SECS = 0.1;

    // intake
    public static final CameraConfig CAMERA_CONFIG_100 = new CameraConfig(
        "7",
        new Transform3d(
            0, 0, 0.5334,
            new Rotation3d(-Math.toRadians(50), 0, 0)),
        1);

    // hopper
    public static final CameraConfig CAMERA_CONFIG_101 = new CameraConfig(
        "7",
        new Transform3d(
            0.28, 0, 0,
            new Rotation3d(0, -Math.toRadians(5), 0)),
        1);

    // the big 3 cameras
    public static final CameraConfig CAMERA_CONFIG_1 = new CameraConfig(// climb camera
        "1",
        new Transform3d(
            Units.inchesToMeters(27.5 / 2 - 16.75),
            Units.inchesToMeters(27.5 / 2 - 2.75),
            Units.inchesToMeters(11.5),
            new Rotation3d(Math.PI, -Math.toRadians(30), -Math.PI / 2.0)),
        1);

    // intake
    public static final CameraConfig CAMERA_CONFIG_2 = new CameraConfig(// shooter camera
        "2",
        new Transform3d(
            Units.inchesToMeters(-(27.5 / 2 - 1.5)),
            Units.inchesToMeters(-(27.5 / 2 - 6.5)),
            Units.inchesToMeters(18.75),
            ZyzToXyzEulerConverter.zyxToXyz(-Math.PI / 2, -Math.toRadians(21), 0)),
        1);

    // hopper
    public static final CameraConfig CAMERA_CONFIG_3 = new CameraConfig(// auxillary camera
        "3",
        new Transform3d(
            Units.inchesToMeters(-(27.5 / 2 - 2.5)),
            Units.inchesToMeters(-(27.5 / 2 - 10.5)),
            Units.inchesToMeters(15),
            ZyzToXyzEulerConverter.zyxToXyz(Math.PI, -Math.toRadians(11), 0)),
        1);
}
