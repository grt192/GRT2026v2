package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.ZyzToXyzEulerConverter;

public final class VisionConstants {
    public static final double FIELD_X = 16.54;
    public static final double FIELD_Y = 8.07;
    public static final double ROBOT_RADIUS = 0.762;
    public static final double[] STD_DEV_DIST = new double[] {
            0.75, 1.00, 1.3, 1.69, 2., 2.51, 2.78, 3.07, 3.54, 4.1, 4.52
    };
    public static final double[] X_STD_DEV = new double[] {
            0.002, 0.005, 0.007, 0.014, 0.029, 0.074, 0.101, 0.12, 0.151, 0.204, 0.287
    };
    public static final double[] Y_STD_DEV = new double[] {
            0.002, 0.005, 0.013, 0.020, 0.067, 0.080, 0.095, 0.160, 0.206, 0.259, 0.288
    };
    public static final double[] O_STD_DEV = new double[] {
            0.002, 0.004, 0.005, 0.011, 0.031, 0.4, 1.72, 1.89, 2.05, 2.443, 2.804
    };

    // intake
    public static final CameraConfig CAMERA_CONFIG_100 = new CameraConfig(
        "7",
        new Transform3d(
            0, 0, 0.5334,
            new Rotation3d(-Math.toRadians(50), 0, 0)));

    // hopper
    public static final CameraConfig CAMERA_CONFIG_101 = new CameraConfig(
        "7",
        new Transform3d(
            0.28, 0, 0,
            new Rotation3d(0, -Math.toRadians(5), 0)));

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

    public static final PolynomialRegression X_STD_DEV_MODEL = new PolynomialRegression(
        STD_DEV_DIST, X_STD_DEV, 2);
    public static final PolynomialRegression Y_STD_DEV_MODEL = new PolynomialRegression(
        STD_DEV_DIST, Y_STD_DEV, 2);
    public static final PolynomialRegression O_STD_DEV_MODEL = new PolynomialRegression(
        STD_DEV_DIST, O_STD_DEV, 1);

}
