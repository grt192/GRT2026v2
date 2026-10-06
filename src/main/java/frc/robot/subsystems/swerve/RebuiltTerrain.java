package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;

/**
 * Floor height of the 2026 REBUILT field for simulation, which is flat except for the four BUMPs.
 * maple-sim is 2D, so this supplies the robot's height and tilt when it drives over them.
 *
 * <p>
 * Per game manual 5.5, each BUMP is 73.0in wide, 44.4in deep and 6.513in tall with a 15 degree ramp
 * down each side (toward the NEUTRAL ZONE and toward the ALLIANCE ZONE), so it is a ridge along the
 * field's Y axis. One BUMP sits on each side of each HUB; they are modelled as touching the HUB.
 */
public final class RebuiltTerrain {
    private static final double BUMP_WIDTH_Y_M = Units.inchesToMeters(73.0);
    private static final double BUMP_DEPTH_X_M = Units.inchesToMeters(44.4);
    private static final double BUMP_HEIGHT_M = Units.inchesToMeters(6.513);
    private static final double RAMP_SLOPE = Math.tan(Math.toRadians(15.0));

    // HUB centers and size (same values maple-sim uses for its field)
    private static final double[] HUB_X_M = {4.5974, 11.938};
    private static final double HUB_Y_M = 4.034536;
    private static final double HUB_WIDTH_Y_M = Units.inchesToMeters(47.0);

    // Where the wheels touch the floor, relative to the robot center (FL, FR, BL, BR)
    private static final Translation2d[] WHEELS = {
            SwerveConstants.FL_POS, SwerveConstants.FR_POS, SwerveConstants.BL_POS, SwerveConstants.BR_POS
    };
    private static final double WHEELBASE_X_M = SwerveConstants.FL_POS.getX() - SwerveConstants.BL_POS.getX();
    private static final double TRACK_WIDTH_Y_M = SwerveConstants.FL_POS.getY() - SwerveConstants.FR_POS.getY();

    private RebuiltTerrain() {}

    /** Floor height in meters at a point on the field. */
    public static double heightAt(Translation2d point) {
        for (double hubX : HUB_X_M) {
            double dx = Math.abs(point.getX() - hubX);
            double dy = Math.abs(point.getY() - HUB_Y_M);
            boolean onBump = dx <= BUMP_DEPTH_X_M / 2.0
                && dy >= HUB_WIDTH_Y_M / 2.0
                && dy <= HUB_WIDTH_Y_M / 2.0 + BUMP_WIDTH_Y_M;
            if (onBump) {
                return Math.max(0.0, BUMP_HEIGHT_M - dx * RAMP_SLOPE);
            }
        }
        return 0.0;
    }

    /**
     * The robot's 3D pose when sitting on the floor at a 2D pose: height and tilt come from the floor
     * height under each wheel.
     */
    public static Pose3d poseOnTerrain(Pose2d pose) {
        double[] wheelHeights = new double[WHEELS.length];
        for (int i = 0; i < WHEELS.length; i++) {
            wheelHeights[i] = heightAt(pose.getTranslation().plus(WHEELS[i].rotateBy(pose.getRotation())));
        }
        double front = (wheelHeights[0] + wheelHeights[1]) / 2.0;
        double back = (wheelHeights[2] + wheelHeights[3]) / 2.0;
        double left = (wheelHeights[0] + wheelHeights[2]) / 2.0;
        double right = (wheelHeights[1] + wheelHeights[3]) / 2.0;

        // Rotation3d is CCW+ about each axis: front higher = nose up = negative pitch, left higher =
        // positive roll
        double pitch = Math.atan2(back - front, WHEELBASE_X_M);
        double roll = Math.atan2(left - right, TRACK_WIDTH_Y_M);
        double z = (front + back) / 2.0;

        return new Pose3d(
            pose.getX(),
            pose.getY(),
            z,
            new Rotation3d(roll, pitch, pose.getRotation().getRadians()));
    }
}
