// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// BOOSTED SWERVE

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ==================== GLOBAL ====================
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode SIM_MODE = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public enum CANType {
        RIO(CANBus.roboRIO().getName()),
        SWERVE("BetaBot");

        private final String busName;

        CANType(String busName) {
            this.busName = busName;
        }

        public String busName() {
            return busName;
        }
    }

    // debug mode / pushing hella stuff to NT tables
    // Subsystem Enable/Disable
    public static final boolean TUNING_MODE = true;

    // ==================== CONTROLLERS ====================

    public static class ControllerConstants {
        // Stick deadbands (fraction of full stick travel)
        public static final double PS5_STICK_DEADBAND = 0.1;
    }

    // ==================== DRIVETRAIN ====================

    public static class SwerveDriveConstants {

        // Motor Configuration

        // Current Limits
        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 70;

        public static final double DRIVE_STATOR_CURRENT_LIMIT = 120;

        public static final boolean DRIVE_CURRENT_LIMIT_ENABLE = true;

        // Physical Measurements (
        public static final double DRIVE_WHEEL_RADIUS_METERS = 0.051; // meters
        public static final double DRIVE_WHEEL_CIRCUMFERENCE_METERS = 2.0 * Math.PI * DRIVE_WHEEL_RADIUS_METERS; // meters
        public static final double DRIVE_GEAR_REDUCTION = 8.25; // R1 gearing

        // Measured max drive speed
        public static final double TRUE_MAX_DRIVE_SPEED = 3.87; // put robot in the air and measure from nt

        // Velocity PID (VelocityVoltage: volts per wheel rot/s). Starting points, not tuned on BetaBot:
        // CTRE's swerve defaults per motor-rotor rot/s, scaled by the gear reduction since the TalonFX
        // reports wheel rotations. kV is 12 V over a Kraken X60's ~100 rot/s free speed
        public static final double kP = 0.1 * DRIVE_GEAR_REDUCTION;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.15; // volts
        public static final double kV = 0.12 * DRIVE_GEAR_REDUCTION;
    }

    public static class SwerveSteerConstants {
        // Current Limits (optimized for Kraken motors - steer needs less current)
        public static final double STEER_SUPPLY_CURRENT_LIMIT = 30; // Prevents brownouts
        public static final double STEER_STATOR_CURRENT_LIMIT = 50; // Sufficient for steering
        public static final boolean STEER_CURRENT_LIMIT_ENABLE = true;

        // Physical Measurements
        public static final double STEER_GEAR_REDUCTION = 160.0 / 7.0; // ~22.86:1

        // Position PID (PositionVoltage: volts per module rotation). CTRE's swerve defaults, not tuned
        // on BetaBot
        public static final double kP = 100;
        public static final double kI = 0;
        public static final double kD = 0.5;
        public static final double kS = 0.1;
        public static final double kV = 0;
    }

    /** Physical model for the maple-sim drivetrain. Estimates -- update with real numbers when known. */
    public static class SwerveSimConstants {
        // Physics step. The CTRE sim devices run in real time, so stepping faster than the 20 ms
        // robot loop keeps the motor controllers' closed loops stable (254 uses 5 ms too)
        public static final double PERIOD_SECONDS = 0.005;

        public static final double ROBOT_MASS_KG = 60.0; // with bumpers and battery
        public static final double BUMPER_LENGTH_X_METERS = 0.76;
        public static final double BUMPER_WIDTH_Y_METERS = 0.76;

        public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.2;
        public static final double DRIVE_FRICTION_VOLTS = 0.1;
        public static final double STEER_FRICTION_VOLTS = 0.15;
        // maple-sim's COTS module default; much smaller goes numerically unstable at a 5 ms step
        public static final double STEER_MOMENT_OF_INERTIA_KG_M2 = 0.03;
    }

    public static class SwerveConstants {
        // ID
        public static final int PIGEON_ID = 42;
        // Module CAN IDs and encoder offsets (per README)
        // The offset is the CANcoder reading (rotations) when the wheel faces forward. The old
        // SteerMotor/KrakenSwerveModule code read the wheel angle as (CANcoder - 0.5 rot), so 0.5
        // keeps the same zero as before.
        public static final int FL_DRIVE = 0;
        public static final int FL_STEER = 1;
        public static final int FL_ENCODER = 8;
        public static final double FL_ENCODER_OFFSET_ROT = 0.5;

        public static final int FR_DRIVE = 2;
        public static final int FR_STEER = 3;
        public static final int FR_ENCODER = 9;
        public static final double FR_ENCODER_OFFSET_ROT = 0.5;

        public static final int BL_DRIVE = 4;
        public static final int BL_STEER = 5;
        public static final int BL_ENCODER = 10;
        public static final double BL_ENCODER_OFFSET_ROT = 0.5;

        public static final int BR_DRIVE = 6;
        public static final int BR_STEER = 7;
        public static final int BR_ENCODER = 11;
        public static final double BR_ENCODER_OFFSET_ROT = 0.5;

        // Module Positions (meters, relative to robot center)
        // WPILib: +X = front, +Y = left
        public static final Translation2d FL_POS = new Translation2d(0.289878, 0.289878); // front-left
        public static final Translation2d FR_POS = new Translation2d(0.289878, -0.289878); // front-right
        public static final Translation2d BL_POS = new Translation2d(-0.289878, 0.289878); // back-left
        public static final Translation2d BR_POS = new Translation2d(-0.289878, -0.289878); // back-right

        // Kinematic Limits (using measured true max speed)
        public static final double MAX_VEL = SwerveDriveConstants.TRUE_MAX_DRIVE_SPEED; // 3.87 m/s
        public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();

        // Chassis Acceleration Limits (set to max - no software limiting)
        // increase = faster response, decrease = smoother
        public static final double MAX_LINEAR_ACCELERATION = 7; // how fast robot speeds up (m/s²)
        public static final double MAX_LINEAR_DECELERATION = 11; // how fast robot stops (m/s²)

        public static final double MAX_ANGULAR_ACCELERATION = 20; // how fast robot starts spinning (rad/s²)
        public static final double MAX_ANGULAR_DECELERATION = 30; // how fast robot stops spinning (rad/s²)

        // Practice starting position: 1.5 meters in front of the red hub (which is at x=11.9). Also
        // where the robot spawns in sim
        public static final Pose2d STARTING_POSE = new Pose2d(10.4, 4.0, Rotation2d.kZero);

        // Modules X-lock after the drivetrain has been commanded to zero for this long
        public static final double LOCK_TIMEOUT_SECONDS = 1.0;

        // How much the pose estimator trusts odometry vs vision (smaller = trust odometry more, so vision
        // nudges the pose instead of yanking it). 6328's 2025 values; they're 2D, so z reuses xy
        public static final double ODOMETRY_XY_STD_DEV_M = 0.003;
        public static final double ODOMETRY_Z_STD_DEV_M = 0.003;
        public static final double ODOMETRY_THETA_STD_DEV_RAD = 0.002;

        // Odometry sample rate on a CAN FD bus; non-FD buses fall back to 100 Hz
        public static final double ODOMETRY_FREQUENCY_FD_HZ = 250.0;
        public static final double ODOMETRY_FREQUENCY_HZ = 100.0;
    }

    // ==================== ALIGN TO TAG ====================

    public static class AlignToTagConstants {
        // How far the camera stops from the tag (horizontal, camera lens to tag face) for a given tag
        // height (floor to tag center). Heights in between are interpolated, heights outside are
        // clamped. With the camera 8 in up and tilted 28.6 degrees, both ends keep the whole tag in
        // frame: about 11-20 degrees above level for the low tag and 32-33 for the high one, inside
        // the camera's roughly 5-53 degree view
        public static final double LOW_TAG_HEIGHT_M = Units.feetToMeters(1.5);
        public static final double LOW_TAG_DISTANCE_M = Units.feetToMeters(3);
        public static final double HIGH_TAG_HEIGHT_M = Units.feetToMeters(7);
        public static final double HIGH_TAG_DISTANCE_M = Units.feetToMeters(10);

        // Translation (m/s per m of error) and rotation (rad/s per rad of error) gains
        public static final double TRANSLATION_P = 2.0;
        public static final double ROTATION_P = 4.0;

        // Kept well under the drivetrain max so the demo stays gentle
        public static final double MAX_VELOCITY_MPS = 1.5;
        public static final double MAX_OMEGA_RADPS = 3.0;

        // A locked tag unseen for this long is dropped, so the command can pick a new one
        public static final double TAG_LOST_TIMEOUT_SECONDS = 0.5;
    }
}
