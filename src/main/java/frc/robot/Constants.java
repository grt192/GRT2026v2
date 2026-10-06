// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// BOOSTED SWERVE

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
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
    // Moment-of-inertia conversion factor: 1 lb·in² = (kg/lb)·(m/in)² kg·m².
    public static final double LB_IN2_TO_KG_M2 =
        Units.lbsToKilograms(1.0) * Units.inchesToMeters(1.0) * Units.inchesToMeters(1.0);

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
        MECH("mechCAN"),
        SWERVE("swerveCAN");

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
    public static final boolean SWERVE_ENABLED = true;
    public static final boolean MECH_ENABLED = true;

    // ==================== CONTROLLERS ====================

    public static class ControllerConstants {
        // Stick deadbands (fraction of full stick travel)
        public static final double PS5_STICK_DEADBAND = 0.035;
        public static final double XBOX_STICK_DEADBAND = 0.15;
    }

    // ==================== DRIVETRAIN ====================

    public static class SwerveDriveConstants {

        // Motor Configuration

        // Current Limits
        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 70;

        public static final double DRIVE_STATOR_CURRENT_LIMIT = 200; // hardware safety cutoff
        public static final double DRIVE_PEAK_STATOR_CURRENT = 120; // max current FOC control can request

        public static final boolean DRIVE_CURRENT_LIMIT_ENABLE = true;

        // Physical Measurements (
        public static final double DRIVE_WHEEL_RADIUS_METERS = 0.051; // meters
        public static final double DRIVE_WHEEL_CIRCUMFERENCE_METERS = 2.0 * Math.PI * DRIVE_WHEEL_RADIUS_METERS; // meters
        public static final double DRIVE_GEAR_REDUCTION = 8.25; // L2 gearing

        // Measured max drive speed
        public static final double TRUE_MAX_DRIVE_SPEED = 3.87; // put robot in the air and measure from nt

        // Velocity PID (VelocityTorqueCurrentFOC: amps per wheel rot/s). These were tuned against
        // motor-rotor rot/s, so they're scaled by the gear reduction now that the TalonFX reports
        // wheel rotations.
        public static final double kP = 9.5 * DRIVE_GEAR_REDUCTION;
        public static final double kI = 0.0;
        public static final double kD = 0.1 * DRIVE_GEAR_REDUCTION;
        public static final double kS = 0.5; // amps
        public static final double kV = 0.12 * DRIVE_GEAR_REDUCTION;
    }

    public static class SwerveSteerConstants {
        // Motor Configuration
        public static final double STEER_PEAK_STATOR_CURRENT = 40;

        // Current Limits (optimized for Kraken motors - steer needs less current)
        public static final double STEER_SUPPLY_CURRENT_LIMIT = 30; // Prevents brownouts
        public static final double STEER_STATOR_CURRENT_LIMIT = 50; // Sufficient for steering
        public static final boolean STEER_CURRENT_LIMIT_ENABLE = true;

        // Physical Measurements
        public static final double STEER_GEAR_REDUCTION = 160.0 / 7.0; // ~22.86:1

        // Position PID (PositionTorqueCurrentFOC: amps per module rotation)
        public static final double kP = 190;
        public static final double kI = 0;
        public static final double kD = 7;
        public static final double kS = 1;
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
        public static final int PIGEON_ID = 24;
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

        // Boost Mode (R2 held) bypasses the drive speed-limit multiplier and the
        // software acceleration limiter entirely in DriveSubsystem.setDrivePowers,
        // so there are no separate boost constants -- it runs at MAX_VEL / MAX_OMEGA
        // with raw commands straight to the modules.

        // Slow Mode Constants (R1 held)
        public static final double SLOW_MODE_SPEED_LIMIT = 0.3; // 30% speed when R1 held

        // Practice starting position: 1.5 meters in front of the red hub (which is at x=11.9). Also
        // where the robot spawns in sim
        public static final Pose2d STARTING_POSE = new Pose2d(10.4, 4.0, Rotation2d.kZero);

        // Modules X-lock after the drivetrain has been commanded to zero for this long
        public static final double LOCK_TIMEOUT_SECONDS = 1.0;

        // Odometry sample rate on a CAN FD bus; non-FD buses fall back to 100 Hz
        public static final double ODOMETRY_FREQUENCY_FD_HZ = 250.0;
        public static final double ODOMETRY_FREQUENCY_HZ = 100.0;

        // PathPlanner Auto PID Constants
        public static final double AUTO_TRANSLATION_P = 1.6; // increased 4/12/26 TONY
        public static final double AUTO_TRANSLATION_I = 0.0;
        public static final double AUTO_TRANSLATION_D = 0.0;
        public static final double AUTO_ROTATION_P = 4.0; // increased 4/12/26 TONY
        public static final double AUTO_ROTATION_I = 0.0;
        public static final double AUTO_ROTATION_D = 0.0;
    }

    public static class RotateToAngleConstants {
        public static final double kP = 0.009;
        public static final double kI = 0.0;
        public static final double kD = 0.0005;
        public static final double TOLERANCE_DEGREES = 0.0;
    }

    // ==================== SHOOTER ====================

    public static class TowerConstants {
        public static final int KRAKEN_CAN_ID = 26;

        public enum TowerIntake {
            BALL_UP,
            BALL_DOWN,
            STOP
        }

        // maths
        public static final double GEAR_REDUCTION = 4.0;

        public static final double TARGET_BPS = 4.0; // frequency
        public static final double WHEEL_RADIUS = 1.0; // distance
        public static final double BALL_DIAMETER = 6.0; // distance
        public static final double TARGET_VELO_RPS = 30.0; // TARGET_BPS * BALL_DIAMETER / WHEEL_RADIUS;

        // Velocity control PID (SysID Derived - Voltage)
        public static final double kP = 0.00625;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.231;
        public static final double kV = 0.388;
        public static final double kA = 0.00582;

        public static final double SIM_P = 0.00625;
        public static final double SIM_V = 0.388;

        public static final double VELOCITY_TOLERANCE_RPS = 7.58;

        // motion magic
        public static final double MM_ACCEL_RPS2 = 1000.0;
        public static final double MM_MAX_VELO_RPS = 100.0;
        public static final double MM_JERK_RPS3 = 100.0;

        // Current limits
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 80.0;
        public static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
        public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;

        // Motor config
        public static final InvertedValue HOPPER_INVERTED = InvertedValue.Clockwise_Positive;
    }

    public static class ShooterConstants {

        // ---- Flywheel ----
        public static class Flywheel {
            public static final InvertedValue F_INVERTED_VALUE = InvertedValue.Clockwise_Positive;

            public static final int UPPER_MOTOR_ID = 17;
            public static final int SECOND_MOTOR_ID = 25;

            public static final double GEAR_RATIO = 1.0;

            // Velocity control PID
            public static final double kP = 10;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.0;
            public static final double kV = 0.12;
            public static final double kA = 0.0;

            // Sim-only PID gains (no friction / different plant).
            public static final double SIM_KP = 0.8;
            public static final double SIM_KV = 0.0;

            // Motion Magic
            public static final double MM_ACCEL_RPS2 = 100.0;
            public static final double MM_MAX_VELO_RPS = 500.0;
            public static final double MM_JERK_RPS3 = 150.0;

            // Velocity tolerance for "at speed" check
            public static final double VELOCITY_TOLERANCE_RPS = 2.0;

            public static final double FLYWHEEL_MAX_SPEED_RPS = 120.0;
        }

        // ---- Hood ----
        public static class Hood {
            public static final int MOTOR_ID = 16;
            public static final int ENCODER_ID = 18;

            public static final double GEAR_RATIO = 244.411765;

            // Position control PID (BAD BAD BAD)
            public static final double kP = 2000;
            public static final double kI = 0;
            public static final double kD = 60;
            public static final double kS = 120;

            // Sim-only PID gains
            public static final double SIM_P = 60.0;
            public static final double SIM_D = 2.0;

            // Plant model (matches Intake pivot until Hood CAD numbers exist)
            public static final double MOMENT_OF_INERTIA_KG_M2 = 598.456909 * LB_IN2_TO_KG_M2;
            public static final double COM_LENGTH_M = Units.inchesToMeters(Math.hypot(0.121549, 9.035458));

            // Angle limits (rotations)
            public static final double UPPER_ANGLE_LIMIT_ROT = 0.1;
            public static final double LOWER_ANGLE_LIMIT_ROT = 0.0;
            public static final double INIT_ANGLE_ROT = UPPER_ANGLE_LIMIT_ROT;
            public static final double MAGNET_OFFSET = -0.05688;

            // Current limits
            public static final double STATOR_CURRENT_LIMIT_AMPS = 50.0;
            public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
            public static final boolean CURRENT_LIMIT_ENABLE = true;

            public static final double ANGLE_TOLERANCE_ROT = 0.01;
        }
    }

    // ==================== SUBSYSTEMS ====================

    public static class IntakeConstants {
        // Roller Motor
        public static final int ROLLER_CAN_ID = 14;
        public static final double ROLLER_IN_SPEED_RPS = -85.0;
        public static final double ROLLER_OUT_SPEED_RPS = 85.0;
        public static final double ROLLER_VELOCITY_TOLERANCE_RPS = 5.0;
        public static final double ROLLER_CURRENT_LIMIT_AMPS = 120.0;
        public static final double ROLLER_STATOR_CURRENT_LIMIT_AMPS = 120.0;
        public static final double ROLLER_OPEN_LOOP_RAMP = 0.0;
        public static final InvertedValue ROLLER_INVERTED = InvertedValue.CounterClockwise_Positive;

        // Roller Position control PID (SysID Derived - Voltage)
        public static final double ROLLER_P = 0.0930;
        public static final double ROLLER_I = 0.0;
        public static final double ROLLER_D = 0.0;
        public static final double ROLLER_S = 0.866;
        public static final double ROLLER_V = 0.100;
        public static final double ROLLER_A = 0.0116;

        // Roller Sim PID
        public static final double ROLLER_SIM_P = 0.9;
        public static final double ROLLER_SIM_V = 0.12;

        // Pivot Motor
        public static final int PIVOT_MOTOR_ID = 12;
        public static final int PIVOT_CANCODER_ID = 13;
        public static final double MANUAL_PIVOT_SPEED = 1;
        public static final double PIVOT_STATOR_CURRENT_LIMIT_AMPS = 40.0;
        public static final boolean PIVOT_STATOR_CURRENT_LIMIT_ENABLE = true;

        // Pivot PID
        public static final double PIVOT_P = 60.0;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 4;
        public static final double PIVOT_S = 0.2;
        public static final double PIVOT_G = 0.0;
        public static final double PIVOT_V = 3.18;
        public static final double PIVOT_A = 0.27;

        public static final double PIVOT_SIM_P = 40.0;
        public static final double PIVOT_SIM_D = .7;
        public static final double PIVOT_SIM_G = 0.73;

        public static final double GEAR_RATIO = 20.0;
        public static final double PIVOT_MOMENT_OF_INERTIA_KG_M2 = 598.456909 * LB_IN2_TO_KG_M2; // Onshape
        public static final double PIVOT_COM_LENGTH_M =
            Units.inchesToMeters(Math.hypot(0.121549, 9.035458)); // Onshape COM offset (x, y) from pivot

        // Pivot Positions (in encoder rotations)
        public static final double PIVOT_FORWARD_LIMIT_ROT = 0.3568;
        public static final double PIVOT_REVERSE_LIMIT_ROT = 0.000;

        public static final double PIVOT_OUT_POS_ROT = 0.0;
        public static final double PIVOT_IN_POS_ROT = 0.33;
        public static final double PIVOT_MID_UPPER_ROT = 0.175;
        public static final double PIVOT_MID_LOWER_ROT = 0.091;

        public static final double PIVOT_POSITION_TOLERANCE_ROT = 5.0 / 360.0;

        // Pivot Duty Cycle
        public static final double PIVOT_UP_DUTY_CYCLE = 0.3;
        public static final double PIVOT_DOWN_DUTY_CYCLE = -0.3;
        public static final double PIVOT_UP_DURATION_SECONDS = 1.5;
        public static final double PIVOT_DOWN_DURATION_SECONDS = 1.5;
    }

    public static class HopperConstants {
        public static final int KRAKEN_CAN_ID = 15;

        public enum HopperIntake {
            BALL_IN,
            BALL_OUT,
            STOP
        }

        // Velocity control PID (SysID Derived - Voltage)
        public static final double kP = 0.0673;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.464;
        public static final double kA = 0.0127;
        public static final double VELOCITY_TOLERANCE_RPS = 6.47;

        // Sim-only PID gains (no friction / different plant — match the dashboard defaults).
        public static final double SIM_KP = 0.8;
        public static final double SIM_KI = 0.0;
        public static final double SIM_KD = 0.0;
        public static final double SIM_KS = 0.0;
        public static final double SIM_KV = 0.0;
        public static final double SIM_KA = 0.0;

        // Motion Magic Constants
        public static final double MM_ACCEL_RPS2 = 100.0;
        public static final double MM_MAX_VELO_RPS = 100.0;
        public static final double MM_JERK_RPS3 = 100.0;

        // balls stuff
        public static final double TARGET_BPS = 4.0; // frequency
        public static final double GEAR_REDUCTION = 4.0; // dummy value -Tony 3.3.26
        public static final double TARGET_RPS = TARGET_BPS / 4.0; // divided by 4 cuz 4 vains on spinner

        // Current limits
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 80.0;
        public static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
        public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;

        // Voltage and ramping
        public static final double OPEN_LOOP_RAMP = 0.5;
        public static final double DUTY_CYCLE_OPEN_LOOP_RAMP = 0.05;

        // Motor config
        public static final InvertedValue HOPPER_INVERTED = InvertedValue.CounterClockwise_Positive;
    }

    // ==================== ALIGNMENT ====================

    public static class AlignToHubConstants {
        public static final Translation2d HUB_POSITION = new Translation2d(12.51204, 4);
    }

    public static class AlignConstants {
        public static final Translation2d BLUE_HUB_TRANS = new Translation2d(4.625, 4);
        public static final Translation2d RED_HUB_TRANS = new Translation2d(11.9, 4);
        public static final double RED_WALL_X = 11.9;
        public static final double BLUE_WALL_X = 4.625;
        public static final double HUB_Y = 4;
        public static final Translation2d BLUE_AIM_TOP = new Translation2d(2.4, 6);
        public static final Translation2d BLUE_AIM_BOTTOM = new Translation2d(2.4, 2);
        public static final Translation2d RED_AIM_TOP = new Translation2d(14.3, 6);
        public static final Translation2d RED_AIM_BOTTOM = new Translation2d(14.3, 2);
    }

    // ==================== LOGGING & DEBUG ====================

    public static class LoggingConstants {
        public static final String SENSOR_TABLE = "Sensors";
    }

    // ==================== SMASH AND SHOOT ==================== thing
    public static class SmashAndShootConstants {
        // Hood position (rotations) - between 0.06 and 0.169
        public static final double HOOD_POSITION_ROT = 0.014;

        // Flywheel speed (RPS)
        public static final double FLYWHEEL_VELO_RPS = 40.0;

        // Tower duty cycle
        public static final double TOWER_DUTY_CYCLE = 1;

        // Indexer/Hopper duty cycle
        public static final double INDEXER_DUTY_CYCLE = -1;

        // Pivot timing
        public static final double INITIAL_DELAY_SECONDS = 2.0;
        public static final double TOGGLE_INTERVAL_SECONDS = 0.5;
    }

    // ==================== CYCLE SHOOTER ====================
    public static class CycleShooterConstants {
        // Hood position (rotations)
        public static final double HOOD_POSITION_ROT = 0.096;

        // Flywheel speed (RPS)
        public static final double FLYWHEEL_VELO_RPS = 60.0;

        // Tower duty cycle
        public static final double TOWER_DUTY_CYCLE = 1;

        // Indexer/Hopper duty cycle
        public static final double INDEXER_DUTY_CYCLE = -1.0;

        // Pivot timing
        public static final double INITIAL_DELAY_SECONDS = 2.0;
        public static final double TOGGLE_INTERVAL_SECONDS = 0.5;
    }

    // ==================== TOWER SHOOT ====================
    public static class TowerShootConstants {
        // Hood position (rotations) - placeholder, tune on robot
        public static final double HOOD_POSITION_ROT = 0.05;

        // Flywheel speed (RPS) - placeholder, tune on robot
        public static final double FLYWHEEL_VELO_RPS = 49.0;
    }
}
