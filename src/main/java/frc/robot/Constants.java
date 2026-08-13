// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// BOOSTED SWERVE

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
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

    // ==================== DRIVETRAIN ====================

    public static class SwerveDriveConstants {

        // Motor Configuration

        // Current Limits (defaults - tunable via NetworkTables)
        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 70;

        public static final double DRIVE_STATOR_CURRENT_LIMIT = 200; // hardware safety cutoff
        public static final double DRIVE_PEAK_STATOR_CURRENT = 120; // max current FOC control can request

        public static final boolean DRIVE_CURRENT_LIMIT_ENABLE = true;
        public static final double DRIVE_RAMP_RATE = 0.0;

        // Physical Measurements (
        public static final double DRIVE_WHEEL_RADIUS_METERS = 0.051; // meters
        public static final double DRIVE_WHEEL_CIRCUMFERENCE_METERS = 2.0 * Math.PI * DRIVE_WHEEL_RADIUS_METERS; // meters
        public static final double DRIVE_GEAR_REDUCTION = 8.25; // L2 gearing

        // Measured max drive speed
        public static final double TRUE_MAX_DRIVE_SPEED = 3.87; // put robot in the air and measure from nt

        // MotionMagic parameters for drive motors (tested values)
        public static final double DRIVE_MAX_VELOCITY_RPS = 100.0; // 90
        public static final double DRIVE_MAX_ACCELERATION = 300.0; // 170
    }

    public static class SwerveSteerConstants {
        // Motor Configuration
        public static final double STEER_PEAK_STATOR_CURRENT = 40;
        public static final double STEER_RAMP_RATE = 0;

        // Current Limits (optimized for Kraken motors - steer needs less current)
        public static final double STEER_SUPPLY_CURRENT_LIMIT = 30; // Prevents brownouts
        public static final double STEER_STATOR_CURRENT_LIMIT = 50; // Sufficient for steering
        public static final boolean STEER_CURRENT_LIMIT_ENABLE = true;

        // Physical Measurements
        public static final double STEER_GEAR_REDUCTION = 160.0 / 7.0; // ~22.86:1
        public static final double STEER_FREE_SPEED_RPM = 7530.0; // Kraken X44

        // Motion Magic (theoretical max from motor specs)
        // 7530 RPM / 22.86 gear ratio / 60 = 5.49 rot/sec output
        public static final double STEER_CRUISE_VELOCITY = STEER_FREE_SPEED_RPM / STEER_GEAR_REDUCTION / 60.0;
        // 10x velocity = reach max in 0.1 sec
        public static final double STEER_ACCELERATION = STEER_CRUISE_VELOCITY * 10.0;
    }

    public static class SwerveConstants {

        // Drive PID (Velocity Control)
        public static final double[] DRIVE_P = {9.5, 9.5, 9.5, 9.5};
        public static final double[] DRIVE_I = {0, 0, 0, 0};
        public static final double[] DRIVE_D = {0.1, 0.1, 0.1, 0.1};
        public static final double[] DRIVE_S = {0.5, 0.5, 0.5, 0.5};
        public static final double[] DRIVE_V = {0.12, 0.12, 0.12, 0.12};

        // Steer PID (Position Control)
        public static final double[] STEER_P = {190, 190, 190, 190};
        public static final double[] STEER_I = {0, 0, 0, 0};
        public static final double[] STEER_D = {7, 7, 7, 7};
        public static final double[] STEER_S = {1, 1, 1, 1};

        // ID
        public static final int PIGEON_ID = 24;
        // Module CAN IDs and Offsets (per README)
        public static final int FL_DRIVE = 0;
        public static final int FL_STEER = 1;
        public static final int FL_ENCODER = 8;
        public static final double FL_OFFSET = 0;

        public static final int FR_DRIVE = 2;
        public static final int FR_STEER = 3;
        public static final int FR_ENCODER = 9;
        public static final double FR_OFFSET = 0;

        public static final int BL_DRIVE = 4;
        public static final int BL_STEER = 5;
        public static final int BL_ENCODER = 10;
        public static final double BL_OFFSET = 0;

        public static final int BR_DRIVE = 6;
        public static final int BR_STEER = 7;
        public static final int BR_ENCODER = 11;
        public static final double BR_OFFSET = 0;

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

        // Boost Mode (L1 held) bypasses the drive speed-limit multiplier and the
        // software acceleration limiter entirely in SwerveSubsystem.setDrivePowers,
        // so there are no separate boost constants -- it runs at MAX_VEL / MAX_OMEGA
        // with raw commands straight to the modules.

        // Slow Mode Constants (R1 held)
        public static final double SLOW_MODE_SPEED_LIMIT = 0.3; // 30% speed when R1 held

        // Chassis Rotation PID (for heading lock / field-oriented rotation)
        public static final double ROTATION_P = 4.0;
        public static final double ROTATION_I = 0.0;
        public static final double ROTATION_D = 0.2;

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


    // ==================== SUBSYSTEMS ====================


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
        public static final String SWERVE_TABLE = "SwerveStats";
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

    public static class DebugConstants {
        public static final boolean MASTER_DEBUG = false;
        public static final boolean DRIVE_DEBUG = false;
        public static final boolean STEER_DEBUG = false;
        public static final boolean STATE_DEBUG = false;
    }
}
