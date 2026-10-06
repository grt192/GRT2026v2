package frc.robot.subsystems.swerve;

import static frc.robot.Constants.DebugConstants.*;
import static frc.robot.Constants.LoggingConstants.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.SwerveSteerConstants.STEER_CRUISE_VELOCITY;
import static frc.robot.Constants.SwerveSteerConstants.STEER_GEAR_REDUCTION;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.vision.TimestampedVisionUpdate;

public class SwerveSubsystem extends SubsystemBase {

    private final KrakenSwerveModule frontLeftModule;
    private final KrakenSwerveModule frontRightModule;
    private final KrakenSwerveModule backLeftModule;
    private final KrakenSwerveModule backRightModule;
    private SwerveModuleState[] states = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };
    SwerveModuleState testState = new SwerveModuleState();
    private Pose2d estimatedPose = new Pose2d(10, 4, new Rotation2d());
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d driverHeadingOffset = new Rotation2d();

    private final Pigeon2 pidgey;
    private final CANBus canivore;
    private Timer lockTimer;
    private double currentCruiseVelocityRPM = STEER_CRUISE_VELOCITY * STEER_GEAR_REDUCTION * 60.0;

    // Acceleration limiting fields
    private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
    private double lastUpdateTime = 0.0;

    // logging
    private NetworkTableInstance ntInstance;
    private NetworkTable swerveTable;

    private StructArrayPublisher<SwerveModuleState> swerveStatesPublisher;
    private StructArrayPublisher<SwerveModuleState> desiredStatesPublisher;

    private StructPublisher<Pose2d> estimatedPosePublisher;
    // private StructLogEntry<Pose2d> estimatedPoseLogEntry =
    // StructLogEntry.create(
    // DataLogManager.getLog(),
    // "estimatedPose",
    // Pose2d.struct
    // );

    // NT Accel Config
    public double maxLinearAcceleration = MAX_LINEAR_ACCELERATION; // meters per second squared
    public double maxLinearDeceleration = MAX_LINEAR_DECELERATION; // meters per second squared
    public double maxAngularAcceleration = MAX_ANGULAR_ACCELERATION; // radians per second squared
    public double maxAngularDeceleration = MAX_ANGULAR_DECELERATION; // radians per second squared

    // Boost mode flag
    private boolean boostModeEnabled = false;

    // Robot-relative mode flag (held button -- release returns to field-relative)
    private boolean robotRelativeEnabled = false;

    public SwerveSubsystem(CANBus canBus) {
        canivore = canBus;
        rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        // initialize and reset the NavX gyro
        pidgey = new Pigeon2(SwerveConstants.PIGEON_ID, canivore);
        pidgey.reset();

        frontLeftModule = new KrakenSwerveModule(FL_DRIVE, FL_STEER, FL_OFFSET, FL_ENCODER, canivore);
        frontRightModule = new KrakenSwerveModule(FR_DRIVE, FR_STEER, FR_OFFSET, FR_ENCODER, canivore);
        backLeftModule = new KrakenSwerveModule(BL_DRIVE, BL_STEER, BL_OFFSET, BL_ENCODER, canivore);
        backRightModule = new KrakenSwerveModule(BR_DRIVE, BR_STEER, BR_OFFSET, BR_ENCODER, canivore);

        // sets swerve
        kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroHeading(),
            getModulePositions(),
            new Pose2d());

        buildAuton();
        initNt();

        if (DRIVE_DEBUG) {
            enableDriveDebug();
        }
        if (STEER_DEBUG) {
            enableSteerDebug();
        }

        lockTimer = new Timer();

        initAccelValues();
    }

    private final PIDController rotationPIDController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);

    @Override
    public void periodic() {
        // update the poseestimator with curent gyro reading
        estimatedPose = poseEstimator.update(
            getGyroHeading(),
            getModulePositions());

        // If all commanded velocities are 0, the system is idle (drivers / commands are
        // not supplying input).
        boolean isIdle = states[0].speedMetersPerSecond == 0.0
            && states[1].speedMetersPerSecond == 0.0
            && states[2].speedMetersPerSecond == 0.0
            && states[3].speedMetersPerSecond == 0.0;

        // Start lock timer when idle
        if (isIdle) {
            lockTimer.start();
        } else {
            lockTimer.stop();
            lockTimer.reset();
        }

        // Lock the swerve module if the lock timeout has elapsed, or set them to their
        // setpoints if drivers are supplying non-idle input.
        if (lockTimer.hasElapsed(1)) {
            applyLock();
        } else {
            // update the swerve modules based on the current desired states from states[]
            frontLeftModule.setDesiredState(states[0]);
            frontRightModule.setDesiredState(states[1]);
            backLeftModule.setDesiredState(states[2]);
            backRightModule.setDesiredState(states[3]);
        }

        // logging
        // estimatedPoseLogEntry.append(estimatedPose, GRTUtil.getFpgaTime());
        SmartDashboard.putNumber("Steer/Current RPM", frontLeftModule.getSteerVelocityRPM());
        SmartDashboard.putNumber("Steer/Max RPM", currentCruiseVelocityRPM);

        publishStats();
        logStats();

        // Update current limits from NetworkTables (only first module needed since they share NT entries)
        frontLeftModule.updateCurrentLimits();
        frontRightModule.updateCurrentLimits();
        backLeftModule.updateCurrentLimits();
        backRightModule.updateCurrentLimits();
    }

    /**
     * Sets the powers of the drivetrain through PIDs. Relative to the driver
     * heading on the field.
     *
     * @param xPower [-1, 1] The forward power.
     * @param yPower [-1, 1] The left power.
     * @param angularPower [-1, 1] The rotational power.
     */
    public void setDrivePowers(double xPower, double yPower, double angularPower) {
        // Boost mode ignores the drive speed-limit multiplier (L2 / R1 slow) and
        // bypasses the software acceleration limiter -- raw commands straight to
        // the modules, capped only by the physical motor maxes.
        double speedLimit = boostModeEnabled ? 1.0 : driveSpeedLimit;
        double limitedMaxVel = MAX_VEL * speedLimit;
        double limitedMaxOmega = MAX_OMEGA * speedLimit;

        // Robot-relative (held button): interpret stick forward as the robot's
        // own +X, which is the physical front (see FL_POS / FR_POS in Constants).
        // Field-relative (default): rotate stick axes into the robot frame using
        // the driver heading.
        ChassisSpeeds desiredSpeeds;
        if (robotRelativeEnabled) {
            desiredSpeeds = new ChassisSpeeds(
                xPower * limitedMaxVel,
                yPower * limitedMaxVel,
                angularPower * limitedMaxOmega);
        } else {
            Rotation2d heading = getDriverHeading();
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xPower * limitedMaxVel,
                yPower * limitedMaxVel,
                angularPower * limitedMaxOmega,
                heading);
        }

        ChassisSpeeds speeds;
        if (boostModeEnabled) {
            // Skip accel limiting entirely, but keep the limiter's state in sync
            // so releasing boost doesn't cause a jerk from a stale previousSpeeds.
            speeds = desiredSpeeds;
            previousSpeeds = desiredSpeeds;
            lastUpdateTime = Timer.getFPGATimestamp();
        } else {
            speeds = limitAcceleration(desiredSpeeds);
        }

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            limitedMaxVel, limitedMaxVel, limitedMaxOmega);
    }

    private void initAccelValues() {
        SmartDashboard.setDefaultNumber("SwerveAccel/maxLinearAccel", MAX_LINEAR_ACCELERATION);
        SmartDashboard.setDefaultNumber("SwerveAccel/maxLinearDecel", MAX_LINEAR_DECELERATION);
        SmartDashboard.setDefaultNumber("SwerveAccel/maxAngularAccel", MAX_ANGULAR_ACCELERATION);
        SmartDashboard.setDefaultNumber("SwerveAccel/maxAngularDecel", MAX_ANGULAR_DECELERATION);

        SmartDashboard.setDefaultBoolean("imu/brownOut", false);
    }

    private void updateAccelValues() {
        // Boost mode skips limitAcceleration entirely, so there's no boost branch
        // here -- these values are only consulted on the non-boost path.
        maxLinearAcceleration = SmartDashboard.getNumber("SwerveAccel/maxLinearAccel", MAX_LINEAR_ACCELERATION);
        maxLinearDeceleration = SmartDashboard.getNumber("SwerveAccel/maxLinearDecel", MAX_LINEAR_DECELERATION);
        maxAngularAcceleration = SmartDashboard.getNumber("SwerveAccel/maxAngularAccel", MAX_ANGULAR_ACCELERATION);
        maxAngularDeceleration = SmartDashboard.getNumber("SwerveAccel/maxAngularDecel", MAX_ANGULAR_DECELERATION);
    }

    /**
     * Enables or disables boost mode.
     * When enabled, uses higher max velocity and acceleration values.
     * 
     * @param enabled true to enable boost mode, false to disable
     */
    public void setBoostMode(boolean enabled) {
        this.boostModeEnabled = enabled;
    }

    /**
     * Enables or disables robot-relative drive. When enabled, stick inputs are
     * interpreted in the robot's own frame (forward stick = robot front). When
     * disabled, stick inputs are field-relative as usual.
     *
     * @param enabled true for robot-relative, false for field-relative
     */
    public void setRobotRelative(boolean enabled) {
        this.robotRelativeEnabled = enabled;
    }

    /**
     * Gets whether boost mode is currently enabled.
     *
     * @return true if boost mode is enabled
     */
    public boolean isBoostModeEnabled() {
        return boostModeEnabled;
    }

    /**
     * Limits acceleration but allows full deceleration
     * 
     * @param desiredSpeeds The desired chassis speeds
     * @return Limited chassis speeds
     */
    private ChassisSpeeds limitAcceleration(ChassisSpeeds desiredSpeeds) {
        updateAccelValues();
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;

        // If this is the first update or dt is too large, don't limit
        if (lastUpdateTime == 0.0 || dt > 0.1 || dt <= 0.0) {
            lastUpdateTime = currentTime;
            previousSpeeds = desiredSpeeds;
            return desiredSpeeds;
        }

        // Calculate current linear velocity magnitude
        double currentLinearVel = Math.hypot(previousSpeeds.vxMetersPerSecond, previousSpeeds.vyMetersPerSecond);
        double desiredLinearVel = Math.hypot(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond);

        // Limit both acceleration and deceleration
        double vx = desiredSpeeds.vxMetersPerSecond;
        double vy = desiredSpeeds.vyMetersPerSecond;
        double deltaV = desiredLinearVel - currentLinearVel;

        if (deltaV > 0) {
            // We're accelerating - apply acceleration limits
            double maxDeltaV = maxLinearAcceleration * dt;
            if (deltaV > maxDeltaV) {
                double limitedLinearVel = currentLinearVel + maxDeltaV;
                double scale = limitedLinearVel / desiredLinearVel;
                vx = desiredSpeeds.vxMetersPerSecond * scale;
                vy = desiredSpeeds.vyMetersPerSecond * scale;
            }
        } else if (deltaV < 0) {
            // We're decelerating - apply deceleration limits
            double maxDeltaV = maxLinearDeceleration * dt;
            if (-deltaV > maxDeltaV) {
                double limitedLinearVel = currentLinearVel - maxDeltaV;
                if (desiredLinearVel > 0.001) {
                    double scale = limitedLinearVel / desiredLinearVel;
                    vx = desiredSpeeds.vxMetersPerSecond * scale;
                    vy = desiredSpeeds.vyMetersPerSecond * scale;
                } else {
                    // Decelerating toward zero - use direction from previous speeds
                    double prevMag = Math.hypot(previousSpeeds.vxMetersPerSecond, previousSpeeds.vyMetersPerSecond);
                    if (prevMag > 0.001) {
                        vx = (previousSpeeds.vxMetersPerSecond / prevMag) * limitedLinearVel;
                        vy = (previousSpeeds.vyMetersPerSecond / prevMag) * limitedLinearVel;
                    }
                }
            }
        }

        // Limit angular acceleration and deceleration separately
        double omega = desiredSpeeds.omegaRadiansPerSecond;
        double deltaOmega = omega - previousSpeeds.omegaRadiansPerSecond;

        // Determine if we're accelerating or decelerating (speeding up or slowing down
        // rotation)
        boolean isAngularAccelerating = Math.abs(omega) > Math.abs(previousSpeeds.omegaRadiansPerSecond);
        double maxDeltaOmega = (isAngularAccelerating ? maxAngularAcceleration : maxAngularDeceleration) * dt;

        if (Math.abs(deltaOmega) > maxDeltaOmega) {
            omega = previousSpeeds.omegaRadiansPerSecond + Math.signum(deltaOmega) * maxDeltaOmega;
        }

        ChassisSpeeds limitedSpeeds = new ChassisSpeeds(vx, vy, omega);

        lastUpdateTime = currentTime;
        previousSpeeds = limitedSpeeds;

        return limitedSpeeds;
    }

    /**
     * Executes swerve X locking, putting swerve's wheels into an X configuration to
     * prevent motion.
     */
    public void applyLock() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        frontRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        backLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        backRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
    }

    public void addVisionMeasurements(TimestampedVisionUpdate update) {
        poseEstimator.addVisionMeasurement(
            update.pose().toPose2d(),
            update.timestamp(),
            update.stdDevs());
    }

    /**
     * Gets the module positions.
     * 
     * @return The array of module positions.
     *         The angles are in radians, and the distances are in meters.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    /**
     * Gets the states of the module
     * 
     * @return The array of module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
        };
    }

    /**
     * Gets the driver heading.
     *
     * @return The angle of the robot relative to the driver heading.
     */
    public Rotation2d getDriverHeading() {
        Rotation2d robotHeading = getGyroHeading();
        return robotHeading.minus(driverHeadingOffset);
    }

    /**
     * Resets the driver heading.
     *
     * @param currentRotation The new driver heading.
     */
    public void resetDriverHeading(Rotation2d currentRotation) {
        driverHeadingOffset = getGyroHeading().minus(currentRotation);
    }

    /**
     * Resets the driver heading to 0.
     */
    public void resetDriverHeading() {
        resetDriverHeading(new Rotation2d());
    }

    /**
     * Resets the driver heading with a 90 degree offset.
     */
    public void resetDriverHeadingOffset90() {
        resetDriverHeading(Rotation2d.fromDegrees(90));
    }

    /** Gets the gyro heading. */
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(pidgey.getYaw().getValueAsDouble()); // Might need to flip depending on the robot setup
    }

    /**
     * Gets the current robot pose.
     *
     * @return The robot Pose2d.
     */
    public Pose2d getRobotPosition() {
        return estimatedPose;
    }

    public void resetPose(Pose2d currentPose) {
        poseEstimator.resetPosition(
            getGyroHeading(),
            getModulePositions(),
            currentPose);
    }

    /**
     * Resets the robot pose to the default starting position in front of the red
     * hub.
     * Use this for testing/practice when starting in a known position.
     */
    public void resetToStartingPosition() {
        // Starting position: 1.5 meters in front of red hub (which is at x=11.9)
        Pose2d startingPose = new Pose2d(10.4, 4.0, new Rotation2d());
        resetPose(startingPose);
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setRobotRelativeDrivePowers(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds,
            new Rotation2d(0));

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    /**
     * Sets the power of the drivetrain through PIDs. Relative to the robot with the
     * intake in the front.
     *
     * @param xPower [-1, 1] The forward power.
     * @param yPower [-1, 1] The left power.
     * @param angularPower [-1, 1] The rotational power.
     */
    public void setRobotRelativeDrivePowers(double xPower, double yPower, double angularPower) {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xPower * MAX_VEL,
            yPower * MAX_VEL,
            angularPower * MAX_OMEGA,
            new Rotation2d(0));

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    // Drive speed limit multiplier (controlled by left trigger)
    private double driveSpeedLimit = 1.0;

    /**
     * Sets the drive speed limit multiplier. This scales max velocity and
     * acceleration.
     * 
     * @param limit [0, 1] fraction of max speed. 1.0 = full speed, 0.0 = stopped.
     */
    public void setDriveSpeedLimit(double limit) {
        this.driveSpeedLimit = Math.max(0.0, Math.min(1.0, limit));
    }

    /**
     * Gets the current drive speed limit.
     * 
     * @return The current speed limit multiplier [0, 1]
     */
    public double getDriveSpeedLimit() {
        return driveSpeedLimit;
    }

    /**
     * Limits all steer motor speeds by scaling the MotionMagic cruise velocity.
     *
     * @param limit [0, 1] fraction of max cruise velocity. 1.0 = full speed, 0.25 =
     *        quarter speed.
     */
    public void setSteerSpeedLimit(double limit) {
        double velocity = STEER_CRUISE_VELOCITY * limit;
        currentCruiseVelocityRPM = velocity * STEER_GEAR_REDUCTION * 60.0;
        frontLeftModule.setSteerCruiseVelocity(velocity);
        frontRightModule.setSteerCruiseVelocity(velocity);
        backLeftModule.setSteerCruiseVelocity(velocity);
        backRightModule.setSteerCruiseVelocity(velocity);
    }

    private void initNt() {
        ntInstance = NetworkTableInstance.getDefault();
        swerveTable = ntInstance.getTable(SWERVE_TABLE);

        swerveStatesPublisher = swerveTable.getStructArrayTopic(
            "SwerveStates", SwerveModuleState.struct).publish();

        desiredStatesPublisher = swerveTable.getStructArrayTopic(
            "DesiredStates", SwerveModuleState.struct).publish();

        estimatedPosePublisher = swerveTable.getStructTopic(
            "estimatedPose",
            Pose2d.struct).publish();
    }

    /**
     * publishes swerve stats to NT
     */
    private void publishStats() {
        estimatedPosePublisher.set(estimatedPose);

        SmartDashboard.putBoolean("imu/brownOut", pidgey.getStickyFault_Undervoltage().getValue());

        if (STATE_DEBUG || DRIVE_DEBUG || STEER_DEBUG) {
            swerveStatesPublisher.set(getModuleStates());
            desiredStatesPublisher.set(states);
        }

        if (DRIVE_DEBUG) {
            frontLeftModule.publishDriveStats();
            frontRightModule.publishDriveStats();
            backLeftModule.publishDriveStats();
            backRightModule.publishDriveStats();
        }

        if (STEER_DEBUG) {
            frontLeftModule.publishSteerStats();
            frontRightModule.publishSteerStats();
            backLeftModule.publishSteerStats();
            backRightModule.publishSteerStats();
        }
    }

    private void logStats() {
        // Subsystem-level
        Logger.recordOutput("swerve/estimatedPose", estimatedPose);
        Logger.recordOutput("swerve/gyroHeading", getGyroHeading().getDegrees());
        Logger.recordOutput("swerve/steerCruiseRPM", currentCruiseVelocityRPM);

        // Chassis speeds
        ChassisSpeeds speeds = getRobotRelativeChassisSpeeds();
        Logger.recordOutput("swerve/chassisVx", speeds.vxMetersPerSecond);
        Logger.recordOutput("swerve/chassisVy", speeds.vyMetersPerSecond);
        Logger.recordOutput("swerve/chassisOmega", speeds.omegaRadiansPerSecond);
        // Per-module logging
        frontLeftModule.logStats();
        frontRightModule.logStats();
        backLeftModule.logStats();
        backRightModule.logStats();
    }

    /**
     * Enables drive debug
     */
    private void enableDriveDebug() {
        frontLeftModule.driveDebug();
        frontRightModule.driveDebug();
        backLeftModule.driveDebug();
        backRightModule.driveDebug();
    }

    /**
     * Enables steer debug
     */
    private void enableSteerDebug() {
        frontLeftModule.steerDebug();
        frontRightModule.steerDebug();
        backLeftModule.steerDebug();
        backRightModule.steerDebug();
    }

    /**
     * Builds the auton builder
     */
    private void buildAuton() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            // Handle exception as needed, maybe use default values or fallback
        }

        AutoBuilder.configure(
            this::getRobotPosition,
            this::resetPose,
            this::getRobotRelativeChassisSpeeds,
            (speeds, feedforwards) -> setRobotRelativeDrivePowers(speeds),

            new PPHolonomicDriveController(
                new PIDConstants(AUTO_TRANSLATION_P, AUTO_TRANSLATION_I, AUTO_TRANSLATION_D),
                new PIDConstants(AUTO_ROTATION_P, AUTO_ROTATION_I, AUTO_ROTATION_D)),

            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this);
    }
}
