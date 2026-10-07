package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveConstants.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;

public class DriveSubsystem extends SubsystemBase {
    public enum SwerveModule {
        FL("Front Left"),
        FR("Front Right"),
        BL("Back Left"),
        BR("Back Right");

        private String fullName;

        SwerveModule(String fullName) {
            this.fullName = fullName;
        }

        @Override
        public String toString() {
            return fullName + " Swerve Module";
        }

        public String toKey() {
            return fullName.replace(" ", "");
        }
    }

    /** Held while reading odometry queues so the odometry thread can't add samples mid-read. */
    static final Lock ODOMETRY_LOCK = new ReentrantLock();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SingleModule[] modules = new SingleModule[4]; // FL, FR, BL, BR

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);
    private final SwerveDrivePoseEstimator3d poseEstimator;

    // Odometry
    // Full gyro orientation; yaw drives the driver heading, pitch/roll let odometry climb the BUMPs
    private Rotation3d rawGyroRotation = Rotation3d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    private boolean odometryInitialized = false;
    private Consumer<Pose3d> poseResetListener = (pose) -> {
    };
    private Rotation2d driverHeadingOffset = Rotation2d.kZero;

    // Setpoints, applied to the modules in periodic()
    private SwerveModuleState[] desiredStates = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };
    private final Timer lockTimer = new Timer();

    // Acceleration limiting
    private final LoggedTunableNumber maxLinearAcceleration =
        new LoggedTunableNumber("Swerve/MaxLinearAccel_mps2", MAX_LINEAR_ACCELERATION);
    private final LoggedTunableNumber maxLinearDeceleration =
        new LoggedTunableNumber("Swerve/MaxLinearDecel_mps2", MAX_LINEAR_DECELERATION);
    private final LoggedTunableNumber maxAngularAcceleration =
        new LoggedTunableNumber("Swerve/MaxAngularAccel_radps2", MAX_ANGULAR_ACCELERATION);
    private final LoggedTunableNumber maxAngularDeceleration =
        new LoggedTunableNumber("Swerve/MaxAngularDecel_radps2", MAX_ANGULAR_DECELERATION);
    private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
    private double lastUpdateTime = 0.0;

    // Boost mode flag
    @AutoLogOutput(key = "Swerve/BoostMode")
    private boolean boostModeEnabled = false;

    // Robot-relative mode flag (held button -- release returns to field-relative)
    @AutoLogOutput(key = "Swerve/RobotRelative")
    private boolean robotRelativeEnabled = false;

    // Drive speed limit multiplier (controlled by left trigger)
    @AutoLogOutput(key = "Swerve/DriveSpeedLimit")
    private double driveSpeedLimit = 1.0;

    public DriveSubsystem(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new SingleModule(flModuleIO, SwerveModule.FL);
        modules[1] = new SingleModule(frModuleIO, SwerveModule.FR);
        modules[2] = new SingleModule(blModuleIO, SwerveModule.BL);
        modules[3] = new SingleModule(brModuleIO, SwerveModule.BR);

        poseEstimator = new SwerveDrivePoseEstimator3d(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            Pose3d.kZero,
            VecBuilder.fill(ODOMETRY_XY_STD_DEV_M, ODOMETRY_XY_STD_DEV_M, ODOMETRY_Z_STD_DEV_M, ODOMETRY_THETA_STD_DEV_RAD),
            // Unused: there is no vision to feed the estimator
            VecBuilder.fill(0.9, 0.9, 0.9, 0.9));

        // Sim feeds odometry one sample per loop instead (see ModuleIOTalonFXSim)
        if (Constants.CURRENT_MODE == Mode.REAL) {
            PhoenixOdometryThread.getInstance().start();
        }
    }

    @Override
    public void periodic() {
        ODOMETRY_LOCK.lock(); // Prevents odometry updates while reading data
        try {
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("Swerve/Gyro", gyroInputs);
            for (SingleModule module : modules) {
                module.updateInputs();
            }
        } finally {
            ODOMETRY_LOCK.unlock();
        }
        LoggedTracer.record("Swerve/Inputs");

        for (SingleModule module : modules) {
            module.periodic();
        }

        updateOdometry();

        // If all commanded velocities are 0, the system is idle (drivers / commands are
        // not supplying input).
        boolean isIdle = true;
        for (SwerveModuleState state : desiredStates) {
            isIdle &= state.speedMetersPerSecond == 0.0;
        }

        // Start lock timer when idle
        if (isIdle) {
            lockTimer.start();
        } else {
            lockTimer.stop();
            lockTimer.reset();
        }

        // Lock the swerve module if the lock timeout has elapsed, or set them to their
        // setpoints if drivers are supplying non-idle input.
        if (lockTimer.hasElapsed(LOCK_TIMEOUT_SECONDS)) {
            applyLock();
        } else {
            for (int i = 0; i < 4; i++) {
                modules[i].setModuleState(desiredStates[i]);
            }
        }

        Logger.recordOutput("Swerve/SwerveStates/Setpoints", desiredStates);
        Logger.recordOutput("Swerve/Locked", lockTimer.hasElapsed(LOCK_TIMEOUT_SECONDS));
        Logger.recordOutput("Swerve/DriverHeading", getDriverHeading());

        LoggedTracer.record("Swerve/Periodic");
    }

    /**
     * Feeds every odometry sample collected since the last loop into the pose estimator, each at its
     * own timestamp. On hardware that's ~5 samples per loop at 250 Hz; in sim it's one.
     */
    private void updateOdometry() {
        double[] sampleTimestamps = Constants.CURRENT_MODE == Mode.SIM
            ? new double[] {Timer.getTimestamp()}
            : gyroInputs.odometryTimestamps; // All signals are sampled together

        int sampleCount = sampleTimestamps.length;
        for (SingleModule module : modules) {
            sampleCount = Math.min(sampleCount, module.getOdometryPositions().length);
        }
        if (gyroInputs.connected) {
            sampleCount = Math.min(sampleCount, gyroInputs.odometryRotations.length);
        }

        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for (int j = 0; j < 4; j++) {
                modulePositions[j] = modules[j].getOdometryPositions()[i];
            }

            // The drive encoders don't start at zero after a code restart, so line odometry up with
            // the first real sample instead of treating it as one huge wheel movement
            if (!odometryInitialized) {
                lastModulePositions = modulePositions;
            }

            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int j = 0; j < 4; j++) {
                moduleDeltas[j] = new SwerveModulePosition(
                    modulePositions[j].distanceMeters - lastModulePositions[j].distanceMeters,
                    modulePositions[j].angle);
            }

            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryRotations[i];
            } else {
                // Gyro is gone, so integrate the heading from wheel motion instead and assume level
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = new Rotation3d(0.0, 0.0, rawGyroRotation.getZ() + twist.dtheta);
            }

            if (!odometryInitialized) {
                Pose3d currentPose = poseEstimator.getEstimatedPosition();
                poseEstimator.resetPosition(
                    rawGyroRotation, modulePositions, withGyroTilt(currentPose.toPose2d(), currentPose.getZ()));
                odometryInitialized = true;
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
            lastModulePositions = modulePositions;
        }

        Logger.recordOutput("Swerve/OdometrySamples", sampleCount);
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
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xPower * limitedMaxVel,
                yPower * limitedMaxVel,
                angularPower * limitedMaxOmega,
                getDriverHeading());
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

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            limitedMaxVel, limitedMaxVel, limitedMaxOmega);
        desiredStates = states;
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
            double maxDeltaV = maxLinearAcceleration.get() * dt;
            if (deltaV > maxDeltaV) {
                double limitedLinearVel = currentLinearVel + maxDeltaV;
                double scale = limitedLinearVel / desiredLinearVel;
                vx = desiredSpeeds.vxMetersPerSecond * scale;
                vy = desiredSpeeds.vyMetersPerSecond * scale;
            }
        } else if (deltaV < 0) {
            // We're decelerating - apply deceleration limits
            double maxDeltaV = maxLinearDeceleration.get() * dt;
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
        double maxDeltaOmega =
            (isAngularAccelerating ? maxAngularAcceleration.get() : maxAngularDeceleration.get()) * dt;

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
        modules[0].setModuleState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        modules[1].setModuleState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        modules[2].setModuleState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        modules[3].setModuleState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
    }


    /**
     * Gets the module positions.
     *
     * @return The array of module positions.
     *         The angles are in radians, and the distances are in meters.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    /**
     * Gets the states of the module
     *
     * @return The array of module states
     */
    @AutoLogOutput(key = "Swerve/SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
    }

    /**
     * Gets the driver heading.
     *
     * @return The angle of the robot relative to the driver heading.
     */
    public Rotation2d getDriverHeading() {
        return rawGyroRotation.toRotation2d().minus(driverHeadingOffset);
    }

    /**
     * Resets the driver heading.
     *
     * @param currentRotation The new driver heading.
     */
    public void resetDriverHeading(Rotation2d currentRotation) {
        driverHeadingOffset = rawGyroRotation.toRotation2d().minus(currentRotation);
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

    /**
     * Gets the current robot pose on the field (the 3D estimate flattened).
     *
     * @return The robot Pose2d.
     */
    @AutoLogOutput(key = "Swerve/EstimatedPose")
    public Pose2d getRobotPosition() {
        return poseEstimator.getEstimatedPosition().toPose2d();
    }

    /** Gets the full 3D pose estimate, including height and tilt (e.g. on a BUMP). */
    @AutoLogOutput(key = "Swerve/EstimatedPose3d")
    public Pose3d getRobotPose3d() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the pose on the field. Height and tilt aren't given, so they're kept from the current
     * estimate and the gyro.
     */
    public void resetPose(Pose2d currentPose) {
        Pose3d pose3d = withGyroTilt(currentPose, getRobotPose3d().getZ());
        poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose3d);
        poseResetListener.accept(pose3d);
    }

    /**
     * A 2D pose lifted to 3D with the gyro's current pitch and roll. Resetting with zero tilt while the
     * gyro reads some would bake that tilt into the estimator's gyro offset.
     */
    private Pose3d withGyroTilt(Pose2d pose, double z) {
        return new Pose3d(
            pose.getX(),
            pose.getY(),
            z,
            new Rotation3d(rawGyroRotation.getX(), rawGyroRotation.getY(), pose.getRotation().getRadians()));
    }

    /**
     * Sets a callback that runs every time the pose is reset (including PathPlanner's auto start reset).
     *
     * @param listener receives the new pose
     */
    public void setPoseResetListener(Consumer<Pose3d> listener) {
        poseResetListener = listener;
    }

    /**
     * Resets the robot pose to the default starting position in front of the red
     * hub.
     * Use this for testing/practice when starting in a known position.
     */
    public void resetToStartingPosition() {
        resetPose(STARTING_POSE);
    }

    @AutoLogOutput(key = "Swerve/ChassisSpeeds/Measured")
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setRobotRelativeDrivePowers(ChassisSpeeds robotRelativeSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, robotRelativeSpeeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
        desiredStates = states;
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
        setRobotRelativeDrivePowers(new ChassisSpeeds(
            xPower * MAX_VEL,
            yPower * MAX_VEL,
            angularPower * MAX_OMEGA));
    }

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
}
