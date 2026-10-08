package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveConstants.*;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveSteerConstants;
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

    // Robot-relative speeds asked for by the driver or a command. periodic() runs them through the
    // setpoint generator and sends the result to the modules
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    // Steps desiredSpeeds toward what the modules can physically do in one loop (steering speed, motor
    // torque, wheel grip). It also discretizes the speeds, so nothing upstream should
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(),
        new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        },
        DriveFeedforwards.zeros(4));
    private final Timer lockTimer = new Timer();

    // Heading hold
    private final LoggedTunableNumber headingHoldKP =
        new LoggedTunableNumber("Swerve/HeadingHold/kP", HEADING_HOLD_KP);
    private final LoggedTunableNumber headingHoldKD =
        new LoggedTunableNumber("Swerve/HeadingHold/kD", HEADING_HOLD_KD);
    private final LoggedTunableNumber.Watcher headingHoldGainsWatcher =
        LoggedTunableNumber.watch(headingHoldKP, headingHoldKD);
    private final PIDController headingHoldController = new PIDController(HEADING_HOLD_KP, 0.0, HEADING_HOLD_KD);
    // Gyro yaw to hold, or empty while the driver is rotating or not translating
    private Optional<Rotation2d> headingHoldTarget = Optional.empty();

    // Robot-relative mode flag (held button -- release returns to field-relative)
    @AutoLogOutput(key = "Swerve/RobotRelative")
    private boolean robotRelativeEnabled = false;


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

        RobotConfig robotConfig = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI_KG_M2,
            new ModuleConfig(
                SwerveDriveConstants.DRIVE_WHEEL_RADIUS_METERS,
                MAX_VEL,
                WHEEL_COEFFICIENT_OF_FRICTION,
                DCMotor.getKrakenX60(1), // Not FOC: BetaBot has no Pro license
                SwerveDriveConstants.DRIVE_GEAR_REDUCTION,
                SwerveDriveConstants.DRIVE_STATOR_CURRENT_LIMIT,
                1),
            FL_POS, FR_POS, BL_POS, BR_POS);
        setpointGenerator = new SwerveSetpointGenerator(robotConfig, SwerveSteerConstants.MAX_STEER_VELOCITY_RAD_PER_SEC);

        headingHoldController.enableContinuousInput(-Math.PI, Math.PI);

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

        headingHoldGainsWatcher.ifChanged(() -> headingHoldController.setPID(headingHoldKP.get(), 0.0, headingHoldKD.get()));

        if (DriverStation.isDisabled()) {
            // Commands don't run while disabled, so drop the last request instead of acting on it the
            // moment the robot is enabled
            desiredSpeeds = new ChassisSpeeds();
            headingHoldTarget = Optional.empty();
        }

        // Nothing is asking the drivetrain to move (compared against a small threshold rather than
        // exactly 0, since filtered and computed speeds may never land on exactly 0)
        boolean isIdle =
            Math.hypot(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond) < IDLE_LINEAR_VELOCITY_MPS
                && Math.abs(desiredSpeeds.omegaRadiansPerSecond) < IDLE_ANGULAR_VELOCITY_RAD_PER_SEC;

        // Start lock timer when idle
        if (isIdle) {
            lockTimer.start();
        } else {
            lockTimer.stop();
            lockTimer.reset();
        }

        boolean locked = lockTimer.hasElapsed(LOCK_TIMEOUT_SECONDS);
        if (DriverStation.isDisabled() || locked) {
            // The modules aren't following the generator here, so restart it from where they really are
            previousSetpoint = new SwerveSetpoint(getRobotRelativeChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
        }

        // Lock the swerve module if the lock timeout has elapsed, or set them to their
        // setpoints if drivers are supplying non-idle input.
        if (locked) {
            applyLock();
        } else {
            previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, desiredSpeeds, LoggedRobot.defaultPeriodSecs);
            for (int i = 0; i < 4; i++) {
                modules[i].setModuleState(previousSetpoint.moduleStates()[i]);
            }
        }

        Logger.recordOutput("Swerve/ChassisSpeeds/Desired", desiredSpeeds);
        Logger.recordOutput("Swerve/ChassisSpeeds/Setpoint", previousSetpoint.robotRelativeSpeeds());
        Logger.recordOutput("Swerve/SwerveStates/Setpoints", previousSetpoint.moduleStates());
        Logger.recordOutput("Swerve/Locked", locked);
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
        // Robot-relative (held button): interpret stick forward as the robot's
        // own +X, which is the physical front (see FL_POS / FR_POS in Constants).
        // Field-relative (default): rotate stick axes into the robot frame using
        // the driver heading.
        ChassisSpeeds requestedSpeeds;
        if (robotRelativeEnabled) {
            requestedSpeeds = new ChassisSpeeds(
                xPower * MAX_VEL,
                yPower * MAX_VEL,
                angularPower * MAX_OMEGA);
        } else {
            requestedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xPower * MAX_VEL,
                yPower * MAX_VEL,
                angularPower * MAX_OMEGA,
                getDriverHeading());
        }

        double omega = applyHeadingHold(
            requestedSpeeds.omegaRadiansPerSecond, angularPower != 0.0, xPower != 0.0 || yPower != 0.0, MAX_OMEGA);
        ChassisSpeeds speeds =
            new ChassisSpeeds(requestedSpeeds.vxMetersPerSecond, requestedSpeeds.vyMetersPerSecond, omega);

        // Scale down together if translating and rotating would ask a module for more than the limit
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
        desiredSpeeds = kinematics.toChassisSpeeds(states);
    }

    /**
     * Holds the gyro heading while the driver translates with the rotation stick released, so carpet
     * drag and uneven wheel grip don't slowly turn the robot.
     *
     * @param omegaRadPerSec the rotation already being commanded
     * @param rotateRequested whether the driver is using the rotation stick
     * @param translateRequested whether the driver is using the translation stick
     * @param maxOmegaRadPerSec the most rotation heading hold may add
     * @return the rotation to command, with any heading correction added
     */
    private double applyHeadingHold(
        double omegaRadPerSec, boolean rotateRequested, boolean translateRequested, double maxOmegaRadPerSec) {
        // Off while stopped too: drift only comes from translating, and holding while parked would just
        // fight anyone pushing the robot (and keep it from ever going idle and X-locking)
        if (rotateRequested || !translateRequested) {
            headingHoldTarget = Optional.empty();
            Logger.recordOutput("Swerve/HeadingHold/Active", false);
            return omegaRadPerSec;
        }

        Rotation2d heading = rawGyroRotation.toRotation2d();
        if (headingHoldTarget.isEmpty()) {
            // Wait for the robot to stop coasting from the last turn, or it would snap back to where the
            // stick was released
            if (Math.abs(getRobotRelativeChassisSpeeds().omegaRadiansPerSecond) > HEADING_HOLD_SETTLED_VELOCITY_RAD_PER_SEC) {
                Logger.recordOutput("Swerve/HeadingHold/Active", false);
                return omegaRadPerSec;
            }
            headingHoldTarget = Optional.of(heading);
            headingHoldController.reset();
        }

        double correction = MathUtil.clamp(
            headingHoldController.calculate(heading.getRadians(), headingHoldTarget.get().getRadians()),
            -maxOmegaRadPerSec,
            maxOmegaRadPerSec);
        Logger.recordOutput("Swerve/HeadingHold/Active", true);
        Logger.recordOutput("Swerve/HeadingHold/Target", headingHoldTarget.get());
        Logger.recordOutput("Swerve/HeadingHold/Error_rad", headingHoldController.getError());
        Logger.recordOutput("Swerve/HeadingHold/Correction_radps", correction);
        return omegaRadPerSec + correction;
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

    /** Drives at these robot-relative speeds. The setpoint generator keeps every module under MAX_VEL. */
    public void setRobotRelativeDrivePowers(ChassisSpeeds robotRelativeSpeeds) {
        // Whatever is driving this way controls its own heading
        headingHoldTarget = Optional.empty();
        desiredSpeeds = robotRelativeSpeeds;
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
}
