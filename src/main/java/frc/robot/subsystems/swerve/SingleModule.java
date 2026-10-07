package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.DriveSubsystem.SwerveModule;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;

public class SingleModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final String logKey;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    private final LoggedTunableNumber driveKP;
    private final LoggedTunableNumber driveKI;
    private final LoggedTunableNumber driveKD;
    private final LoggedTunableNumber driveKS;
    private final LoggedTunableNumber driveKV;

    private final LoggedTunableNumber steerKP;
    private final LoggedTunableNumber steerKI;
    private final LoggedTunableNumber steerKD;
    private final LoggedTunableNumber steerKS;
    private final LoggedTunableNumber steerKV;

    private final LoggedTunableNumber.Watcher drivePIDWatcher;
    private final LoggedTunableNumber.Watcher driveFeedforwardWatcher;
    private final LoggedTunableNumber.Watcher steerPIDWatcher;

    private SimpleMotorFeedforward driveFeedforwardModel;

    private double commandedDriveFeedforwardVolts = 0.0;
    private double commandedDriveVelocityRPS = 0.0;

    private double commandedSteerPositionRot = 0.0;

    public SingleModule(ModuleIO io, SwerveModule module) {
        this.io = io;
        PIDConstants drivePID = io.getDefaultDrivePID();
        PIDConstants steerPID = io.getDefaultSteerPID();

        logKey = "Swerve/" + module.toKey();
        driveKP = new LoggedTunableNumber(logKey + "/Drive/kP", drivePID.kP());
        driveKI = new LoggedTunableNumber(logKey + "/Drive/kI", drivePID.kI());
        driveKD = new LoggedTunableNumber(logKey + "/Drive/kD", drivePID.kD());
        driveKS = new LoggedTunableNumber(logKey + "/Drive/kS", drivePID.kS());
        driveKV = new LoggedTunableNumber(logKey + "/Drive/kV", drivePID.kV());

        steerKP = new LoggedTunableNumber(logKey + "/Steer/kP", steerPID.kP());
        steerKI = new LoggedTunableNumber(logKey + "/Steer/kI", steerPID.kI());
        steerKD = new LoggedTunableNumber(logKey + "/Steer/kD", steerPID.kD());
        steerKS = new LoggedTunableNumber(logKey + "/Steer/kS", steerPID.kS());
        steerKV = new LoggedTunableNumber(logKey + "/Steer/kV", steerPID.kV());

        drivePIDWatcher = LoggedTunableNumber.watch(driveKP, driveKI, driveKD);
        driveFeedforwardWatcher = LoggedTunableNumber.watch(driveKS, driveKV);
        steerPIDWatcher = LoggedTunableNumber.watch(steerKP, steerKI, steerKD, steerKS, steerKV);

        driveFeedforwardModel = new SimpleMotorFeedforward(driveKS.get(), driveKV.get());
        io.setDrivePID(driveKP.get(), driveKI.get(), driveKD.get());
        io.setSteerPID(steerKP.get(), steerKI.get(), steerKD.get(), steerKS.get(), steerKV.get());
    }

    /** Reads and logs inputs. Called by the drivetrain while it holds the odometry lock. */
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
    }

    public void periodic() {
        drivePIDWatcher.ifChanged(
            () -> io.setDrivePID(driveKP.get(), driveKI.get(), driveKD.get()));

        driveFeedforwardWatcher.ifChanged(
            () -> driveFeedforwardModel = new SimpleMotorFeedforward(driveKS.get(), driveKV.get()));

        steerPIDWatcher.ifChanged(
            () -> io.setSteerPID(steerKP.get(), steerKI.get(), steerKD.get(), steerKS.get(), steerKV.get()));

        // Build this loop's odometry samples (every signal is sampled together by the odometry thread)
        int sampleCount = Math.min(inputs.odometryDrivePositionsRads.length, inputs.odometrySteerPositions.length);
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRads[i] * SwerveDriveConstants.DRIVE_WHEEL_RADIUS_METERS;
            odometryPositions[i] = new SwerveModulePosition(positionMeters, inputs.odometrySteerPositions[i]);
        }

        Logger.recordOutput(logKey + "/CommandedDriveVelocity_rps", commandedDriveVelocityRPS);
        Logger.recordOutput(logKey + "/CommandedDriveFeedforward_volts", commandedDriveFeedforwardVolts);
        Logger.recordOutput(logKey + "/CommandedSteerPosition_rot", commandedSteerPositionRot);

        LoggedTracer.record(logKey);
    }

    /** Optimizes and cosine-scales the state against the current angle, then commands it. */
    public void setModuleState(SwerveModuleState desiredState) {
        // Copy so optimizing doesn't mutate the caller's state
        SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        Rotation2d currentAngle = getAngle();
        state.optimize(currentAngle);
        state.cosineScale(currentAngle);

        double idealDriveVelocityRPS = state.speedMetersPerSecond / SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS;

        commandedSteerPositionRot = state.angle.getRotations();
        commandedDriveVelocityRPS = idealDriveVelocityRPS;
        commandedDriveFeedforwardVolts = driveFeedforwardModel.calculate(idealDriveVelocityRPS);

        io.setDriveVelocity(commandedDriveVelocityRPS, commandedDriveFeedforwardVolts);
        io.setSteerPosition(commandedSteerPositionRot);
    }

    public void stop() {
        commandedDriveVelocityRPS = 0.0;
        commandedDriveFeedforwardVolts = 0.0;
        io.stopDrive();
        io.stopSteer();
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveLinVelocityMPS(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionRot * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS,
            getAngle());
    }

    /** Module positions sampled by the odometry thread since the last loop, oldest first. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Current module angle, 0 = wheel facing robot forward. */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.steerPositionRot);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(inputs.encoderAbsolutePositionRot);
    }

    public double getDriveAngVelocityRPS() {
        return inputs.driveVelocityRPS;
    }

    public double getDriveLinVelocityMPS() {
        return inputs.driveVelocityRPS * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS;
    }
}
