package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveConstants.DRIVE_D;
import static frc.robot.Constants.SwerveConstants.DRIVE_I;
import static frc.robot.Constants.SwerveConstants.DRIVE_P;
import static frc.robot.Constants.SwerveConstants.DRIVE_S;
import static frc.robot.Constants.SwerveConstants.DRIVE_V;
import static frc.robot.Constants.SwerveConstants.STEER_D;
import static frc.robot.Constants.SwerveConstants.STEER_I;
import static frc.robot.Constants.SwerveConstants.STEER_P;
import static frc.robot.Constants.SwerveConstants.STEER_S;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.EnumSet;

public class KrakenSwerveModule {

    private final DriveMotor driveMotor;
    private final SteerMotor steerMotor;

    private int drivePort;
    private int steerPort;

    private int driveIndex;
    private int steerIndex;

    private double offsetRads = 0;

    /**
     * Constructs a Swerve Module.
     *
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     * @param offsetRads The offset of the absolute encoder (0 for our cases)
     * @param canCoderPort The CAN ID of the steer motor encoder
     */
    public KrakenSwerveModule(int drivePort, int steerPort, double offsetRads, int canCoderPort, CANBus canivore) {

        this.drivePort = drivePort;
        this.steerPort = steerPort;

        // very efficient code VVV :)
        driveIndex = drivePort / 2; // this means that drive must be even IE: 0, 2, 4, 8
        steerIndex = (steerPort - 1) / 2; // this means that steer must be odd IE: 1,3,5,7

        // steerMotor = new SteerMotor(steerPort, canCoderPort);
        // steerMotor.configPID(
        // STEER_P[steerIndex],
        // STEER_I[steerIndex],
        // STEER_D[steerIndex],
        // STEER_FF[steerIndex]
        // );
        steerMotor = new SteerMotor(steerPort, canCoderPort, canivore);
        steerMotor.configPID(
            STEER_P[steerIndex],
            STEER_I[steerIndex],
            STEER_D[steerIndex],
            STEER_S[steerIndex]);

        driveMotor = new DriveMotor(drivePort, canivore);
        driveMotor.configPID(
            DRIVE_P[driveIndex],
            DRIVE_I[driveIndex],
            DRIVE_D[driveIndex],
            DRIVE_S[driveIndex],
            DRIVE_V[driveIndex]);

        this.offsetRads = offsetRads;
    }

    /**
     * Sets the un optimized desired state of this swerve module through setting the
     * PID targets.
     *
     * @param state The desired SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentAngle = getWrappedAngle();
        state.optimize(currentAngle);

        double targetAngleRads = state.angle.getRadians() - offsetRads;
        double angleErrorRads = state.angle.minus(currentAngle).getRadians();
        double targetVelocity = state.speedMetersPerSecond * Math.cos(angleErrorRads);

        driveMotor.setVelocity(targetVelocity);
        steerMotor.setPosition(targetAngleRads);

    }

    /**
     * Sets the optimized desired state of this swerve module through setting the
     * PID targets.
     *
     * @param state The desired SwerveModuleState
     */
    // public void setOptomizedDesiredState(SwerveModuleState state) {
    // driveMotor.setVelocity(state.speedMetersPerSecond);
    // steerMotor.setPosition(state.angle.getRadians());
    // }

    /**
     * Gets the current state of the swerve module.
     *
     * @return The current SwerveModulePosition of this module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getDistance(),
            getWrappedAngle());
    }

    /**
     * Gets the state of the swerve module (drive velo in m/s + angle 0-1 )
     * 
     * @return state of the module (velo is m/s and angle is double from 0 to 1)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getVelocity(),
            getWrappedAngle());
    }

    /**
     * Gets the current angle of the module.
     *
     * @return Wrapped angle in radians from -pi to pi
     */
    public Rotation2d getWrappedAngle() {
        // returned a 0-1 value
        double angleDouble = steerMotor.getPosition(); // 0..1

        double angleRads = (2.0 * Math.PI * angleDouble) - Math.PI; // [-π, π)
        // double wrappedAngleRads = MathUtil.angleModulus(angleRads + offsetRads);
        angleRads = MathUtil.angleModulus(angleRads + offsetRads);

        return new Rotation2d(angleRads);
    }

    /**
     * Gets the distance the distance driven by the drive motor.
     *
     * @return The distance driven in meters.
     */
    public double getDistanceDriven() {
        return driveMotor.getDistance();
    }

    /**
     * Gets the velocity of the drive motor.
     *
     * @return The velocity of the drive motor in meters/second.
     */
    public double getDriveVelocity() {
        return driveMotor.getVelocity();
    }

    public void steerDebug() {
        NetworkTableInstance.getDefault().getTable("steerDebug")
            .getEntry("AllSwerve" + "PIDF")
            .setDoubleArray(
                new double[] {
                        STEER_P[steerIndex],
                        STEER_I[steerIndex],
                        STEER_D[steerIndex],
                        STEER_S[steerIndex]
                });
        NetworkTableInstance.getDefault().getTable("steerDebug").addListener(
            "AllSwerve" + "PIDF",
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table, key, event) -> {
                double[] pidf = event.valueData.value.getDoubleArray();
                steerMotor.configPID(pidf[0], pidf[1], pidf[2], pidf[3]);
            });
    }

    public void driveDebug() {
        NetworkTableInstance.getDefault().getTable("driveDebug")
            .getEntry("AllSwerve" + "PIDSV")
            .setDoubleArray(
                new double[] {
                        DRIVE_P[driveIndex],
                        DRIVE_I[driveIndex],
                        DRIVE_D[driveIndex],
                        DRIVE_S[driveIndex],
                        DRIVE_V[driveIndex]
                });
        NetworkTableInstance.getDefault().getTable("driveDebug").addListener(
            "AllSwerve" + "PIDSV",
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table, key, event) -> {
                double[] pidsv = event.valueData.value.getDoubleArray();
                driveMotor.configPID(pidsv[0], pidsv[1], pidsv[2], pidsv[3], pidsv[4]);
            });
    }

    /**
     * Publishes drive motor statistics to NetworkTables
     */
    public void publishDriveStats() {
        driveMotor.publishStats();
    }

    /**
     * Updates drive current limits from NetworkTables if changed.
     * Call this periodically.
     */
    public void updateCurrentLimits() {
        driveMotor.updateCurrentLimitsFromNt();
    }

    /**
     * Sets the steer motor cruise velocity for MotionMagic.
     * 
     * @param velocity cruise velocity in rotations per second
     */
    public void setSteerCruiseVelocity(double velocity) {
        steerMotor.setCruiseVelocity(velocity);
    }

    public double getSteerVelocityRPM() {
        return steerMotor.getVelocityRPM();
    }

    /**
     * Refreshes the cached status signals for both motors. Call once per loop
     * before reading any module state (positions, velocities, logging).
     */
    public void refreshSignals() {
        driveMotor.refreshSignals();
        steerMotor.refreshSignals();
    }

    /**
     * Publishes steer motor statistics to NetworkTables
     */
    public void publishSteerStats() {
        // steerMotor.publishStats();
    }

    /**
     * Logs all motor statistics to data log
     */
    public void logStats() {
        driveMotor.logStats();
        steerMotor.logStats();
    }
}
