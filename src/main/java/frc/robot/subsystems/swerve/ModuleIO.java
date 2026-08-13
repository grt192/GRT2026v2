package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.DriveSubsystem.SwerveModule;
import frc.robot.util.Hardware.EncoderHealth;
import frc.robot.util.Hardware.MotorControlMode;
import frc.robot.util.PIDConstants;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRot = 0.0;
        public double driveVelocityRPS = 0.0;
        public double driveAccelerationRPS2 = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveTorqueCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveTempC = 0.0;
        public boolean driveTempFault = false;
        public boolean driveMotorConnected = false;

        public MotorControlMode driveControlMode = MotorControlMode.Disabled;
        public double driveAppliedDutyCycle = 0.0;
        public double driveClosedLoopSetpoint = 0.0;
        public double driveClosedLoopOutput = 0.0;

        public double steerPositionRot = 0.0;
        public double steerVelocityRPS = 0.0;
        public double steerAccelerationRPS2 = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerSupplyCurrentAmps = 0.0;
        public double steerTorqueCurrentAmps = 0.0;
        public double steerStatorCurrentAmps = 0.0;
        public double steerTempC = 0.0;
        public boolean steerTempFault = false;
        public boolean steerMotorConnected = false;

        public MotorControlMode steerControlMode = MotorControlMode.Disabled;
        public double steerAppliedDutyCycle = 0.0;
        public double steerClosedLoopSetpoint = 0.0;
        public double steerClosedLoopOutput = 0.0;

        public double encoderAbsolutePositionRot = 0.0;
        public EncoderHealth encoderHealth = EncoderHealth.Unknown;
        public boolean encoderConnected = false;

        public double[] odometryDrivePositionsRads = new double[] {};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {}


    public default PIDConstants getDefaultDrivePID() {
        return PIDConstants.ZERO;
    }

    public default PIDConstants getDefaultSteerPID() {
        return PIDConstants.ZERO;
    }

    public default SwerveModule getModule() {
        return SwerveModule.FL;
    }

    public default void setDriveVelocity(double velocityRPS, double feedForwardVolts) {}

    public default void setDriveVelocity(double velocityRPS) {}

    public default void setDriveVoltage(double volts) {}

    public default void setSteerPosition(double positionRot) {}

    public default void setSteerVoltage(double volts) {}

    public default void stopSteer() {}

    public default void stopDrive() {}

    public default void setDrivePID(double kP, double kI, double kD) {}

    public default void setSteerPID(double kP, double kI, double kD, double kS, double kV) {}

}
