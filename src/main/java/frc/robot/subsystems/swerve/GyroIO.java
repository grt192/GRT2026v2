package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double yawPositionDeg = 0.0;
        public double pitchPositionDeg = 0.0;
        public double rollPositionDeg = 0.0;

        public double yawVelocityDegPerSec = 0.0;
        public double pitchVelocityDegPerSec = 0.0;
        public double rollVelocityDegPerSec = 0.0;

        // Brown Out and Reset Detection
        public double upTimeSec = 0.0;
        public double supplyVoltage = 0.0;

        public double tempC = 0.0;
        public boolean connected = false;

        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void resetHeading() {}
}
