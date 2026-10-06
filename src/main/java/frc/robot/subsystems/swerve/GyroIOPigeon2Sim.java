package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LoggedCanivore;

/**
 * Real Pigeon 2 IO with its sim state fed every physics tick: yaw from maple-sim's simulated gyro,
 * pitch/roll from the robot's tilt on the field (maple-sim itself is 2D).
 */
public class GyroIOPigeon2Sim extends GyroIOPigeon2 {
    public GyroIOPigeon2Sim(LoggedCanivore canivore, SwerveDriveSim driveSim) {
        super(canivore);

        Pigeon2SimState pigeonSimState = pigeon.getSimState();
        SimulatedArena.getInstance().addCustomSimulation((subTickNum) -> {
            pigeonSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage().in(Volts));
            pigeonSimState.setRawYaw(driveSim.getGyro().getGyroReading().getDegrees());
            pigeonSimState.setAngularVelocityZ(driveSim.getGyro().getMeasuredAngularVelocity().in(DegreesPerSecond));

            Rotation3d tilt = driveSim.getPose3d().getRotation();
            pigeonSimState.setPitch(Units.radiansToDegrees(tilt.getY()));
            pigeonSimState.setRoll(Units.radiansToDegrees(tilt.getX()));
        });
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        super.updateInputs(inputs);

        // The odometry thread doesn't run in sim, so hand odometry the one sample from this loop
        inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
        inputs.odometryRotations = new Rotation3d[] {
                toRotation3d(inputs.yawPositionDeg, inputs.pitchPositionDeg, inputs.rollPositionDeg)
        };
    }
}
