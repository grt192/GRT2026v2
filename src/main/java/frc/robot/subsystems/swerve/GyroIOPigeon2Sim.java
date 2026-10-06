package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LoggedCanivore;

/** Real Pigeon 2 IO with its sim state fed from maple-sim's simulated gyro every physics tick. */
public class GyroIOPigeon2Sim extends GyroIOPigeon2 {
    public GyroIOPigeon2Sim(LoggedCanivore canivore, GyroSimulation gyroSimulation) {
        super(canivore);

        Pigeon2SimState pigeonSimState = pigeon.getSimState();
        SimulatedArena.getInstance().addCustomSimulation((subTickNum) -> {
            pigeonSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage().in(Volts));
            pigeonSimState.setRawYaw(gyroSimulation.getGyroReading().getDegrees());
            pigeonSimState.setAngularVelocityZ(gyroSimulation.getMeasuredAngularVelocity().in(DegreesPerSecond));
        });
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        super.updateInputs(inputs);

        // The odometry thread doesn't run in sim, so hand odometry the one sample from this loop
        inputs.odometryYawTimestamps = new double[] {Timer.getTimestamp()};
        inputs.odometryYawPositions = new Rotation2d[] {Rotation2d.fromDegrees(inputs.yawPositionDeg)};
    }
}
