package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.DriveSubsystem.SwerveModule;
import frc.robot.util.LoggedCanivore;

/**
 * Real TalonFX module IO driven by a maple-sim module. maple-sim calls the motor controller adapters
 * below every physics tick: they hand the simulated wheel/module motion to the CTRE sim states and
 * return the voltage the TalonFX firmware is applying. Adapters based on 254's 2025 code.
 *
 * <p>
 * Uses the robot's real gains: with the physics stepped every 5 ms they track the same as on the
 * robot, so there are no sim-only gains to keep in sync.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
    public ModuleIOTalonFXSim(
        SwerveModule module,
        int driveMotorID,
        int steerMotorID,
        int cancoderID,
        double encoderOffsetRot,
        LoggedCanivore canivore,
        SwerveModuleSimulation moduleSimulation) {
        super(module, driveMotorID, steerMotorID, cancoderID, encoderOffsetRot, canivore);

        TalonFXSimState driveSimState = driveMotor.getSimState();
        TalonFXSimState steerSimState = steerMotor.getSimState();
        // Match the motor inverts so positive output moves the module (and CANcoder) positive
        driveSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        steerSimState.Orientation = ChassisReference.Clockwise_Positive;
        // Steer is a Kraken X44; the sim otherwise models an X60, whose back-EMF fights the X44 plant
        steerSimState.setMotorType(TalonFXSimState.MotorType.KrakenX44);

        moduleSimulation.useDriveMotorController(new TalonFXMotorControllerSim(driveMotor));
        moduleSimulation.useSteerMotorController(
            new TalonFXMotorControllerWithCANcoderSim(steerMotor, steerEncoder, encoderOffsetRot));
    }


    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // The odometry thread doesn't run in sim, so hand odometry the one sample from this loop
        inputs.odometryDrivePositionsRads = new double[] {Units.rotationsToRadians(inputs.drivePositionRot)};
        inputs.odometrySteerPositions = new Rotation2d[] {Rotation2d.fromRotations(inputs.steerPositionRot)};
    }

    /** Feeds a TalonFX's rotor from maple-sim and returns its output voltage. */
    private static class TalonFXMotorControllerSim implements SimulatedMotorController {
        private final TalonFXSimState talonFXSimState;

        TalonFXMotorControllerSim(TalonFX talonFX) {
            talonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    /** Same as above, plus the module's CANcoder (which sits after the gearbox). */
    private static class TalonFXMotorControllerWithCANcoderSim extends TalonFXMotorControllerSim {
        private final CANcoderSimState cancoderSimState;
        private final double encoderOffsetRot;

        TalonFXMotorControllerWithCANcoderSim(TalonFX talonFX, CANcoder cancoder, double encoderOffsetRot) {
            super(talonFX);
            cancoderSimState = cancoder.getSimState();
            cancoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;
            this.encoderOffsetRot = encoderOffsetRot;
        }

        @Override
        public Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity) {
            cancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage().in(Volts));
            // The IO subtracts the offset to get the module angle, so the simulated CANcoder reads
            // module angle + offset (a module facing forward reads the offset, like the real one)
            cancoderSimState.setRawPosition(mechanismAngle.plus(Rotations.of(encoderOffsetRot)));
            cancoderSimState.setVelocity(mechanismVelocity);
            return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }
}
