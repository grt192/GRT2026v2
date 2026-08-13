package frc.robot.util.subsystems.talonfx;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;

import frc.robot.util.LoggedCanivore;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismState;

/**
 * A simulated CANcoder, fed from the motor's plant.
 *
 * <p>
 * The encoder has no plant of its own — in the real mechanism it is bolted to the same shaft the
 * motor turns. Rather than duplicate the physics, it reads the motor's output-shaft state each
 * loop and writes it into the device sim state. This is the one piece of wiring that has to
 * happen where both IO objects are visible, which is why {@link TalonFXHardware} builds them
 * together.
 */
class EncoderIOCancoderSim extends EncoderIOCancoder {

    private final CANcoderSimState simState;
    private final Supplier<MechanismState> plant;

    EncoderIOCancoderSim(
        MechanismConfig config, LoggedCanivore bus, Supplier<MechanismState> plant) {
        super(config, bus);

        this.plant = plant;
        this.simState = cancoder.getSimState();
        this.simState.Orientation =
            appliedConfig.MagnetSensor.SensorDirection == SensorDirectionValue.Clockwise_Positive
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void updateInputs(EncoderInputs inputs) {
        MechanismState state = plant.get();
        simState.setRawPosition(state.positionRot());
        simState.setVelocity(state.velocityRps());

        super.updateInputs(inputs);
    }
}
