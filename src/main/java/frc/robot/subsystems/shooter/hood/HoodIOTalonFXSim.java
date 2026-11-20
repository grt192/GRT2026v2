package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;

public class HoodIOTalonFXSim extends HoodIOTalonFX {
    private static final double LOOP_PERIOD_SECONDS = 0.02;

    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(ShooterConstants.Hood.SIM_P)
        .withKD(ShooterConstants.Hood.SIM_D);

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    private final TalonFXSimState motorSimState;
    private final CANcoderSimState cancoderSimState;
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        gearbox,
        ShooterConstants.Hood.GEAR_RATIO,
        ShooterConstants.Hood.MOMENT_OF_INERTIA_KG_M2,
        ShooterConstants.Hood.COM_LENGTH_M,
        Units.rotationsToRadians(ShooterConstants.Hood.LOWER_ANGLE_LIMIT_ROT),
        Units.rotationsToRadians(ShooterConstants.Hood.UPPER_ANGLE_LIMIT_ROT),
        false,
        Units.rotationsToRadians(ShooterConstants.Hood.LOWER_ANGLE_LIMIT_ROT));

    public HoodIOTalonFXSim(LoggedCanivore canivore) {
        super(canivore);

        motorSimState = motor.getSimState();
        cancoderSimState = cancoder.getSimState();

        // Match the sign-flip baked into the real motor's feedback config (negative
        // RotorToSensorRatio) so simulated rotor motion produces the expected
        // mechanism-direction position.
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;
        cancoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(motorSimState.getMotorVoltage());
        sim.update(LOOP_PERIOD_SECONDS);

        double mechanismRotations = Units.radiansToRotations(sim.getAngleRads());
        double mechanismVelocityRPS = Units.radiansToRotations(sim.getVelocityRadPerSec());

        cancoderSimState.setRawPosition(mechanismRotations);
        cancoderSimState.setVelocity(mechanismVelocityRPS);

        motorSimState.setRawRotorPosition(mechanismRotations * ShooterConstants.Hood.GEAR_RATIO);
        motorSimState.setRotorVelocity(mechanismVelocityRPS * ShooterConstants.Hood.GEAR_RATIO);

        super.updateInputs(inputs);
    }
}
