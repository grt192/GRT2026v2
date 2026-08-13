package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.CANType;
import frc.robot.util.Hardware.ClosedLoopOutput;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismConfig.Sim;
import frc.robot.util.subsystems.MechanismConfig.SlotSpec;
import frc.robot.util.subsystems.MechanismConfig.SysIdSpec;
import frc.robot.util.subsystems.MechanismConfig.Visualizer;

/** Every device setting and mechanism constant for the shooter hood. */
public final class HoodConfig {

    private HoodConfig() {}

    /** Compile-time constant so log keys have exactly one source of truth. */
    public static final String LOG_KEY = "Shooter/Hood";

    private static final int MOTOR_ID = 16;
    private static final int ENCODER_ID = 18;

    /** Rotor rotations per rotation of the hood. */
    private static final double GEAR_RATIO = 244.411765;

    public static final double LOWER_LIMIT_ROT = 0.0;
    public static final double UPPER_LIMIT_ROT = 0.1;
    public static final double INIT_ANGLE_ROT = UPPER_LIMIT_ROT;
    public static final double TOLERANCE_ROT = 0.01;

    // Plant model, matching the intake pivot until Hood CAD numbers exist.
    private static final double MOI_KG_M2 = 598.456909 * Constants.LB_IN2_TO_KG_M2;
    private static final double COM_LENGTH_M =
        Units.inchesToMeters(Math.hypot(0.121549, 9.035458));

    public static final MechanismConfig CONFIG =
        MechanismConfig.of(LOG_KEY, CANType.MECH, MOTOR_ID)
            .withFx(fx -> {
                fx.MotorOutput
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake);

                fx.CurrentLimits
                    .withStatorCurrentLimit(50.0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40.0)
                    .withSupplyCurrentLimitEnable(true);

                // The encoder counts opposite the desired mechanism convention, so both the
                // rotor-to-sensor and sensor-to-mechanism ratios are negated. Note that this
                // also flips the simulated rotor orientation, which MechanismConfig derives.
                fx.Feedback
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withFeedbackRemoteSensorID(ENCODER_ID)
                    .withRotorToSensorRatio(-1.0 * GEAR_RATIO)
                    .withSensorToMechanismRatio(-1.0);

                fx.SoftwareLimitSwitch
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(UPPER_LIMIT_ROT)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(LOWER_LIMIT_ROT);

                fx.Slot0.withKP(2000.0).withKI(0.0).withKD(60.0).withKS(120.0);
            })
            .withSimOverrides(fx -> fx.Slot0.withKP(60.0).withKD(2.0).withKS(0.0))
            .withEncoder(ENCODER_ID, cc -> cc.MagnetSensor
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive))
            .withSlots(SlotSpec.position().withOutput(ClosedLoopOutput.TorqueCurrent))
            .withToleranceRot(TOLERANCE_ROT)
            .withVisualizer(Visualizer.arm(0.4)
                .withColor(new Color8Bit(Color.kCornflowerBlue)))
            .withSim(new Sim.Arm(
                DCMotor.getKrakenX60Foc(1),
                GEAR_RATIO,
                MOI_KG_M2,
                COM_LENGTH_M,
                LOWER_LIMIT_ROT,
                UPPER_LIMIT_ROT,
                false,
                INIT_ANGLE_ROT))
            .withSysId(SysIdSpec.SERVO);
}
