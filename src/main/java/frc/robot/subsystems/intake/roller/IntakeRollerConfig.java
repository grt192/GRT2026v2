package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.CANType;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismConfig.Sim;
import frc.robot.util.subsystems.MechanismConfig.SlotSpec;
import frc.robot.util.subsystems.MechanismConfig.SysIdSpec;
import frc.robot.util.subsystems.MechanismConfig.Visualizer;

/** Every device setting and mechanism constant for the intake roller. */
public final class IntakeRollerConfig {

    private IntakeRollerConfig() {}

    /** Compile-time constant so log keys have exactly one source of truth. */
    public static final String LOG_KEY = "Intake/Roller";

    private static final int MOTOR_ID = 14;

    /** Direct drive: the roller is on the rotor, so reported units are mechanism units. */
    private static final double MECHANICAL_REDUCTION = 1.0;

    private static final double kA = 0.0116;

    public static final double IN_SPEED_RPS = 85.0;
    public static final double OUT_SPEED_RPS = 85.0;
    public static final double TOLERANCE_RPS = 5.0;

    public static final MechanismConfig CONFIG =
        MechanismConfig.of(LOG_KEY, CANType.MECH, MOTOR_ID)
            .withFx(fx -> {
                fx.MotorOutput
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake);

                fx.CurrentLimits
                    .withStatorCurrentLimit(120.0)
                    .withStatorCurrentLimitEnable(true);

                fx.Feedback.withSensorToMechanismRatio(MECHANICAL_REDUCTION);

                // Velocity control gains, SysId derived against voltage output.
                fx.Slot0.withKP(0.0930).withKI(0.0).withKD(0.0).withKS(0.866).withKV(0.100)
                    .withKA(kA);
            })
            .withSimOverrides(fx -> fx.Slot0.withKP(0.9).withKS(0.0).withKV(0.12).withKA(0.0))
            .withSlots(SlotSpec.velocity())
            .withToleranceRps(TOLERANCE_RPS)
            .withVisualizer(Visualizer.spinner(0.2))
            .withSim(Sim.RotatingMass.fromKa(
                DCMotor.getKrakenX60Foc(1), MECHANICAL_REDUCTION, kA))
            .withSysId(SysIdSpec.ROLLER);
}
