package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.CANType;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismConfig.Sim;
import frc.robot.util.subsystems.MechanismConfig.SlotSpec;
import frc.robot.util.subsystems.MechanismConfig.SysIdSpec;
import frc.robot.util.subsystems.MechanismConfig.Visualizer;

/** Every device setting and mechanism constant for the hopper spindexer. */
public final class HopperConfig {

    private HopperConfig() {}

    /** Compile-time constant so log keys have exactly one source of truth. */
    public static final String LOG_KEY = "Hopper";

    private static final int MOTOR_ID = 15;

    /** Rotor rotations per rotation of the spindexer. */
    private static final double GEAR_REDUCTION = 4.0;

    private static final double kA = 0.0127;

    /** Vanes on the spindexer — four of them, so a ball goes by every quarter turn. */
    public static final int VANES = 4;

    private static final double TARGET_BALLS_PER_SEC = 4.0;
    public static final double TARGET_RPS = TARGET_BALLS_PER_SEC / VANES;
    public static final double TOLERANCE_RPS = 6.47;

    /** Which way the spindexer is being asked to turn. */
    public enum HopperIntake {
        BALL_IN,
        BALL_OUT,
        STOP
    }

    public static final MechanismConfig CONFIG =
        MechanismConfig.of(LOG_KEY, CANType.MECH, MOTOR_ID)
            .withFx(fx -> {
                fx.MotorOutput
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast);

                // Configured but deliberately left disabled, as it has been all season.
                fx.CurrentLimits
                    .withStatorCurrentLimit(120.0)
                    .withStatorCurrentLimitEnable(false);

                fx.Feedback.withSensorToMechanismRatio(GEAR_REDUCTION);

                // Velocity control gains, SysId derived against voltage output.
                fx.Slot0.withKP(0.0673).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.464)
                    .withKA(kA);

                // Configured, but nothing issues a Motion Magic request today.
                fx.MotionMagic
                    .withMotionMagicCruiseVelocity(100.0)
                    .withMotionMagicAcceleration(100.0)
                    .withMotionMagicJerk(100.0);
            })
            .withSimOverrides(fx -> fx.Slot0.withKP(0.8).withKS(0.0).withKV(0.0).withKA(0.0))
            .withSlots(SlotSpec.velocity())
            .withToleranceRps(TOLERANCE_RPS)
            .withVisualizer(Visualizer.spinner(0.4, VANES))
            .withSim(Sim.RotatingMass.fromKa(DCMotor.getKrakenX60Foc(1), GEAR_REDUCTION, kA))
            .withSysId(SysIdSpec.ROLLER);
}
