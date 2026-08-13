package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.CANType;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismConfig.Follower;
import frc.robot.util.subsystems.MechanismConfig.Sim;
import frc.robot.util.subsystems.MechanismConfig.SlotSpec;
import frc.robot.util.subsystems.MechanismConfig.SysIdSpec;
import frc.robot.util.subsystems.MechanismConfig.Visualizer;

/** Every device setting and mechanism constant for the shooter flywheel. */
public final class FlywheelConfig {

    private FlywheelConfig() {}

    /** Compile-time constant so log keys have exactly one source of truth. */
    public static final String LOG_KEY = "Shooter/Flywheel";

    private static final int LEADER_ID = 17;
    private static final int FOLLOWER_ID = 25;

    /** Direct drive: the wheels are on the rotors. */
    private static final double MECHANICAL_REDUCTION = 1.0;

    /** Flywheel kA is not characterized yet, so the plant inertia is a hand-picked estimate. */
    private static final double MOI_KG_M2 = 0.025;

    public static final double MAX_SPEED_RPS = 120.0;
    public static final double TOLERANCE_RPS = 2.0;

    public static final MechanismConfig CONFIG =
        MechanismConfig.of(LOG_KEY, CANType.MECH, LEADER_ID)
            .withFx(fx -> {
                fx.MotorOutput
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast);

                // No current limits are configured on the flywheel today.

                fx.Feedback.withSensorToMechanismRatio(MECHANICAL_REDUCTION);

                fx.Slot0.withKP(10.0).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.12)
                    .withKA(0.0);

                // Configured, but nothing issues a Motion Magic request today.
                fx.MotionMagic
                    .withMotionMagicCruiseVelocity(500.0)
                    .withMotionMagicAcceleration(100.0)
                    .withMotionMagicJerk(150.0);
            })
            .withSimOverrides(fx -> fx.Slot0.withKP(0.8).withKV(0.0))
            // The second wheel is mechanically opposed to the first.
            .withFollowers(new Follower(FOLLOWER_ID, true))
            .withSlots(SlotSpec.velocity())
            .withToleranceRps(TOLERANCE_RPS)
            .withVisualizer(Visualizer.spinner(0.4))
            .withSim(new Sim.RotatingMass(
                DCMotor.getKrakenX60Foc(2), MECHANICAL_REDUCTION, MOI_KG_M2))
            .withSysId(SysIdSpec.ROLLER);
}
