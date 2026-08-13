package frc.robot.subsystems.shooter.tower;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.CANType;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismConfig.Sim;
import frc.robot.util.subsystems.MechanismConfig.SlotSpec;
import frc.robot.util.subsystems.MechanismConfig.SysIdSpec;
import frc.robot.util.subsystems.MechanismConfig.Visualizer;

/** Every device setting and mechanism constant for the shooter tower. */
public final class TowerConfig {

    private TowerConfig() {}

    /** Compile-time constant so log keys have exactly one source of truth. */
    public static final String LOG_KEY = "Shooter/Tower";

    private static final int MOTOR_ID = 26;

    /**
     * Rotor rotations per rotation of the tower shaft — the gearbox, used by the sim plant.
     *
     * <p>
     * Deliberately <b>not</b> folded into the feedback ratio below: the tower reports rotor
     * rotations, so its speeds and tolerance are in rotor units while the hopper's are in
     * mechanism units. That has always been true; stating the 1:1 explicitly just stops it
     * being an unwritten default.
     */
    private static final double MECHANICAL_REDUCTION = 4.0;

    private static final double REPORTING_RATIO = 1.0;

    private static final double kA = 0.00582;

    /** Rotor rot/s, not mechanism rot/s — see {@link #MECHANICAL_REDUCTION}. */
    public static final double TARGET_VELO_RPS = 30.0;
    public static final double TOLERANCE_RPS = 7.58;

    /** Which way the tower is being asked to move a ball. */
    public enum TowerIntake {
        BALL_UP,
        BALL_DOWN,
        STOP
    }

    public static final MechanismConfig CONFIG =
        MechanismConfig.of(LOG_KEY, CANType.MECH, MOTOR_ID)
            .withFx(fx -> {
                fx.MotorOutput
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast);

                // Configured but deliberately left disabled, as it has been all season.
                fx.CurrentLimits
                    .withStatorCurrentLimit(120.0)
                    .withStatorCurrentLimitEnable(false);

                fx.Feedback.withSensorToMechanismRatio(REPORTING_RATIO);

                // Velocity control gains, SysId derived against voltage output.
                fx.Slot0.withKP(0.00625).withKI(0.0).withKD(0.0).withKS(0.231).withKV(0.388)
                    .withKA(kA);

                // Configured, but nothing issues a Motion Magic request today.
                fx.MotionMagic
                    .withMotionMagicCruiseVelocity(100.0)
                    .withMotionMagicAcceleration(1000.0)
                    .withMotionMagicJerk(100.0);
            })
            .withSimOverrides(fx -> fx.Slot0.withKP(0.00625).withKS(0.0).withKV(0.388).withKA(0.0))
            .withSlots(SlotSpec.velocity())
            .withToleranceRps(TOLERANCE_RPS)
            .withVisualizer(Visualizer.spinner(0.3))
            .withSim(Sim.RotatingMass.fromKa(
                DCMotor.getKrakenX60Foc(1), MECHANICAL_REDUCTION, kA))
            .withSysId(SysIdSpec.ROLLER);
}
