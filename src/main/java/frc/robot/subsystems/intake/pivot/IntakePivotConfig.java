package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.CANType;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismConfig.Sim;
import frc.robot.util.subsystems.MechanismConfig.SlotSpec;
import frc.robot.util.subsystems.MechanismConfig.SysIdSpec;
import frc.robot.util.subsystems.MechanismConfig.Visualizer;
import frc.robot.util.subsystems.MechanismConfig.Visualizer.Source;

/** Every device setting and mechanism constant for the intake pivot. */
public final class IntakePivotConfig {

    private IntakePivotConfig() {}

    /** Compile-time constant so log keys have exactly one source of truth. */
    public static final String LOG_KEY = "Intake/Pivot";

    private static final int MOTOR_ID = 12;
    private static final int ENCODER_ID = 13;

    /** Rotor rotations per rotation of the pivot shaft. */
    private static final double GEAR_RATIO = 20.0;

    public static final double REVERSE_LIMIT_ROT = 0.000;
    public static final double FORWARD_LIMIT_ROT = 0.3568;
    public static final double TOLERANCE_ROT = 5.0 / 360.0;

    public static final double OUT_POS_ROT = 0.0;
    public static final double IN_POS_ROT = 0.33;
    public static final double MID_UPPER_ROT = 0.175;
    public static final double MID_LOWER_ROT = 0.091;

    // Onshape: moment of inertia, and the centre-of-mass offset from the pivot.
    private static final double MOI_KG_M2 = 598.456909 * Constants.LB_IN2_TO_KG_M2;
    private static final double COM_LENGTH_M =
        Units.inchesToMeters(Math.hypot(0.121549, 9.035458));

    // Visualization geometry, in metres.
    static final double CANVAS_WIDTH_M = Units.inchesToMeters(26.5);
    static final double CANVAS_HEIGHT_M = Units.inchesToMeters(15);
    static final double ARM_LENGTH_M = Units.inchesToMeters(13.10);
    static final double WALL_HEIGHT_M = Units.inchesToMeters(12);
    static final double ROOT_X_M = Units.inchesToMeters(9);
    static final double ROOT_Y_M = Units.inchesToMeters(1.5);

    public static final MechanismConfig CONFIG =
        MechanismConfig.of(LOG_KEY, CANType.MECH, MOTOR_ID)
            .withFx(fx -> {
                fx.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

                fx.CurrentLimits
                    .withStatorCurrentLimit(40.0)
                    .withStatorCurrentLimitEnable(true);

                // Position comes straight from the CANcoder, so a rotation of the reported
                // value is a rotation of the pivot regardless of the gearbox.
                fx.Feedback
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withFeedbackRemoteSensorID(ENCODER_ID)
                    .withRotorToSensorRatio(GEAR_RATIO)
                    .withSensorToMechanismRatio(1.0);

                fx.SoftwareLimitSwitch
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(FORWARD_LIMIT_ROT)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(REVERSE_LIMIT_ROT);

                fx.Slot0
                    .withKP(60.0).withKI(0.0).withKD(4.0)
                    .withKS(0.2).withKG(0.0).withKV(3.18).withKA(0.27)
                    .withGravityType(GravityTypeValue.Arm_Cosine);
            })
            .withSimOverrides(fx -> fx.Slot0
                .withKP(40.0).withKD(0.7).withKS(0.0).withKG(0.73).withKV(0.0).withKA(0.0))
            .withEncoder(ENCODER_ID, cc -> cc.MagnetSensor
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive))
            .withSlots(SlotSpec.position())
            .withToleranceRot(TOLERANCE_ROT)
            .withVisualizer(Visualizer.arm(ARM_LENGTH_M)
                .withCanvas(CANVAS_WIDTH_M, CANVAS_HEIGHT_M)
                .withRoot(ROOT_X_M, ROOT_Y_M)
                .withColor(new Color8Bit(Color.kBlueViolet))
                .withSource(Source.ENCODER_ABSOLUTE))
            .withSim(new Sim.Arm(
                DCMotor.getKrakenX60Foc(1),
                GEAR_RATIO,
                MOI_KG_M2,
                COM_LENGTH_M,
                REVERSE_LIMIT_ROT,
                FORWARD_LIMIT_ROT,
                true,
                0.0))
            .withSysId(SysIdSpec.SERVO);
}
