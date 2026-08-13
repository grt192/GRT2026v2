package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import frc.robot.subsystems.hopper.HopperConfig;
import frc.robot.subsystems.intake.pivot.IntakePivotConfig;
import frc.robot.subsystems.intake.roller.IntakeRollerConfig;
import frc.robot.subsystems.shooter.flywheel.FlywheelConfig;
import frc.robot.subsystems.shooter.hood.HoodConfig;
import frc.robot.subsystems.shooter.tower.TowerConfig;
import frc.robot.util.subsystems.MechanismConfig;

import org.junit.jupiter.api.Test;

/**
 * Pins each mechanism's device configuration to the values its hand-written {@code *IOTalonFX}
 * applied before the migration.
 *
 * <p>
 * The migration's whole contract is that behavior does not change, and the way that contract
 * most plausibly breaks is a mistyped gain or a dropped ratio while moving settings from
 * {@code Constants} into a config. These expectations were read off the pre-migration IO classes,
 * so a failure here means the port changed the robot rather than just its shape.
 */
class MechanismConfigParityTest {

    private static final double EPSILON = 1e-9;

    @Test
    void intakeRollerMatchesPreMigrationConfig() {
        TalonFXConfiguration fx = IntakeRollerConfig.CONFIG.fx();

        assertEquals(InvertedValue.CounterClockwise_Positive, fx.MotorOutput.Inverted);
        assertEquals(NeutralModeValue.Brake, fx.MotorOutput.NeutralMode);
        assertTrue(fx.CurrentLimits.StatorCurrentLimitEnable);
        assertEquals(120.0, fx.CurrentLimits.StatorCurrentLimit, EPSILON);

        // Direct drive: the roller never had a feedback ratio applied.
        assertEquals(1.0, fx.Feedback.SensorToMechanismRatio, EPSILON);

        assertEquals(0.0930, fx.Slot0.kP, EPSILON);
        assertEquals(0.866, fx.Slot0.kS, EPSILON);
        assertEquals(0.100, fx.Slot0.kV, EPSILON);
        assertEquals(0.0116, fx.Slot0.kA, EPSILON);
    }

    @Test
    void intakePivotMatchesPreMigrationConfig() {
        TalonFXConfiguration fx = IntakePivotConfig.CONFIG.fx();

        assertEquals(NeutralModeValue.Brake, fx.MotorOutput.NeutralMode);
        assertTrue(fx.CurrentLimits.StatorCurrentLimitEnable);
        assertEquals(40.0, fx.CurrentLimits.StatorCurrentLimit, EPSILON);

        assertEquals(FeedbackSensorSourceValue.RemoteCANcoder, fx.Feedback.FeedbackSensorSource);
        assertEquals(13, fx.Feedback.FeedbackRemoteSensorID);
        assertEquals(20.0, fx.Feedback.RotorToSensorRatio, EPSILON);
        assertEquals(1.0, fx.Feedback.SensorToMechanismRatio, EPSILON);

        assertTrue(fx.SoftwareLimitSwitch.ForwardSoftLimitEnable);
        assertEquals(0.3568, fx.SoftwareLimitSwitch.ForwardSoftLimitThreshold, EPSILON);
        assertTrue(fx.SoftwareLimitSwitch.ReverseSoftLimitEnable);
        assertEquals(0.0, fx.SoftwareLimitSwitch.ReverseSoftLimitThreshold, EPSILON);

        assertEquals(60.0, fx.Slot0.kP, EPSILON);
        assertEquals(4.0, fx.Slot0.kD, EPSILON);
        assertEquals(0.2, fx.Slot0.kS, EPSILON);
        assertEquals(3.18, fx.Slot0.kV, EPSILON);
        assertEquals(0.27, fx.Slot0.kA, EPSILON);
        assertEquals(GravityTypeValue.Arm_Cosine, fx.Slot0.GravityType);
    }

    @Test
    void hopperMatchesPreMigrationConfig() {
        TalonFXConfiguration fx = HopperConfig.CONFIG.fx();

        assertEquals(InvertedValue.CounterClockwise_Positive, fx.MotorOutput.Inverted);
        assertEquals(NeutralModeValue.Coast, fx.MotorOutput.NeutralMode);
        assertFalse(fx.CurrentLimits.StatorCurrentLimitEnable, "hopper limit was configured off");
        assertEquals(4.0, fx.Feedback.SensorToMechanismRatio, EPSILON);

        assertEquals(0.0673, fx.Slot0.kP, EPSILON);
        assertEquals(0.464, fx.Slot0.kV, EPSILON);
        assertEquals(0.0127, fx.Slot0.kA, EPSILON);

        assertEquals(100.0, fx.MotionMagic.MotionMagicCruiseVelocity, EPSILON);
        assertEquals(100.0, fx.MotionMagic.MotionMagicAcceleration, EPSILON);
    }

    @Test
    void towerReportsRotorRotationsWhileTurningAReduction() {
        TalonFXConfiguration fx = TowerConfig.CONFIG.fx();

        assertEquals(InvertedValue.Clockwise_Positive, fx.MotorOutput.Inverted);
        assertEquals(NeutralModeValue.Coast, fx.MotorOutput.NeutralMode);
        assertFalse(fx.CurrentLimits.StatorCurrentLimitEnable, "tower limit was configured off");

        // The tower has always reported rotor rotations - it simply never called withFeedback.
        // Stating the 1:1 explicitly must not change that, while the sim plant keeps the real
        // 4:1 gearbox.
        assertEquals(1.0, fx.Feedback.SensorToMechanismRatio, EPSILON);
        assertEquals(4.0, TowerConfig.CONFIG.sim().mechanicalReduction(), EPSILON);

        assertEquals(0.00625, fx.Slot0.kP, EPSILON);
        assertEquals(0.231, fx.Slot0.kS, EPSILON);
        assertEquals(0.388, fx.Slot0.kV, EPSILON);
    }

    @Test
    void flywheelMatchesPreMigrationConfig() {
        MechanismConfig config = FlywheelConfig.CONFIG;
        TalonFXConfiguration fx = config.fx();

        assertEquals(InvertedValue.Clockwise_Positive, fx.MotorOutput.Inverted);
        assertEquals(NeutralModeValue.Coast, fx.MotorOutput.NeutralMode);

        // The flywheel never configures current limits, so it runs on Phoenix's defaults -
        // which are enabled, not absent. Pinning it to a fresh configuration keeps that true
        // if CTRE ever changes the defaults, rather than silently adopting new ones.
        TalonFXConfiguration defaults = new TalonFXConfiguration();
        assertEquals(defaults.CurrentLimits.StatorCurrentLimitEnable,
            fx.CurrentLimits.StatorCurrentLimitEnable);
        assertEquals(defaults.CurrentLimits.StatorCurrentLimit,
            fx.CurrentLimits.StatorCurrentLimit, EPSILON);
        assertEquals(defaults.CurrentLimits.SupplyCurrentLimitEnable,
            fx.CurrentLimits.SupplyCurrentLimitEnable);
        assertEquals(defaults.CurrentLimits.SupplyCurrentLimit,
            fx.CurrentLimits.SupplyCurrentLimit, EPSILON);

        assertEquals(10.0, fx.Slot0.kP, EPSILON);
        assertEquals(0.12, fx.Slot0.kV, EPSILON);

        assertEquals(1, config.followers().length);
        assertEquals(25, config.followers()[0].canId());
        assertTrue(config.followers()[0].opposeLeader());
    }

    @Test
    void hoodMatchesPreMigrationConfig() {
        MechanismConfig config = HoodConfig.CONFIG;
        TalonFXConfiguration fx = config.fx();

        assertEquals(InvertedValue.CounterClockwise_Positive, fx.MotorOutput.Inverted);
        assertEquals(NeutralModeValue.Brake, fx.MotorOutput.NeutralMode);
        assertEquals(50.0, fx.CurrentLimits.StatorCurrentLimit, EPSILON);
        assertEquals(40.0, fx.CurrentLimits.SupplyCurrentLimit, EPSILON);

        // The encoder counts backwards, expressed as a negated ratio pair.
        assertEquals(FeedbackSensorSourceValue.RemoteCANcoder, fx.Feedback.FeedbackSensorSource);
        assertEquals(18, fx.Feedback.FeedbackRemoteSensorID);
        assertEquals(-244.411765, fx.Feedback.RotorToSensorRatio, EPSILON);
        assertEquals(-1.0, fx.Feedback.SensorToMechanismRatio, EPSILON);

        assertEquals(2000.0, fx.Slot0.kP, EPSILON);
        assertEquals(60.0, fx.Slot0.kD, EPSILON);
        assertEquals(120.0, fx.Slot0.kS, EPSILON);

        assertEquals(0.1, fx.SoftwareLimitSwitch.ForwardSoftLimitThreshold, EPSILON);
        assertEquals(0.0, fx.SoftwareLimitSwitch.ReverseSoftLimitThreshold, EPSILON);
    }

    /**
     * The naive derivation (orientation follows the configured inversion) reproduces four of the
     * six mechanisms and silently breaks the hood, whose negative rotor-to-sensor ratio flips it
     * a second time. These are the values the hand-written sim IO classes used.
     */
    @Test
    void simRotorOrientationMatchesPreMigrationSimClasses() {
        assertEquals(ChassisReference.Clockwise_Positive,
            HoodConfig.CONFIG.simOrientation(), "hood is inverted twice");
        assertEquals(ChassisReference.CounterClockwise_Positive,
            IntakePivotConfig.CONFIG.simOrientation());
        assertEquals(ChassisReference.CounterClockwise_Positive,
            IntakeRollerConfig.CONFIG.simOrientation());
        assertEquals(ChassisReference.CounterClockwise_Positive,
            HopperConfig.CONFIG.simOrientation());
        assertEquals(ChassisReference.Clockwise_Positive, TowerConfig.CONFIG.simOrientation());
        assertEquals(ChassisReference.Clockwise_Positive, FlywheelConfig.CONFIG.simOrientation());
    }

    /** Every mechanism must declare which slot serves which control mode, or nothing tunes. */
    @Test
    void everyMechanismDeclaresExactlyOneGainSlot() {
        for (MechanismConfig config : new MechanismConfig[] {
                IntakeRollerConfig.CONFIG, IntakePivotConfig.CONFIG, HopperConfig.CONFIG,
                TowerConfig.CONFIG, FlywheelConfig.CONFIG, HoodConfig.CONFIG}) {
            assertEquals(1, config.slots().length, config.name() + " gain slots");
            assertEquals(0, config.slots()[0].slot(), config.name() + " slot number");
            assertNotNull(config.sim(), config.name() + " sim plant");
        }
    }

    /** Witherers must copy, or two mechanisms could end up sharing one device configuration. */
    @Test
    void witherersDoNotAliasTheDeviceConfiguration() {
        MechanismConfig base = HoodConfig.CONFIG;
        MechanismConfig derived = base.withFx(fx -> fx.Slot0.withKP(1.0));

        assertEquals(2000.0, base.fx().Slot0.kP, EPSILON, "original was mutated");
        assertEquals(1.0, derived.fx().Slot0.kP, EPSILON);
    }
}
