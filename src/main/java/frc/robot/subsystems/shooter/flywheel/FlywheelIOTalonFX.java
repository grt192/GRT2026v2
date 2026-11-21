package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.GatedAlert;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {
    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(ShooterConstants.Flywheel.kP)
        .withKI(ShooterConstants.Flywheel.kI)
        .withKD(ShooterConstants.Flywheel.kD)
        .withKS(ShooterConstants.Flywheel.kS)
        .withKV(ShooterConstants.Flywheel.kV)
        .withKA(ShooterConstants.Flywheel.kA);

    protected final TalonFX leader;
    protected final TalonFX follower;

    private final Slot0Configs pidConfig;
    private final MotionMagicConfigs mmConfigs;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);

    private static final String LEADER_ALERT_PREFIX = "Flywheel Leader Motor (ID " + ShooterConstants.Flywheel.UPPER_MOTOR_ID + "): ";
    private static final String FOLLOWER_ALERT_PREFIX = "Flywheel Follower Motor (ID " + ShooterConstants.Flywheel.SECOND_MOTOR_ID + "): ";

    private boolean leaderConnected = false;
    private boolean followerConnected = false;

    private final Alert leaderDisconnectedAlert = new Alert(LEADER_ALERT_PREFIX + "Disconnected", AlertType.kError);
    private final Alert followerDisconnectedAlert = new Alert(FOLLOWER_ALERT_PREFIX + "Disconnected", AlertType.kError);

    private final GatedAlert failedToSetFrequencyAlert = new GatedAlert(LEADER_ALERT_PREFIX + "Failed to set status signal frequency", AlertType.kError, () -> leaderConnected);
    private final GatedAlert failedToConfigureLeaderAlert = new GatedAlert(LEADER_ALERT_PREFIX + "Failed to configure motor", AlertType.kError, () -> leaderConnected);
    private final GatedAlert failedToConfigureFollowerAlert = new GatedAlert(FOLLOWER_ALERT_PREFIX + "Failed to configure motor", AlertType.kError, () -> followerConnected);
    private final GatedAlert failedToSetFollowerAlert = new GatedAlert(FOLLOWER_ALERT_PREFIX + "Failed to set follower control", AlertType.kError, () -> followerConnected);

    private final GatedAlert didNotOptimizeCANAlert = new GatedAlert(LEADER_ALERT_PREFIX + "Didn't optimize CAN", AlertType.kWarning, () -> leaderConnected);
    private final GatedAlert pidNotSetAlert = new GatedAlert(LEADER_ALERT_PREFIX + "PID was not saved", AlertType.kWarning, () -> leaderConnected);
    private final GatedAlert mmNotSetAlert = new GatedAlert(LEADER_ALERT_PREFIX + "Motion Magic configs were not saved", AlertType.kWarning, () -> leaderConnected);

    private final List<BaseStatusSignal> signals;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temp;
    private final StatusSignal<Boolean> tempFault;
    private final StatusSignal<ControlModeValue> controlMode;
    private final StatusSignal<Double> appliedDutyCycle;
    private final StatusSignal<Double> closedLoopReference;
    private final StatusSignal<Double> closedLoopOutput;

    private final StatusSignal<Voltage> followerAppliedVoltage;
    private final StatusSignal<Current> followerSupplyCurrent;
    private final StatusSignal<Current> followerStatorCurrent;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Temperature> followerTemp;
    private final List<BaseStatusSignal> followerSignals;

    public FlywheelIOTalonFX(LoggedCanivore canivore) {
        leader = new TalonFX(ShooterConstants.Flywheel.UPPER_MOTOR_ID, canivore);
        follower = new TalonFX(ShooterConstants.Flywheel.SECOND_MOTOR_ID, canivore);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(ShooterConstants.Flywheel.F_INVERTED_VALUE));

        config.withFeedback(
            new FeedbackConfigs().withSensorToMechanismRatio(ShooterConstants.Flywheel.GEAR_RATIO));

        // Applied to both motors, since the follower gets the same config
        config.withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ShooterConstants.Flywheel.SUPPLY_CURRENT_LIMIT_AMPS)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(ShooterConstants.Flywheel.STATOR_CURRENT_LIMIT_AMPS));

        pidConfig = new Slot0Configs()
            .withKP(ShooterConstants.Flywheel.kP)
            .withKI(ShooterConstants.Flywheel.kI)
            .withKD(ShooterConstants.Flywheel.kD)
            .withKS(ShooterConstants.Flywheel.kS)
            .withKV(ShooterConstants.Flywheel.kV)
            .withKA(ShooterConstants.Flywheel.kA);
        config.withSlot0(pidConfig);

        mmConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(ShooterConstants.Flywheel.MM_ACCEL_RPS2)
            .withMotionMagicCruiseVelocity(ShooterConstants.Flywheel.MM_MAX_VELO_RPS)
            .withMotionMagicJerk(ShooterConstants.Flywheel.MM_JERK_RPS3);
        config.withMotionMagic(mmConfigs);

        tryUntilOk(5, () -> leader.getConfigurator().apply(config), failedToConfigureLeaderAlert);
        tryUntilOk(5, () -> follower.getConfigurator().apply(config), failedToConfigureFollowerAlert);

        // Follower spins opposite to the leader (mirrors the existing FlywheelSubsystem setup).
        tryUntilOk(5,
            () -> follower.setControl(
                new Follower(ShooterConstants.Flywheel.UPPER_MOTOR_ID, MotorAlignmentValue.Opposed)),
            failedToSetFollowerAlert);

        position = leader.getPosition(false);
        velocity = leader.getVelocity(false);
        appliedVoltage = leader.getMotorVoltage(false);
        supplyCurrent = leader.getSupplyCurrent(false);
        statorCurrent = leader.getStatorCurrent(false);
        torqueCurrent = leader.getTorqueCurrent(false);
        temp = leader.getDeviceTemp(false);
        tempFault = leader.getFault_DeviceTemp(false);
        controlMode = leader.getControlMode(false);
        appliedDutyCycle = leader.getDutyCycle(false);
        closedLoopReference = leader.getClosedLoopReference(false);
        closedLoopOutput = leader.getClosedLoopOutput(false);

        followerAppliedVoltage = follower.getMotorVoltage(false);
        followerSupplyCurrent = follower.getSupplyCurrent(false);
        followerStatorCurrent = follower.getStatorCurrent(false);
        followerTorqueCurrent = follower.getTorqueCurrent(false);
        followerTemp = follower.getDeviceTemp(false);

        signals = List.of(
            position,
            velocity,
            appliedVoltage,
            supplyCurrent,
            statorCurrent,
            torqueCurrent,
            temp,
            tempFault,
            controlMode,
            appliedDutyCycle,
            closedLoopReference,
            closedLoopOutput);

        followerSignals = List.of(
            followerAppliedVoltage,
            followerSupplyCurrent,
            followerStatorCurrent,
            followerTorqueCurrent,
            followerTemp);

        List<BaseStatusSignal> allSignals = new ArrayList<>(signals);
        allSignals.addAll(followerSignals);

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(100.0, allSignals), failedToSetFrequencyAlert);
        tryUntilOk(5, () -> leader.optimizeBusUtilization(0, 1.0), didNotOptimizeCANAlert);
        tryUntilOk(5, () -> follower.optimizeBusUtilization(0, 1.0), didNotOptimizeCANAlert);
        PhoenixUtil.registerSignals(canivore.getCanType(), allSignals);

        BaseStatusSignal.refreshAll(allSignals);
        refreshLeaderAlerts(BaseStatusSignal.isAllGood(signals));
        refreshFollowerAlerts(BaseStatusSignal.isAllGood(followerSignals));
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.positionRot = position.getValueAsDouble();
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempC = temp.getValueAsDouble();
        inputs.tempFault = tempFault.getValue();
        inputs.connected = BaseStatusSignal.isAllGood(signals);
        inputs.controlMode = PhoenixUtil.toMotorControlMode(controlMode.getValue());
        inputs.appliedDutyCycle = appliedDutyCycle.getValue();
        inputs.closedLoopSetpoint = closedLoopReference.getValue();
        inputs.closedLoopOutput = closedLoopOutput.getValue();

        inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
        inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
        inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
        inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
        inputs.followerTempC = followerTemp.getValueAsDouble();
        inputs.followerConnected = BaseStatusSignal.isAllGood(followerSignals);

        refreshLeaderAlerts(inputs.connected);
        refreshFollowerAlerts(inputs.followerConnected);
    }

    @Override
    public void updatePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        pidConfig.withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV).withKA(kA);
        tryUntilOk(5, () -> leader.getConfigurator().apply(pidConfig), pidNotSetAlert);
        tryUntilOk(5, () -> follower.getConfigurator().apply(pidConfig), pidNotSetAlert);
    }

    @Override
    public void updateMotionMagicConfig(double accelRPS2, double veloRPS, double jerkRPS3) {
        mmConfigs.withMotionMagicAcceleration(accelRPS2)
            .withMotionMagicCruiseVelocity(veloRPS)
            .withMotionMagicJerk(jerkRPS3);
        tryUntilOk(5, () -> leader.getConfigurator().apply(mmConfigs), mmNotSetAlert);
        tryUntilOk(5, () -> follower.getConfigurator().apply(mmConfigs), mmNotSetAlert);
    }

    @Override
    public void setDutyCycleOut(double dutyCycle) {
        leader.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setVoltageOut(double volts) {
        leader.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setVelocityOut(double velocityRPS) {
        leader.setControl(velocityControl.withVelocity(velocityRPS));
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }

    private void refreshLeaderAlerts(boolean connected) {
        leaderConnected = connected;
        leaderDisconnectedAlert.set(!connected);
    }

    private void refreshFollowerAlerts(boolean connected) {
        followerConnected = connected;
        followerDisconnectedAlert.set(!connected);
    }
}
