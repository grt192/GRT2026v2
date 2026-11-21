package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.HopperConstants;
import frc.robot.util.GatedAlert;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;
import frc.robot.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO {
    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(HopperConstants.kP)
        .withKI(HopperConstants.kI)
        .withKD(HopperConstants.kD)
        .withKS(HopperConstants.kS)
        .withKV(HopperConstants.kV)
        .withKA(HopperConstants.kA);

    protected final TalonFX motor;
    private final Slot0Configs pidConfig;
    private final MotionMagicConfigs mmConfigs;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);

    private static final String MOTOR_ALERT_PREFIX = "Hopper Motor (ID " + HopperConstants.KRAKEN_CAN_ID + "): ";

    private boolean motorConnected = false;

    private final Alert motorDisconnectedAlert = new Alert(MOTOR_ALERT_PREFIX + "Disconnected", AlertType.kError);

    private final GatedAlert failedToSetFrequencyAlert = new GatedAlert(MOTOR_ALERT_PREFIX + "Failed to set status signal frequency", AlertType.kError, () -> motorConnected);
    private final GatedAlert failedToConfigureMotorAlert = new GatedAlert(MOTOR_ALERT_PREFIX + "Failed to configure motor", AlertType.kError, () -> motorConnected);

    private final GatedAlert didNotOptimizeCANAlert = new GatedAlert(MOTOR_ALERT_PREFIX + "Didn't optimize CAN", AlertType.kWarning, () -> motorConnected);
    private final GatedAlert pidNotSetAlert = new GatedAlert(MOTOR_ALERT_PREFIX + "PID was not saved", AlertType.kWarning, () -> motorConnected);
    private final GatedAlert mmNotSetAlert = new GatedAlert(MOTOR_ALERT_PREFIX + "Motion Magic configs were not saved", AlertType.kWarning, () -> motorConnected);

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

    public HopperIOTalonFX(LoggedCanivore canivore) {
        motor = new TalonFX(HopperConstants.KRAKEN_CAN_ID, canivore);

        TalonFXConfiguration config = new TalonFXConfiguration();
        // Motor output
        config.withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(HopperConstants.HOPPER_INVERTED));

        // Current limits
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(HopperConstants.STATOR_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(HopperConstants.STATOR_CURRENT_LIMIT_AMPS)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(HopperConstants.SUPPLY_CURRENT_LIMIT_AMPS));

        config.withFeedback(
            new FeedbackConfigs().withSensorToMechanismRatio(HopperConstants.GEAR_REDUCTION));

        // Velocity control PID (Slot 0)
        pidConfig = new Slot0Configs()
            .withKP(HopperConstants.kP)
            .withKI(HopperConstants.kI)
            .withKD(HopperConstants.kD)
            .withKS(HopperConstants.kS)
            .withKV(HopperConstants.kV)
            .withKA(HopperConstants.kA);
        config.withSlot0(pidConfig);

        mmConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(HopperConstants.MM_ACCEL_RPS2)
            .withMotionMagicCruiseVelocity(HopperConstants.MM_MAX_VELO_RPS)
            .withMotionMagicJerk(HopperConstants.MM_JERK_RPS3);
        config.withMotionMagic(mmConfigs);

        tryUntilOk(5, () -> motor.getConfigurator().apply(config), failedToConfigureMotorAlert);

        position = motor.getPosition(false);
        velocity = motor.getVelocity(false);
        appliedVoltage = motor.getMotorVoltage(false);
        supplyCurrent = motor.getSupplyCurrent(false);
        statorCurrent = motor.getStatorCurrent(false);
        torqueCurrent = motor.getTorqueCurrent(false);
        temp = motor.getDeviceTemp(false);
        tempFault = motor.getFault_DeviceTemp(false);
        controlMode = motor.getControlMode(false);
        appliedDutyCycle = motor.getDutyCycle(false);
        closedLoopReference = motor.getClosedLoopReference(false);
        closedLoopOutput = motor.getClosedLoopOutput(false);

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

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(100.0, signals), failedToSetFrequencyAlert);
        tryUntilOk(5, () -> motor.optimizeBusUtilization(0, 1.0), didNotOptimizeCANAlert);
        PhoenixUtil.registerSignals(canivore.getCanType(), signals);

        BaseStatusSignal.refreshAll(signals);
        refreshMotorAlerts(BaseStatusSignal.isAllGood(signals));
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
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
        refreshMotorAlerts(inputs.connected);
    }

    @Override
    public void updatePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        pidConfig.withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV).withKA(kA);
        tryUntilOk(5, () -> motor.getConfigurator().apply(pidConfig), pidNotSetAlert);
    }

    @Override
    public void updateMotionMagicConfig(double accelRPS2, double veloRPS, double jerkRPS3) {
        mmConfigs.withMotionMagicAcceleration(accelRPS2)
            .withMotionMagicCruiseVelocity(veloRPS)
            .withMotionMagicJerk(jerkRPS3);
        tryUntilOk(5, () -> motor.getConfigurator().apply(mmConfigs), mmNotSetAlert);
    }

    @Override
    public void setDutyCycleOut(double dutyCycle) {
        motor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setVoltageOut(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setVelocityOut(double velocityOutRPS) {
        motor.setControl(velocityControl.withVelocity(velocityOutRPS));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    private void refreshMotorAlerts(boolean connected) {
        motorConnected = connected;
        motorDisconnectedAlert.set(!connected);
    }
}
