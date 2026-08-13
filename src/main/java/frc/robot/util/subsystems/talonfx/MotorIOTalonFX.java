package frc.robot.util.subsystems.talonfx;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.GatedAlert;
import frc.robot.util.Hardware.ClosedLoopOutput;
import frc.robot.util.Hardware.MotorControlMode;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismState;
import frc.robot.util.subsystems.MechanismConfig.SlotSpec;
import frc.robot.util.subsystems.MotorIO;
import frc.robot.util.subsystems.talonfx.TalonFXTranslator.SetpointRequest;

/**
 * A TalonFX behind {@link MotorIO}.
 *
 * <p>
 * One class serves every mechanism: what varies between them lives in
 * {@link MechanismConfig#fx()}, which is applied verbatim. Followers use this same class — they
 * are configured identically to their leader, told once to follow, and then only read.
 */
class MotorIOTalonFX implements MotorIO {

    private static final double SIGNAL_FREQUENCY_HZ = 100.0;
    private static final int CONFIG_ATTEMPTS = 5;

    protected final TalonFX talon;
    protected final MechanismConfig config;

    /** The configuration actually applied — the simulation overlay is already folded in. */
    protected final TalonFXConfiguration appliedConfig;

    private final Map<MotorControlMode, SlotSpec> slotByMode =
        new EnumMap<>(MotorControlMode.class);

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);
    private final SetpointRequest positionControl;
    private final SetpointRequest velocityControl;

    private boolean connected = false;

    private final Alert disconnectedAlert;
    private final GatedAlert failedToConfigureAlert;
    private final GatedAlert failedToSetSignalFrequencyAlert;
    private final GatedAlert didNotOptimizeCanAlert;
    private final GatedAlert gainsNotSetAlert;
    private final GatedAlert motionMagicNotSetAlert;
    private final GatedAlert failedToSetFollowerAlert;

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

    MotorIOTalonFX(MechanismConfig config, int canId, String role, LoggedCanivore bus) {
        this.config = config;
        this.talon = new TalonFX(canId, bus);

        String prefix = config.displayName() + " " + role + " (ID " + canId + "): ";
        disconnectedAlert = new Alert(prefix + "Disconnected", AlertType.kError);
        failedToConfigureAlert =
            new GatedAlert(prefix + "Failed to configure motor", AlertType.kError, this::isConnected);
        failedToSetSignalFrequencyAlert = new GatedAlert(
            prefix + "Failed to set status signal frequency", AlertType.kError, this::isConnected);
        didNotOptimizeCanAlert =
            new GatedAlert(prefix + "Didn't optimize CAN", AlertType.kWarning, this::isConnected);
        gainsNotSetAlert =
            new GatedAlert(prefix + "Gains were not saved", AlertType.kWarning, this::isConnected);
        motionMagicNotSetAlert = new GatedAlert(
            prefix + "Motion Magic configs were not saved", AlertType.kWarning, this::isConnected);
        failedToSetFollowerAlert = new GatedAlert(
            prefix + "Failed to set follower control", AlertType.kError, this::isConnected);

        appliedConfig = config.fx().clone();
        if (Constants.CURRENT_MODE == Mode.SIM) {
            config.simOverrides().accept(appliedConfig);
        }
        tryUntilOk(CONFIG_ATTEMPTS,
            () -> talon.getConfigurator().apply(appliedConfig), failedToConfigureAlert);

        for (SlotSpec spec : config.slots()) {
            if (spec.slot() < 0 || spec.slot() > 2) {
                throw new IllegalArgumentException(
                    config.name() + ": gain slot must be 0-2, got " + spec.slot());
            }
            if (slotByMode.put(spec.mode(), spec) != null) {
                throw new IllegalArgumentException(
                    config.name() + ": more than one gain slot declared for " + spec.mode());
            }
        }
        positionControl = TalonFXTranslator.positionRequest(outputFor(MotorControlMode.Position));
        velocityControl = TalonFXTranslator.velocityRequest(outputFor(MotorControlMode.Velocity));

        position = talon.getPosition(false);
        velocity = talon.getVelocity(false);
        appliedVoltage = talon.getMotorVoltage(false);
        supplyCurrent = talon.getSupplyCurrent(false);
        statorCurrent = talon.getStatorCurrent(false);
        torqueCurrent = talon.getTorqueCurrent(false);
        temp = talon.getDeviceTemp(false);
        tempFault = talon.getFault_DeviceTemp(false);
        controlMode = talon.getControlMode(false);
        appliedDutyCycle = talon.getDutyCycle(false);
        closedLoopReference = talon.getClosedLoopReference(false);
        closedLoopOutput = talon.getClosedLoopOutput(false);
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

        tryUntilOk(CONFIG_ATTEMPTS,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_FREQUENCY_HZ, signals),
            failedToSetSignalFrequencyAlert);
        tryUntilOk(CONFIG_ATTEMPTS,
            () -> talon.optimizeBusUtilization(0, 1.0), didNotOptimizeCanAlert);
        PhoenixUtil.registerSignals(bus.getCanType(), signals);

        BaseStatusSignal.refreshAll(signals);
        refreshAlerts(BaseStatusSignal.isAllGood(signals));
    }

    /** Mirrors a leader. Issued once; a follower is never commanded again. */
    void follow(int leaderCanId, boolean opposeLeader) {
        MotorAlignmentValue alignment =
            opposeLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned;
        tryUntilOk(CONFIG_ATTEMPTS,
            () -> talon.setControl(new Follower(leaderCanId, alignment)),
            failedToSetFollowerAlert);
    }

    private boolean isConnected() {
        return connected;
    }

    /**
     * Output-shaft plant state. Meaningful only in simulation, where a simulated absolute encoder
     * reads it instead of modelling the same shaft twice.
     */
    MechanismState mechanismState() {
        return MechanismState.ZERO;
    }

    private ClosedLoopOutput outputFor(MotorControlMode mode) {
        SlotSpec spec = slotByMode.get(mode);
        return spec == null ? ClosedLoopOutput.Voltage : spec.output();
    }

    private int slotFor(MotorControlMode mode) {
        SlotSpec spec = slotByMode.get(mode);
        return spec == null ? 0 : spec.slot();
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        inputs.positionRot = position.getValueAsDouble();
        inputs.velocityRps = velocity.getValueAsDouble();
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.appliedDutyCycle = appliedDutyCycle.getValue();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempC = temp.getValueAsDouble();
        inputs.tempFault = tempFault.getValue();
        inputs.connected = BaseStatusSignal.isAllGood(signals);

        inputs.controlMode = TalonFXTranslator.toMotorControlMode(controlMode.getValue());
        inputs.closedLoopSetpoint = closedLoopReference.getValue();
        inputs.closedLoopOutput = closedLoopOutput.getValue();

        refreshAlerts(inputs.connected);
    }

    @Override
    public PIDConstants defaultGains(MotorControlMode mode) {
        return TalonFXTranslator.toPidConstants(
            TalonFXTranslator.slotConfigs(appliedConfig, slotFor(mode)));
    }

    @Override
    public void setGains(MotorControlMode mode, PIDConstants gains) {
        SlotConfigs slot = TalonFXTranslator.withGains(
            TalonFXTranslator.slotConfigs(appliedConfig, slotFor(mode)), gains);
        tryUntilOk(CONFIG_ATTEMPTS,
            () -> talon.getConfigurator().apply(slot), gainsNotSetAlert);
    }

    @Override
    public void setMotionMagic(double cruiseRps, double accelRps2, double jerkRps3) {
        appliedConfig.MotionMagic
            .withMotionMagicCruiseVelocity(cruiseRps)
            .withMotionMagicAcceleration(accelRps2)
            .withMotionMagicJerk(jerkRps3);
        tryUntilOk(CONFIG_ATTEMPTS,
            () -> talon.getConfigurator().apply(appliedConfig.MotionMagic),
            motionMagicNotSetAlert);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        talon.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setVoltage(double volts) {
        talon.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setPositionRot(double positionRot) {
        talon.setControl(positionControl.with(positionRot, slotFor(MotorControlMode.Position)));
    }

    @Override
    public void setVelocityRps(double velocityRps) {
        talon.setControl(velocityControl.with(velocityRps, slotFor(MotorControlMode.Velocity)));
    }

    @Override
    public void stop() {
        talon.stopMotor();
    }

    @Override
    public void setCurrentPositionRot(double positionRot) {
        tryUntilOk(CONFIG_ATTEMPTS, () -> talon.setPosition(positionRot), failedToConfigureAlert);
    }

    @Override
    public void setSoftLimitsEnabled(boolean forward, boolean reverse) {
        appliedConfig.SoftwareLimitSwitch
            .withForwardSoftLimitEnable(forward)
            .withReverseSoftLimitEnable(reverse);
        tryUntilOk(CONFIG_ATTEMPTS,
            () -> talon.getConfigurator().apply(appliedConfig.SoftwareLimitSwitch),
            failedToConfigureAlert);
    }

    private void refreshAlerts(boolean isConnected) {
        connected = isConnected;
        disconnectedAlert.set(!isConnected);
    }
}
