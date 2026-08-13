package frc.robot.util.subsystems.talonfx;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.GatedAlert;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.subsystems.EncoderIO;
import frc.robot.util.subsystems.MechanismConfig;

/** A CANcoder behind {@link EncoderIO}. */
class EncoderIOCancoder implements EncoderIO {

    private static final double SIGNAL_FREQUENCY_HZ = 100.0;
    private static final int CONFIG_ATTEMPTS = 5;

    protected final CANcoder cancoder;
    protected final CANcoderConfiguration appliedConfig = new CANcoderConfiguration();

    private boolean connected = false;

    private final Alert disconnectedAlert;
    private final GatedAlert configRefreshAlert;
    private final GatedAlert configAlert;
    private final GatedAlert failedToSetSignalFrequencyAlert;
    private final GatedAlert didNotOptimizeCanAlert;

    private final List<BaseStatusSignal> signals;
    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<MagnetHealthValue> magnetHealth;

    EncoderIOCancoder(MechanismConfig config, LoggedCanivore bus) {
        MechanismConfig.Encoder spec = config.encoder();
        cancoder = new CANcoder(spec.canId(), bus);

        String prefix = config.displayName() + " Cancoder (ID " + spec.canId() + "): ";
        disconnectedAlert = new Alert(prefix + "Disconnected", AlertType.kError);
        configRefreshAlert =
            new GatedAlert(prefix + "Failed to refresh config", AlertType.kError, this::isConnected);
        configAlert =
            new GatedAlert(prefix + "Failed to configure", AlertType.kError, this::isConnected);
        failedToSetSignalFrequencyAlert = new GatedAlert(
            prefix + "Failed to set status signal frequency", AlertType.kError, this::isConnected);
        didNotOptimizeCanAlert =
            new GatedAlert(prefix + "Didn't optimize CAN", AlertType.kWarning, this::isConnected);

        // Refresh before modifying so a magnet offset burned into the device survives. Only the
        // fields the mechanism actually specifies are overwritten.
        tryUntilOk(CONFIG_ATTEMPTS,
            () -> cancoder.getConfigurator().refresh(appliedConfig), configRefreshAlert);

        appliedConfig.MagnetSensor.withSensorDirection(spec.cc().MagnetSensor.SensorDirection);
        if (spec.cc().MagnetSensor.MagnetOffset != 0.0) {
            appliedConfig.MagnetSensor.withMagnetOffset(spec.cc().MagnetSensor.MagnetOffset);
        }
        appliedConfig.MagnetSensor
            .withAbsoluteSensorDiscontinuityPoint(discontinuityPoint(config));

        tryUntilOk(CONFIG_ATTEMPTS,
            () -> cancoder.getConfigurator().apply(appliedConfig), configAlert);

        absolutePosition = cancoder.getAbsolutePosition(false);
        velocity = cancoder.getVelocity(false);
        magnetHealth = cancoder.getMagnetHealth(false);
        signals = List.of(absolutePosition, velocity, magnetHealth);

        tryUntilOk(CONFIG_ATTEMPTS,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_FREQUENCY_HZ, signals),
            failedToSetSignalFrequencyAlert);
        tryUntilOk(CONFIG_ATTEMPTS,
            () -> cancoder.optimizeBusUtilization(0, 1.0), didNotOptimizeCanAlert);
        PhoenixUtil.registerSignals(bus.getCanType(), signals);

        BaseStatusSignal.refreshAll(signals);
        refreshAlerts(BaseStatusSignal.isAllGood(signals));
    }

    /**
     * Puts the wrap point of the absolute reading opposite the middle of travel, so the encoder
     * never rolls over anywhere the mechanism can actually be.
     */
    private static double discontinuityPoint(MechanismConfig config) {
        if (!config.hasSoftLimits()) {
            return config.encoder().cc().MagnetSensor.AbsoluteSensorDiscontinuityPoint;
        }
        double mid = config.softLimitMidpointRot();
        return ((mid + 0.5) % 1.0 + 1.0) % 1.0;
    }

    private boolean isConnected() {
        return connected;
    }

    @Override
    public void updateInputs(EncoderInputs inputs) {
        inputs.absolutePositionRot = absolutePosition.getValueAsDouble();
        inputs.velocityRps = velocity.getValueAsDouble();
        inputs.health = TalonFXTranslator.toEncoderHealth(magnetHealth.getValue());
        inputs.connected = BaseStatusSignal.isAllGood(signals);
        refreshAlerts(inputs.connected);
    }

    private void refreshAlerts(boolean isConnected) {
        connected = isConnected;
        disconnectedAlert.set(!isConnected);
    }
}
