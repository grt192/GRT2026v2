package frc.robot.subsystems.swerve;

import java.util.Queue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.GatedAlert;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PhoenixUtil;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    protected final Pigeon2 pigeon;

    private final StatusSignal<Angle> yaw;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> roll;

    private final StatusSignal<AngularVelocity> yawVelocity;
    private final StatusSignal<AngularVelocity> pitchVelocity;
    private final StatusSignal<AngularVelocity> rollVelocity;

    private final StatusSignal<Time> upTime;
    private final StatusSignal<Voltage> supplyVoltage;

    private final StatusSignal<Temperature> temperature;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    private static final String PIGEON_ALERT_PREFIX = "Swerve Pigeon (ID " + SwerveConstants.PIGEON_ID + "): ";

    private boolean pigeonConnected = false;

    private final Alert pigeonDisconnectedAlert = new Alert(PIGEON_ALERT_PREFIX + "Disconnected", AlertType.kError);
    private final GatedAlert failedToConfigureAlert = new GatedAlert(PIGEON_ALERT_PREFIX + "Failed to configure", AlertType.kError, () -> pigeonConnected);

    public GyroIOPigeon2(LoggedCanivore canivore) {
        pigeon = new Pigeon2(SwerveConstants.PIGEON_ID, canivore);
        PhoenixUtil.tryUntilOk(5, () -> pigeon.getConfigurator().apply(new Pigeon2Configuration()), failedToConfigureAlert);
        pigeon.getConfigurator().setYaw(0.0);

        yaw = pigeon.getYaw();
        pitch = pigeon.getPitch();
        roll = pigeon.getRoll();

        yawVelocity = pigeon.getAngularVelocityZWorld();
        pitchVelocity = pigeon.getAngularVelocityYWorld();
        rollVelocity = pigeon.getAngularVelocityXWorld();

        upTime = pigeon.getUpTime();
        supplyVoltage = pigeon.getSupplyVoltage();

        temperature = pigeon.getTemperature();

        // Yaw and yaw rate feed odometry, so they run at the odometry rate (250 Hz on CAN FD)
        BaseStatusSignal.setUpdateFrequencyForAll(
            PhoenixOdometryThread.getInstance().getFrequencyHz(), yaw, yawVelocity);

        BaseStatusSignal.setUpdateFrequencyForAll(120.0, pitch, roll, rollVelocity, pitchVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(4.0, upTime, supplyVoltage, temperature);
        pigeon.optimizeBusUtilization();

        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone(), yawVelocity.clone());

        refreshPigeonAlerts(BaseStatusSignal.refreshAll(
            yaw, yawVelocity, pitch, roll, rollVelocity, pitchVelocity, upTime, supplyVoltage, temperature).isOK());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
            yaw, yawVelocity, pitch, roll, rollVelocity, pitchVelocity, upTime, supplyVoltage, temperature)
            .equals(StatusCode.OK);
        inputs.tempC = temperature.getValueAsDouble();

        inputs.yawPositionDeg = yaw.getValueAsDouble();
        inputs.pitchPositionDeg = pitch.getValueAsDouble();
        inputs.rollPositionDeg = roll.getValueAsDouble();

        inputs.yawVelocityDegPerSec = yawVelocity.getValueAsDouble();
        inputs.pitchVelocityDegPerSec = pitchVelocity.getValueAsDouble();
        inputs.rollVelocityDegPerSec = rollVelocity.getValueAsDouble();

        inputs.upTimeSec = upTime.getValueAsDouble();
        inputs.supplyVoltage = supplyVoltage.getValueAsDouble();

        refreshPigeonAlerts(inputs.connected);

        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
            yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    private void refreshPigeonAlerts(boolean connected) {
        pigeonConnected = connected;
        pigeonDisconnectedAlert.set(!connected);
    }
}
