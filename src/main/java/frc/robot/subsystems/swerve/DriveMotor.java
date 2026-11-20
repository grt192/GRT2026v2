package frc.robot.subsystems.swerve;

import static frc.robot.Constants.LoggingConstants.SWERVE_TABLE;
import static frc.robot.Constants.SwerveDriveConstants.DRIVE_CURRENT_LIMIT_ENABLE;
import static frc.robot.Constants.SwerveDriveConstants.DRIVE_GEAR_REDUCTION;
import static frc.robot.Constants.SwerveDriveConstants.DRIVE_MAX_ACCELERATION;
import static frc.robot.Constants.SwerveDriveConstants.DRIVE_MAX_VELOCITY_RPS;
import static frc.robot.Constants.SwerveDriveConstants.DRIVE_PEAK_STATOR_CURRENT;
import static frc.robot.Constants.SwerveDriveConstants.DRIVE_RAMP_RATE;
import static frc.robot.Constants.SwerveDriveConstants.DRIVE_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.SwerveDriveConstants.DRIVE_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class DriveMotor {

    // Motor instance for controlling the drive motor
    private TalonFX motor;
    private int motorId;

    // Configuration for Kraken stored in one Object
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // For fine control of velocity and torque using FOC (Field-Oriented Control)
    private VelocityTorqueCurrentFOC torqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withSlot(0);

    // Target speed in rotations per second
    private double targetRotationsPerSec = 0;

    // logging
    private NetworkTableInstance ntInstance;
    private NetworkTable swerveStatsTable;
    private DoublePublisher veloErrorPublisher;
    private DoublePublisher veloPublisher;
    private DoublePublisher appliedVlotsPublisher;
    private DoublePublisher supplyCurrentPublisher;
    private DoublePublisher torqueCurrentPublisher;
    private DoublePublisher targetRPSPublisher;
    private DoublePublisher positionPublisher;

    // Tunable current limits via NetworkTables
    private DoubleEntry supplyCurrentLimitEntry;
    private DoubleEntry statorCurrentLimitEntry;
    private double lastSupplyLimit = DRIVE_SUPPLY_CURRENT_LIMIT;
    private double lastStatorLimit = DRIVE_STATOR_CURRENT_LIMIT;

    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Voltage> appliedVoltsSignal;
    private StatusSignal<Current> supplyCurrentSignal;
    private StatusSignal<Current> torqueCurrentSignal; // torqueCurrent is Pro
    private StatusSignal<Temperature> deviceTempSignal;

    public DriveMotor(int motorID, CANBus canivore) {
        this.motorId = motorID;

        // Set Motor and reset Encoder
        motor = new TalonFX(motorID, canivore);
        motor.setPosition(0);

        // Configure CANcoder and Kraken
        configureMotor();
        initNt(motorID);
        initSignals();
    }

    /**
     * initializes network table and entries
     * 
     * @param canId motor's CAN ID
     */
    private void initNt(int canId) {
        ntInstance = NetworkTableInstance.getDefault();
        swerveStatsTable = ntInstance.getTable(SWERVE_TABLE);
        positionPublisher = swerveStatsTable.getDoubleTopic(canId + "position").publish();
        targetRPSPublisher = swerveStatsTable.getDoubleTopic(canId + "targetRPS").publish();
        veloErrorPublisher = swerveStatsTable.getDoubleTopic(canId + "veloError").publish();
        veloPublisher = swerveStatsTable.getDoubleTopic(canId + "velo").publish();
        appliedVlotsPublisher = swerveStatsTable.getDoubleTopic(canId + "appliedVolts").publish();
        supplyCurrentPublisher = swerveStatsTable.getDoubleTopic(canId + "supplyCurrent").publish();
        torqueCurrentPublisher = swerveStatsTable.getDoubleTopic(canId + "torqueCurrent").publish();

        // Tunable current limits (shared across all drive motors)
        NetworkTable tuningTable = ntInstance.getTable("SwerveTuning");
        supplyCurrentLimitEntry = tuningTable.getDoubleTopic("driveSupplyCurrentLimit").getEntry(DRIVE_SUPPLY_CURRENT_LIMIT);
        statorCurrentLimitEntry = tuningTable.getDoubleTopic("driveStatorCurrentLimit").getEntry(DRIVE_STATOR_CURRENT_LIMIT);
        supplyCurrentLimitEntry.set(DRIVE_SUPPLY_CURRENT_LIMIT);
        statorCurrentLimitEntry.set(DRIVE_STATOR_CURRENT_LIMIT);
    }

    /**
     * Initializes Phoenix 6's signals
     */
    private void initSignals() {
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        appliedVoltsSignal = motor.getMotorVoltage();
        torqueCurrentSignal = motor.getTorqueCurrent();
        supplyCurrentSignal = motor.getSupplyCurrent();
        deviceTempSignal = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            250.0, positionSignal, velocitySignal,
            appliedVoltsSignal, torqueCurrentSignal, supplyCurrentSignal, deviceTempSignal);
        motor.optimizeBusUtilization(0, 1.0);
    }

    /**
     * Set Configurations for Kraken drive
     */
    public void configureMotor() {

        // Set peak current for torque limiting for stall prevention
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_PEAK_STATOR_CURRENT;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_PEAK_STATOR_CURRENT;

        // Current limits (optimized for swerve drive)
        motorConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_CURRENT_LIMIT_ENABLE;
        motorConfig.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = DRIVE_CURRENT_LIMIT_ENABLE;

        // How fast can the code change torque for the motor
        motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DRIVE_RAMP_RATE;

        // By Default Robot will not move
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Configure MotionMagic for velocity control with acceleration limiting
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = DRIVE_MAX_VELOCITY_RPS;
        motorConfig.MotionMagic.MotionMagicAcceleration = DRIVE_MAX_ACCELERATION;

        // Apply motor config with retries (max 5 attempts)
        for (int i = 0; i < 5; i++) {
            if (motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK) {
                break; // Success
            }
        }
    }

    /**
     * Sets Drive Motor Velocity in Meters/Seconds
     * 
     * @param TargetMetersPerSec
     */
    public void setVelocity(double metersPerSec) {

        targetRotationsPerSec = metersPerSec / DRIVE_WHEEL_CIRCUMFERENCE_METERS * DRIVE_GEAR_REDUCTION; // turns meters per sec into wheel rotation per sec
        // disabled set velocity for noise reduction
        motor.setControl(torqueCurrentFOC.withVelocity(targetRotationsPerSec)); // apply this constant speed

    }

    /**
     * Reset Encoder Position to 0, resets Distance
     */
    public void resetEncoder() {
        motor.setPosition(0);
    }

    /**
     * Configures drive motor's PIDSV
     * 
     * @param p :Determines how much the config will react to the error
     * @param i :Corrects recurring errors over time by stacking past errors
     * @param d :Reacting to the rate of change of the error, preventing
     *        overshooting and damping oscillations
     * @param s :Helps with overcoming initial friction or resistance in systems
     * @param v :Compensates for the velocity or speed at which the system is moving
     */
    public void configPID(double p, double i, double d, double s, double v) {

        Slot0Configs slot0Configs = new Slot0Configs(); // used to store and update PID values

        /*
         * Think of P as how much we want it to correct, as an example imagine you are
         * parking a car
         * If you’re too far from the spot, you might turn the steering wheel sharply (a
         * large correction) to get closer.
         * If you’re very close to the spot, you might make smaller, finer adjustments
         * (a smaller correction).
         * 
         * The proportional control is like how hard you turn the steering wheel based
         * on how far you are from the target spot. If you’re far away, you turn more;
         * if you’re close, you turn less.
         */

        slot0Configs.kP = p;

        /*
         * Integral Control's job is to correct recurring errors over time by stacking
         * past errors.
         * It sums up previous errors, so it looks at how many errors you have had over
         * time.
         * 
         * Imagine the motor is consistently below the target position due to friction.
         * kI will accumulate this error over time and then apply this correction to
         * bring the motor to its target.
         * 
         * Keep in mind that this will also apply if something is blocking the mechanism
         * from working, e.g., a rock.
         * This will lead to a crazy increase in errors, potentially causing motors to
         * burn out.
         */
        slot0Configs.kI = i;

        /*
         * The Derivitive Controls job is to look at the Rate of Change (slope) of how
         * fast the error is changing (def of derrivitive)
         * If the error is chaning too fast, the kD will slow it down so we do not
         * overshoot
         * 
         * Imagine you’re driving a car toward a stop sign. As you approach the stop
         * sign, you start to apply the brakes.
         * The derivative control is like reacting to how quickly you’re approaching the
         * stop sign:
         * If you’re coming in fast, the derivative term would apply more braking force
         * to slow you down before you overshoot the stop sign.
         * If you’re coming in slowly, the derivative term would apply less braking
         * force, just enough to prevent overshooting but not too much.
         * 
         * kD responds to the rate of change of the error, preventing overshooting and
         * damping oscillations, too high will make mech slower
         */
        slot0Configs.kD = d;

        /*
         * Static is added to make sure the motor can spin properly even without
         * presence of errors,
         * this includes things like power needed to overcome friction, drag, or the
         * inertia
         * 
         * Imagine trying to turn a gear.
         * Even if you’re not trying to change its position, you still need to exert
         * some force to overcome static friction — the resistance between the object
         * and the surface, after doing so it will be more easy to turn
         * kS provides that initial extra force to overcome the friction and start
         * turning the gear.
         * Once the gear starts moving, we will switch to using kP, kI, and kD terms.
         * 
         * kS helps with overcoming initial friction or resistance in systems
         */
        slot0Configs.kS = s;

        /*
         * Velocity compensates for the velocity or speed at which the system is moving,
         * think if it like correction of speed
         * 
         * The kV term is proportional to the velocity or speed of the system, meaning
         * that as the speed increases:
         * the control system will apply an appropriate amount of correction based on
         * the speed at which the system is moving.
         * 
         * Imagine driving a car and trying to maintain a speed. The kV term is like a
         * cruise control system that adjusts the throttle based on how fast you’re
         * going:
         * If you’re going too fast, the kV term will reduce the throttle (power) to
         * slow you down.
         * If you’re going too slow, it will increase the throttle to speed up.
         * 
         * kV adjusts the system’s response based on its speed, helping to prevent
         * overshooting and stabilize motion with goal of keeping constant speed.
         */
        slot0Configs.kV = v;

        motor.getConfigurator().apply(slot0Configs);
    }

    /**
     * Gets the distance the drive wheel has traveled.
     * 
     * @return distance the drive wheel has traveled in meters
     */
    public double getDistance() {
        return DRIVE_WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_REDUCTION * positionSignal.getValueAsDouble();
    }

    /**
     * Get swerve wheel's velocity in m/s
     *
     * @return swerve wheel's velocity in m/s
     */
    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    /**
     * Gets the tempature of the motor
     *
     * @return temperature of the motor in double
     */
    public double getTemperature() {
        return deviceTempSignal.getValueAsDouble();
    }

    /**
     * Checks NetworkTables for updated current limits and applies them if changed.
     * Call this periodically (e.g., in periodic()).
     */
    public void updateCurrentLimitsFromNt() {
        double newSupplyLimit = supplyCurrentLimitEntry.get(DRIVE_SUPPLY_CURRENT_LIMIT);
        double newStatorLimit = statorCurrentLimitEntry.get(DRIVE_STATOR_CURRENT_LIMIT);

        // Only reconfigure if values changed
        if (newSupplyLimit != lastSupplyLimit || newStatorLimit != lastStatorLimit) {
            lastSupplyLimit = newSupplyLimit;
            lastStatorLimit = newStatorLimit;

            motorConfig.CurrentLimits.SupplyCurrentLimit = newSupplyLimit;
            motorConfig.CurrentLimits.StatorCurrentLimit = newStatorLimit;
            motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = newStatorLimit;
            motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -newStatorLimit;

            motor.getConfigurator().apply(motorConfig.CurrentLimits, 0.02);
            motor.getConfigurator().apply(motorConfig.TorqueCurrent, 0.02);
        }
    }

    /**
     * Refreshes all cached status signals so the latest values are read.
     * Required because the signals are read from cached objects (not the
     * auto-refreshing device getters) and optimizeBusUtilization() was applied.
     * Call once per loop before reading/logging any cached signal.
     */
    public void refreshSignals() {
        BaseStatusSignal.refreshAll(
            positionSignal, velocitySignal, appliedVoltsSignal,
            supplyCurrentSignal, torqueCurrentSignal, deviceTempSignal);
    }

    /**
     * Publishes drive motor statistics to NetworkTables
     */
    public void publishStats() {
        positionPublisher.set(getDistance());
        targetRPSPublisher.set(targetRotationsPerSec);
        veloErrorPublisher.set(0.0); // TODO: Calculate actual velocity error
        veloPublisher.set(getVelocity());
        appliedVlotsPublisher.set(appliedVoltsSignal.getValueAsDouble());
        supplyCurrentPublisher.set(supplyCurrentSignal.getValueAsDouble());
        torqueCurrentPublisher.set(torqueCurrentSignal.getValueAsDouble());
    }

    /**
     * Logs drive motor statistics to data log
     */
    public void logStats() {
        Logger.recordOutput("drive/" + motorId + "/position", getDistance());
        Logger.recordOutput("drive/" + motorId + "/veloError", targetRotationsPerSec - velocitySignal.getValueAsDouble());
        Logger.recordOutput("drive/" + motorId + "/velo", getVelocity());
        Logger.recordOutput("drive/" + motorId + "/targetVelo", targetRotationsPerSec);
        Logger.recordOutput("drive/" + motorId + "/appliedVolts", appliedVoltsSignal.getValueAsDouble());
        Logger.recordOutput("drive/" + motorId + "/supplyCurrent", supplyCurrentSignal.getValueAsDouble());
        Logger.recordOutput("drive/" + motorId + "/torqueCurrent", torqueCurrentSignal.getValueAsDouble());
        Logger.recordOutput("drive/" + motorId + "/temperature", getTemperature());
    }

}
