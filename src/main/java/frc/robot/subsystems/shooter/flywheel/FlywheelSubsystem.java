package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ComponentStatus.MotorControlMode;
import frc.robot.util.LoggedSetpointTracker;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;
import frc.robot.util.RollerMechanism2D;

public class FlywheelSubsystem extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber motionMagicAccel =
        new LoggedTunableNumber("Flywheel/motionMagicAccel_rotPerSec2", ShooterConstants.Flywheel.MM_ACCEL_RPS2);
    private final LoggedTunableNumber motionMagicVelo =
        new LoggedTunableNumber("Flywheel/motionMagicVelocity_rotPerSec", ShooterConstants.Flywheel.MM_MAX_VELO_RPS);
    private final LoggedTunableNumber motionMagicJerk =
        new LoggedTunableNumber("Flywheel/motionMagicJerk_rotPerSec3", ShooterConstants.Flywheel.MM_JERK_RPS3);

    private final LoggedTunableNumber.Watcher pidWatcher;
    private final LoggedTunableNumber.Watcher motionMagicWatcher =
        LoggedTunableNumber.watch(motionMagicAccel, motionMagicVelo, motionMagicJerk);

    private final LoggedSetpointTracker setpointTracker = new LoggedSetpointTracker(
        "Flywheel",
        MotorControlMode.DutyCycle,
        MotorControlMode.Voltage,
        MotorControlMode.Velocity);

    private final SysIdRoutine sysIdRoutine;

    private final RollerMechanism2D mechanism = new RollerMechanism2D(0.4);
    @AutoLogOutput(key = "Flywheel/Mechanism2D")
    private final LoggedMechanism2d mechanism2d = mechanism.getMechanism2d();

    public FlywheelSubsystem(FlywheelIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Flywheel/kP", pid.kP());
        kI = new LoggedTunableNumber("Flywheel/kI", pid.kI());
        kD = new LoggedTunableNumber("Flywheel/kD", pid.kD());
        kS = new LoggedTunableNumber("Flywheel/kS", pid.kS());
        kV = new LoggedTunableNumber("Flywheel/kV", pid.kV());
        kA = new LoggedTunableNumber("Flywheel/kA", pid.kA());
        pidWatcher = LoggedTunableNumber.watch(kP, kI, kD, kS, kV, kA);

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds),
                Volts.of(7),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Flywheel/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this));
    }

    public void setDutyCycle(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        io.setDutyCycleOut(speed);
        setpointTracker.updateSetpoint(speed, MotorControlMode.DutyCycle);
    }

    public void setVoltage(double volts) {
        double clampedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        io.setVoltageOut(clampedVolts);
        setpointTracker.updateSetpoint(clampedVolts, MotorControlMode.Voltage);
    }

    private void setVoltage(Voltage volts) {
        setVoltage(volts.in(Volts));
    }

    public void setVelocity(double velocityRPS) {
        io.setVelocityOut(velocityRPS);
        setpointTracker.updateSetpoint(velocityRPS, MotorControlMode.Velocity);
    }

    public void stop() {
        io.stop();
        setpointTracker.setControlMode(MotorControlMode.Disabled);
    }

    public boolean atSetpoint() {
        return setpointTracker.atSetpoint(
            MotorControlMode.Velocity, inputs.velocityRPS, ShooterConstants.Flywheel.VELOCITY_TOLERANCE_RPS);
    }

    public double getVelocity() {
        return inputs.velocityRPS;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        if (DriverStation.isDisabled()) {
            setpointTracker.setControlMode(MotorControlMode.Disabled);
        }

        setpointTracker.logAll();
        Logger.recordOutput("Flywheel/atVelocitySetpoint", atSetpoint());

        mechanism.setPosition(inputs.positionRot);

        pidWatcher.ifChanged(
            () -> io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()));

        motionMagicWatcher.ifChanged(
            () -> io.updateMotionMagicConfig(motionMagicAccel.get(), motionMagicVelo.get(), motionMagicJerk.get()));

        LoggedTracer.record("Flywheel");
    }

    public Command runSysID() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command setFlywheelManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command setFlywheelVelocity(DoubleSupplier rpsSupplier) {
        return this.run(() -> setVelocity(rpsSupplier.getAsDouble()))
            .finallyDo(this::stop);
    }

    public Command setFlywheelVelocity(double velocityRPS) {
        return setFlywheelVelocity(() -> velocityRPS);
    }

    public Command stopFlywheel() {
        return this.runOnce(this::stop);
    }
}
