package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperConstants.HopperIntake;
import frc.robot.util.LoggedSetpointTracker;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;
import frc.robot.util.ComponentStatus.MotorControlMode;

public class HopperSubsystem extends SubsystemBase {
    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber motionMagicAccel =
        new LoggedTunableNumber("Hopper/motionMagicAccel_rotPerSec2", HopperConstants.MM_ACCEL_RPS2);
    private final LoggedTunableNumber motionMagicVelo =
        new LoggedTunableNumber("Hopper/motionMagicVelocity_rotPerSec", HopperConstants.MM_MAX_VELO_RPS);
    private final LoggedTunableNumber motionMagicJerk =
        new LoggedTunableNumber("Hopper/motionMagicJerk_rotPerSec3", HopperConstants.MM_JERK_RPS3);

    private final LoggedTunableNumber.Watcher pidWatcher;
    private final LoggedTunableNumber.Watcher motionMagicWatcher =
        LoggedTunableNumber.watch(motionMagicAccel, motionMagicVelo, motionMagicJerk);

    private final LoggedSetpointTracker setpointTracker = new LoggedSetpointTracker(
        "Hopper",
        MotorControlMode.DutyCycle,
        MotorControlMode.Voltage,
        MotorControlMode.Velocity);

    private final SysIdRoutine sysIdRoutine;

    private static final int HOPPER_VANES = 4;
    @AutoLogOutput(key = "Hopper/Mechanism2d")
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("HopperSpinner", 0.5, 0.5);
    private final LoggedMechanismLigament2d[] vaneLigaments = new LoggedMechanismLigament2d[HOPPER_VANES];

    public HopperSubsystem(HopperIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Hopper/kP", pid.kP());
        kI = new LoggedTunableNumber("Hopper/kI", pid.kI());
        kD = new LoggedTunableNumber("Hopper/kD", pid.kD());
        kS = new LoggedTunableNumber("Hopper/kS", pid.kS());
        kV = new LoggedTunableNumber("Hopper/kV", pid.kV());
        kA = new LoggedTunableNumber("Hopper/kA", pid.kA());
        pidWatcher = LoggedTunableNumber.watch(kP, kI, kD, kS, kV, kA);

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());

        for (int i = 0; i < HOPPER_VANES; i++) {
            vaneLigaments[i] = mechanismRoot.append(
                new LoggedMechanismLigament2d(
                    "Vane" + i,
                    0.4,
                    i * (360.0 / HOPPER_VANES),
                    6.0,
                    new Color8Bit(Color.kBlueViolet)));
        }

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds), // ramp rate: 1 V/s
                Volts.of(7), // step voltage
                Seconds.of(10), // timeout
                (state) -> Logger.recordOutput("Hopper/SysIdTestState", state.toString())),
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
            MotorControlMode.Velocity, inputs.velocityRPS, HopperConstants.VELOCITY_TOLERANCE_RPS);
    }

    public void setHopperState(HopperIntake state) {
        switch (state) {
            case BALL_IN:
                setVelocity(HopperConstants.TARGET_RPS);
                break;
            case BALL_OUT:
                setVelocity(-HopperConstants.TARGET_RPS);
                break;
            default:
                stop();
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);

        if (DriverStation.isDisabled()) {
            setpointTracker.setControlMode(MotorControlMode.Disabled);
        }

        setpointTracker.logAll();
        Logger.recordOutput("Hopper/atVelocitySetpoint", atSetpoint());

        double spinnerDeg = Units.rotationsToDegrees(inputs.positionRot);
        for (int i = 0; i < HOPPER_VANES; i++) {
            vaneLigaments[i].setAngle(spinnerDeg + i * (360.0 / HOPPER_VANES));
        }

        pidWatcher.ifChanged(
            () -> io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()));

        motionMagicWatcher.ifChanged(
            () -> io.updateMotionMagicConfig(motionMagicAccel.get(), motionMagicVelo.get(), motionMagicJerk.get()));

        LoggedTracer.record("Hopper");
    }

    public Command runSysID() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command setHopperManualSpeed(DoubleSupplier speedSupplier) {
        return run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(interrupted -> stop());
    }

    public Command runHopperState(HopperIntake state) {
        return this.runEnd(
            () -> {
                setHopperState(state);
            },
            this::stop);
    }

    public Command runHopperOut() {
        return runHopperState(HopperIntake.BALL_OUT);
    }

    public Command runHopperIn() {
        return runHopperState(HopperIntake.BALL_IN);
    }

    public Command runHopperDutyCycle(double dutyCycle) {
        return this.runEnd(
            () -> setDutyCycle(dutyCycle),
            this::stop);
    }

    public Command stopHopper() {
        return this.runOnce(this::stop);
    }
}
