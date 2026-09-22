package frc.robot.subsystems.shooter.hood;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants.Hood;
import frc.robot.util.ComponentStatus.MotorControlMode;
import frc.robot.util.LoggedSetpointTracker;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;

public class HoodSubsystem extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber.Watcher pidWatcher;

    private final LoggedSetpointTracker setpointTracker = new LoggedSetpointTracker(
        "Hood",
        MotorControlMode.DutyCycle,
        MotorControlMode.Voltage,
        MotorControlMode.Position);

    private final SysIdRoutine sysIdRoutine;

    @AutoLogOutput(key = "Hood/Mechanism2D")
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("Hood", 0.5, 0.2);
    private final LoggedMechanismLigament2d hoodMech =
        mechanismRoot.append(new LoggedMechanismLigament2d(
            "HoodArm",
            0.4,
            0,
            6.0,
            new Color8Bit(Color.kCornflowerBlue)));

    public HoodSubsystem(HoodIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Hood/kP", pid.kP());
        kI = new LoggedTunableNumber("Hood/kI", pid.kI());
        kD = new LoggedTunableNumber("Hood/kD", pid.kD());
        kS = new LoggedTunableNumber("Hood/kS", pid.kS());
        kV = new LoggedTunableNumber("Hood/kV", pid.kV());
        kA = new LoggedTunableNumber("Hood/kA", pid.kA());
        pidWatcher = LoggedTunableNumber.watch(kP, kI, kD, kS, kV, kA);

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(1),
                Seconds.of(5),
                (state) -> Logger.recordOutput("Hood/SysIdTestState", state.toString())),
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

    public void setPosition(double positionRot) {
        double clampedPositionRot = MathUtil.clamp(
            positionRot,
            Hood.LOWER_ANGLE_LIMIT_ROT,
            Hood.UPPER_ANGLE_LIMIT_ROT);

        io.setPositionOut(clampedPositionRot);
        setpointTracker.updateSetpoint(clampedPositionRot, MotorControlMode.Position);
    }

    public void stop() {
        io.stop();
        setpointTracker.setControlMode(MotorControlMode.Disabled);
    }

    public boolean atPositionSetpoint() {
        return setpointTracker.atSetpoint(
            MotorControlMode.Position, inputs.positionRot, Hood.ANGLE_TOLERANCE_ROT);
    }

    public double getPosition() {
        return inputs.positionRot;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        if (DriverStation.isDisabled()) {
            setpointTracker.setControlMode(MotorControlMode.Disabled);
        }

        setpointTracker.logAll();
        Logger.recordOutput("Hood/atPositionSetpoint", atPositionSetpoint());

        hoodMech.setAngle(Units.rotationsToDegrees(inputs.positionRot));

        pidWatcher.ifChanged(
            () -> io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()));

        LoggedTracer.record("Hood");
    }

    public Command runSysID() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command setHoodManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble()))
            .finallyDo(this::stop);
    }

    public Command goToPosition(double positionRot) {
        return this.runOnce(() -> setPosition(positionRot));
    }

    public Command holdPosition(double positionRot) {
        return this.startEnd(
            () -> setPosition(positionRot),
            this::stop);
    }

    public Command holdPositionThenHide(double positionRot) {
        return this.startEnd(
            () -> setPosition(positionRot),
            () -> setPosition(Hood.LOWER_ANGLE_LIMIT_ROT));
    }

    public Command holdPositionThenHide(DoubleSupplier positionRotSupplier) {
        // run(), not startEnd(), so the supplier is re-sampled every loop and the hood
        // tracks live adjustments while the command is held.
        return this.run(() -> setPosition(positionRotSupplier.getAsDouble()))
            .finallyDo(() -> setPosition(Hood.LOWER_ANGLE_LIMIT_ROT));
    }

    public Command hideHood() {
        return this.runOnce(() -> setPosition(Hood.LOWER_ANGLE_LIMIT_ROT))
            .andThen(Commands.waitUntil(this::atPositionSetpoint));
    }

    public Command holdDownHood() {
        return this.run(() -> setPosition(Hood.LOWER_ANGLE_LIMIT_ROT));
    }

    public Command jiggleHood() {
        // Jiggle within the middle 50% of the hood's travel range.
        double range = Hood.UPPER_ANGLE_LIMIT_ROT - Hood.LOWER_ANGLE_LIMIT_ROT;
        double lowPos = Hood.LOWER_ANGLE_LIMIT_ROT + range * 0.25;
        double highPos = Hood.LOWER_ANGLE_LIMIT_ROT + range * 0.25;

        Command jiggleHoodCommand = Commands.sequence(
            this.runOnce(() -> setPosition(highPos)),
            Commands.waitSeconds(0.5),
            this.runOnce(() -> setPosition(lowPos)),
            Commands.waitSeconds(0.5)).repeatedly();
        jiggleHoodCommand.addRequirements(this);

        return jiggleHoodCommand;
    }

    public Command stopHood() {
        return this.runOnce(this::stop);
    }
}
