package frc.robot.subsystems.intake.pivot;

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
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.ComponentStatus.MotorControlMode;
import frc.robot.util.LoggedSetpointTracker;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;

public class PivotSubsystem extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kG;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber.Watcher pidWatcher;

    private final LoggedSetpointTracker setpointTracker = new LoggedSetpointTracker(
        "Pivot",
        MotorControlMode.DutyCycle,
        MotorControlMode.Voltage,
        MotorControlMode.Position);
    private boolean intakeDeployed = false;

    private final SysIdRoutine sysIdRoutine;

    private static final double CANVAS_WIDTH_M = Units.inchesToMeters(26.5);
    private static final double CANVAS_HEIGHT_M = Units.inchesToMeters(15);
    private static final double PIVOT_ARM_LENGTH_M = Units.inchesToMeters(13.10);
    private static final double WALL_HEIGHT_M = Units.inchesToMeters(12);
    private static final double PIVOT_ROOT_X = Units.inchesToMeters(9);
    private static final double PIVOT_ROOT_Y = Units.inchesToMeters(1.5);

    @AutoLogOutput(key = "Pivot/Mechanism2D")
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(CANVAS_WIDTH_M, CANVAS_HEIGHT_M);
    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("Pivot", PIVOT_ROOT_X, PIVOT_ROOT_Y);
    private final LoggedMechanismLigament2d pivotMech =
        mechanismRoot.append(new LoggedMechanismLigament2d(
            "PivotArm",
            PIVOT_ARM_LENGTH_M,
            0,
            6.0,
            new Color8Bit(Color.kBlueViolet)));
    private final LoggedMechanismRoot2d wallRoot =
        mechanism.getRoot("WallRoot", PIVOT_ROOT_X, PIVOT_ROOT_Y);
    @SuppressWarnings("unused")
    private final LoggedMechanismLigament2d wallMech =
        wallRoot.append(new LoggedMechanismLigament2d(
            "Wall",
            WALL_HEIGHT_M,
            90.0,
            6.0,
            new Color8Bit(Color.kDarkRed)));

    // Found empirically in Desmos by Daniel: https://www.desmos.com/calculator/hcsxnghm5o
    static double getWallDistanceFromPivotRoot(double theta) {
        double s = theta + 0.31666353127;
        return 2.606810 * Math.sin(s) + 7.162190 * Math.cos(s) + 7.758382;
    }

    public PivotSubsystem(PivotIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Pivot/kP", pid.kP());
        kI = new LoggedTunableNumber("Pivot/kI", pid.kI());
        kD = new LoggedTunableNumber("Pivot/kD", pid.kD());
        kS = new LoggedTunableNumber("Pivot/kS", pid.kS());
        kG = new LoggedTunableNumber("Pivot/kG", pid.kG());
        kV = new LoggedTunableNumber("Pivot/kV", pid.kV());
        kA = new LoggedTunableNumber("Pivot/kA", pid.kA());
        pidWatcher = LoggedTunableNumber.watch(kP, kI, kD, kS, kG, kV, kA);

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get());

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds), // ramp rate: 0.5 V/s (slow for limited range)
                Volts.of(1), // step voltage
                Seconds.of(5), // timeout
                (state) -> Logger.recordOutput("Pivot/SysIdTestState", state.toString())),
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
            IntakeConstants.PIVOT_REVERSE_LIMIT_ROT,
            IntakeConstants.PIVOT_FORWARD_LIMIT_ROT);
        io.setPositionOut(clampedPositionRot);
        setpointTracker.updateSetpoint(clampedPositionRot, MotorControlMode.Position);
    }

    public void stop() {
        io.stop();
        setpointTracker.setControlMode(MotorControlMode.Disabled);
    }

    public boolean atPositionSetpoint() {
        return setpointTracker.atSetpoint(
            MotorControlMode.Position, inputs.positionRot, IntakeConstants.PIVOT_POSITION_TOLERANCE_ROT);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        if (DriverStation.isDisabled()) {
            setpointTracker.setControlMode(MotorControlMode.Disabled);
        }

        setpointTracker.logAll();
        Logger.recordOutput("Pivot/atPositionSetpoint", atPositionSetpoint());

        pivotMech.setAngle(Units.rotationsToDegrees(inputs.encoderAbsolutePositionRot));
        wallRoot.setPosition(
            PIVOT_ROOT_X + Units.inchesToMeters(getWallDistanceFromPivotRoot(
                Units.rotationsToRadians(inputs.encoderAbsolutePositionRot))),
            PIVOT_ROOT_Y);

        pidWatcher.ifChanged(
            () -> io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get()));

        LoggedTracer.record("Pivot");
    }

    public Command runSysID() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command setPivotManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble()))
            .finallyDo(this::stop);
    }

    public Command deployPivot() {
        return this.runOnce(() -> {
            intakeDeployed = true;
            setPosition(IntakeConstants.PIVOT_OUT_POS_ROT);
        });
    }

    public Command retractPivot() {
        return this.runOnce(() -> {
            intakeDeployed = false;
            setPosition(IntakeConstants.PIVOT_IN_POS_ROT);
        });
    }

    public Command togglePivot() {
        return Commands.either(retractPivot(), deployPivot(), () -> intakeDeployed);
    }

    public Command jigglePivot() {
        Command jigglePivotCommand = Commands.sequence(
            this.runOnce(() -> setPosition(IntakeConstants.PIVOT_OUT_POS_ROT)),
            Commands.waitSeconds(0.5),
            this.runOnce(() -> setPosition(IntakeConstants.PIVOT_FORWARD_LIMIT_ROT)),
            Commands.waitSeconds(0.5)).repeatedly();
        jigglePivotCommand.addRequirements(this);

        return jigglePivotCommand;
    }

    public Command cyclePivotMid(double initialDelaySeconds, double toggleIntervalSeconds) {
        return Commands.sequence(
            this.runOnce(() -> setPosition(IntakeConstants.PIVOT_MID_LOWER_ROT)),
            Commands.waitSeconds(initialDelaySeconds),
            Commands.sequence(
                this.runOnce(() -> setPosition(IntakeConstants.PIVOT_MID_UPPER_ROT)),
                Commands.waitSeconds(toggleIntervalSeconds),
                this.runOnce(() -> setPosition(IntakeConstants.PIVOT_MID_LOWER_ROT)),
                Commands.waitSeconds(toggleIntervalSeconds)).repeatedly())
            .finallyDo(() -> setPosition(IntakeConstants.PIVOT_OUT_POS_ROT));
    }

    public Command holdPivotOut() {
        return this.startEnd(
            () -> setPosition(IntakeConstants.PIVOT_OUT_POS_ROT),
            () -> setPosition(IntakeConstants.PIVOT_IN_POS_ROT));
    }

    public Command stopPivot() {
        return this.runOnce(this::stop);
    }
}
