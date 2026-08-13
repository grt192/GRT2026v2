package frc.robot.subsystems.intake.pivot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.SysIdFactory;
import frc.robot.util.subsystems.MechanismHardware;
import frc.robot.util.subsystems.MechanismIOBundle;
import frc.robot.util.subsystems.ServoSubsystem;

/** The intake pivot: swings the intake down to the floor and back into the frame. */
public class PivotSubsystem extends ServoSubsystem {

    /**
     * The bumper wall slides as the pivot swings, so the visualization draws it at a distance
     * that depends on the pivot angle. Found empirically in Desmos by Daniel:
     * https://www.desmos.com/calculator/hcsxnghm5o
     */
    static double getWallDistanceFromPivotRoot(double theta) {
        double s = theta + 0.31666353127;
        return 2.606810 * Math.sin(s) + 7.162190 * Math.cos(s) + 7.758382;
    }

    private final LoggedMechanismRoot2d wallRoot;

    public PivotSubsystem() {
        this(MechanismHardware.create(IntakePivotConfig.CONFIG));
    }

    /** Injection point for tests and for a hand-built IO. */
    public PivotSubsystem(MechanismIOBundle io) {
        super(IntakePivotConfig.CONFIG, io);

        wallRoot = visualizer().mechanism2d().getRoot(
            "WallRoot", IntakePivotConfig.ROOT_X_M, IntakePivotConfig.ROOT_Y_M);
        wallRoot.append(new LoggedMechanismLigament2d(
            "Wall", IntakePivotConfig.WALL_HEIGHT_M, 90.0, 6.0,
            new Color8Bit(Color.kDarkRed)));
    }

    @Override
    protected void onPeriodic() {
        wallRoot.setPosition(
            IntakePivotConfig.ROOT_X_M + Units.inchesToMeters(getWallDistanceFromPivotRoot(
                Units.rotationsToRadians(getEncoderAbsolutePositionRot()))),
            IntakePivotConfig.ROOT_Y_M);
    }

    public Command setPivotManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command deployPivot() {
        return this.runOnce(() -> setPositionRot(IntakePivotConfig.OUT_POS_ROT));
    }

    public Command retractPivot() {
        return this.runOnce(() -> setPositionRot(IntakePivotConfig.IN_POS_ROT));
    }

    public Command jigglePivot() {
        Command jiggle = Commands.sequence(
            this.runOnce(() -> setPositionRot(IntakePivotConfig.OUT_POS_ROT)),
            Commands.waitSeconds(0.5),
            this.runOnce(() -> setPositionRot(IntakePivotConfig.FORWARD_LIMIT_ROT)),
            Commands.waitSeconds(0.5)).repeatedly();
        jiggle.addRequirements(this);

        return jiggle;
    }

    public Command holdPivotOut() {
        return this.startEnd(
            () -> setPositionRot(IntakePivotConfig.OUT_POS_ROT),
            () -> setPositionRot(IntakePivotConfig.IN_POS_ROT));
    }

    public Command stopPivot() {
        return this.runOnce(this::stop);
    }

    public Command runSysId() {
        return SysIdFactory.fullSweep(SysIdFactory.routine(
            this, IntakePivotConfig.LOG_KEY, IntakePivotConfig.CONFIG.sysId(), this::setVoltage));
    }
}
