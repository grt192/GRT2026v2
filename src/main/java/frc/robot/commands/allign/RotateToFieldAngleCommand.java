package frc.robot.commands.allign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateToAngleConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import java.util.function.BooleanSupplier;

public class RotateToFieldAngleCommand extends Command {
    private final DriveSubsystem swerve;
    private final PIDController pid;
    private final double targetDegrees;
    private final BooleanSupplier cancelCondition;

    private static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("AutoAim");
    private static final NetworkTableEntry KP_ENTRY = TABLE.getEntry("kP");
    private static final NetworkTableEntry KI_ENTRY = TABLE.getEntry("kI");
    private static final NetworkTableEntry KD_ENTRY = TABLE.getEntry("kD");
    private static final NetworkTableEntry TOLERANCE_ENTRY = TABLE.getEntry("Tolerance");
    private static final NetworkTableEntry GOAL_ENTRY = TABLE.getEntry("Goal");
    private static final NetworkTableEntry ACTUAL_ENTRY = TABLE.getEntry("Actual");
    private static final NetworkTableEntry ERROR_ENTRY = TABLE.getEntry("Error");
    private static final NetworkTableEntry OUTPUT_ENTRY = TABLE.getEntry("Output");

    private static boolean initialized = false;

    public RotateToFieldAngleCommand(DriveSubsystem swerve, double targetDegrees, BooleanSupplier cancelCondition) {
        this.swerve = swerve;
        this.targetDegrees = targetDegrees;

        this.cancelCondition = cancelCondition;
        this.pid = new PIDController(
            RotateToAngleConstants.kP,
            RotateToAngleConstants.kI,
            RotateToAngleConstants.kD);
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(RotateToAngleConstants.TOLERANCE_DEGREES);
        addRequirements(swerve);

        if (!initialized) {
            KP_ENTRY.setDouble(RotateToAngleConstants.kP);
            KI_ENTRY.setDouble(RotateToAngleConstants.kI);
            KD_ENTRY.setDouble(RotateToAngleConstants.kD);
            TOLERANCE_ENTRY.setDouble(RotateToAngleConstants.TOLERANCE_DEGREES);

            GOAL_ENTRY.setDouble(Double.NaN);
            ACTUAL_ENTRY.setDouble(Double.NaN);
            ERROR_ENTRY.setDouble(Double.NaN);
            OUTPUT_ENTRY.setDouble(Double.NaN);
            initialized = true;
        }
    }

    @Override
    public void initialize() {
        pid.setP(KP_ENTRY.getDouble(RotateToAngleConstants.kP));
        pid.setI(KI_ENTRY.getDouble(RotateToAngleConstants.kI));
        pid.setD(KD_ENTRY.getDouble(RotateToAngleConstants.kD));
        pid.setTolerance(TOLERANCE_ENTRY.getDouble(RotateToAngleConstants.TOLERANCE_DEGREES));
        pid.reset();
    }

    @Override
    public void execute() {
        // Live update PID values from NetworkTables
        pid.setP(KP_ENTRY.getDouble(RotateToAngleConstants.kP));
        pid.setI(KI_ENTRY.getDouble(RotateToAngleConstants.kI));
        pid.setD(KD_ENTRY.getDouble(RotateToAngleConstants.kD));
        pid.setTolerance(TOLERANCE_ENTRY.getDouble(RotateToAngleConstants.TOLERANCE_DEGREES));

        double currentAngle = normalizeAngle(swerve.getRobotPosition().getRotation().getDegrees());
        double normalizedTarget = normalizeAngle(targetDegrees);
        double rotationPower = pid.calculate(currentAngle, normalizedTarget);
        swerve.setDrivePowers(0, 0, rotationPower);

        // Publish feedback to NetworkTables
        GOAL_ENTRY.setDouble(normalizedTarget);
        ACTUAL_ENTRY.setDouble(currentAngle);
        double error = normalizedTarget - currentAngle;
        if (error > 180) {
            error -= 360;
        }
        if (error < -180) {
            error += 360;
        }
        ERROR_ENTRY.setDouble(error);
        OUTPUT_ENTRY.setDouble(rotationPower);
    }

    /**
     * Normalizes an angle to the range [-180, 180]
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees % 360;
        if (angle > 180) {
            angle -= 360;
        }
        if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || cancelCondition.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setDrivePowers(0, 0, 0);
    }
}
