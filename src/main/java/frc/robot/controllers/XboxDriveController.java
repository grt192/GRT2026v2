package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.PS5ControllerEmulator;

/**
 * A single Xbox controller on port 0. Buttons sit in the same spots as on {@link PS5DriveController}
 * (A = cross, X = square, Y = triangle, Start = options, LT = L2).
 *
 * <p>
 * Reads through {@link PS5ControllerEmulator}, which handles macOS reporting the Xbox axes in a
 * different order than WPILib's XboxController expects (plain XboxController reads the resting
 * right trigger, -1, as the right stick, so the robot spins at full speed).
 */
public class XboxDriveController extends BaseDriveController {

    private final PS5ControllerEmulator driveController = new PS5ControllerEmulator(0);
    private final Trigger leftBumper = driveController.L1();
    private final Trigger rightBumper = driveController.R1();
    private final Trigger a = driveController.cross();
    private final Trigger x = driveController.square();
    private double deadZone = 0;

    // Deadbands are rescaled, so output ramps up from 0 at the edge of the deadband instead of jumping

    @Override
    public double getForwardPower() {
        return MathUtil.applyDeadband(-driveController.getLeftY(), deadZone);
    }

    @Override
    public double getLeftPower() {
        return MathUtil.applyDeadband(-driveController.getLeftX(), deadZone);
    }

    @Override
    public double getRotatePower() {
        return MathUtil.applyDeadband(-driveController.getRightX(), deadZone);
    }

    @Override
    public boolean getDriverHeadingResetButton() {
        return a.getAsBoolean();
    }

    @Override
    public boolean getLeftBumper() {
        return leftBumper.getAsBoolean();
    }

    @Override
    public boolean getRightBumper() {
        return rightBumper.getAsBoolean();
    }

    @Override
    public double getLeftTriggerAxis() {
        return driveController.getL2Axis();
    }

    @Override
    public double getRightTriggerAxis() {
        return driveController.getR2Axis();
    }

    @Override
    public Trigger getAimToHub() {
        return driveController.triangle();
    }

    @Override
    public Trigger getHoldHoodDown() {
        return driveController.L2();
    }

    @Override
    public Trigger getForceIntakeIn() {
        return x;
    }

    @Override
    public Trigger getResetPose() {
        return driveController.options();
    }

    @Override
    public void bindDriverHeadingReset(
        Runnable command, Subsystem requiredSubsystem) {
        a.onTrue(new InstantCommand(command, requiredSubsystem));
    }

    @Override
    public void setDeadZone(double deadZone) {
        this.deadZone = deadZone;
    }

    @Override
    public Trigger getAlignToReef() {
        return x;
    }

    @Override
    public Trigger getAlignToSource() {
        return a;
    }
}
