package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5DriveController extends BaseDriveController {

    private final CommandPS5Controller driveController = new CommandPS5Controller(0);
    private Trigger cross = new Trigger(driveController.cross());
    private double deadZone = 0;

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
        return cross.getAsBoolean();
    }

    @Override
    public Trigger getAlignToTag() {
        return driveController.triangle();
    }

    @Override
    public void bindDriverHeadingReset(
        Runnable command, Subsystem requiredSubsystem) {
        InstantCommand instantCommand = new InstantCommand(
            command,
            requiredSubsystem);
        new Trigger(this::getDriverHeadingResetButton).onTrue(instantCommand);
    }

    @Override
    public void setDeadZone(double deadZone) {
        this.deadZone = deadZone;
    }
}
