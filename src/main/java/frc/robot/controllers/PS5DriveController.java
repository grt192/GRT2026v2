package frc.robot.controllers;

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
        double forwardPower = -driveController.getLeftY();
        if (Math.abs(forwardPower) > deadZone) {
            return -driveController.getLeftY();
        } else {
            return 0;
        }
    }

    @Override
    public double getLeftPower() {
        double leftPower = -driveController.getLeftX();
        if (Math.abs(leftPower) > deadZone) {
            return -driveController.getLeftX();
        } else {
            return 0;
        }
    }

    @Override
    public double getRotatePower() {
        double rotatePower = -driveController.getRightX();
        if (Math.abs(rotatePower) > deadZone) {
            return -driveController.getRightX();
        } else {
            return 0;
        }
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
