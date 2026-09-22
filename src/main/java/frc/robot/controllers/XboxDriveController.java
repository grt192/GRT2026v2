package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A single Xbox controller on port 0.
 */
public class XboxDriveController extends BaseDriveController {

    private final CommandXboxController driveController = new CommandXboxController(0);
    private Trigger leftBumper = new Trigger(driveController.leftBumper());
    private Trigger rightBumper = new Trigger(driveController.rightBumper());
    private Trigger x = new Trigger(driveController.x());
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
        return x.getAsBoolean();
    }

    @Override
    public boolean getLeftBumper() {
        return leftBumper.getAsBoolean();
    }

    public Trigger getLeftBumperTrigger() {
        return leftBumper;
    }

    public Trigger getRightBumperTrigger() {
        return rightBumper;
    }

    @Override
    public boolean getRightBumper() {
        return rightBumper.getAsBoolean();
    }

    public boolean getRightTrigger() {
        return driveController.getRightTriggerAxis() > .1;
    }

    public boolean getLeftTrigger() {
        return driveController.getLeftTriggerAxis() > .1;
    }

    /**
     * Gets the raw left trigger axis value.
     *
     * @return Value from 0.0 (not pressed) to 1.0 (fully pressed)
     */
    public double getLeftTriggerAxis() {
        return driveController.getLeftTriggerAxis();
    }

    /**
     * Gets the raw right trigger axis value.
     *
     * @return Value from 0.0 (not pressed) to 1.0 (fully pressed)
     */
    public double getRightTriggerAxis() {
        return driveController.getRightTriggerAxis();
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

    public int getPOV() {
        return driveController.getHID().getPOV();
    }

    public Trigger y() {
        return driveController.y();
    }

    public Trigger b() {
        return driveController.b();
    }

    public CommandXboxController getController() {
        return driveController;
    }

    public Trigger start() {
        return driveController.start();
    }

    public Trigger x() {
        return x;
    }

    public Trigger back() {
        return driveController.back();
    }

    @Override
    public Trigger getAlignToReef() {
        return x;
    }

    @Override
    public Trigger getAlignToSource() {
        return driveController.a();
    }
}
