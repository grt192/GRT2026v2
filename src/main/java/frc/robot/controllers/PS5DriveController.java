package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;

public class PS5DriveController extends BaseDriveController {

    private final CommandPS5Controller driveController = new CommandPS5Controller(0);
    private Trigger cross = new Trigger(driveController.cross());
    private double deadZone = 0;

    @Override
    public double getForwardPower() {
        return getTranslation().getX();
    }

    @Override
    public double getLeftPower() {
        return getTranslation().getY();
    }

    @Override
    public double getRotatePower() {
        double rotatePower = MathUtil.applyDeadband(-driveController.getRightX(), deadZone);
        return Math.copySign(Math.pow(Math.abs(rotatePower), ControllerConstants.ROTATION_STICK_EXPONENT), rotatePower);
    }

    /**
     * The left stick as (forward, left) with the deadband and power curve applied to its magnitude,
     * so the stick's direction is kept. A per-axis deadband would snap near-diagonal pushes onto an
     * axis and clip small diagonal pushes entirely.
     */
    private Translation2d getTranslation() {
        Translation2d stick = new Translation2d(-driveController.getLeftY(), -driveController.getLeftX());
        // The stick's corners read past 1, which would let diagonals outrun straight lines
        double magnitude = Math.min(stick.getNorm(), 1.0);
        // Rescales so the output ramps up from 0 at the deadband edge instead of jumping
        magnitude = MathUtil.applyDeadband(magnitude, deadZone);
        if (magnitude == 0.0) {
            return Translation2d.kZero;
        }
        magnitude = Math.pow(magnitude, ControllerConstants.TRANSLATION_STICK_EXPONENT);
        return new Translation2d(magnitude, stick.getAngle());
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
