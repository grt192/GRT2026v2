package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** The base class for a drive controller. Contains all needed methods for driving the robot (without mechs) */
public abstract class BaseDriveController {

    /**
     * Gets the forward power commanded by the controller.
     *
     * @return The [-1.0, 1.0] forward power.
     */
    public abstract double getForwardPower();

    /**
     * Gets the left power commanded by the controller.
     *
     * @return The [-1.0, 1.0] left power.
     */
    public abstract double getLeftPower();

    /**
     * Gets the rotational power commanded by the controller.
     *
     * @return The [-1.0, 1.0] angular power.
     */
    public abstract double getRotatePower();

    /**
     * Gets the button to reset the driver heading.
     *
     * @return The JoystickButton to reset the driver heading.
     */
    public abstract boolean getDriverHeadingResetButton();

    /**
     * Gets the left bumper or equivalent. Used in testSingleModuleSwerveSubsystem to move between tests.
     *
     * @return The JoystickButton of the left bumper or equivalent.
     */
    public abstract boolean getLeftBumper();

    /**
     * Gets the right bumper or equivalent. Used in testSingleModuleSwerveSubsystem to move between tests.
     *
     * @return The JoystickButton of the right bumper or equivalent.
     */
    public abstract boolean getRightBumper();

    // public abstract boolean getAlignToReef();

    /**
     * Gets the left trigger axis.
     *
     * @return Value from 0.0 (not pressed) to 1.0 (fully pressed)
     */
    public abstract double getLeftTriggerAxis();

    /**
     * Gets the right trigger axis.
     *
     * @return Value from 0.0 (not pressed) to 1.0 (fully pressed)
     */
    public abstract double getRightTriggerAxis();

    /** Held to auto-rotate toward the hub. */
    public abstract Trigger getAimToHub();

    /** Held to hold the hood down. */
    public abstract Trigger getHoldHoodDown();

    /** Held to force the intake in (pivot up + rollers stopped). */
    public abstract Trigger getForceIntakeIn();

    /** Pressed to reset the pose to the starting position. */
    public abstract Trigger getResetPose();

    public abstract void bindDriverHeadingReset(
        Runnable command, Subsystem requiredSubsystem);

    public abstract void setDeadZone(double deadZone);

    public abstract Trigger getAlignToReef();

    public abstract Trigger getAlignToSource();

}
