package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** The base class for a drive controller. Contains all needed methods for driving the robot */
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

    public abstract void bindDriverHeadingReset(
        Runnable command, Subsystem requiredSubsystem);

    public abstract void setDeadZone(double deadZone);
}
