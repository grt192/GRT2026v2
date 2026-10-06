// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;
import com.ctre.phoenix6.SignalLogger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.util.GatedAlert;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.LoggedTracer;
import frc.robot.util.PhoenixUtil;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    private final RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        // Record metadata
        // BUILDCONSTANTS IS GENERATED DURING BUILD
        // IF THIS IS THROWING AN ERROR JUST BUILD
        Logger.recordMetadata("BuildType", Constants.CURRENT_MODE.toString());
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
            "GitDirty",
            switch (BuildConstants.DIRTY) {
                case 0 -> "All changes committed";
                case 1 -> "Uncommitted changes";
                default -> "Unknown";
            });

        switch (Constants.CURRENT_MODE) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replayed"), AdvantageScopeOpenBehavior.AUTO));
                break;
            default:
                break;
        }

        Logger.start();
        SignalLogger.enableAutoLogging(false);

        DriverStation.silenceJoystickConnectionWarning(true);

        // Instantiate our RobotContainer. This will perform all our button bindings.
        robotContainer = new RobotContainer();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        robotContainer.updateAlerts();

        LoggedTracer.reset();

        PhoenixUtil.refreshAllStatusSignals();
        LoggedTracer.record("PhoenixRefresh");

        CommandScheduler.getInstance().run();
        LoggedTracer.record("Commands");

        LoggedCanivore.updateCanivoreStatuses();
        LoggedTracer.record("Canivore");

        GatedAlert.pushAll();
        LoggedTracer.record("Alerts");
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        robotContainer.simulationPeriodic();
    }
}
