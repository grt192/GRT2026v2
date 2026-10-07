// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.CANType;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AlignToTagCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem.SwerveModule;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;
import frc.robot.subsystems.swerve.GyroIOPigeon2Sim;
import frc.robot.subsystems.swerve.ModuleIOTalonFXSim;
import frc.robot.subsystems.swerve.SwerveDriveSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.TracerSentinel;
import java.util.function.Supplier;

/**
 * Vision demo: drive around with the sticks, hold the align button and the robot squares up to the
 * AprilTag it sees, standing further back the higher the tag is.
 */
public class RobotContainer {
    // Must be the first SubsystemBase constructed
    // Captures pre-subsystem scheduler overhead
    @SuppressWarnings("unused")
    private final TracerSentinel tracerSentinel = new TracerSentinel();

    private BaseDriveController driveController;
    private final LoggedCanivore swerveCan = new LoggedCanivore(CANType.SWERVE);

    private final DriveSubsystem swerveSubsystem;
    // Physics for the drivetrain, only in sim
    private SwerveDriveSim swerveDriveSim = null;

    private final VisionSubsystem vision;

    private final Alert driveControllerDisconnectedAlert = new Alert("Drive Controller Disconnected", AlertType.kWarning);

    public RobotContainer() {
        // Swerve is built first: the sim cameras need its physics
        swerveSubsystem = createSwerve();

        switch (Constants.CURRENT_MODE) {
            case REAL:
                vision = createVision(new VisionIOPhoton(VisionConstants.CAMERA_CONFIG));
                break;
            case SIM:
                // The camera sees the physics sim's true pose
                Supplier<Pose3d> truePose = swerveDriveSim::getPose3d;
                vision = createVision(new VisionIOPhotonSim(VisionConstants.CAMERA_CONFIG, truePose));
                break;
            case REPLAY:
            default:
                vision = createVision(new VisionIO() {});
                break;
        }

        // Move the simulated robot along with the odometry so they stay in agreement
        if (swerveDriveSim != null) {
            swerveSubsystem.setPoseResetListener((pose) -> swerveDriveSim.teleport(pose.toPose2d()));
            // The sim robot spawns at the starting pose, so start odometry there too
            swerveSubsystem.resetToStartingPosition();
        }

        constructController();
        configureBindings();
    }

    private DriveSubsystem createSwerve() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                return new DriveSubsystem(
                    new GyroIOPigeon2(swerveCan),
                    new ModuleIOTalonFX(SwerveModule.FL, SwerveConstants.FL_DRIVE, SwerveConstants.FL_STEER,
                        SwerveConstants.FL_ENCODER, SwerveConstants.FL_ENCODER_OFFSET_ROT, swerveCan),
                    new ModuleIOTalonFX(SwerveModule.FR, SwerveConstants.FR_DRIVE, SwerveConstants.FR_STEER,
                        SwerveConstants.FR_ENCODER, SwerveConstants.FR_ENCODER_OFFSET_ROT, swerveCan),
                    new ModuleIOTalonFX(SwerveModule.BL, SwerveConstants.BL_DRIVE, SwerveConstants.BL_STEER,
                        SwerveConstants.BL_ENCODER, SwerveConstants.BL_ENCODER_OFFSET_ROT, swerveCan),
                    new ModuleIOTalonFX(SwerveModule.BR, SwerveConstants.BR_DRIVE, SwerveConstants.BR_STEER,
                        SwerveConstants.BR_ENCODER, SwerveConstants.BR_ENCODER_OFFSET_ROT, swerveCan));
            case SIM:
                swerveDriveSim = new SwerveDriveSim(SwerveConstants.STARTING_POSE);
                DriveSubsystem simSwerve = new DriveSubsystem(
                    new GyroIOPigeon2Sim(swerveCan, swerveDriveSim),
                    new ModuleIOTalonFXSim(SwerveModule.FL, SwerveConstants.FL_DRIVE, SwerveConstants.FL_STEER,
                        SwerveConstants.FL_ENCODER, SwerveConstants.FL_ENCODER_OFFSET_ROT, swerveCan,
                        swerveDriveSim.getModule(SwerveModule.FL)),
                    new ModuleIOTalonFXSim(SwerveModule.FR, SwerveConstants.FR_DRIVE, SwerveConstants.FR_STEER,
                        SwerveConstants.FR_ENCODER, SwerveConstants.FR_ENCODER_OFFSET_ROT, swerveCan,
                        swerveDriveSim.getModule(SwerveModule.FR)),
                    new ModuleIOTalonFXSim(SwerveModule.BL, SwerveConstants.BL_DRIVE, SwerveConstants.BL_STEER,
                        SwerveConstants.BL_ENCODER, SwerveConstants.BL_ENCODER_OFFSET_ROT, swerveCan,
                        swerveDriveSim.getModule(SwerveModule.BL)),
                    new ModuleIOTalonFXSim(SwerveModule.BR, SwerveConstants.BR_DRIVE, SwerveConstants.BR_STEER,
                        SwerveConstants.BR_ENCODER, SwerveConstants.BR_ENCODER_OFFSET_ROT, swerveCan,
                        swerveDriveSim.getModule(SwerveModule.BR)));
                swerveDriveSim.start();
                return simSwerve;
            case REPLAY:
            default:
                return new DriveSubsystem(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
        }
    }

    private VisionSubsystem createVision(VisionIO io) {
        return new VisionSubsystem(
            swerveSubsystem::getRobotPose3d,
            new VisionCamera(io, VisionConstants.CAMERA_CONFIG));
    }

    /** Call from {@link Robot#simulationPeriodic()}. */
    public void simulationPeriodic() {
        if (swerveDriveSim != null) {
            Logger.recordOutput("Swerve/SimGroundTruthPose", swerveDriveSim.getPose3d());
        }
    }

    /** Update controller-connection alerts. Call from {@link Robot#robotPeriodic()}. */
    public void updateAlerts() {
        driveControllerDisconnectedAlert.set(!DriverStation.isJoystickConnected(0));
    }

    private void configureBindings() {
        // Left stick translates (field-relative), right stick rotates
        swerveSubsystem.setDefaultCommand(
            new RunCommand(() -> swerveSubsystem.setDrivePowers(
                driveController.getForwardPower(),
                driveController.getLeftPower(),
                driveController.getRotatePower()),
                swerveSubsystem));

        // Cross = reset the field axes to the current robot axes
        driveController.bindDriverHeadingReset(swerveSubsystem::resetDriverHeading, swerveSubsystem);

        // Triangle (hold) = align to the AprilTag in view
        driveController.getAlignToTag().whileTrue(new AlignToTagCommand(swerveSubsystem, vision));
    }

    /** A DualSense on port 0, on the robot and in sim */
    private void constructController() {
        driveController = new PS5DriveController();
        driveController.setDeadZone(ControllerConstants.PS5_STICK_DEADBAND);
    }
}
