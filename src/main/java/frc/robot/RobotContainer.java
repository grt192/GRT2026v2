// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.CANType;
import frc.robot.Constants.CycleShooterConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ShooterConstants.Flywheel;
import frc.robot.Constants.ShooterConstants.Hood;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.CycleShot;
import frc.robot.commands.SmashShot;
import frc.robot.commands.TowerShot;
import frc.robot.commands.auton.ShootAndLeaveAuton;
import frc.robot.commands.intake.PivotAndRollerIntakeCommand;
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.hopper.HopperIOTalonFXSim;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFX;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFXSim;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOTalonFX;
import frc.robot.subsystems.intake.roller.RollerIOTalonFXSim;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFXSim;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFXSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.tower.TowerIO;
import frc.robot.subsystems.shooter.tower.TowerIOTalonFX;
import frc.robot.subsystems.shooter.tower.TowerIOTalonFXSim;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.swerve.AimSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem.SwerveModule;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOPigeon2Sim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;
import frc.robot.subsystems.swerve.ModuleIOTalonFXSim;
import frc.robot.subsystems.swerve.SwerveDriveSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.TimestampedVisionUpdate;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.TracerSentinel;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Must be the first SubsystemBase constructed
    // Captures pre-subsystem scheduler overhead
    @SuppressWarnings("unused")
    private final TracerSentinel tracerSentinel = new TracerSentinel();

    private double cycleFlywheelVelo = CycleShooterConstants.FLYWHEEL_VELO_RPS;
    private DoubleSupplier cycleFlywheelOffsetGetter = () -> (cycleFlywheelVelo - CycleShooterConstants.FLYWHEEL_VELO_RPS);

    private static final double CYCLE_FLYWHEEL_VELO_STEP_RPS = 5.0; // rotations per second
    private static final double CYCLE_HOOD_POS_STEP_ROT = 0.01; // rotations
    private double cycleHoodPos = CycleShooterConstants.HOOD_POSITION_ROT;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private PS5DriveController driveController;
    private final LoggedCanivore swerveCan = new LoggedCanivore(CANType.SWERVE);
    private final LoggedCanivore mechCan = new LoggedCanivore(CANType.MECH);

    // Physics for the drivetrain, only in sim. No initializer: createSwerve() sets it during the
    // swerveSubsystem initializer below, and an initializer here would run after and wipe it
    private SwerveDriveSim swerveDriveSim;
    private final DriveSubsystem swerveSubsystem = Constants.SWERVE_ENABLED ? createSwerve() : null;
    private final FieldManagementSubsystem fmsSubsystem = new FieldManagementSubsystem(cycleFlywheelOffsetGetter);
    private final Field2d field = new Field2d();

    private final PivotSubsystem pivot;
    private final RollerSubsystem roller;
    private final HopperSubsystem hopper;
    private final TowerSubsystem tower;
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    @SuppressWarnings("unused")
    private final AimSubsystem aimSubsystem =
        (Constants.SWERVE_ENABLED && swerveSubsystem != null)
            ? new AimSubsystem(swerveSubsystem, fmsSubsystem)
            : null;

    private final VisionSubsystem vision;
    private UsbCamera driverCam;

    private final Alert controllerDisconnectedAlert = new Alert("Controller Disconnected", AlertType.kWarning);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                pivot = new PivotSubsystem(new PivotIOTalonFX(mechCan));
                roller = new RollerSubsystem(new RollerIOTalonFX(mechCan));
                hopper = new HopperSubsystem(new HopperIOTalonFX(mechCan));
                tower = new TowerSubsystem(new TowerIOTalonFX(mechCan));
                flywheel = new FlywheelSubsystem(new FlywheelIOTalonFX(mechCan));
                hood = new HoodSubsystem(new HoodIOTalonFX(mechCan));
                vision = createVision(
                    new VisionIOPhoton(VisionConstants.CAMERA_CONFIG_1),
                    new VisionIOPhoton(VisionConstants.CAMERA_CONFIG_2),
                    new VisionIOPhoton(VisionConstants.CAMERA_CONFIG_3));
                break;
            case SIM:
                pivot = new PivotSubsystem(new PivotIOTalonFXSim(mechCan));
                roller = new RollerSubsystem(new RollerIOTalonFXSim(mechCan));
                hopper = new HopperSubsystem(new HopperIOTalonFXSim(mechCan));
                tower = new TowerSubsystem(new TowerIOTalonFXSim(mechCan));
                flywheel = new FlywheelSubsystem(new FlywheelIOTalonFXSim(mechCan));
                hood = new HoodSubsystem(new HoodIOTalonFXSim(mechCan));
                vision = createVision(new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
                break;
            case REPLAY:
            default:
                pivot = new PivotSubsystem(new PivotIO() {});
                roller = new RollerSubsystem(new RollerIO() {});
                hopper = new HopperSubsystem(new HopperIO() {});
                tower = new TowerSubsystem(new TowerIO() {});
                flywheel = new FlywheelSubsystem(new FlywheelIO() {});
                hood = new HoodSubsystem(new HoodIO() {});
                vision = createVision(new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
                break;
        }
        if (Constants.SWERVE_ENABLED && swerveSubsystem != null) {
            swerveSubsystem.setPoseResetListener((pose) -> {
                vision.resetHeading(Timer.getTimestamp(), pose.toPose2d().getRotation());
                // Move the simulated robot along with the odometry so they stay in agreement
                if (swerveDriveSim != null) {
                    swerveDriveSim.teleport(pose.toPose2d());
                }
            });
            // The sim robot spawns at the starting pose, so start odometry there too
            if (swerveDriveSim != null) {
                swerveSubsystem.resetToStartingPosition();
            }
        }
        constructController();
        configureBindings();
        configureAutoChooser();

        if (Constants.CURRENT_MODE == Mode.REAL) {
            // Driver cam — publishes to NT at /CameraPublisher/Driver Cam/streams
            driverCam = CameraServer.startAutomaticCapture("Driver Cam", 0);
            driverCam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

            // Throttle what actually streams to the dashboard (the source-side FPS
            // is just a hint — Arducams often ignore it). These caps are authoritative.
            MjpegServer driverCamServer = (MjpegServer) CameraServer.getServer("serve_Driver Cam");
            driverCamServer.setResolution(320, 240);
            driverCamServer.setFPS(20);
            driverCamServer.setCompression(50); // 0 = worst, 100 = best; lower = less bandwidth
        }

        SmartDashboard.putData("Field", field);
        NamedCommands.registerCommand("deployIntake", pivot.deployPivot());
        NamedCommands.registerCommand("runRollers", roller.runRollerIn());
        NamedCommands.registerCommand("pivotAndRollerIntake", new PivotAndRollerIntakeCommand(pivot, roller));
        NamedCommands.registerCommand("pivotdownandrunrollers", new PivotAndRollerIntakeCommand(pivot, roller));
        NamedCommands.registerCommand("shootSequence", new AutonShooterSequence(flywheel, hood, tower, hopper, pivot));
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

    /** Call from {@link Robot#simulationPeriodic()}. */
    public void simulationPeriodic() {
        if (swerveDriveSim != null) {
            Logger.recordOutput("Swerve/SimGroundTruthPose", swerveDriveSim.getPose3d());
        }
    }

    private VisionSubsystem createVision(VisionIO io1, VisionIO io2, VisionIO io3) {
        boolean hasSwerve = Constants.SWERVE_ENABLED && swerveSubsystem != null;
        Consumer<TimestampedVisionUpdate> visionConsumer = hasSwerve
            ? swerveSubsystem::addVisionMeasurements
            : (update) -> {
            };
        Supplier<Rotation2d> visionHeadingSupplier = hasSwerve
            ? () -> swerveSubsystem.getRobotPosition().getRotation()
            : () -> Rotation2d.kZero;

        return new VisionSubsystem(
            visionConsumer,
            visionHeadingSupplier,
            new VisionCamera(io1, VisionConstants.CAMERA_CONFIG_1),
            new VisionCamera(io2, VisionConstants.CAMERA_CONFIG_2),
            new VisionCamera(io3, VisionConstants.CAMERA_CONFIG_3));
    }

    /** Update controller-connection alerts. Call from {@link Robot#robotPeriodic()}. */
    public void updateAlerts() {
        controllerDisconnectedAlert.set(!DriverStation.isJoystickConnected(0));
    }

    /**
     * One PS5 controller on port 0 drives the swerve and runs every mech, with
     * live cycle-shot tuning on the D-pad.
     */
    private void configureBindings() {
        if (Constants.SWERVE_ENABLED && swerveSubsystem != null) {
            // Left stick = translate, right stick = rotate (field-relative)
            swerveSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    swerveSubsystem.setDrivePowers(
                        driveController.getForwardPower(),
                        driveController.getLeftPower(),
                        driveController.getRotatePower());
                }, swerveSubsystem));

            // Cross = reset the field axes to the current robot axes
            driveController.bindDriverHeadingReset(
                () -> {
                    swerveSubsystem.resetDriverHeading();
                }, swerveSubsystem);

            // Options = reset pose to starting position (in front of red hub)
            driveController.options()
                .onTrue(Commands.runOnce(() -> swerveSubsystem.resetToStartingPosition(), swerveSubsystem));
        }
        if (Constants.MECH_ENABLED) {
            CommandPS5Controller controller = driveController.getController();

            // R1 (hold) = cycle shot at the live-tuned flywheel velocity and hood position
            controller.R1().whileTrue(
                new CycleShot(flywheel, hood, tower, hopper, pivot, () -> cycleFlywheelVelo, () -> cycleHoodPos));
            // R2 (hold) = roller out
            controller.R2().whileTrue(roller.runRollerOut());

            // L1 = toggle pivot, L2 (hold) = roller in
            controller.L1().toggleOnTrue(pivot.togglePivot());
            controller.L2().whileTrue(roller.runRollerIn());

            // Triangle = smash shot (toggle), Circle = tower shot (toggle)
            controller.triangle().toggleOnTrue(new SmashShot(flywheel, hood, tower, hopper, pivot));
            controller.circle().toggleOnTrue(new TowerShot(flywheel, hood, tower, hopper, pivot));

            // Square = jiggle pivot and hood (toggle)
            controller.square().toggleOnTrue(Commands.parallel(pivot.jigglePivot(), hood.jiggleHood()));

            // D-pad up/down = cycle flywheel velocity, right/left = cycle hood position
            publishCycleTuning();
            controller.povUp().onTrue(
                Commands.runOnce(() -> adjustCycleFlywheelVelo(CYCLE_FLYWHEEL_VELO_STEP_RPS)));
            controller.povDown().onTrue(
                Commands.runOnce(() -> adjustCycleFlywheelVelo(-CYCLE_FLYWHEEL_VELO_STEP_RPS)));
            controller.povRight().onTrue(
                Commands.runOnce(() -> adjustCycleHoodPos(CYCLE_HOOD_POS_STEP_ROT)));
            controller.povLeft().onTrue(
                Commands.runOnce(() -> adjustCycleHoodPos(-CYCLE_HOOD_POS_STEP_ROT)));

            roller.setDefaultCommand(roller.stopRoller());
            hopper.setDefaultCommand(hopper.stopHopper());
            tower.setDefaultCommand(tower.stopTower());
        }
    }

    private void adjustCycleFlywheelVelo(double deltaRotationsPerSecond) {
        cycleFlywheelVelo = MathUtil.clamp(
            cycleFlywheelVelo + deltaRotationsPerSecond,
            0.0,
            Flywheel.FLYWHEEL_MAX_SPEED_RPS);
        publishCycleTuning();
    }

    private void adjustCycleHoodPos(double deltaRotations) {
        cycleHoodPos = MathUtil.clamp(
            cycleHoodPos + deltaRotations,
            Hood.LOWER_ANGLE_LIMIT_ROT,
            Hood.UPPER_ANGLE_LIMIT_ROT);
        publishCycleTuning();
    }

    /** Publishes the live cycle-shot offsets so the operator can see what they are tuning. */
    private void publishCycleTuning() {
        Logger.recordOutput("CycleShot/FlywheelVeloRPS", cycleFlywheelVelo - CycleShooterConstants.FLYWHEEL_VELO_RPS);
        Logger.recordOutput("CycleShot/HoodPosRotations", cycleHoodPos - CycleShooterConstants.HOOD_POSITION_ROT);
    }

    /** Constructs the single PS5 controller on port 0. */
    private void constructController() {
        driveController = new PS5DriveController();
        driveController.setDeadZone(0.035);
    }

    /**
     * Config the autonomous command chooser
     */
    private void configureAutoChooser() {
        // Add auton here
        autoChooser.setDefaultOption("Do Nothing", null);

        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return new ShootAndLeaveAuton(swerveSubsystem, flywheel, hood, hopper, tower, pivot, roller);

        // return new ToDepotAndShoot(flywheel, hoodSubsystem, tower, hopper, pivotIntake, intakeSubsystem, learner);
        // return new PathPlannerAuto("90degturn");

        // rehturn new PathPlannerAuto("swerve90");

        // return new CNeutralIntakeTOWERAuton(flywheel, hoodSubsystem, tower, hopper, pivotIntake, intakeSubsystem, learner);
        // return new PathPlannerAuto("auton2");
        // return new NeutralDefenseAuton(flywheel, hoodSubsystem, tower, hopper, pivotIntake, intakeSubsystem);
        // return new ANeutralIntakeAuton(flywheel, hoodSubsystem, tower, hopper, pivotIntake, intakeSubsystem);
        // return new PathPlannerAuto("auton1");

        // Run ManualShooterSequence for 10 seconds
        // return new AutonShooterSequence(
        // flywheel,
        // hoodSubsystem,
        // tower,
        // hopper,
        // pivotIntake).withTimeout(10);

    }

    /**
     * Called when teleop starts to reset driver heading with 90 degree offset.
     */
    public void onTeleopInit() {
        if (swerveSubsystem != null) {
            swerveSubsystem.resetDriverHeadingOffset90();
        }
    }

    /**
     * Called when autonomous starts to zero the pivot encoder.
     */
    public void onAutonInit() {
        // pivotIntake.zeroEncoder();
    }
    // return new ShootAndLeaveAuton(swerveSubsystem, flywheel, hoodSubsystem, hopper, tower, pivotIntake);
    // }


}
