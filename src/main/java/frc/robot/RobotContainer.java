// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANType;
import frc.robot.Constants.CycleShooterConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ShooterConstants.Flywheel;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.CycleShot;
import frc.robot.commands.SmashShot;
import frc.robot.commands.TowerShot;
import frc.robot.commands.allign.AimToHubCommand;
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
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.TracerSentinel;
import java.util.function.DoubleSupplier;

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

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private PS5DriveController driveController;
    private CommandPS5Controller mechController;
    private final LoggedCanivore swerveCan = new LoggedCanivore(CANType.SWERVE);
    private final LoggedCanivore mechCan = new LoggedCanivore(CANType.MECH);

    private SwerveSubsystem swerveSubsystem = Constants.SWERVE_ENABLED ? new SwerveSubsystem(swerveCan) : null;
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
    private final AimToHubCommand aimHelper =
        (Constants.SWERVE_ENABLED && swerveSubsystem != null)
            ? new AimToHubCommand(swerveSubsystem, fmsSubsystem)
            : null;

    // private final FuelDetectionSubsystem fuelDetectionSubsystem = new FuelDetectionSubsystem(VisionConstants.FUEL_DETECTION_CONFIG);

    private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
        VisionConstants.CAMERA_CONFIG_1);
    private final VisionSubsystem visionSubsystem2 = new VisionSubsystem(
        VisionConstants.CAMERA_CONFIG_2);
    private final VisionSubsystem visionSubsystem3 = new VisionSubsystem(
        VisionConstants.CAMERA_CONFIG_3);
    private UsbCamera driverCam;

    private double desiredHoodSpeed = 0;
    // private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
    // VisionConstants.CAMERA_CONFIG_11);

    private final Alert driveControllerDisconnectedAlert = new Alert("Drive Controller Disconnected", AlertType.kWarning);
    private final Alert mechControllerDisconnectedAlert = new Alert("Mech Controller Disconnected", AlertType.kWarning);

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
                break;
            case SIM:
                pivot = new PivotSubsystem(new PivotIOTalonFXSim(mechCan));
                roller = new RollerSubsystem(new RollerIOTalonFXSim(mechCan));
                hopper = new HopperSubsystem(new HopperIOTalonFXSim(mechCan));
                tower = new TowerSubsystem(new TowerIOTalonFXSim(mechCan));
                flywheel = new FlywheelSubsystem(new FlywheelIOTalonFXSim(mechCan));
                hood = new HoodSubsystem(new HoodIOTalonFXSim(mechCan));
                break;
            case REPLAY:
            default:
                pivot = new PivotSubsystem(new PivotIO() {});
                roller = new RollerSubsystem(new RollerIO() {});
                hopper = new HopperSubsystem(new HopperIO() {});
                tower = new TowerSubsystem(new TowerIO() {});
                flywheel = new FlywheelSubsystem(new FlywheelIO() {});
                hood = new HoodSubsystem(new HoodIO() {});
                break;
        }
        visionStuff();
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

    /** Update controller-connection alerts. Call from {@link Robot#robotPeriodic()}. */
    public void updateAlerts() {
        driveControllerDisconnectedAlert.set(!DriverStation.isJoystickConnected(0));
        mechControllerDisconnectedAlert.set(!DriverStation.isJoystickConnected(mechController.getHID().getPort()));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named f`actories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */

    private void configureBindings() {
        /*
         * Driving -- One joystick controls translation, the other rotation. If the
         * robot-relative button is held down,
         * the robot is controlled along its own axes, otherwise controls apply to the
         * field axes by default. If the
         * swerve aim button is held down, the robot will rotate automatically to always
         * face a target, and only
         * translation will be manually controllable.
         */
        if (Constants.SWERVE_ENABLED && swerveSubsystem != null) {
            swerveSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    // R2 (>= 90% pressed) = boost mode
                    swerveSubsystem.setBoostMode(driveController.getRightTriggerAxis() > 0.9);

                    // L1 (hold) = robot-relative drive; release returns to field-relative
                    swerveSubsystem.setRobotRelative(driveController.getLeftBumper());

                    // R1 = slow mode (30% speed), L2 = variable speed limit
                    double speedLimit;
                    if (driveController.getRightBumper()) {
                        speedLimit = Constants.SwerveConstants.SLOW_MODE_SPEED_LIMIT;
                    } else {
                        double leftTrigger = driveController.getLeftTriggerAxis();
                        speedLimit = 1.0 - leftTrigger;
                    }
                    swerveSubsystem.setDriveSpeedLimit(speedLimit);

                    swerveSubsystem.setDrivePowers(
                        driveController.getForwardPower(),
                        driveController.getLeftPower(),
                        driveController.getRotatePower());
                },
                    swerveSubsystem));

            // Create button = switch cameras
            // driveController.create().onTrue(
            // Commands.runOnce(() -> {
            // if (isCamera1Active) {
            // cameraServer.setSource(camera2);
            // } else {
            // cameraServer.setSource(camera1);
            // }
            // isCamera1Active = !isCamera1Active;
            // }));

            /* Pressing the button resets the field axes to the current robot axes. */
            driveController.bindDriverHeadingReset(
                () -> {
                    swerveSubsystem.resetDriverHeading();
                },
                swerveSubsystem);

            // Triangle (drive) = auto-rotate to face the hub. Held-while-true; rotating
            // the right stick past the deadband cancels it so the driver can override.
            // Wrapped in Commands.defer so the target angle is recomputed at every press.
            if (aimHelper != null) {
                driveController.getController().triangle().whileTrue(
                    Commands.defer(
                        () -> aimHelper.createAimCommand(() -> false),
                        java.util.Set.of(swerveSubsystem)));
            }

            driveController.getController().L2().whileTrue(hood.holdDownHood());
        }
        if (Constants.MECH_ENABLED) {
            // ==================== INTAKE ROLLER ====================
            // R1 (mech) = intake out, L1 (mech) = intake in (duty cycle control)
            mechController.L1().whileTrue(roller.runRollerIn());
            mechController.R1().whileTrue(roller.runRollerOut());
            roller.setDefaultCommand(roller.stopRoller());

            // ==================== INTAKE PIVOT ====================
            // D-pad left = pivot down (timed), D-pad right = pivot up (timed)
            mechController.povLeft().whileTrue(pivot.deployPivot());
            mechController.povRight().whileTrue(pivot.retractPivot());

            // L2 (mech) = spin spindexer (hopper) at max RPM and tower at full duty cycle
            mechController.L2().whileTrue(Commands.run(() -> {
                hopper.setDutyCycle(-1.0); // Max duty cycle for spindexer
                tower.setDutyCycle(1.0); // Full duty cycle for tower
            }, hopper, tower));
            hopper.setDefaultCommand(hopper.stopHopper());

            // Square (drive) = emergency force intake in (pivot up + stop rollers) - hold to override
            driveController.square()
                .whileTrue(Commands.parallel(pivot.retractPivot(), roller.stopRoller()));

            // ==================== INTAKE PIVOT ====================
            // Right stick Y controls pivot manually
            /*
             * pivotIntake.setDefaultCommand(Commands.run(() -> {
             * double pivotInput = -mechController.getRightY();
             * if (Math.abs(pivotInput) > 0.1) {
             * pivotIntake.setManualSpeed(pivotInput * 0.3);
             * } else {
             * pivotIntake.stop();
             * }
             * }, pivotIntake));
             */

            // ==================== SHOOTING PRESETS ====================
            // Square (mech) = smash-and-shoot preset (close-range)
            mechController.square().whileTrue(
                Commands.defer(
                    () -> new SmashShot(
                        flywheel,
                        hood,
                        tower,
                        hopper,
                        pivot),
                    java.util.Set.of(
                        flywheel,
                        hood,
                        hopper,
                        tower,
                        pivot)));

            // ==================== INTERPOLATION TABLE CALIBRATION ====================
            // D-pad up/down: bump flywheel RPM offset (5 RPS per press)
            // Triangle/Circle: bump hood angle offset (0.005 rotations per press)
            // Options: log current (distance, rpm, angle) to riolog
            // Share: reset both offsets to zero
            // mechController.povUp().onTrue(CalibrationCommands.rpmUp(learner));
            // mechController.povDown().onTrue(CalibrationCommands.rpmDown(learner));
            // mechController.triangle().onTrue(CalibrationCommands.hoodUp(learner));
            // mechController.circle().onTrue(CalibrationCommands.hoodDown(learner));
            // mechController.create().onTrue(CalibrationCommands.resetOffsets(learner));
            // if (aimSubsystem != null) {
            // mechController.options().onTrue(CalibrationCommands.logPoint(learner, aimSubsystem));
            // }

            // Joystick movement cancels it
            // Trigger joystickMoved = new Trigger(() -> Math.abs(driveController.getForwardPower()) > 0.1 ||
            // Math.abs(driveController.getLeftPower()) > 0.1 ||
            // Math.abs(driveController.getRotatePower()) > 0.1);
            // joystickMoved.onTrue(Commands.runOnce(() -> manualShooterCmd.cancel()));

            // ==================== SHOOTER ====================
            // R2 = flywheel (analog speed control)
            // Left stick Y = hood manual control
            flywheel.setDefaultCommand(Commands.run(() -> {
                if (!DriverStation.isJoystickConnected(1)) {
                    flywheel.stop();
                    return;
                }

                // R2 rests at -1 and reads 1 fully pressed, so shift it into [0, 2/3]
                double flywheelManualVeloCommand = (mechController.getR2Axis() + 1) / 3;

                // Closed loop holding 0 fights the flywheel, so stop it outright instead
                if (Math.abs(flywheelManualVeloCommand) > 1e-6) {
                    flywheel.setVelocity(flywheelManualVeloCommand);
                } else {
                    flywheel.stop();
                }
            }, flywheel));

            tower.setDefaultCommand(Commands.run(() -> {
                tower.stop(); // Stop tower by default
            }, tower));

            hood.setDefaultCommand(Commands.run(() -> {
                if (mechController.L3().getAsBoolean()) {
                    desiredHoodSpeed = 0.15;
                    hood.setDutyCycle(0.15);
                } else if (mechController.R3().getAsBoolean()) {
                    desiredHoodSpeed = -0.15;
                    hood.setDutyCycle(-0.15);
                } else {
                    if (desiredHoodSpeed != 0) {
                        desiredHoodSpeed = 0;
                        hood.stop();

                    }
                }
            }, hood));

            mechController.povUp().onTrue(Commands.runOnce(() -> {
                if (Flywheel.FLYWHEEL_MAX_SPEED_RPS > cycleFlywheelVelo) {
                    cycleFlywheelVelo += 5;
                }
            }));

            mechController.povDown().onTrue(Commands.runOnce(() -> {
                if (cycleFlywheelVelo > 0) {
                    cycleFlywheelVelo -= 5;
                }
            }));

            // Cross = passing shot — flywheel pinned to max (120 RPS)
            mechController.cross().whileTrue(new CycleShot(
                flywheel,
                hood,
                tower,
                hopper,
                () -> cycleFlywheelVelo));

            // Touchpad = tower shoot preset
            mechController.triangle().whileTrue(new TowerShot(
                flywheel,
                hood,
                tower,
                hopper,
                pivot));

            // Swerve-dependent drive controller commands
            if (Constants.SWERVE_ENABLED && swerveSubsystem != null) {
                // Options button = reset pose to starting position (in front of red hub)
                driveController.options()
                    .onTrue(Commands.runOnce(() -> swerveSubsystem.resetToStartingPosition(), swerveSubsystem));
            }
        }

    }

    /**
     * Constructs the drive controller based on the name of the controller at port
     * 0
     */
    private void constructController() {
        driveController = new PS5DriveController();
        driveController.setDeadZone(0.035);
        mechController = new CommandPS5Controller(1);
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

    // vision shit
    public void visionStuff() {
        visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);
        visionSubsystem2.setInterface(swerveSubsystem::addVisionMeasurements);
        visionSubsystem3.setInterface(swerveSubsystem::addVisionMeasurements);

        // CommandScheduler.getInstance().schedule(
        // new GetCameraDisplacement(visionSubsystem1,
        // new Transform3d(
        // Units.inchesToMeters(0),
        // Units.inchesToMeters(-43 - 15),
        // Units.inchesToMeters(44.25),
        // new Rotation3d(0, 0, Math.PI / 2))));

    }

}
