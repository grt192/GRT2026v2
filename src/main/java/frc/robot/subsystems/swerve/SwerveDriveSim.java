package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.dyn4j.dynamics.Body;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveSimConstants;
import frc.robot.Constants.SwerveSteerConstants;
import frc.robot.subsystems.swerve.DriveSubsystem.SwerveModule;

/**
 * maple-sim physics for the drivetrain: robot mass, wheel grip, and collisions with the field.
 * The sim module and gyro IOs feed their CTRE sim states from it, so the real IO code runs
 * unchanged on top. Based on 254's 2025 MapleSimSwerveDrivetrain.
 *
 * <p>
 * Construct it, build the sim IOs from it, then call {@link #start()}.
 */
public class SwerveDriveSim {
    private final TeleportableArena arena = new TeleportableArena();
    private final SwerveDriveSimulation driveSimulation;
    private final Notifier notifier = new Notifier(() -> arena.simulationPeriodic());

    public SwerveDriveSim(Pose2d startingPose) {
        // One physics sub-tick per 5 ms notifier call
        SimulatedArena.overrideSimulationTimings(Seconds.of(SwerveSimConstants.PERIOD_SECONDS), 1);
        SimulatedArena.overrideInstance(arena);

        DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(SwerveSimConstants.ROBOT_MASS_KG))
            .withBumperSize(
                Meters.of(SwerveSimConstants.BUMPER_LENGTH_X_METERS),
                Meters.of(SwerveSimConstants.BUMPER_WIDTH_Y_METERS))
            .withGyro(COTS.ofPigeon2())
            .withCustomModuleTranslations(new Translation2d[] {
                    SwerveConstants.FL_POS, SwerveConstants.FR_POS, SwerveConstants.BL_POS, SwerveConstants.BR_POS
            })
            .withSwerveModule(new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX44Foc(1),
                SwerveDriveConstants.DRIVE_GEAR_REDUCTION,
                SwerveSteerConstants.STEER_GEAR_REDUCTION,
                Volts.of(SwerveSimConstants.DRIVE_FRICTION_VOLTS),
                Volts.of(SwerveSimConstants.STEER_FRICTION_VOLTS),
                Meters.of(SwerveDriveConstants.DRIVE_WHEEL_RADIUS_METERS),
                KilogramSquareMeters.of(SwerveSimConstants.STEER_MOMENT_OF_INERTIA_KG_M2),
                SwerveSimConstants.WHEEL_COEFFICIENT_OF_FRICTION));

        driveSimulation = new SwerveDriveSimulation(config, startingPose);
        arena.addDriveTrainSimulation(driveSimulation);
        notifier.setName("SwerveDriveSim");
    }

    /** Starts stepping the physics. Call once every module has its motor controllers attached. */
    public void start() {
        notifier.startPeriodic(SwerveSimConstants.PERIOD_SECONDS);
    }

    /** The simulated module, in the same FL, FR, BL, BR order as the module translations. */
    public SwerveModuleSimulation getModule(SwerveModule module) {
        return driveSimulation.getModules()[module.ordinal()];
    }

    public GyroSimulation getGyro() {
        return driveSimulation.getGyroSimulation();
    }

    /** Where the robot actually is in the simulated world (ground truth, not the odometry estimate). */
    public Pose2d getPose() {
        return driveSimulation.getSimulatedDriveTrainPose();
    }

    /**
     * Moves the simulated robot. The simulated gyro integrates rotation rate, so it keeps reading
     * continuously through the move, like a real gyro when odometry is reset.
     */
    public void teleport(Pose2d pose) {
        // simulationPeriodic() locks the arena, so this can't land in the middle of a physics step
        synchronized (arena) {
            driveSimulation.setSimulationWorldPose(pose);
            // setSimulationWorldPose only zeroes linear velocity
            driveSimulation.setAngularVelocity(0.0);
            // dyn4j finds contacts at the end of each step for the next one, so contacts from the old
            // spot (e.g. pressed against the hub) would otherwise shove the robot right after the move
            arena.clearContacts(driveSimulation);
        }
    }

    /** The 2026 field, plus a way to drop a body's contacts (the physics world is protected). */
    private static class TeleportableArena extends Arena2026Rebuilt {
        void clearContacts(Body body) {
            physicsWorld.removeBody(body);
            physicsWorld.addBody(body);
        }
    }
}
