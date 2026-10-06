package frc.robot.commands;

import static frc.robot.Constants.AlignToTagConstants.*;

import java.util.Comparator;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.TagSighting;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Drives so the camera that sees an AprilTag ends up square in front of it, backing off further the
 * higher the tag is. Works with any tag (field layout not needed): it locks onto the closest tag it
 * sees and keeps following it, even if the tag moves.
 *
 * <p>
 * Each camera frame sets a goal pose, which is held in the odometry frame so the robot keeps driving
 * smoothly between frames and through short dropouts.
 */
public class AlignToTagCommand extends Command {
    private static final String LOG_KEY = "AlignToTag";

    private final DriveSubsystem drive;
    private final VisionSubsystem vision;

    private TagSighting lockedTag = null;
    private double lastSeenTime = Double.NEGATIVE_INFINITY;
    private Pose2d goal = null;

    public AlignToTagCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        lockedTag = null;
        goal = null;
    }

    @Override
    public void execute() {
        if (lockedTag != null && Timer.getTimestamp() - lastSeenTime > TAG_LOST_TIMEOUT_SECONDS) {
            lockedTag = null;
            goal = null;
        }

        findTag().ifPresent(this::updateGoal);

        if (goal == null) {
            drive.setRobotRelativeDrivePowers(new ChassisSpeeds());
        } else {
            driveToGoal();
        }

        Logger.recordOutput(LOG_KEY + "/HasTag", lockedTag != null);
        Logger.recordOutput(LOG_KEY + "/TagId", lockedTag != null ? lockedTag.tagId() : -1);
        Logger.recordOutput(LOG_KEY + "/Goal", goal != null ? goal : new Pose2d());
    }

    /** The newest sighting of the locked tag, or the closest tag of any kind if nothing is locked. */
    private Optional<TagSighting> findTag() {
        return vision.getSightings().stream()
            .filter(sighting -> lockedTag == null
                || (sighting.camera().equals(lockedTag.camera()) && sighting.tagId() == lockedTag.tagId()))
            .min(lockedTag == null
                ? Comparator.comparingDouble(sighting -> sighting.robotToTag().getTranslation().getNorm())
                : Comparator.comparingDouble(sighting -> -sighting.timestamp()));
    }

    private void updateGoal(TagSighting sighting) {
        lockedTag = sighting;
        lastSeenTime = Timer.getTimestamp();

        // The robot frame sits on the floor, so the tag's z is its height
        double tagHeight = sighting.robotToTag().getZ();
        double distance = MathUtil.interpolate(
            LOW_TAG_DISTANCE_M,
            HIGH_TAG_DISTANCE_M,
            MathUtil.inverseInterpolate(LOW_TAG_HEIGHT_M, HIGH_TAG_HEIGHT_M, tagHeight));

        // All in the robot's frame at the moment of the frame. A tag's +X points out of its face, so
        // the camera goal is that far out along it, turned around to look back at the tag
        Pose2d tag = Pose2d.kZero.transformBy(flatten(sighting.robotToTag()));
        Pose2d cameraGoal = tag.transformBy(new Transform2d(distance, 0, Rotation2d.kPi));
        Pose2d robotGoal = cameraGoal.transformBy(flatten(sighting.camera().robotToCamera()).inverse());

        // Move it into the odometry frame using where the robot was when the frame was taken
        Pose2d robotThen = drive.getRobotPositionAt(sighting.timestamp()).orElse(drive.getRobotPosition());
        goal = robotThen.transformBy(new Transform2d(robotGoal.getTranslation(), robotGoal.getRotation()));

        Logger.recordOutput(LOG_KEY + "/TagHeightM", tagHeight);
        Logger.recordOutput(LOG_KEY + "/TargetDistanceM", distance);
    }

    private void driveToGoal() {
        Pose2d pose = drive.getRobotPosition();

        Translation2d velocity = goal.getTranslation().minus(pose.getTranslation()).times(TRANSLATION_P);
        if (velocity.getNorm() > MAX_VELOCITY_MPS) {
            velocity = velocity.times(MAX_VELOCITY_MPS / velocity.getNorm());
        }
        double omega = MathUtil.clamp(
            goal.getRotation().minus(pose.getRotation()).getRadians() * ROTATION_P,
            -MAX_OMEGA_RADPS,
            MAX_OMEGA_RADPS);

        drive.setRobotRelativeDrivePowers(
            ChassisSpeeds.fromFieldRelativeSpeeds(velocity.getX(), velocity.getY(), omega, pose.getRotation()));
    }

    /**
     * Drops a 3D transform to the floor, keeping the direction its +X axis points. Plain yaw isn't enough:
     * cameras can be mounted upside down or tilted, which shuffles the Euler angles.
     */
    private static Transform2d flatten(Transform3d transform) {
        Translation3d forward = new Translation3d(1, 0, 0).rotateBy(transform.getRotation());
        return new Transform2d(
            transform.getX(),
            transform.getY(),
            new Rotation2d(forward.getX(), forward.getY()));
    }

    @Override
    public void end(boolean interrupted) {
        drive.setRobotRelativeDrivePowers(new ChassisSpeeds());
    }
}
