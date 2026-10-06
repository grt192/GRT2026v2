package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * One AprilTag seen by one camera in one frame.
 *
 * @param camera the camera that saw it
 * @param tagId the tag's ID
 * @param timestamp when the frame was captured (FPGA seconds)
 * @param robotToTag where the tag was relative to the robot's center on the floor
 */
public record TagSighting(CameraConfig camera, int tagId, double timestamp, Transform3d robotToTag) {}
