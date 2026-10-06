package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * @param cameraName the camera's name in PhotonVision
 * @param robotToCamera where the camera sits on the robot, from the robot's center on the floor
 */
public record CameraConfig(String cameraName, Transform3d robotToCamera) {}
