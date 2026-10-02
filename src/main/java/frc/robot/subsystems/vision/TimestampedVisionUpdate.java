package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record TimestampedVisionUpdate(
    double timestamp,
    Pose3d pose,
    boolean isMultiTag,
    Matrix<N3, N1> stdDevs) {}
