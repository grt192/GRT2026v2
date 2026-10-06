package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

public record TimestampedVisionUpdate(
    double timestamp,
    Pose3d pose,
    boolean isMultiTag,
    Matrix<N4, N1> stdDevs) {}
