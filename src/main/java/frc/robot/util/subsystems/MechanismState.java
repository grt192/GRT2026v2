package frc.robot.util.subsystems;

/**
 * A simulated mechanism's output-shaft state, in mechanism rotations and rotations per second.
 *
 * <p>
 * Only used to link a simulated motor's plant to a simulated encoder: the encoder has no plant
 * of its own, so it reads the motor's each loop and writes the result into its device sim state.
 */
public record MechanismState(double positionRot, double velocityRps) {

    public static final MechanismState ZERO = new MechanismState(0.0, 0.0);
}
