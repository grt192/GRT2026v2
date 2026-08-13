package frc.robot.util.subsystems;

/**
 * The IO objects one mechanism needs: a leader, zero or more followers, and an optional absolute
 * encoder.
 *
 * <p>
 * Bundled so a subsystem constructor takes one argument instead of three, and so
 * {@link MechanismHardware} has one thing to return.
 */
public record MechanismIOBundle(MotorIO motor, MotorIO[] followers, EncoderIO encoder) {

    /** All-defaults IO, used in replay and for a mechanism disabled by a feature flag. */
    public static MechanismIOBundle noOp(MechanismConfig config) {
        MotorIO[] followers = new MotorIO[config.followers().length];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new MotorIO() {};
        }
        return new MechanismIOBundle(new MotorIO() {}, followers, new EncoderIO() {});
    }
}
