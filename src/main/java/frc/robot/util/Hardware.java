package frc.robot.util;

/**
 * Vendor-neutral hardware vocabulary shared by IO layers, logging and configs.
 *
 * <p>
 * Everything here crosses the AdvantageKit IO boundary, so it must never reference a vendor
 * type: replayed logs have to be readable without Phoenix on the classpath. Vendor enums that
 * only ever appear on the hardware side of that boundary (Phoenix's {@code InvertedValue},
 * {@code NeutralModeValue}, {@code GravityTypeValue}) are used directly in configs instead of
 * being mirrored here.
 */
public final class Hardware {

    private Hardware() {}

    /** How a motor is currently being commanded. */
    public enum MotorControlMode {
        Disabled,
        Follower,
        DutyCycle,
        Voltage,
        TorqueCurrent,
        Position,
        Velocity;
    }

    /** Absolute-encoder magnet health. */
    public enum EncoderHealth {
        Good,
        Marginal,
        Bad,
        Unknown
    }

    /**
     * What a closed-loop request commands the motor with.
     *
     * <p>
     * This is not a configuration value — it selects the control <i>request</i> type
     * ({@code PositionVoltage} vs {@code PositionTorqueCurrentFOC} vs {@code PositionDutyCycle}),
     * which lives in code rather than in a device configuration. It is paired with a gain slot,
     * since gains are only meaningful for the output units they were tuned against.
     */
    public enum ClosedLoopOutput {
        Voltage,
        TorqueCurrent,
        DutyCycle
    }
}
