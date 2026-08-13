package frc.robot.util.subsystems;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.Hardware.EncoderHealth;

/**
 * An absolute encoder, behind the AdvantageKit IO boundary.
 *
 * <p>
 * Telemetry only. When the encoder is the motor controller's remote feedback source — which is
 * how both of our servos are wired — mechanism position already arrives through
 * {@link MotorIO.MotorInputs#positionRot}, scaled by the controller. This interface exists so
 * absolute position, magnet health and connectivity are logged and alerted on, and so encoder
 * hardware can be swapped without touching the motor side.
 */
public interface EncoderIO {

    /** Everything read back from one absolute encoder each loop. */
    @AutoLog
    public static class EncoderInputs {
        public double absolutePositionRot = 0.0;
        public double velocityRps = 0.0;
        public EncoderHealth health = EncoderHealth.Unknown;
        public boolean connected = false;
    }

    default void updateInputs(EncoderInputs inputs) {}
}
