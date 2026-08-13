package frc.robot.util.subsystems;

import frc.robot.util.LoggedTunableNumber;

/**
 * Dashboard-backed Motion Magic constraints, for mechanisms that configure them.
 *
 * <p>
 * Note that nothing currently issues a Motion Magic control request — the constraints are
 * configured and then plain position/velocity requests are used. This carries that state
 * forward unchanged rather than quietly changing how three mechanisms move.
 */
class MotionMagicTuner {

    private final LoggedTunableNumber cruise;
    private final LoggedTunableNumber accel;
    private final LoggedTunableNumber jerk;
    private final LoggedTunableNumber.Watcher watcher;

    MotionMagicTuner(MechanismConfig config) {
        String prefix = config.name() + "/MotionMagic/";
        cruise = new LoggedTunableNumber(prefix + "cruiseVelocity_rps", config.motionMagicCruiseRps());
        accel = new LoggedTunableNumber(prefix + "acceleration_rps2", config.motionMagicAccelRps2());
        jerk = new LoggedTunableNumber(prefix + "jerk_rps3", config.motionMagicJerkRps3());
        watcher = LoggedTunableNumber.watch(cruise, accel, jerk);
    }

    void applyIfChanged(MotorIO motor) {
        watcher.ifChanged(() -> motor.setMotionMagic(cruise.get(), accel.get(), jerk.get()));
    }
}
