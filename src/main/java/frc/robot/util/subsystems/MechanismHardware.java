package frc.robot.util.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.CanBuses;
import frc.robot.util.subsystems.MechanismConfig.Controller;
import frc.robot.util.subsystems.talonfx.TalonFXHardware;

/**
 * The one place the real / simulation / replay and vendor decisions are made.
 *
 * <p>
 * Previously this choice was written out once per subsystem per mode inside
 * {@code RobotContainer} — eighteen lines for six mechanisms, growing by three with every new
 * one. Taking it here means a subsystem declaration reads identically in all three modes:
 *
 * <pre>
 * private final HoodSubsystem hood = new HoodSubsystem();
 * </pre>
 *
 * <p>
 * Adding a motor-controller vendor costs one {@link Controller} constant, one case below, and
 * the vendor's own package. Nothing in the template classes or the subsystems changes, and the
 * exhaustive {@code switch} makes a forgotten case a compile error rather than a runtime one.
 */
public final class MechanismHardware {

    private MechanismHardware() {}

    public static MechanismIOBundle create(MechanismConfig config) {
        if (Constants.CURRENT_MODE == Mode.REPLAY || config.controller() == Controller.NONE) {
            return MechanismIOBundle.noOp(config);
        }

        boolean simulated = Constants.CURRENT_MODE == Mode.SIM;

        switch (config.controller()) {
            case TALON_FX:
                return TalonFXHardware.create(config, CanBuses.of(config.bus()), simulated);
            case NONE:
            default:
                return MechanismIOBundle.noOp(config);
        }
    }
}
