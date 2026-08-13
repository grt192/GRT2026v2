package frc.robot.util.subsystems;

import java.util.EnumMap;
import java.util.Map;

import frc.robot.util.Hardware.MotorControlMode;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;
import frc.robot.util.subsystems.MechanismConfig.SlotSpec;

/**
 * Dashboard-backed gains, one set per closed-loop control mode the mechanism declares.
 *
 * <p>
 * Keys are {@code <name>/Gains/<Mode>/kP} and so on, so a mechanism that closed-loops on both
 * position and velocity gets two independently tunable sets rather than one shared one. Values
 * are seeded from whatever the IO reports it started with — configured gains on real hardware,
 * the simulation overlay in sim — and re-applied only when one actually changes.
 */
class GainTuner {

    private final Map<MotorControlMode, Entry> entries = new EnumMap<>(MotorControlMode.class);

    GainTuner(MechanismConfig config, MotorIO motor) {
        for (SlotSpec spec : config.slots()) {
            entries.put(spec.mode(), new Entry(config.name(), spec.mode(),
                motor.defaultGains(spec.mode())));
        }
        applyAll(motor);
    }

    /** Pushes every set to the motor. Called once at construction. */
    final void applyAll(MotorIO motor) {
        entries.forEach((mode, entry) -> motor.setGains(mode, entry.current()));
    }

    void applyIfChanged(MotorIO motor) {
        entries.forEach((mode, entry) -> entry.watcher.ifChanged(() -> motor.setGains(mode, entry.current())));
    }

    private static final class Entry {
        private final LoggedTunableNumber kP;
        private final LoggedTunableNumber kI;
        private final LoggedTunableNumber kD;
        private final LoggedTunableNumber kS;
        private final LoggedTunableNumber kG;
        private final LoggedTunableNumber kV;
        private final LoggedTunableNumber kA;
        private final LoggedTunableNumber.Watcher watcher;

        private Entry(String name, MotorControlMode mode, PIDConstants seed) {
            String prefix = name + "/Gains/" + mode + "/";
            kP = new LoggedTunableNumber(prefix + "kP", seed.kP());
            kI = new LoggedTunableNumber(prefix + "kI", seed.kI());
            kD = new LoggedTunableNumber(prefix + "kD", seed.kD());
            kS = new LoggedTunableNumber(prefix + "kS", seed.kS());
            kG = new LoggedTunableNumber(prefix + "kG", seed.kG());
            kV = new LoggedTunableNumber(prefix + "kV", seed.kV());
            kA = new LoggedTunableNumber(prefix + "kA", seed.kA());
            watcher = LoggedTunableNumber.watch(kP, kI, kD, kS, kG, kV, kA);
        }

        private PIDConstants current() {
            return new PIDConstants(
                kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get());
        }
    }
}
