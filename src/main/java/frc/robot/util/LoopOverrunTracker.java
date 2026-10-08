package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/**
 * Counts loop overruns: cycles that started late because the previous one (user code, logging, or a GC
 * pause) ran past the loop period. Timing from cycle start to cycle start catches all of that, where
 * timing only our own code would miss logging and GC.
 */
public class LoopOverrunTracker {
    private LoopOverrunTracker() {}

    // A little slack so notifier jitter doesn't count as an overrun
    private static final double OVERRUN_THRESHOLD_SECS = LoggedRobot.defaultPeriodSecs * 1.1;

    private static final String ALERT_PREFIX = "Robot Loop: ";
    private static final Alert overrunAlert = new Alert(ALERT_PREFIX + "Overran", AlertType.kWarning);

    private static double lastCycleStartTime = -1.0;
    private static int overrunCount = 0;

    /**
     * Call once per loop, at the start of robotPeriodic and before {@link LoggedTracer#reset()}, so the
     * tracer still holds the previous cycle's slowest epoch.
     */
    public static void update() {
        double now = Timer.getFPGATimestamp();
        double cycleSecs = now - lastCycleStartTime;
        boolean overran = lastCycleStartTime > 0.0 && cycleSecs > OVERRUN_THRESHOLD_SECS;
        lastCycleStartTime = now;

        Logger.recordOutput("LoopOverrun/Overran", overran);
        if (!overran) {
            return;
        }

        overrunCount++;
        String slowestEpoch = LoggedTracer.getSlowestEpochName();
        Logger.recordOutput("LoopOverrun/Count", overrunCount);
        Logger.recordOutput("LoopOverrun/LastCycleMS", cycleSecs * 1000.0);
        Logger.recordOutput("LoopOverrun/LastSlowestEpoch", slowestEpoch);
        Logger.recordOutput("LoopOverrun/LastSlowestEpochMS", LoggedTracer.getSlowestEpochMS());

        overrunAlert.setText(String.format(
            "%sOverran %d times (last %.1f ms, slowest part %s)",
            ALERT_PREFIX, overrunCount, cycleSecs * 1000.0, slowestEpoch.isEmpty() ? "unknown" : slowestEpoch));
        overrunAlert.set(true);
    }
}
