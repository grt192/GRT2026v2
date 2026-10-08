// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for logging code execution times.
 * Taken from team 6328 Mechanical Advantage's 2025 codebase
 */
public class LoggedTracer {
    private LoggedTracer() {}

    private static double startTime = -1.0;

    // Longest epoch since the last reset, so a loop overrun can name what was slow
    private static String slowestEpochName = "";
    private static double slowestEpochSecs = 0.0;

    /** Reset the clock. */
    public static void reset() {
        startTime = Timer.getFPGATimestamp();
        slowestEpochName = "";
        slowestEpochSecs = 0.0;
    }

    /** Save the time elapsed since the last reset or record. */
    public static void record(String epochName) {
        double now = Timer.getFPGATimestamp();
        double elapsedSecs = now - startTime;
        Logger.recordOutput("LoggedTracer/" + epochName + "MS", elapsedSecs * 1000.0);
        if (elapsedSecs > slowestEpochSecs) {
            slowestEpochName = epochName;
            slowestEpochSecs = elapsedSecs;
        }
        startTime = now;
    }

    /** The longest epoch recorded since the last reset, or "" if none was. */
    public static String getSlowestEpochName() {
        return slowestEpochName;
    }

    /** How long the slowest epoch since the last reset took, in milliseconds. */
    public static double getSlowestEpochMS() {
        return slowestEpochSecs * 1000.0;
    }
}
