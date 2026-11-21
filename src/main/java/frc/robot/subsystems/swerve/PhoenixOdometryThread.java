// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.Constants.CANType;
import frc.robot.Constants.SwerveConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>
 * This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CAN FD CANivore, the thread uses the "waitForAll" blocking method to enable more consistent
 * sampling. This also allows Phoenix Pro users to benefit from lower latency between devices using
 * CANivore time synchronization.
 *
 * <p>
 * Every position is registered with its velocity and sampled latency compensated
 * ({@link BaseStatusSignal#getLatencyCompensatedValueAsDouble}), the same way CTRE's swerve
 * odometry thread (used by 254) does. That projects each reading forward to the moment it was
 * sampled, so all values in a sample line up with the sample's FPGA timestamp.
 *
 * Adapted from the AdvantageKit swerve template and 6328's 2025 code
 */
public class PhoenixOdometryThread extends Thread {
    private final Lock signalsLock =
        new ReentrantLock(); // Prevents conflicts when registering signals
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<StatusSignal<Angle>> phoenixPositionSignals = new ArrayList<>();
    private final List<StatusSignal<AngularVelocity>> phoenixVelocitySignals = new ArrayList<>();
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private final boolean isCANFD = new CANBus(CANType.SWERVE.busName()).isNetworkFD();
    private final double frequencyHz =
        isCANFD ? SwerveConstants.ODOMETRY_FREQUENCY_FD_HZ : SwerveConstants.ODOMETRY_FREQUENCY_HZ;

    private static PhoenixOdometryThread instance = null;

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new PhoenixOdometryThread();
        }
        return instance;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
    }

    /** Rate the odometry signals should be published at, in Hz. */
    public double getFrequencyHz() {
        return frequencyHz;
    }

    @Override
    public void start() {
        if (timestampQueues.size() > 0) {
            super.start();
        }
    }

    /**
     * Registers a Phoenix position signal to be read from the thread, latency compensated with its
     * velocity. Both signals should be clones so the thread doesn't race the main loop's refresh.
     */
    public Queue<Double> registerSignal(StatusSignal<Angle> position, StatusSignal<AngularVelocity> velocity) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        DriveSubsystem.ODOMETRY_LOCK.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 2];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = position;
            newSignals[phoenixSignals.length + 1] = velocity;
            phoenixSignals = newSignals;
            phoenixPositionSignals.add(position);
            phoenixVelocitySignals.add(velocity);
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            DriveSubsystem.ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        DriveSubsystem.ODOMETRY_LOCK.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            signalsLock.unlock();
            DriveSubsystem.ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        DriveSubsystem.ODOMETRY_LOCK.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            DriveSubsystem.ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        // Real-time priority so samples land on time even when the main loop is busy. Safe because
        // the loop always blocks (waitForAll or sleep) between samples. See
        // https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template#real-time-thread-priority
        Threads.setCurrentThreadPriority(true, 1);

        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (isCANFD && phoenixSignals.length > 0) {
                    BaseStatusSignal.waitForAll(2.0 / frequencyHz, phoenixSignals);
                } else {
                    // "waitForAll" does not support blocking on multiple signals with a bus
                    // that is not CAN FD, regardless of Pro licensing.
                    Thread.sleep((long) (1000.0 / frequencyHz));
                    if (phoenixSignals.length > 0) {
                        BaseStatusSignal.refreshAll(phoenixSignals);
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            // Save new data to queues
            DriveSubsystem.ODOMETRY_LOCK.lock();
            try {
                // Values are latency compensated up to now, so the sample is stamped with the
                // current FPGA time rather than an averaged CAN latency
                double timestamp = RobotController.getFPGATime() / 1e6;

                // Add new samples to queues
                for (int i = 0; i < phoenixPositionSignals.size(); i++) {
                    phoenixQueues.get(i).offer(
                        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                            phoenixPositionSignals.get(i), phoenixVelocitySignals.get(i)));
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            } finally {
                DriveSubsystem.ODOMETRY_LOCK.unlock();
            }
        }
    }
}
