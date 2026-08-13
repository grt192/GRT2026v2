package frc.robot.util;

import java.util.EnumMap;
import java.util.Map;

import frc.robot.Constants.CANType;

/**
 * One {@link LoggedCanivore} per physical CAN bus, created on first use.
 *
 * <p>
 * A {@code LoggedCanivore} spawns a polling thread and registers itself for dashboard status, so
 * there must be exactly one per bus. Previously that was enforced only by the instances happening
 * to be fields of {@code RobotContainer}, which meant every IO constructor had to be handed one.
 *
 * <p>
 * Creation is lazy on purpose: in replay no IO ever asks for a bus, so no CANivore is constructed
 * and no thread starts.
 */
public final class CanBuses {

    private static final Map<CANType, LoggedCanivore> BUSES = new EnumMap<>(CANType.class);

    private CanBuses() {}

    public static synchronized LoggedCanivore of(CANType type) {
        return BUSES.computeIfAbsent(type, LoggedCanivore::new);
    }
}
