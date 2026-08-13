package frc.robot.util.subsystems.talonfx;

import frc.robot.util.LoggedCanivore;
import frc.robot.util.subsystems.EncoderIO;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismConfig.Follower;
import frc.robot.util.subsystems.MechanismIOBundle;
import frc.robot.util.subsystems.MotorIO;

/**
 * Builds the Phoenix IO for one mechanism.
 *
 * <p>
 * The single public entry point into this package, so every class that touches a Phoenix type
 * can stay package-private. Adding a second motor-controller vendor means adding a sibling
 * package with a class shaped like this one, plus one case in
 * {@code MechanismHardware}.
 */
public final class TalonFXHardware {

    private TalonFXHardware() {}

    public static MechanismIOBundle create(
        MechanismConfig config, LoggedCanivore bus, boolean simulated) {

        MotorIOTalonFX leader = simulated
            ? new MotorIOTalonFXSim(config, config.canId(), "Motor", bus)
            : new MotorIOTalonFX(config, config.canId(), "Motor", bus);

        Follower[] followerSpecs = config.followers();
        MotorIO[] followers = new MotorIO[followerSpecs.length];
        for (int i = 0; i < followerSpecs.length; i++) {
            String role = "Follower " + i;
            MotorIOTalonFX follower = simulated
                ? new MotorIOTalonFXSim(config, followerSpecs[i].canId(), role, bus)
                : new MotorIOTalonFX(config, followerSpecs[i].canId(), role, bus);
            follower.follow(config.canId(), followerSpecs[i].opposeLeader());
            followers[i] = follower;
        }

        EncoderIO encoder;
        if (config.encoder() == null) {
            encoder = new EncoderIO() {};
        } else if (simulated) {
            encoder = new EncoderIOCancoderSim(config, bus, leader::mechanismState);
        } else {
            encoder = new EncoderIOCancoder(config, bus);
        }

        return new MechanismIOBundle(leader, followers, encoder);
    }
}
