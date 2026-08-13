package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

public class RampFlywheel extends Command {

    private final FlywheelSubsystem fly;

    public RampFlywheel(FlywheelSubsystem h) {
        this.fly = h;
        addRequirements(fly);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        fly.setVelocityRps(SmashAndShootConstants.FLYWHEEL_VELO_RPS);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        fly.stop();
    }

}
