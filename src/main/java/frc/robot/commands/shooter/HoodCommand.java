package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class HoodCommand extends Command {

    private final HoodSubsystem hood;

    public HoodCommand(HoodSubsystem hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        hood.setPositionRot(SmashAndShootConstants.HOOD_POSITION_ROT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
