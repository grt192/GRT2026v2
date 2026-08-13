package frc.robot.commands.shooter.tower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.shooter.tower.TowerConfig.TowerIntake;
import frc.robot.subsystems.shooter.tower.TowerConfig;

public class TowerRoll extends Command {

    private final TowerSubsystem tower;

    public TowerRoll(TowerSubsystem tower) {
        this.tower = tower;

        addRequirements(tower);
    }

    @Override
    public void execute() {
        tower.setDutyCycle(SmashAndShootConstants.TOWER_DUTY_CYCLE);
        // t.setTower(TOWER_INTAKE.BALL_UP);
        // if (fly.wantedVel() && hd.wantedAngl()) {
        // t.setTower(TOWER_INTAKE.BALL_UP);
        // } else {
        // t.setTower(TOWER_INTAKE.STOP);
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        tower.setTower(TowerIntake.STOP);
    }
}
