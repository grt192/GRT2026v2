package frc.robot.commands.allign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.commands.allign.AimToHubCommand;
import frc.robot.commands.allign.AimToPointCommand;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

public class AimBot extends Command {

    private DriveSubsystem swerve;
    private FieldManagementSubsystem field;
    private boolean redTeam = false;

    public AimBot(DriveSubsystem swerveSubsystem, FieldManagementSubsystem fms, boolean red) {
        swerve = swerveSubsystem;
        field = fms;
        redTeam = red;
    }

    @Override
    public void initialize() {
        if (redTeam) {
            if (swerve.getRobotPosition().getX() > AlignConstants.RED_WALL_X) {
                new AimToHubCommand(swerve, field).schedule();
            } else {
                if (swerve.getRobotPosition().getY() > AlignConstants.HUB_Y) {
                    new AimToPointCommand(swerve, field, AlignConstants.RED_AIM_TOP).schedule();
                } else {
                    new AimToPointCommand(swerve, field, AlignConstants.RED_AIM_BOTTOM).schedule();
                }
            }

        } else {
            if (swerve.getRobotPosition().getX() < AlignConstants.BLUE_WALL_X) {
                new AimToHubCommand(swerve, field).schedule();
            } else {
                if (swerve.getRobotPosition().getY() > AlignConstants.HUB_Y) {
                    new AimToPointCommand(swerve, field, AlignConstants.BLUE_AIM_TOP).schedule();
                } else {
                    new AimToPointCommand(swerve, field, AlignConstants.BLUE_AIM_BOTTOM).schedule();
                }
            }
        }
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}

}
