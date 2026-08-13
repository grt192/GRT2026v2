package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.shooter.Intertable;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class HoodAuto extends Command {

    private HoodSubsystem hood;
    private Intertable tableThing = Intertable.getInstance();
    private boolean redTeam = false;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SWERVE_TABLE_NAME");
    StructSubscriber<Pose2d> poseSub = table.getStructTopic("estimatedPose", Pose2d.struct).subscribe(new Pose2d());

    public HoodAuto(HoodSubsystem hood, boolean red) {
        redTeam = red;
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double angleRot;

        if (redTeam) {
            if (poseSub.get().getX() > AlignConstants.RED_WALL_X) {
                angleRot = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.RED_HUB_TRANS));
            } else {
                if (poseSub.get().getY() > AlignConstants.HUB_Y) {
                    angleRot = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.RED_AIM_TOP));
                } else {
                    angleRot = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.RED_AIM_BOTTOM));
                }
            }

        } else {
            if (poseSub.get().getX() < AlignConstants.BLUE_WALL_X) {
                angleRot = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_HUB_TRANS));
            } else {
                if (poseSub.get().getY() > AlignConstants.HUB_Y) {
                    angleRot = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_AIM_TOP));
                } else {
                    angleRot = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_AIM_BOTTOM));
                }
            }
        }

        System.out.println("hood angle: " + angleRot);
        hood.setPositionRot(angleRot);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
