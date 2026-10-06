package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateToAngleConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.OldVisionSubsystem;
import java.io.Serial;
import java.util.function.BooleanSupplier;
import javax.xml.crypto.dsig.Transform;

public class GetCameraDisplacement extends Command {
    private final OldVisionSubsystem visionSubsytem;
    private final Transform3d robotToApriltag;
    private final String camID;
    private Transform3d robotToCamera;
    private Transform3d cameraToApriltag;
    // private StructPublisher<Pose2d> cameraPosePublisher;

    public GetCameraDisplacement(OldVisionSubsystem visionSubsytem, Transform3d robotToApriltag) {
        this.visionSubsytem = visionSubsytem;
        this.robotToApriltag = robotToApriltag;
        this.camID = this.visionSubsytem.getCamID();
        addRequirements(visionSubsytem);
        // Publish initial PID values to NetworkTables so you can edit them in Shuffleboard/SmartDashboard
        // SmartDashboard.putNumber("RotateToAngle/kP", RotateToAngleConstants.kP);
    }

    @Override
    public void initialize() {

        // cameraPosePublisher = swerveTable.getStructTopic(
        // "estimatedPose",
        // Pose2d.struct
        // ).publish();

    }

    @Override
    public void execute() {

        cameraToApriltag = visionSubsytem.cameraToApriltag();
        robotToCamera = robotToApriltag.plus(cameraToApriltag.inverse()); // this is what we want
        // Pose3d cameraPos = new Pose3d().plus(robotToCamera);
        SmartDashboard.putNumber(camID + "/x", robotToCamera.getX());
        SmartDashboard.putNumber(camID + "/y", robotToCamera.getY());
        SmartDashboard.putNumber(camID + "/z", robotToCamera.getZ());
        SmartDashboard.putString(camID + "/quat", robotToCamera.getRotation().getQuaternion().toString());

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
