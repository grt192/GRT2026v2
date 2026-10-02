package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
    private String cameraName;
    private Transform3d cameraPose;
    private double camStdDevFactor;

    public CameraConfig(
        String cameraName, Transform3d cameraPose, double camStdDevFactor) {
        this.cameraName = cameraName;
        this.cameraPose = cameraPose;
        this.camStdDevFactor = camStdDevFactor;
    }

    public String getCameraName() {
        return cameraName;
    }

    public Transform3d getCameraPose() {
        return cameraPose;
    }

    public double getStdDevFactor() {
        return camStdDevFactor;
    }

}
