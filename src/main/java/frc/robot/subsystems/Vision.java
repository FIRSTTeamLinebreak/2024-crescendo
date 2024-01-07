package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    PhotonCamera camera;

    public Vision(String name) {
        camera = new PhotonCamera(name);
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        // PhotonPipelineResult result = camera.getLatestResult();
        // if (result.hasTargets()) {
        //     SmartDashboard.putString("Target Data", result.getBestTarget().getYaw() + "");
        // }
        // else {
        //     SmartDashboard.putString("Target Data", "No Targets");
        // }
    }

    public PhotonPipelineResult getResult() {
        return camera.getLatestResult();
    }
}
