package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    // private PhotonCamera camera;
    // private PhotonPipelineResult result;

    public Vision(String name) {
        // camera = new PhotonCamera(name);
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        // result = camera.getLatestResult();
    }

    public double getTargetAngle() {
        // if (result.hasTargets()) {
        //     return result.getBestTarget().getYaw();
        // }
        return 0;
    }
    
    public boolean hasTargets() {
        // return result.hasTargets();
        return false;
    }
}
