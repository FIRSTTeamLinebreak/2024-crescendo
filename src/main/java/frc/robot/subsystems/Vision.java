package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {

    private LimelightResults results;
    private double lastTagSeen = -1;

    public Vision() {}

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Last Tag Seen", lastTagSeen);
        results = LimelightHelpers.getLatestResults("limelight");
        lastTagSeen = results.targetingResults.targets_Fiducials.length > 0 ? results.targetingResults.targets_Fiducials[0].fiducialID : lastTagSeen;
    }

    public boolean hasTargets() {
        return results.targetingResults.targets_Fiducials.length > 0;
    }

    /**
     * current pose of the robot
     * @return Pose2d of the robot
     */
    public PoseEstimate getRobotPose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    public int lastTagSeen() {
        return (int)lastTagSeen;
    }

    public void enableLight() {
        LimelightHelpers.setLEDMode_ForceOn("limelight");
    }

    public void disableLight() {
        LimelightHelpers.setLEDMode_ForceOff("limelight");
    }
}
