package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final NetworkTable table;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry tid;
    private final NetworkTableEntry tagPose;
    private final NetworkTableEntry botPose;
    private final double maxLauncherSetpoint = .88;
    private final double minLauncherSetpoint = .77;
    private long lastTagSeen = -1;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tv = table.getEntry("tv");
        tid = table.getEntry("tid");
        tagPose = table.getEntry("targetpose_robotspace");
        botPose = table.getEntry("botpose_wpiblue");
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot To Base", getLengthToBase());
        SmartDashboard.putNumber("Last April Tag Seen", this.lastTagSeen());
    }

    // x y z roll pitch yaw

    /** returns yaw */
    public double getTargetAngle() {
        return tagPose.getDoubleArray(new Double[]{0.0,0.0,0.0,0.0,0.0,0.0})[5];
    }
    
    public boolean hasTargets() {
        return tv.getDouble(0.0) == 1.0;
    }

    public double getLengthToBase() {
        double lengthToBase = tagPose.getDoubleArray(new Double[]{0.0,0.0,0.0,0.0,0.0,0.0})[2];
        return lengthToBase;
    }

    public Double[] getRobotPose() {
        return botPose.getDoubleArray(new Double[]{0.0,0.0,0.0,0.0,0.0,0.0});
    }

    public double getVisionClawSetpoint() {
        double slope = (maxLauncherSetpoint - minLauncherSetpoint) / 1.6;
        return (getLengthToBase() * slope) + minLauncherSetpoint;
    }

    public long getAprilTagID() {
        return tid.getInteger(-1);
    }

    public long lastTagSeen() {
        if(getAprilTagID() != -1) {
            lastTagSeen = getAprilTagID();
        }
        return lastTagSeen;
    }
}


