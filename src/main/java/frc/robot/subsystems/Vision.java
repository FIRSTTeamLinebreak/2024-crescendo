package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final NetworkTable table;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry tid;
    private final NetworkTableEntry tagPose;
    private final NetworkTableEntry botPose;
    private long lastTagSeen = -1;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        ta = table.getEntry("ta");
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
        return tagPose.getDoubleArray(new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 })[5];
    }

    public boolean hasTargets() {
        return tv.getDouble(0.0) > 0.0;
    }

    public double getLengthToBase() {
        double lengthToBase = tagPose.getDoubleArray(new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 })[2];
        return lengthToBase;
    }

    public Double[] getRobotPose() {
        return botPose.getDoubleArray(new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        // return new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    }

    public long getAprilTagID() {
        return tid.getInteger(-1);
    }

    public long lastTagSeen() {
        if (getAprilTagID() != -1) {
            lastTagSeen = getAprilTagID();
        }
        return lastTagSeen;
    }

    public Pose2d getVisionPose() {
        Double[] botPose = this.getRobotPose();
        return new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
    }

    public double getPoseLatensy() {
        return this.getRobotPose()[6];
    }

    public int getNumberOfTargetsVisible() {
        return this.getRobotPose()[7].intValue();
    }

    public double getBestTargetArea() {
        return this.ta.getDouble(0.0);
    }
}
