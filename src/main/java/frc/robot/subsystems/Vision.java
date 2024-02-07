package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final NetworkTable table;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry tagPose;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tv = table.getEntry("tv");
        tagPose = table.getEntry("targetpose_cameraspace");
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {}

    public double getTargetAngle() {
        return tagPose.getDoubleArray(new Double[]{0.0,0.0,0.0,0.0,0.0,0.0})[5];
    }
    
    public boolean hasTargets() {
        return tv.getDouble(0.0) == 1.0;
    }
}
