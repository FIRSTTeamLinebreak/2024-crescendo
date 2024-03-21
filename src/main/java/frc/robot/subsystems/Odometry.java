package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Kinematics;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Odometry extends SubsystemBase {

    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDrive m_driveSubsystem;
    private final Vision m_vision;

    private final StructPublisher<Pose2d> m_poseOdoPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Pose_Odo", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> m_poseVisPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Pose_Vis", Pose2d.struct).publish();

    public Odometry(SwerveDrive driveSubsystem, Vision vison) {
        m_driveSubsystem = driveSubsystem;
        m_vision = vison;

        m_poseEstimator = new SwerveDrivePoseEstimator(Kinematics.driveKinematics,
                m_driveSubsystem.getRotation2d(),
                m_driveSubsystem.getModulePositions(),
                this.getAlliance()
                        ? new Pose2d(16.56, 2.15, new Rotation2d())
                        : new Pose2d(0.56, 2.15, new Rotation2d(Math.PI)));

        AutoBuilder.configureHolonomic(
                this::getRobotPose,
                (Pose2d pose) -> m_poseEstimator.resetPosition(
                        m_driveSubsystem.getRotation2d(),
                        m_driveSubsystem.getModulePositions(),
                        pose),
                m_driveSubsystem::getRobotRelativeSpeeds,
                m_driveSubsystem::setSpeed,
                SwerveConstants.HolonomicConfig,
                this::getAlliance,
                m_driveSubsystem);
    }

    public Pose2d getRobotPose() {
        Pose2d pose = m_poseEstimator.getEstimatedPosition();
        m_poseOdoPublisher.set(pose);
        return pose;
    }

    /**
     * Get the Robots alliance color
     * 
     * @return true if red alliance, false if blue alliance
     */
    public boolean getAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void updatePoseEstimatorWithVisionBotPose() {
        PoseEstimate visionPose = m_vision.getRobotPose();
        m_poseVisPublisher.set(visionPose.pose);
        // invalid LL data
        if (visionPose.pose.getX() == 0.0) {
            return;
        }

        // distance from current pose to vision estimated pose
        double poseDifference = m_poseEstimator.getEstimatedPosition().getTranslation()
                .getDistance(visionPose.pose.getTranslation());
        double xyStds = 50;
        double degStds = 600;
        if (visionPose.tagCount >= 2) {
            // multiple targets detected
            xyStds = 0.5;
            degStds = 6;
        } else if (visionPose.tagSpan > 0.8 && poseDifference < 0.5) {
            // 1 target with large area and close to estimated pose
            xyStds = 1.0;
            degStds = 12;
        } else if (visionPose.tagSpan > 0.1 && poseDifference < 0.3) {
            // 1 target farther away and estimated pose is close
            xyStds = 2.0;
            degStds = 30;
        }

        m_poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
        m_poseEstimator.addVisionMeasurement(visionPose.pose,
                visionPose.timestampSeconds);
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(
                m_driveSubsystem.getRotation2d(),
                m_driveSubsystem.getModulePositions());

        updatePoseEstimatorWithVisionBotPose();

        SmartDashboard.putBoolean("Aliance", getAlliance());
    }

}
