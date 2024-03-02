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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Kinematics;

public class Odometry extends SubsystemBase {

    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDrive m_driveSubsystem;
    private final Vision m_vision;

    private final StructPublisher<Pose2d> m_poseOdoPublisher;
    private final StructPublisher<Pose2d> m_poseVisPublisher;

    public Odometry(SwerveDrive driveSubsystem, Vision vison) {
        m_driveSubsystem = driveSubsystem;
        m_vision = vison;
        m_poseOdoPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Pose_Odo", Pose2d.struct).publish();
        m_poseVisPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Pose_Vis", Pose2d.struct).publish();

        m_poseEstimator = new SwerveDrivePoseEstimator(Kinematics.driveKinematics,
                m_driveSubsystem.getRotation2d(),
                m_driveSubsystem.getModulePositions(),
                new Pose2d(2.0, 2.0, new Rotation2d()));

        AutoBuilder.configureHolonomic(
                this::getRobotPose, // Needs to be updated to pose estimation
                (Pose2d pose) -> m_poseEstimator.resetPosition(
                        m_driveSubsystem.getRotation2d(),
                        m_driveSubsystem.getModulePositions(),
                        pose),
                m_driveSubsystem::getRobotRelativeSpeeds,
                m_driveSubsystem::setSpeed,
                SwerveConstants.HolonomicConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                m_driveSubsystem);
    }

    public Pose2d getRobotPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void updatePoseEstimatorWithVisionBotPose() {
        Pose2d visionPose = m_vision.getVisionPose();
        double visionLatensy = m_vision.getPoseLatensy();
        // invalid LL data
        if (visionPose.getX() == 0.0) {
            return;
        }

        // distance from current pose to vision estimated pose
        double poseDifference = m_poseEstimator.getEstimatedPosition().getTranslation()
                .getDistance(visionPose.getTranslation());

        double xyStds = 0.75;
        double degStds = 9;
        // multiple targets detected
        if (m_vision.getNumberOfTargetsVisible() >= 2) {
            xyStds = 0.5;
            degStds = 6;
        }
        // 1 target with large area and close to estimated pose
        else if (m_vision.getBestTargetArea() > 0.8 && poseDifference < 0.5) {
            xyStds = 1.0;
            degStds = 12;
        }
        // 1 target farther away and estimated pose is close
        else if (m_vision.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
            xyStds = 2.0;
            degStds = 30;
        } else {
            return;
        }

        m_poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
        m_poseEstimator.addVisionMeasurement(visionPose,
                Timer.getFPGATimestamp() - visionLatensy / 1000.0);
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(
                m_driveSubsystem.getRotation2d(),
                m_driveSubsystem.getModulePositions());
        
        updatePoseEstimatorWithVisionBotPose();

        Double[] visionPoseArr = m_vision.getRobotPose();

        Pose2d odometryPose = getRobotPose();
        Pose2d visionPose = new Pose2d(
                visionPoseArr[0],
                visionPoseArr[1],
                new Rotation2d(Units.degreesToRadians(visionPoseArr[5])));

        m_poseOdoPublisher.set(odometryPose);
        m_poseVisPublisher.set(visionPose);
    }

}
