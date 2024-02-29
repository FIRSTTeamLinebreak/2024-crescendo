package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Kinematics;

public class Odometry extends SubsystemBase {

    private final SwerveDriveOdometry m_driveOdometry;
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

        m_driveOdometry =
                new SwerveDriveOdometry(
                        Kinematics.driveKinematics,
                        m_driveSubsystem.getRotation2d(),
                        m_driveSubsystem.getModulePositions(),
                        new Pose2d(5.0, 13.5, new Rotation2d()));

        AutoBuilder.configureHolonomic(
                this::getRobotPose, // Needs to be updated to pose estimation
                (Pose2d pose) ->
                        m_driveOdometry.resetPosition(
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
        return m_driveOdometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        m_driveOdometry.update(
            m_driveSubsystem.getRotation2d(),
            m_driveSubsystem.getModulePositions()
        );

        Pose2d odometryPose = m_driveOdometry.getPoseMeters();
        Pose2d visionPose = m_vision.getRobotPose();

        m_poseOdoPublisher.set(odometryPose);
        m_poseVisPublisher.set(visionPose);
    }

}
