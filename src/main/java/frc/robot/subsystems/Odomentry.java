package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Kinematics;

public class Odomentry extends SubsystemBase {

    private final SwerveDriveOdometry m_driveOdometry;
    private final SwerveDrive m_dirveSubsystem;
    private final Vision m_vision;

    public Odomentry(SwerveDrive driveSubsystem, Vision vison) {
        m_dirveSubsystem = driveSubsystem;
        m_vision = vison;

        m_driveOdometry =
                new SwerveDriveOdometry(
                        Kinematics.driveKinematics,
                        m_dirveSubsystem.getRotation2d(),
                        m_dirveSubsystem.getModulePositions(),
                        new Pose2d(0.0, 0.0, new Rotation2d()));

        AutoBuilder.configureHolonomic(
                m_driveOdometry::getPoseMeters, // Needs to be updated to pose estimation
                (Pose2d pose) ->
                        m_driveOdometry.resetPosition(
                                m_dirveSubsystem.getRotation2d(),
                                m_dirveSubsystem.getModulePositions(),
                                pose),
                m_dirveSubsystem::getRobotRelativeSpeeds,
                m_dirveSubsystem::setSpeed,
                SwerveConstants.HolonomicConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                m_dirveSubsystem);
    }

    @Override
    public void periodic() {
        m_driveOdometry.update(
            m_dirveSubsystem.getRotation2d(),
            m_dirveSubsystem.getModulePositions()
        );
    }
    
}
