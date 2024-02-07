package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Kinematics;

/** The subsystem that handles swerve drive. */
public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final AHRS gyro;
    private final double gyroOffset = 90.0;

    private final SwerveDriveOdometry driveOdometry;

    /** Initializes a new SwerveDrive subsystem object. */
    public SwerveDrive() {
        frontLeft =
                new SwerveModule(21, false, 22, false, 23, SwerveConstants.Dimensions.magOffsetFL);
        frontRight =
                new SwerveModule(31, false, 32, false, 33, SwerveConstants.Dimensions.magOffsetFR);
        backLeft =
                new SwerveModule(41, false, 42, false, 43, SwerveConstants.Dimensions.magOffsetBL);
        backRight =
                new SwerveModule(51, false, 52, false, 53, SwerveConstants.Dimensions.magOffsetBR);

        frontRight.reset();
        frontLeft.reset();
        backRight.reset();
        backLeft.reset();

        gyro = new AHRS(SerialPort.Port.kMXP);
        this.InitGyro();

        driveOdometry =
                new SwerveDriveOdometry(
                        Kinematics.driveKinematics,
                        gyro.getRotation2d(),
                        new SwerveModulePosition[] {
                            frontLeft.getPosition(),
                            frontRight.getPosition(),
                            backLeft.getPosition(),
                            backRight.getPosition()
                        },
                        new Pose2d(0.0, 0.0, new Rotation2d()));

        AutoBuilder.configureHolonomic(
                driveOdometry::getPoseMeters, // Needs to be updated to pose estimation
                (Pose2d pose) ->
                        driveOdometry.resetPosition(
                                gyro.getRotation2d(),
                                new SwerveModulePosition[] {
                                    frontLeft.getPosition(), frontRight.getPosition(),
                                    backLeft.getPosition(), backRight.getPosition()
                                },
                                pose),
                this::getRobotRelativeSpeeds,
                this::setSpeed,
                SwerveConstants.HolonomicConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    /**
     * Gets the heading of the robot clamped within 360 degrees.
     *
     * @return Robot heading in Rotation2d
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(360 - gyro.getYaw() - gyroOffset);
    }

    /** Stops all motors. */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setDirection(
            double xSpeed, double ySpeed, double turningSpeed, boolean fieldOriented) {

        xSpeed *= JoystickConstants.xySpeedMultiplier;
        ySpeed *= JoystickConstants.xySpeedMultiplier;
        turningSpeed *= JoystickConstants.turningSpeedMultiplier;

        if (fieldOriented) {
            setSpeed(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeed, ySpeed, turningSpeed, this.getRotation2d()));
        } else {
            setSpeed(new ChassisSpeeds(xSpeed, ySpeed, turningSpeed));
        }
    }

    private void setSpeed(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = Kinematics.driveKinematics.toSwerveModuleStates(speeds);
        this.setStates(moduleStates, true);
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = gyro.getRotation2d();

        // Update the pose
        driveOdometry.update(
                gyroAngle,
                new SwerveModulePosition[] {
                    frontLeft.getPosition(), frontRight.getPosition(),
                    backLeft.getPosition(), backRight.getPosition()
                });

        Pose2d pose = driveOdometry.getPoseMeters();
        SmartDashboard.putString("Pose", "x: " + pose.getX() + ", y: " + pose.getY());
    }

    /**
     * Sets the swerve modules to a new state.
     *
     * @param desiredStates The new desired states
     * @param log If the Swerve Modules should log there target state and current state
     */
    private void setStates(SwerveModuleState[] desiredStates, boolean log) {
        // Normalize the speed so we don attempt to overpower motors
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, SwerveConstants.Kinematics.drivePhysicalMaxSpeed);

        frontLeft.setState(desiredStates[0], log);
        frontRight.setState(desiredStates[1], log);
        backLeft.setState(desiredStates[2], log);
        backRight.setState(desiredStates[3], log);
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return Kinematics.driveKinematics.toChassisSpeeds(
                new SwerveModuleState[] {
                    frontLeft.getState(), frontRight.getState(),
                    backLeft.getState(), backRight.getState()
                });
    }

    private void InitGyro() {
        new Thread(
                        () -> {
                            try {
                                Thread.sleep(1000);
                                gyro.reset();
                            } catch (Exception e) {
                                System.out.println("Init failure!");
                                System.out.println(e);
                            }
                        })
                .start();
    }
}
