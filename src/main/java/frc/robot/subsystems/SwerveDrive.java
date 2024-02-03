package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    private final double gyroOffset = 0.0;

    private final SwerveDriveOdometry driveOdometry;
    private Pose2d m_pose;
    private final Pose2d m_startingPose = null;
    private ChassisSpeeds chassisSpeed;
    
    /** Initializes a new SwerveDrive subsystem object. */
    public SwerveDrive() {
        frontRight = new SwerveModule(21, false, 22, false, 23, SwerveConstants.Dimensions.magOffsetFL);
        frontLeft = new SwerveModule(31, false, 32, false, 33, SwerveConstants.Dimensions.magOffsetFR);
        backRight = new SwerveModule(41, false, 42, false, 43, SwerveConstants.Dimensions.magOffsetBL);
        backLeft = new SwerveModule(51, false, 52, false, 53, SwerveConstants.Dimensions.magOffsetBR);

        gyro = new AHRS(SerialPort.Port.kMXP);
        // Can't call `gyro.reset()` when the gyro is calibration so we defer calling it on another thread

        driveOdometry = 
            new SwerveDriveOdometry(
                Kinematics.driveKinematics, gyro.getRotation2d(),
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backLeft.getPosition()
                }, new Pose2d(5.0, 13.5, new Rotation2d()));

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                // this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();
            } catch (Exception e) {
                System.out.println("Init failure!");
                System.out.println(e);
            }
            
        }).start();
    }
    


    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = gyro.getRotation2d();

        // Update the pose
        m_pose = driveOdometry.update(gyroAngle,
            new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()
    });
    }

    /**
     * Gets the heading of the robot clamped within 360 degrees.
     *
     * @return Robot heading in Rotation2d
     */
    public Rotation2d getRotation2d() {
        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360 - gyro.getYaw() - gyroOffset);
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    /** Stops all motors. */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setDirection(double xSpeed, double ySpeed, double turningSpeed, boolean fieldOriented) {


        xSpeed *= JoystickConstants.xySpeedMultiplier;
        ySpeed *= JoystickConstants.xySpeedMultiplier;
        turningSpeed *= JoystickConstants.turningSpeedMultiplier;

        if (fieldOriented) {
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, this.getRotation2d());
        } else {
            // The robot is rotated 90 degrees from what we expect so we change how we pass the speeds
            chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = SwerveConstants.Kinematics.driveKinematics.toSwerveModuleStates(chassisSpeed);
        this.setStates(moduleStates, true);
    }

    /**
     * Sets the swerve modules to a new state.
     *
     * @param desiredStates The new desired states
     * @param log           If the Swerve Modules should log there target state and
     *                      current state
     */
    private void setStates(SwerveModuleState[] desiredStates, boolean log) {
        // Normalize the speed so we don attempt to overpower motors
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.Dimensions.drivePhysicalMaxSpeed);

        frontLeft.setState(desiredStates[0], log);
        frontRight.setState(desiredStates[1], log);
        backLeft.setState(desiredStates[2], log);
        backRight.setState(desiredStates[3], log);
    }

    /**
     * Put some of the available gyro data to the dashboard.
     */
    public void putGyroData() {
        SmartDashboard.putNumber("IMU_Yaw", gyro.getYaw());
        SmartDashboard.putNumber("IMU_Pitch", gyro.getPitch());
        SmartDashboard.putNumber("IMU_Roll", gyro.getRoll());
    }

    public Pose2d getPose() {
        return m_pose;
    }

    public void resetPose(Pose2d pose) {
        m_pose = pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return chassisSpeed;
    }
}
