package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveConstants;

/** The subsystem that handles swerve drive. */
public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final AHRS gyro;
    private final double gyroOffset = 0.0;

    /** Initializes a new SwerveDrive subsystem object. */
    public SwerveDrive() {
        frontLeft = new SwerveModule(21, false, 22, false, 23, SwerveConstants.Dimensions.magOffsetFL);
        frontRight = new SwerveModule(31, false, 32, false, 33, SwerveConstants.Dimensions.magOffsetFR);
        backLeft = new SwerveModule(41, false, 42, false, 43, SwerveConstants.Dimensions.magOffsetBL);
        backRight = new SwerveModule(51, false, 52, false, 53, SwerveConstants.Dimensions.magOffsetBR);

        gyro = new AHRS(SerialPort.Port.kMXP);
        // Can't call `gyro.reset()` when the gyro is calibration so we defer calling it on another thread
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
    public void periodic() {}

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
        ChassisSpeeds chassisSpeed;

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
}
