package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
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

    private final Odometry m_odometry;
    private final Vision m_vision;
    
    private final AHRS gyro;
    private final double gyroOffset = 270.0;
    private double gyroOffsetInit;

    /** Initializes a new SwerveDrive subsystem object. */
    public SwerveDrive(Vision m_vision) {
        this.m_vision = m_vision;

        this.frontLeft =
                new SwerveModule(21, false, 22, false, 23, SwerveConstants.Dimensions.magOffsetFL);
        this.frontRight =
                new SwerveModule(31, false, 32, false, 33, SwerveConstants.Dimensions.magOffsetFR);
        this.backLeft =
                new SwerveModule(41, false, 42, false, 43, SwerveConstants.Dimensions.magOffsetBL);
        this.backRight =
                new SwerveModule(51, false, 52, false, 53, SwerveConstants.Dimensions.magOffsetBR);

        frontRight.reset();
        frontLeft.reset();
        backRight.reset();
        backLeft.reset();

        gyro = new AHRS(SerialPort.Port.kMXP);
        this.InitGyro();
        this.m_odometry = new Odometry(this, m_vision);
    }

    public void init() {
        gyroOffsetInit = m_odometry.getRobotPose().getRotation().getDegrees();
        gyro.setAngleAdjustment(gyroOffsetInit);
    }

    public Odometry getOdometry() {
        return m_odometry;
    }

    /**
     * Gets the heading of the robot clamped within 360 degrees.
     *
     * @return Robot heading in Rotation2d
     */ 
    public Rotation2d getRotation2d() {
        if (gyro == null) return Rotation2d.fromDegrees(0);
        return gyro.getRotation2d();
    }

    /** Stops all motors. */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
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

    public void setSpeed(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = Kinematics.driveKinematics.toSwerveModuleStates(speeds);
        this.setStates(moduleStates, true);
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {}

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

    public ChassisSpeeds getRobotRelativeSpeeds() {
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
