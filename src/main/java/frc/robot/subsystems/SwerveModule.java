package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

/** An individual swerve module. */
public class SwerveModule {
    // Drive
    private final TalonFX driveController;

    // Turning
    private final CANSparkMax turningController;
    private final PIDController turningPid;

    // CAN Coder
    private final CANcoder canCoder;

    /** Creates a new swerve module.
     *
     * @param driveId CAN ID for the drive motor (Falcon)
     * @param isDriveReversed If the drive motor is reversed
     * @param turningId CAN ID for the turning motor (NEO)
     * @param isTurningReversed If the turning motor is reversed
     * @param coderId CAN ID for the CAN coder within the swerve module
     * @param coderOffset The offset to get the CAN coder to true zero (NEED TO BE POSITIVE)
     */
    public SwerveModule(int driveId, boolean isDriveReversed, int turningId, boolean isTurningReversed, int coderId, double coderOffset) {
        // Drive
        driveController = new TalonFX(driveId);
        driveController.setInverted(isDriveReversed);

        // Turning
        turningController = new CANSparkMax(turningId, MotorType.kBrushless);
        turningController.setInverted(isTurningReversed);
        turningController.restoreFactoryDefaults(true);
        turningController.setIdleMode(IdleMode.kBrake);

        turningPid = new PIDController(0.7, 0.0, 0.0);
        turningPid.enableContinuousInput(0, 2 * Math.PI); // Tell the PID controller that it can go through -PI and PI because its a circle
        turningPid.setTolerance(0.05);

        // CAN Coder
        canCoder = new CANcoder(coderId);

        CANcoderConfiguration config = new CANcoderConfiguration();
        // config.sensorCoefficient = 2 * Math.PI / 409V6.0;
        // config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // config.unitString = "rad";
        // config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.MagnetOffset = coderOffset;

        canCoder.getConfigurator().apply(config);
    }

    /** Creates a SwerveModuleState for this swerve module given its position and velocity.
     * This is needed for WPILib functions.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /** Sets the swerve module to a new state.
     *
     * @param state The new desired state
     * @param log If the Swerve Module should log there target state and current state
     */
    public void setState(SwerveModuleState state, boolean log) {
        if (log) {
            SmartDashboard.putString(
                "Swerve " + Integer.toString(driveController.getDeviceID() - 10).charAt(0) + " Target State",
                String.format("Speed: %.3f, Rotation: %.3f", state.speedMetersPerSecond, state.angle.getRadians())
            );
            SmartDashboard.putString(
                "Swerve " + Integer.toString(driveController.getDeviceID() - 10).charAt(0) + " Current State",
                String.format("Speed: %.3f, Rotation: %.3f", getDriveVelocity(), getTurningPositionReadable())
            );
            SmartDashboard.putString(
                "Swerve " + Integer.toString(driveController.getDeviceID() - 10).charAt(0) + " Current Position",
                String.format("Position: %.3f", getDrivePosition())
            );
        }

        if (Math.abs(state.speedMetersPerSecond) <= .01) { // Implements a "deadzone" so releasing the joystick won't make the wheels reset to 0
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getTurningPosition())); // Optimize movements to not move more than 90 deg for any new state
        driveController.set(state.speedMetersPerSecond / SwerveConstants.drivePhysicalMaxSpeed);
        turningController.set(turningPid.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    /** Stops the swerve module. */
    public void stop() {
        driveController.set(0);
        turningController.set(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    }

    /** Gets the drive motor position.
     *
     * @return Drive motor position
     */
    public double getDrivePosition() {
        return driveController.getPosition().getValueAsDouble() * SwerveConstants.driveRotToMeters;
    }

    /** Gets the velocity of the drive motor.
     *
     * @return Drive motor velocity
     */
    public double getDriveVelocity() {
        return driveController.getVelocity().getValueAsDouble() * SwerveConstants.driveRpsToMps; // @TODO use the correct math to convert to mps
    }

    /** Gets the turning motor position in radians.
     *
     * @return Turning motor position
     */
    public double getTurningPosition() {
        return canCoder.getPosition().getValueAsDouble();
    }

    /** Gets the turning motor position in radians in a human readable form.
     *
     * @return Turning motor position
     */
    public double getTurningPositionReadable() {
        return canCoder.getPosition().getValueAsDouble() % 2 * Math.PI;
    }
}