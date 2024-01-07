// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveConstants {
    public static final double wheelDiameter = Units.inchesToMeters(2); // Meters
    public static final double driveGearRatio = 6.12;

    public static final double driveRotToMeters = ((2 * Math.PI * wheelDiameter) / driveGearRatio) / 2048; // Drive motor rotations to meters
    public static final double driveRpsToMps = driveRotToMeters / 10 * 60 * 2; // Drive motor rotations per second to meters per second

    public static final double drivePhysicalMaxSpeed = 5.486; // Physical max speed of the motor in m/s

    public static final double trackWidth = Units.inchesToMeters(23.5); // Distance between the center of the left and right wheels in meters
    public static final double wheelBase = Units.inchesToMeters(21.5); // Distance between the center of the front and back wheels in meters
    public static final Translation2d locationFL = new Translation2d(wheelBase / 2,  trackWidth / 2);
    public static final Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
    public static final Translation2d locationBL = new Translation2d(-wheelBase / 2,  trackWidth / 2);
    public static final Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        locationFL,
        locationFR,
        locationBL,
        locationBR
    );
}

/** Constants for the operator interface (OI). */
public static final class OiConstants {
    public static final int driveControllerId = 0;
    public static final int scoreControllerId = 1;
    
    public static final double joystickDeadZone = 0.1; // The circular zone around "zero" to ignore. Prevents joystick drift from becoming an issue
    public static final double triggerDeadZone = 0.1; // Dead zone on the triggers. Prevents drift from becoming an issue

    public static final double xySpeedMultiplier = 5; // Speed multiplier in m/s(?)
    public static final double turningSpeedMultiplier = 1.25 * Math.PI;
    public static final double slowTurningDivisor = 2;
}
}
