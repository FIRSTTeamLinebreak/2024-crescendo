package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
    /** Contstants pertaining to the Swerve Modules */
    public static final class SwerveConstants {
        public static final class Dimensions {
            public static final double wheelDiameter = Units.inchesToMeters(2);
            public static final double driveGearRatio = 6.12;

            public static final double driveRotToMeters =
                ((2 * Math.PI * wheelDiameter) / driveGearRatio);
            public static final double driveRpsToMps = driveRotToMeters / 10 * 60 * 2;

            public static final double drivePhysicalMaxSpeed = 5.486;

            public static final double wheelBase = Units.inchesToMeters(23.5);
            public static final double trackWidth = Units.inchesToMeters(21.5);

            // public static final double magOffsetFL = 0.469;
            // public static final double magOffsetFR = 0.126;
            // public static final double magOffsetBL = 0.853;
            // public static final double magOffsetBR = 0.520;
            public static final double magOffsetFL = 0.392;
            public static final double magOffsetFR = 0.220;
            public static final double magOffsetBL = 0.715;
            public static final double magOffsetBR = 0.888;
        }

        public static final class Kinematics {
            public static final Translation2d locationFL =
                new Translation2d(Dimensions.wheelBase / 2, Dimensions.trackWidth / 2);
            public static final Translation2d locationFR =
                new Translation2d(Dimensions.wheelBase / 2, -Dimensions.trackWidth / 2);
            public static final Translation2d locationBL =
                new Translation2d(-Dimensions.wheelBase / 2, Dimensions.trackWidth / 2);
            public static final Translation2d locationBR =
                new Translation2d(-Dimensions.wheelBase / 2, -Dimensions.trackWidth / 2);

            public static final SwerveDriveKinematics driveKinematics =
                new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
        }
        

        public static final class PIDConstants {
            public static final double kP= 0.7;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
                    
            public static final double kT = 0.001;
        }

        public static final class HolonomicPathFollowerConfig {

            public static final class AutoPIDConstants {

                public static final double rP = 0.1;
                public static final double rI = 0.0;
                public static final double rD = 0.0;
                
                public static final double tP = 0.1;
                public static final double tI = 0.0;
                public static final double tD = 0.0;

            }

            public static final double autoDriveMaxModuleSpeed = 5.486;

        }
    }

    /** Constants pertaining to the joystick. */
    public static final class JoystickConstants {
        public static final int driveControllerId = 0;
        public static final int scoreControllerId = 1;

        public static final double joystickDeadZone = 0.1;
        public static final double triggerDeadZone = 0.1;

        public static final double xySpeedMultiplier = 1; // Speed multiplier in m/s(?)
        public static final double turningSpeedMultiplier = 1.25;
        public static final double slowTurningDivisor = 2;
    }
}
