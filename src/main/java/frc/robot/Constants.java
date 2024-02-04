package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

public class Constants {
    public static class Chassis {
        public static final double ROBOT_MASS = 23.05;  //kg
        public static final double LOOP_TIME = 0.13;  //s, 20ms + 110ms SPARK MAX velocity lag
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    }

    public static class drivePIDF {
        public static final double driveKP = 0.07;
        public static final double driveKI = 0.007;
        public static final double driveKD = 0;
        public static final double driveIzone = 0;
    }

    public static class turnPIDF {
        public static final double turnKP = 1;
        public static final double turnKI = 0.003;
        public static final double turnKD = 0.005;
        public static final double turnIzone = 0;
    }

    public static class VisionConstants {
        public static final String NAME = "limelight";
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0.46, 0.365, -0.7,  // Unit: Meters
                new Rotation3d(0, 0.125*Math.PI, 0));  // Unit: Radians
    }
}
