package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

public class Constants {
    public static class Chassis {
        public static final double ROBOT_MASS = 25.23;  //kg
        public static final double LOOP_TIME = 0.13;  //s, 20ms + 110ms SPARK MAX velocity lag
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double MODULE_MAX_SPEED = 5.88264;  // L3, NEO
    }

    public static class drivePIDF {
        public static final double P = 0.07;
        public static final double I = 0.007;
        public static final double D = 0;
        public static final double IZone = 0;
    }

    public static class turnPIDF {
        public static final double P = 1;
        public static final double I = 0.003;
        public static final double D = 0.005;
        public static final double IZone = 0;
    }

    public static class AutoAimPID {
        public static final double P = 0.05;
        public static final double I = 0;
        public static final double D = 0;
    }

    public static class VisionConstants {
        public static final String NAME = "limelight";
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0.46, 0.365, -0.7,  // Unit: Meters
                new Rotation3d(0, 0.125*Math.PI, 0));  // Unit: Radians
    }
}
