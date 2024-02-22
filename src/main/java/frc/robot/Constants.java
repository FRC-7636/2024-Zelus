package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

public class Constants {
    public static class Chassis {
        public static final double ROBOT_MASS = 31.8;  //kg
        public static final double LOOP_TIME = 0.13;  //s, 20ms + 110ms SPARK MAX velocity lag
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double MODULE_MAX_SPEED = 5.88264;  // L3, NEO
    }

    public static class AutoDrivePIDF {
        public static final double P = 35;
        public static final double I = 0.001;
        public static final double D = 0;
        public static final double IZone = 0;
    }

    public static class AutoTurnPIDF {
        public static final double P = 0.009;
        public static final double I = 0;
        public static final double D = 0.001;
        public static final double IZone = 0.005;
    }

    public static class AutoAimPID {
        public static final double P = 0.05;
        public static final double I = 0;
        public static final double D = 0;
    }

    public static class ShooterConstants {
        public static class Config {
            public static final int L_ID = 0;
            public static final int R_ID = 0;
            public static final int TRANS_ID = 0;
            public static final int ANGLE_ID = 0;
            public static final boolean SHOOTER_INVERTED = false;
            public static final boolean TRANS_INVERTED = false;
            public static final boolean ANGLE_INVERTED = false;
            public static final int SENSOR_ID = 9;
        }
        public static class Control {
            public static final double SHOOT_VELOCITY = 0;  // Absolute
            public static final double SUCK_SPEED = 0;  // Relative
            public static final double SHOOTER_POSITION = 0;
            public static final double TRANS_SPEED = 0;
        }
        public static class AnglePIDF {
            public static final double P = 0;
            public static final double I = 0;
            public static final double D = 0;
        }
        public static class LeftPIDF {
            public static final double P = 0;
            public static final double I = 0;
            public static final double D = 0;
        }
        public static class RightPIDF {
            public static final double P = 0;
            public static final double I = 0;
            public static final double D = 0;
        }
    }

    public static class VisionConstants {
        public static final String NAME = "limelight";
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0.46, 0.365, -0.7,  // Unit: Meters
                new Rotation3d(0, 0.125*Math.PI, 0));  // Unit: Radians
    }
}
