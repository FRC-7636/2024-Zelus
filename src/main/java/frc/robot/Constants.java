package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

/*
 * CAN ID rules:
 *  0-9: Drive Base (configured in json)
 *  10-19: Shooter
 *  20-29: Intake
 *  30-39: Climber
 *  40+: Other
 */
public class Constants {
    public static class Chassis {
        public static final double ROBOT_MASS = 31.8;  // kg
        public static final double LOOP_TIME = 0.13;  // s, 20ms + 110ms SPARK MAX velocity lag
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double MODULE_MAX_SPEED = 5.88264;  // L3, NEO
    }

    public static class AutoDrivePIDF {
        public static final double P = 6;
        public static final double I = 0.00;
        public static final double D = 0;
        public static final double I_ZONE = 0;
    }

    public static class AutoTurnPIDF {
        public static final double P = 1.55;
        public static final double I = 0.0;
        public static final double D = 0.00;
        public static final double I_ZONE = 0.0;
    }

    public static class AutoAimPID {
        public static final double P = 0.05;
        public static final double I = 0;
        public static final double D = 0;
    }

    public static class ShooterConstants {
        public static class Config {
            // CAN ID: 10-19
            public static final int L_ID = 0;
            public static final int R_ID = 0;
            public static final int TRANS_ID = 0;
            public static final int ANGLE_ID = 0;
            public static final boolean SHOOTER_INVERTED = false;
            public static final boolean TRANS_INVERTED = false;
            public static final boolean ANGLE_INVERTED = false;
            public static final int SENSOR_ID = 9;
            public static final int CURRENT_LIMIT = 35;
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

    public static class IntakeConstants {
        public static class Config {
            // CAN ID: 20-29
            public static final int PIPE_ID = 50;
            public static final int ANGLE_ID = 50;
            public static final int ITS_ID = 0;
            public static final boolean PIPE_INVERTED = false;
            public static final boolean ANGLE_INVERTED = false;
            public static final boolean ITS_INVERTED = false;
            public static final int CURRENT_LIMIT = 35;
        }
        public static class AnglePIDF {
            public static final double P = 0;
            public static final double I = 0;
            public static final double D = 0;
        }
        public static class Control {
            public static final double FLOOR_POSITION = 0;
            public static final double AMP_POSITION = 0;
            public static final double TRAP_POSITION = 0;
            public static final double ORIGIN_POSITION = 0;
            public static final double SHOOT_SPEED = 0;
            public static final double SUCK_SPEED = 0;
            public static final double ITS_SPEED = 0;
        }
    }

    public static class ClimberConstants {
        public static class Config {
            // CAN ID: 30-39
            public static final int F_ID = 0;
            public static final int R_ID = 0;
            public static final boolean INVERTED = false;
            public static final double MAX_ACCEL = 0;
            public static final double MAX_VELOCITY = 0;
        }

        public static class Control {
            public static final double FLOOR = 0;
            public static final double AMP = 0;
            public static final double BALANCE = 0;
            public static final double TRAP = 0;
        }

        public static class PIDF {
            public static final double P = 0;
            public static final double I = 0;
            public static final double D = 0;
            public static final double F = 0;
        }
    }

    public static class VisionConstants {
        public static final String NAME = "limelight";
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0.46, 0.365, -0.7,  // Unit: Meters
                new Rotation3d(0, 0.125*Math.PI, 0));  // Unit: Radians
    }
}
