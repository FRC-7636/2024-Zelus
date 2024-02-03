package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

public class Constants {
    public static class Chassis {
        public static final double ROBOT_MASS = 25.23;  //kg
        public static final double LOOP_TIME = 0.13;  //s, 20ms + 110ms SPARK MAX velocity lag
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double AUTO_MAX_SPEED = 3.81;  //m/s
    }

    public static class IntakeConstants {
        public static final int pipeIntakePort = 50;
        public static final int angleIntakePort = 50;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        //比賽時可做調整
        public static final double kfloorAngle = 0;
        public static final double kAMPangle = 0;
        public static final double ktrapAngle = 0;
        public static final double koriginAngle = 0;
    }

    public static class VisionConstants {
        public static final Pose3d CAMERA_TO_ROBOT = new Pose3d(0, 0, 0, new Rotation3d()); 
        
    }

    public static class drivePIDF {
        public static final double driveKP = 0.07;
        public static final double driveKI = 0;
        public static final double driveKD = 0;
        public static final double driveIzone = 0;                
    }

    public static class turnPIDF {
        public static final double turnKP = 1;
        public static final double turnKI = 0.003;
        public static final double turnKD = 0.005;
        public static final double turnIzone = 0;
    }
}
