package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

public class Constants {
    public class Chassis {
        public static final double ROBOT_MASS = 23.05;  //kg
        public static final double LOOP_TIME = 0.13;  //s, 20ms + 110ms SPARK MAX velocity lag
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    }

    public class drivePIDF{
        public static final double driveKP = 0.009;
        public static final double driveKI = 0.005;
        public static final double driveKD = 0;
        public static final double driveIzone = 0;                
    }

    public class turnPIDF{
        public static final double turnKP = 0.01;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
        public static final double turnIzone = 0;
    }
}
