package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

public class Constants {
    public class Chassis {
        public static final double ROBOT_MASS = 23.05;  //kg
        public static final double LOOP_TIME = 0.13;  //s, 20ms + 110ms sprk max velocity lag
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    }
}
