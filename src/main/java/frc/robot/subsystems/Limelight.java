package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Retro;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight extends SubsystemBase{
    private final Field2d field2d = new Field2d();
    private final Pose2d pose2d = new Pose2d();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    DoubleArraySubscriber CameraSeeBot = table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[]{});

    public Limelight() {
        
    }

    @Override
    public void periodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        field2d.setRobotPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
        SmartDashboard.putData("L_Field2d", field2d);
    }

    public Pose2d getPose2d(){
        return LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    }
}