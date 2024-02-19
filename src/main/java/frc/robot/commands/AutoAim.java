package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

import java.util.ArrayList;


public class AutoAim extends Command {
//    private final PhotonVision photonVision;
    private final Swerve swerve;

//    public AutoAim(PhotonVision photonVision, Swerve swerve) {
//        this.photonVision = photonVision;
//        this.swerve = swerve;
//        // each subsystem used by the command must be passed into the
//        // addRequirements() method (which takes a vararg of Subsystem)
//        addRequirements(this.photonVision, this.swerve);
//    }

    public AutoAim(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        double deltaDeg = Units.degreesToRadians(LimelightHelpers.getTX(VisionConstants.NAME));
        double tagId = LimelightHelpers.getFiducialID(VisionConstants.NAME);
        SmartDashboard.putNumber("Delta Deg.", deltaDeg);
        SmartDashboard.putNumber("Current ID", tagId);
        Trajectory trajToTargetPose;
        if (tagId == 4d || tagId == 7d) {
            Rotation2d targetHeading = new Rotation2d(deltaDeg);
            Pose2d targetPose = swerve.getPose().rotateBy(targetHeading);
            trajToTargetPose = TrajectoryGenerator.generateTrajectory(
                    swerve.getPose(),
                    new ArrayList<>(),
                    targetPose,
                    new TrajectoryConfig(10, 10));
        }
        else {
            trajToTargetPose = new Trajectory();
        }

        swerve.postTrajectory(trajToTargetPose);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
