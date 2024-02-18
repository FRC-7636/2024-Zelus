package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

import java.util.ArrayList;


public class AutoAim extends Command {
//    private final PhotonVision photonVision;
    private final Swerve swerve;
    private Trajectory trajToTargetPose;

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
    public void initialize() {
        Rotation2d targetHeading = new Rotation2d(Units.degreesToRadians(LimelightHelpers.getTX(VisionConstants.NAME)));
        Pose2d targetPose = swerve.getPose().rotateBy(targetHeading);
        trajToTargetPose = TrajectoryGenerator.generateTrajectory(
                swerve.getPose(),
                new ArrayList<>(),
                targetPose,
                new TrajectoryConfig(10, 10));
    }

    @Override
    public void execute() {
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
