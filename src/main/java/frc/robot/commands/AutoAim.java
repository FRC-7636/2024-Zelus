package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAimPID;

import java.util.Arrays;


public class AutoAim extends Command {
//    private final PhotonVision photonVision;
    private final Swerve swerve;
    private final PIDController pidCtrl = new PIDController(AutoAimPID.P, AutoAimPID.I, AutoAimPID.D);

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
        double deltaDeg = LimelightHelpers.getTX("");
        double deltaRad = Units.degreesToRadians(deltaDeg);
        SmartDashboard.putNumber("Delta Rad.", deltaDeg);
        Rotation2d targetHeading = swerve.getHeading().plus(new Rotation2d(deltaRad));
        SmartDashboard.putNumber("Heading offset", targetHeading.getDegrees());
//        Trajectory trajToTargetPose;
//        if (tagId == 4d || tagId == 7d) {
//            Rotation2d targetHeading = new Rotation2d(deltaRad);
//            Pose2d targetPose = swerve.getPose().rotateBy(targetHeading);
//            trajToTargetPose = TrajectoryGenerator.generateTrajectory(
//                    swerve.getPose(),
//                    new ArrayList<>(),
//                    targetPose,
//                    new TrajectoryConfig(10, 10));
//        }
//        else {
//            trajToTargetPose = new Trajectory();
//        }
//
//        swerve.postTrajectory(trajToTargetPose);
        ChassisSpeeds targetSpeeds = swerve.getTargetSpeeds(0, 0, targetHeading);
        SmartDashboard.putString("AutoAim output", Arrays.toString(new Double[]{
                targetSpeeds.vxMetersPerSecond,
                targetSpeeds.vyMetersPerSecond,
                targetSpeeds.omegaRadiansPerSecond
        }));
//        swerve.drive(targetSpeeds);
        if (!isFinished()) {
            swerve.drive(new Translation2d(), pidCtrl.calculate(deltaDeg), false);
        } else {
            swerve.drive(new Translation2d(), 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        double deltaDeg = Math.abs(LimelightHelpers.getTX(""));
        return deltaDeg < 5;
    }
}
