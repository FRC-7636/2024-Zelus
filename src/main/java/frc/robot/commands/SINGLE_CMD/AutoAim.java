package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAimPID;


public class AutoAim extends Command {
//    private final PhotonVision photonVision;
    private final Swerve swerve;
    private final Shooter shooter;
    private final PIDController yawCtrl = new PIDController(AutoAimPID.P, AutoAimPID.I, AutoAimPID.D);

//    public AutoAim(PhotonVision photonVision, Swerve swerve) {
//        this.photonVision = photonVision;
//        this.swerve = swerve;
//        // each subsystem used by the command must be passed into the
//        // addRequirements() method (which takes a vararg of Subsystem)
//        addRequirements(this.photonVision, this.swerve);
//    }

    public AutoAim(Swerve swerve, Shooter shooter) {
        this.swerve = swerve;
        this.shooter = shooter;

        addRequirements(this.swerve, this.shooter);
    }

    @Override
    public void execute() {
        double deltaDegX = LimelightHelpers.getTX("");
        double deltaRad = Math.toRadians(deltaDegX);
        SmartDashboard.putNumber("Delta Rad.", deltaDegX);
        Rotation2d targetHeading = swerve.getHeading().plus(new Rotation2d(deltaRad));
        SmartDashboard.putNumber("Heading offset", targetHeading.getDegrees());
        swerve.drive(new Translation2d(), yawCtrl.calculate(deltaDegX), false);

        Pose3d robotToTarget = LimelightHelpers.getBotPose3d_TargetSpace("");
        double deltaDegY = 90 - Math.toDegrees(Math.atan2(-robotToTarget.getZ(), robotToTarget.getY() + 0.45));
        System.out.println(deltaDegY);
        shooter.setPosition(deltaDegY - 16);
    }
}
