package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.ShooterConstants;


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
                Pose3d robotToTarget = LimelightHelpers.getBotPose3d_TargetSpace("");
        double deltaDegY = 90 - Math.toDegrees(Math.atan2(-robotToTarget.getZ(), robotToTarget.getY() + 0.53));
        System.out.println(deltaDegY);
        shooter.setPosition(MathUtil.clamp(deltaDegY - ShooterConstants.Config.ANGLE_OFFSET, 5, 32));
    }
}
