package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAimPID;
import frc.robot.Constants.ShooterConstants;

public class SmartShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final Swerve swerve;
     private final PIDController yawCtrl = new PIDController(AutoAimPID.P, AutoAimPID.I, AutoAimPID.D);

    public SmartShoot(Shooter shooter, Intake intake, Swerve swerve) {
        this.shooter = shooter;
        this.intake = intake;
        this.swerve = swerve;

        addRequirements(this.shooter, this.intake, this.swerve);
    }


    @Override
    public void execute() {
        Pose3d robotToTarget = LimelightHelpers.getBotPose3d_TargetSpace("");
        double deltaDegY = 90 - Math.toDegrees(Math.atan2(-robotToTarget.getZ(), robotToTarget.getY() + 0.53));
        System.out.println(deltaDegY - ShooterConstants.Config.ANGLE_OFFSET);
        shooter.setPosition(MathUtil.clamp(deltaDegY - ShooterConstants.Config.ANGLE_OFFSET, 5, 45));
//        shooter.setPosition(ShooterConstants.Control.NEARSHOOT_POSITION);
        double deltaDegX = LimelightHelpers.getTX("");
        double deltaRad = Math.toRadians(deltaDegX);
        SmartDashboard.putNumber("Delta Rad.", deltaDegX);
        Rotation2d targetHeading = swerve.getHeading().plus(new Rotation2d(deltaRad));
        SmartDashboard.putNumber("Heading offset", targetHeading.getDegrees());
        shooter.shoot();

        if (!isFinished()) {
            swerve.drive(new Translation2d(), yawCtrl.calculate(deltaDegX), false);
        } else {
            swerve.drive(new Translation2d(), 0, false);
        }

        if (shooter.readyToShoot()) {
            intake.conveyorShoot();
            shooter.transport();
        }
    }

    @Override
    public boolean isFinished() {
        return !shooter.noteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        intake.stopConvey();
    }
}
