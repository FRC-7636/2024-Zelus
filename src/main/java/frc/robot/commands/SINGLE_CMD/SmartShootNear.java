package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

public class SmartShootNear extends Command {
    private final Shooter shooter;
    private final Intake intake;

    public SmartShootNear(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(this.shooter, this.intake);
    }


    @Override
    public void execute() {
        Pose3d robotToTarget = LimelightHelpers.getBotPose3d_TargetSpace("");
        double deltaDegY = 90 - Math.toDegrees(Math.atan2(-robotToTarget.getZ(), robotToTarget.getY() + 0.45));
        System.out.println(deltaDegY);
        shooter.setPosition(deltaDegY - ShooterConstants.Config.ANGLE_OFFSET);

        shooter.shootNear();
        if (shooter.readyToShoot()) {
            intake.conveyorShoot();
            shooter.transport();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: return "true" when the note is out
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        intake.stopConvey();
    }
}