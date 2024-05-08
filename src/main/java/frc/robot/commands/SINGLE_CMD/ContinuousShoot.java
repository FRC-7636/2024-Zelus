package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;

public class ContinuousShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;

    public ContinuousShoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(this.shooter, this.intake);
    }


    @Override
    public void execute() {
        intake.setFloorAngle();
        intake.suck();
        intake.startConvey();
        shooter.transport();
        Pose3d robotToTarget = LimelightHelpers.getBotPose3d_TargetSpace("");
        double deltaDegY = 90 - Math.toDegrees(Math.atan2(-robotToTarget.getZ(), robotToTarget.getY() + 0.53));
        System.out.println(deltaDegY - ShooterConstants.Config.ANGLE_OFFSET);
        shooter.setPosition(MathUtil.clamp(deltaDegY - ShooterConstants.Config.ANGLE_OFFSET, 5, 45));
        shooter.shoot();
    }
}
