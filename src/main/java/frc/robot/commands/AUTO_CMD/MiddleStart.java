package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.SINGLE_CMD.SmartIntake;

public class MiddleStart extends SequentialCommandGroup {

    public MiddleStart(Swerve m_swerve, Shooter shooter, Intake intake) {
        addCommands(new InstantCommand(intake::setFloorAngle));
        addCommands(new InstantCommand(() -> shooter.setPosition(50)));
        addCommands(new AutoShoot(shooter, intake));
//        addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), m_swerve));
        addCommands(new SmartIntake(shooter, intake)
                .alongWith(m_swerve.getAutonomousCommand("blue 2 1", true)));
        addCommands(m_swerve.getAutonomousCommand("blue 2 2", false));
        addCommands(new AutoShoot(shooter, intake));
//        addCommands(new SmartIntake(shooter, intake)
//                .alongWith(m_swerve.getAutonomousCommand("mid 3", false)));
//        addCommands(new AutoShoot(shooter, intake));
    }
}
