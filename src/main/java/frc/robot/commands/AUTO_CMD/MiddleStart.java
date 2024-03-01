package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.AUTO_CMD.AutoIntake;
import frc.robot.commands.AUTO_CMD.AutoIntakeStop;
import frc.robot.commands.SINGLE_CMD.SmartShoot;
import frc.robot.commands.SINGLE_CMD.SmartIntake;

public class MiddleStart extends SequentialCommandGroup {

    public MiddleStart(Swerve m_swerve, Shooter shooter, Intake intake) {
        addCommands(new InstantCommand(intake::floorAngle));
        addCommands(new InstantCommand(() -> shooter.setPosition(50)));
        addCommands(new SmartShoot(shooter, intake));
        addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), m_swerve));
        addCommands(new AutoIntake(shooter, intake)
                .alongWith(m_swerve.getAutonomousCommand("1", false)));
        addCommands(new WaitCommand(3));
        addCommands(new AutoIntakeStop(shooter, intake));
        addCommands(m_swerve.getAutonomousCommand("2", false));
        addCommands(m_swerve.getAutonomousCommand("4", false));
    }
}
