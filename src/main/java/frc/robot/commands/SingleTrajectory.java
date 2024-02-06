package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class SingleTrajectory extends SequentialCommandGroup{
    public SingleTrajectory(Limelight limelight, Swerve m_swerve, String pathName){
        addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(limelight.getPose2d()), m_swerve, limelight));
        addCommands(m_swerve.getAutonomousCommand(pathName, false));
    }
}
