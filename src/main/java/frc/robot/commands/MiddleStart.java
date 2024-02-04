package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class MiddleStart extends SequentialCommandGroup{
    public MiddleStart(PhotonVision m_photonVision, Swerve m_swerve){
        addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(m_photonVision.getLatestEstimatedRobotPose()), m_swerve, m_photonVision));
        addCommands(m_swerve.getAutonomousCommand("1", false));
        addCommands(m_swerve.getAutonomousCommand("2", false));
        addCommands(m_swerve.getAutonomousCommand("2 Shoot", false));
    }
}
