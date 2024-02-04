package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

public class SingleTrajectory extends SequentialCommandGroup{
    public SingleTrajectory(PhotonVision m_photonVision, Swerve m_swerve, String pathName){
        addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(m_photonVision.getLatestEstimatedRobotPose()), m_swerve, m_photonVision));
        addCommands(m_swerve.getAutonomousCommand(pathName, false));
    }
}
