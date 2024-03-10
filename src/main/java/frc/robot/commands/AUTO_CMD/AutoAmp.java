package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.commands.GROUP_CMD.Amp;

public class AutoAmp extends SequentialCommandGroup {
    public AutoAmp(Swerve swerve, Intake intake, Climber climber, Shooter shooter) {
        addCommands(swerve.getAutonomousCommand("red out", true));
//        addCommands(new InstantCommand(intake::setAmpAngle).alongWith(new InstantCommand(climber::setBalanceLevel)));
//        addCommands(new InstantCommand(intake::shoot).withTimeout(1.5));
//        addCommands(new InstantCommand(climber::setFloorLevel));
//        addCommands(swerve.getAutonomousCommand("blue amp 2", false));
    }
}