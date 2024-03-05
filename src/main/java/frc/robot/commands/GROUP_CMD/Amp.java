package frc.robot.commands.GROUP_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class Amp extends SequentialCommandGroup {
    public Amp(Climber climber, Intake intake) {
        addCommands(new InstantCommand(climber::setBalanceLevel, climber));
        addCommands(new InstantCommand(intake::ampAngle, intake));
    }
}
