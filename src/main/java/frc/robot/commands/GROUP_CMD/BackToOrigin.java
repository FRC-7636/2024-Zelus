package frc.robot.commands.GROUP_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BackToOrigin extends SequentialCommandGroup {
    public BackToOrigin(Climber climber, Intake intake, Shooter shooter){
        addCommands(new InstantCommand(climber::setFloorLevel, climber));
        andThen(new InstantCommand(intake::backToZero, intake));
        andThen(new InstantCommand(shooter::originAngle, shooter));
    }
}