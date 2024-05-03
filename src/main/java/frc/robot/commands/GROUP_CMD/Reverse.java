package frc.robot.commands.GROUP_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class Reverse extends SequentialCommandGroup {
    public Reverse(Shooter shooter, Intake intake) {
        addCommands();
        addCommands(new InstantCommand(intake::setFloorAngle));
        addCommands(new InstantCommand(() -> shooter.setPosition(Constants.ShooterConstants.Control.INTAKE_POSITION)).
                andThen(shooter::antiTransport));
        addCommands(new InstantCommand(intake::setReverseAngle).
                andThen(intake::reverseConvey));
        addCommands(new InstantCommand(intake::shoot));

        addRequirements(shooter, intake);
    }
}