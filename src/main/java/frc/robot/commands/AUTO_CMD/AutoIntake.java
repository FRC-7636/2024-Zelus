package frc.robot.commands.AUTO_CMD;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

public class AutoIntake extends SequentialCommandGroup {

    public AutoIntake(Shooter shooter, Intake intake) {
        addCommands(new InstantCommand(() -> shooter.setPosition(ShooterConstants.Control.INTAKE_POSITION)));
        addCommands(new InstantCommand(intake::setFloorAngle));
        addCommands(new InstantCommand(intake::suck));
        addCommands(new InstantCommand(intake::startConvey));
    }
}
