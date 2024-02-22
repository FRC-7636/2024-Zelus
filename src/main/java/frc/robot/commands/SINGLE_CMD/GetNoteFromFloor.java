package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class GetNoteFromFloor extends Command {
    private final Intake intake;
    private final Shooter shooter;

    public GetNoteFromFloor(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(this.intake, this.shooter);
    }

    @Override
    public void execute() {
        intake.floorAngle();
        intake.suck();
        shooter.transport();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopTransport();
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return shooter.noteDetected();
    }
}
