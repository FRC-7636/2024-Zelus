package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;


public class JustShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;

    public JustShoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooter, this.intake);
    }

    @Override
    public void execute() {
        shooter.shootTwo();
        if (shooter.readyToShoot()) {
            intake.startConvey();
            shooter.transport();
        }
    }

    @Override
    public boolean isFinished() {
        return !shooter.noteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        intake.stopConvey();
    }
}
