package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SmartShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;

    public SmartShoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(this.shooter, this.intake);
    }


    @Override
    public void execute() {
        shooter.shoot();
        if (shooter.readyToShoot()) {
            intake.conveyorShoot();
            shooter.transport();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: return "true" when the note is out
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        intake.stopConvey();
    }
}
