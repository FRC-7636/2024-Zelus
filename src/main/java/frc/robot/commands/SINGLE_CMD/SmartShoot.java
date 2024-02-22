package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class SmartShoot extends Command {
    private final Shooter shooter;

    public SmartShoot(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(this.shooter);
    }


    @Override
    public void execute() {
        shooter.shoot();
        if (shooter.readyToShoot()) {
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
    }
}
