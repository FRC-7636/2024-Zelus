package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;


public class SafeClimbTop extends Command {
    private final Climber climber;
    private final Shooter shooter;

    public SafeClimbTop(Climber climber, Shooter shooter) {
        this.climber = climber;
        this.shooter = shooter;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climber, this.shooter);
    }

    @Override
    public void execute() {
        shooter.setPosition(5);
        if (Math.abs(shooter.currentPosition() - 5) <= 2.5) {
            climber.setBalanceLevel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
