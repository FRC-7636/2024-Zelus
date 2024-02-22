package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter;


public class AutoAimShooter extends Command {
    private final Shooter shooter;

    public AutoAimShooter(Shooter shooter) {
        this.shooter = shooter;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooter);
    }


    @Override
    public void execute() {

    }
}
