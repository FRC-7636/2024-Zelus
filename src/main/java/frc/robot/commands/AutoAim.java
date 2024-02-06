package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;


public class AutoAim extends Command {
    private final PhotonVision photonVision;
    private final Swerve swerve;

    public AutoAim(PhotonVision photonVision, Swerve swerve) {
        this.photonVision = photonVision;
        this.swerve = swerve;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.photonVision, this.swerve);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
