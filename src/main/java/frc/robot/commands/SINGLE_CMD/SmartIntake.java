package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SmartIntake extends Command {
    private final Intake intake;
    private final Shooter shooter;

    public SmartIntake(Shooter shooter, Intake intake) {
        this.intake = intake;
        this.shooter = shooter;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake, this.shooter);
    }


    @Override
    public void execute() {
        shooter.setPosition(25);
        intake.floorAngle();
        intake.suck();
        intake.startConvey();
        shooter.transport();
    }

    @Override
    public boolean isFinished() {
        // TODO: return true when Shooter has detected NOTE
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopAll();
        shooter.stopTransport();
    }
}
