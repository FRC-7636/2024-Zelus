package frc.robot.commands.SINGLE_CMD;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAimPID;


public class AutoAim extends Command {
//    private final PhotonVision photonVision;
    private final Swerve swerve;
    private final PIDController yawCtrl = new PIDController(AutoAimPID.P, AutoAimPID.I, AutoAimPID.D);

//    public AutoAim(PhotonVision photonVision, Swerve swerve) {
//        this.photonVision = photonVision;
//        this.swerve = swerve;
//        // each subsystem used by the command must be passed into the
//        // addRequirements() method (which takes a vararg of Subsystem)
//        addRequirements(this.photonVision, this.swerve);
//    }

    public AutoAim(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        double deltaDeg = LimelightHelpers.getTX("");
        double deltaRad = Units.degreesToRadians(deltaDeg);
        SmartDashboard.putNumber("Delta Rad.", deltaDeg);
        Rotation2d targetHeading = swerve.getHeading().plus(new Rotation2d(deltaRad));
        SmartDashboard.putNumber("Heading offset", targetHeading.getDegrees());
        if (!isFinished()) {
            swerve.drive(new Translation2d(), yawCtrl.calculate(deltaDeg), false);
        } else {
            swerve.drive(new Translation2d(), 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        double deltaDeg = Math.abs(LimelightHelpers.getTX(""));
        return deltaDeg < 5;
    }
}
