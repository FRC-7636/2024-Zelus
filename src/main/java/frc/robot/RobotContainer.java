// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.NewFieldDrive;

public class RobotContainer {
  private final Swerve driveBase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/Neo"));
  private final XboxController chassisCtrl = new XboxController(0);
  private final PhotonVision photonVision = new PhotonVision();

  NewFieldDrive NFD = new NewFieldDrive(driveBase, 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getLeftY(), 0.01), 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getLeftX(), 0.01), 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getRightX(), 0.05));


  public RobotContainer() {
    driveBase.resetOdometry(photonVision.getLatestEstimatedRobotPose(driveBase));
    configureBindings();
    
    driveBase.setDefaultCommand(NFD);
  }

  private void configureBindings() {
    new JoystickButton(chassisCtrl, 1).onTrue(new InstantCommand(driveBase::zeroGyro, driveBase)
    .andThen(new InstantCommand(() -> SmartDashboard.putNumber("Delta Heading", 0))));
    new JoystickButton(chassisCtrl, 3).onTrue(new InstantCommand(
      () -> SmartDashboard.putNumber("Delta Heading", SmartDashboard.getNumber("Delta Heading", 0)-driveBase.getHeading().getDegrees())
    ));
  }

  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return driveBase.getAutonomousCommand("2", false);
  }

  
  public void sendGamePadValueToDashboard() {
  }
}
