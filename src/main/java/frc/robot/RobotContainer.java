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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.subsystems.Swerve;
import frc.robot.commands.AbsDrive;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.commands.NewFieldDrive;

public class RobotContainer {
  private Swerve driveBase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/Neo"));
  private XboxController chassisCtrl = new XboxController(0);

  AbsDrive absoluteDrive = new AbsDrive(driveBase, chassisCtrl::getLeftY, chassisCtrl::getLeftX, chassisCtrl::getRightX);
  FieldRelativeDrive fieldRelativeDrive = new FieldRelativeDrive(driveBase, chassisCtrl::getLeftY, chassisCtrl::getLeftX, chassisCtrl::getRightX);
  NewFieldDrive NFD = new NewFieldDrive(driveBase, 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getLeftY(), 0.01), 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getLeftX(), 0.01), 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getRightX(), 0.05));

  Command driveFieldOrientedDirectAngle = driveBase.driveCommand(
        () -> MathUtil.applyDeadband(chassisCtrl.getLeftY(), 0.01),
        () -> MathUtil.applyDeadband(chassisCtrl.getLeftX(), 0.01),
        () -> chassisCtrl.getRightX(),
        () -> chassisCtrl.getRightY());

  Command driveFieldOrientedDirectAngleSim = driveBase.simDriveCommand(
        () -> MathUtil.applyDeadband(chassisCtrl.getLeftY(), 0.01),
        () -> MathUtil.applyDeadband(chassisCtrl.getLeftX(), 0.01),
        () -> chassisCtrl.getRawAxis(4));


  public RobotContainer() {
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
    return driveBase.getAutonomousCommand("StraightForward", false);
  }
  
  public void sendGamePadValueToDashboard() {
    // SmartDashboard.putNumber("X", MathUtil.applyDeadband(chassisCtrl.getLeftX(), 0.01));
    // SmartDashboard.putNumber("Y", MathUtil.applyDeadband(chassisCtrl.getLeftY(), 0.01));
    // SmartDashboard.putNumber("Turning", chassisCtrl.getRightX());
  }
}
