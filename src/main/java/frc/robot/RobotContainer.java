// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Swerve;
import frc.robot.commands.AbsDrive;
import frc.robot.commands.FieldRelativeDrive;

public class RobotContainer {
  private Swerve driveBase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/Neo"));
  private XboxController chassisCtrl = new XboxController(0);
  
  AbsDrive absoluteDrive = new AbsDrive(driveBase, chassisCtrl::getLeftY, chassisCtrl::getLeftX, chassisCtrl::getRightX);
  FieldRelativeDrive fieldRelativeDrive = new FieldRelativeDrive(driveBase, chassisCtrl::getLeftY, chassisCtrl::getLeftX, chassisCtrl::getRightX);

  public RobotContainer() {
    configureBindings();
    driveBase.setDefaultCommand(absoluteDrive);
  }

  private void configureBindings() {
    new JoystickButton(chassisCtrl, 1).onTrue(new InstantCommand(driveBase::zeroGyro, driveBase));
  }
}
