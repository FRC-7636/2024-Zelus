// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.AbsDrive;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.commands.LeftStart;
import frc.robot.commands.MiddleStart;
import frc.robot.commands.NewFieldDrive;
import frc.robot.commands.SingleTrajectory;

public class RobotContainer {
  // Objects, Instances
  private final Swerve driveBase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/Neo"));
  private final XboxController chassisCtrl = new XboxController(0);
  // private final PhotonVision photonVision = new PhotonVision();
  private final Limelight limelight = new Limelight(); 

  // Auto Commands
  private final MiddleStart middleStart = new MiddleStart(limelight, driveBase);
  private final LeftStart leftStart = new LeftStart(limelight, driveBase);
  private final SendableChooser<Command> trajectoryChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // TeleOp Commands
  AbsDrive absoluteDrive = new AbsDrive(driveBase, chassisCtrl::getLeftY, chassisCtrl::getLeftX, chassisCtrl::getRightX);
  FieldRelativeDrive fieldRelativeDrive = new FieldRelativeDrive(driveBase, chassisCtrl::getLeftY, chassisCtrl::getLeftX, chassisCtrl::getRightX);
  NewFieldDrive NFD = new NewFieldDrive(driveBase, 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getLeftY(), 0.05), 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getLeftX(), 0.05), 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getRightX(), 0.05));

  private final static File[] pathFileList = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

    public RobotContainer() {
    // Configure controller buttons
    configureBindings();
    

    driveBase.setDefaultCommand(NFD);
  }

  private void configureBindings() {
    new JoystickButton(chassisCtrl, 1).onTrue(new InstantCommand(driveBase::zeroGyro, driveBase)
    .andThen(new InstantCommand(() -> SmartDashboard.putNumber("Delta Heading", 0))));
    new JoystickButton(chassisCtrl, 3).onTrue(new InstantCommand(
      () -> SmartDashboard.putNumber("Delta Heading", SmartDashboard.getNumber("Delta Heading", 0)-driveBase.getHeading().getDegrees())
    ));

    SmartDashboard.putData("control", driveBase.SwerveLock(driveBase));
  }

  public Command getAutonomousCommand() {
    // return trajectoryChooser.getSelected();
    return autoChooser.getSelected();
    // return middleStart;
  }

  public void initiateTrajectoryChooser() {
    // Initiate SendableChooser
    List<String> trajectoryList = new ArrayList<>();
    for (File file: Objects.requireNonNull(pathFileList)) {
      if (file.isFile()) {
        String pureFileName = file.getName().replaceFirst("[.][^.]+$", "");  // Use regex to remove extension
        trajectoryList.add(pureFileName);
        System.out.println(pureFileName);
      }
    }
    trajectoryChooser.setDefaultOption("Run all", middleStart);
    for (String name: trajectoryList) {
      trajectoryChooser.addOption(name, new SingleTrajectory(limelight, driveBase, name));
    }
    SmartDashboard.putData("Choose Trajectory", trajectoryChooser);
  }

  public void initiateAutoChooser(){
    autoChooser.setDefaultOption("MiddleStart", leftStart);
    autoChooser.addOption("LeftStart", middleStart);

    SmartDashboard.putData("Choose Auto", autoChooser);
  }
}
