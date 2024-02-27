// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.subsystems.*;
import frc.robot.commands.AUTO_CMD.LeftStart;
import frc.robot.commands.AUTO_CMD.MiddleStart;
import frc.robot.commands.AUTO_CMD.SingleTrajectory;
import frc.robot.commands.AUTO_CMD.BlueTest2;
import frc.robot.commands.SINGLE_CMD.AutoAim;
import frc.robot.commands.SINGLE_CMD.SmartShoot;
import frc.robot.commands.SINGLE_CMD.GetNoteFromFloor;
import frc.robot.commands.TEST_CMD.StopEverything;
import frc.robot.commands.SWERVE_CMD.AbsDrive;
import frc.robot.commands.SWERVE_CMD.FieldRelativeDrive;
import frc.robot.commands.SWERVE_CMD.NewFieldDrive;


public class RobotContainer {
  // Objects, Instances
  private final Swerve driveBase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/Neo"));
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final XboxController chassisCtrl = new XboxController(0);
  private final XboxController botCtrl = new XboxController(1);
  //  private final PhotonVision photonVision = new PhotonVision();

  // Auto Commands
  private final MiddleStart middleStart = new MiddleStart(driveBase);
  private final LeftStart leftStart = new LeftStart(driveBase);
  private final SendableChooser<Command> trajectoryChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // TeleOp Commands
  private final AbsDrive absoluteDrive = new AbsDrive(driveBase, chassisCtrl::getLeftY, chassisCtrl::getLeftX, chassisCtrl::getRightX);
  private final FieldRelativeDrive fieldRelativeDrive = new FieldRelativeDrive(driveBase, chassisCtrl::getLeftY, chassisCtrl::getLeftX, chassisCtrl::getRightX);
  private final NewFieldDrive NFD = new NewFieldDrive(driveBase,
                                        () -> MathUtil.applyDeadband(chassisCtrl.getLeftY(), 0.05), 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getLeftX(), 0.05), 
                                        () -> MathUtil.applyDeadband(chassisCtrl.getRightX(), 0.05));
  private final AutoAim autoAim = new AutoAim(driveBase);
  private final StopEverything stopEverything = new StopEverything(intake, shooter);
  private final SmartShoot smartShoot = new SmartShoot(shooter, intake);
  private final GetNoteFromFloor getNoteFromFloor = new GetNoteFromFloor(intake, shooter);

  private final static File[] pathFileList = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

  public RobotContainer() {
    // Configure controller buttons
    configureBindings();

    driveBase.setDefaultCommand(NFD);
  }

  private void configureBindings() {
    new JoystickButton(chassisCtrl, 1).onTrue(new InstantCommand(shooter::shoot)).whileFalse(new InstantCommand(shooter::stopShoot));
    new JoystickButton(chassisCtrl, 2).onTrue(new InstantCommand(shooter::suck)).whileFalse(new InstantCommand(shooter::stopShoot));
    new JoystickButton(chassisCtrl, 3).onTrue(new InstantCommand(shooter::transport)).whileFalse(new InstantCommand(shooter::stopTransport));
//    new JoystickButton(chassisCtrl, 3).onTrue(new InstantCommand(intake::suck, intake).andThen(new InstantCommand(intake::startConvey, intake)))
//            .whileFalse(new InstantCommand(intake::stopIntake, intake).andThen(new InstantCommand(intake::stopConvey, intake)));
    new JoystickButton(chassisCtrl, 4).onTrue(new InstantCommand(intake::shoot)).whileFalse(new InstantCommand(intake::stopIntake));
    new JoystickButton(chassisCtrl, 5).whileTrue(new InstantCommand(driveBase::zeroGyro));
    new JoystickButton(chassisCtrl, 6).onTrue(new InstantCommand(shooter::antiTransport, shooter)).whileFalse(new InstantCommand(shooter::stopTransport, shooter));
    new POVButton(chassisCtrl, 0).onTrue(new InstantCommand(intake::up)).whileFalse(new InstantCommand(intake::stopAngle));
    new POVButton(chassisCtrl, 180).onTrue(new InstantCommand(intake::down)).whileFalse(new InstantCommand(intake::stopAngle));

    new JoystickButton(botCtrl, 1).whileTrue(new InstantCommand(intake::ampAngle));
    new JoystickButton(botCtrl, 2).whileTrue(new InstantCommand(intake::backToZero));
    new JoystickButton(botCtrl, 3).onTrue(new InstantCommand(climber::up)).whileFalse(new InstantCommand(climber::stop));
    new JoystickButton(botCtrl, 4).onTrue(new InstantCommand(climber::down)).whileFalse(new InstantCommand(climber::stop));
    new JoystickButton(botCtrl, 5).onTrue(new InstantCommand(shooter::up)).whileFalse(new InstantCommand(shooter::stopAngle));
    new JoystickButton(botCtrl, 6).onTrue(new InstantCommand(shooter::down)).whileFalse(new InstantCommand(shooter::stopAngle));
  }

  public Command getAutonomousCommand() {
       return new BlueTest2(driveBase);
//       return trajectoryChooser.getSelected();
//       return middleStart;
  }

  public void initiateTrajectoryChooser() {
    // Initiate SendableChooser
    List<String> trajectoryList = new ArrayList<>();
    System.out.println("Detected paths:");
    for (File file: Objects.requireNonNull(pathFileList)) {
      if (file.isFile()) {
        String pureFileName = file.getName().replaceFirst("[.][^.]+$", "");  // Use regex to remove extension
        trajectoryList.add(pureFileName);
        System.out.println(" " + pureFileName);
      }
    }
    trajectoryChooser.setDefaultOption("Run all", middleStart);
    for (String name: trajectoryList) {
      trajectoryChooser.addOption(name, new SingleTrajectory(driveBase, name));
    }
    SmartDashboard.putData("Choose Trajectory", trajectoryChooser);
  }

  public void initiateAutoChooser(){
    autoChooser.setDefaultOption("MiddleStart", leftStart);
    autoChooser.addOption("LeftStart", middleStart);

    SmartDashboard.putData("Choose Auto", autoChooser);
  }

  public void autoResetOdometry() {
    Pose2d currentPose = driveBase.getPose().plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180)));
    driveBase.resetOdometry(currentPose);
  }

  public void rumbleeeeeeee() {
    chassisCtrl.setRumble(GenericHID.RumbleType.kBothRumble, Math.abs(chassisCtrl.getRightY()));
  }
}
