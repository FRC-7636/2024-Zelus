// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.AUTO_CMD.*;
import frc.robot.subsystems.*;
import frc.robot.commands.GROUP_CMD.BackToOrigin;
import frc.robot.commands.GROUP_CMD.Amp;
import frc.robot.commands.SINGLE_CMD.*;
import frc.robot.commands.TEST_CMD.StopEverything;
import frc.robot.commands.SWERVE_CMD.NewFieldDrive;


public class RobotContainer {
    // Objects, Instances
    private final Swerve driveBase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/Neo"));
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();
    private final Candle candle = new Candle();
    private final XboxController chassisCtrl = new XboxController(0);
    private final XboxController assistCtrl = new XboxController(1);
    //  private final PhotonVision photonVision = new PhotonVision();

    // Auto Commands
    private final MiddleStart middleStart = new MiddleStart(driveBase, shooter, intake);
    private final LeftStart leftStart = new LeftStart(driveBase);
    private final RightStart rightStart = new RightStart(driveBase, shooter, intake);
    private final SendableChooser<Command> trajectoryChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // TeleOp Commands
    private final NewFieldDrive NFD = new NewFieldDrive(driveBase,
            chassisCtrl::getLeftY,
            chassisCtrl::getLeftX,
            chassisCtrl::getRightX);
    private final StopEverything stopEverything = new StopEverything(intake, shooter);
    private final SmartShoot smartShoot = new SmartShoot(shooter, intake);
    private final NearShoot nearShoot = new NearShoot(shooter, intake);
    private final FarShoot farShoot = new FarShoot(shooter, intake);
    private final JustShoot justShoot = new JustShoot(shooter, intake);
    private final SmartIntake smartIntake = new SmartIntake(shooter, intake);
    private final BackToOrigin backToOrigin = new BackToOrigin(climber, intake, shooter);
    private final Amp ampCmd = new Amp(climber, shooter, intake);
    private final SafeClimbTop safeClimbTop = new SafeClimbTop(climber, shooter);
    private final ShooterSuck shooterSuck = new ShooterSuck(shooter);

    private SendableChooser<Command> getAutoChooser = new SendableChooser<>();

    private final static File[] pathFileList = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

    public RobotContainer() {
        // Configure controller buttons
        configureBangRenBindings();
        configureYuJieBindings();
        createButtonsOnDS();

        getAutoChooser = AutoBuilder.buildAutoChooser();

        NamedCommands.registerCommand("amplevel", ampCmd);
        NamedCommands.registerCommand("ampshoot", new InstantCommand(intake::shoot));

        SmartDashboard.putData("Auto", getAutoChooser);

        driveBase.setDefaultCommand(NFD);
    }

    private void configureBangRenBindings() {
//        new JoystickButton(chassisCtrl, 1).onTrue(new InstantCommand(() -> shooter.setPosition(50), shooter));
//        new JoystickButton(chassisCtrl, 3).onTrue(shooterSuck).onFalse(new InstantCommand(shooter::stopShoot));
//        new JoystickButton(chassisCtrl, 4).onTrue(nearShoot);

//        new POVButton(chassisCtrl, 0).whileTrue(justShoot);
//        new POVButton(chassisCtrl, 180).onTrue(farShoot);
//    new POVButton(chassisCtrl, 180).whileTrue(smartShootNear);

        new JoystickButton(chassisCtrl, 5).whileTrue(new InstantCommand(intake::reverseConvey)).onFalse(new InstantCommand(intake::stopAll));
        new JoystickButton(chassisCtrl, 6).whileTrue(new InstantCommand(intake::shoot)).onFalse(new InstantCommand(intake::stopAll));

        new JoystickButton(chassisCtrl, 7).onTrue(new InstantCommand(driveBase::setIntakeAsHead));
    }

    private void configureYuJieBindings() {
        new JoystickButton(assistCtrl, 1).onTrue(ampCmd);
        new JoystickButton(assistCtrl, 2).onTrue(backToOrigin);
        new JoystickButton(assistCtrl, 3).onTrue(smartIntake);
        new JoystickButton(assistCtrl, 4).whileTrue(new InstantCommand(intake::suck, intake)).onFalse(new InstantCommand(intake::stopIntake));

        new JoystickButton(assistCtrl, 5).whileTrue(new InstantCommand(intake::setFloorAngle)
                .alongWith(new InstantCommand(intake::slowSuck))
                .alongWith(new InstantCommand(intake::reverseConvey)))
                .onFalse(new InstantCommand(intake::stopAll));
        new JoystickButton(assistCtrl, 6).whileTrue(new InstantCommand(intake::reverseConvey)).onFalse(new InstantCommand(intake::stopAll));

        new POVButton(assistCtrl, 0).whileTrue(new InstantCommand(climber::up, climber)).onFalse(new InstantCommand(climber::stop));
        new POVButton(assistCtrl, 180).whileTrue(new InstantCommand(climber::down, climber)).onFalse(new InstantCommand(climber::stop));
        new POVButton(assistCtrl, 270).whileTrue(new InstantCommand(climber::setFloorLevel, climber));
    }

    public void createButtonsOnDS() {
        // Climber
        SmartDashboard.putData("Climber Up", new InstantCommand(climber::up, climber));
        SmartDashboard.putData("Climber Down", new InstantCommand(climber::down, climber));
        SmartDashboard.putData("Climber Stop", new InstantCommand(climber::stop, climber));

        // Intake
        SmartDashboard.putData("Intake Up", new InstantCommand(intake::up, intake));
        SmartDashboard.putData("Intake Down", new InstantCommand(intake::down, intake));
        SmartDashboard.putData("Intake Stop", new InstantCommand(intake::stopAngle, intake));
        SmartDashboard.putData("Intake Suck", new InstantCommand(intake::suck, intake));
        SmartDashboard.putData("Intake Shoot", new InstantCommand(intake::shoot, intake));
    }

    public Command getAutonomousCommand() {
//       return new BlueTest2(driveBase);
//       return trajectoryChooser.getSelected();
//        return getAutoChooser.getSelected();
        return new AutoAmp(driveBase, intake, climber, shooter);
    }

    public void initiateTrajectoryChooser() {
        // Initiate SendableChooser
        List<String> trajectoryList = new ArrayList<>();
        System.out.println("Detected paths:");
        for (File file : Objects.requireNonNull(pathFileList)) {
            if (file.isFile()) {
                String pureFileName = file.getName().replaceFirst("[.][^.]+$", "");  // Use regex to remove extension
                trajectoryList.add(pureFileName);
                System.out.println(" " + pureFileName);
            }
        }
        trajectoryChooser.setDefaultOption("Run all", middleStart);
        for (String name : trajectoryList) {
            trajectoryChooser.addOption(name, new SingleTrajectory(driveBase, name));
        }
        SmartDashboard.putData("Choose Trajectory", trajectoryChooser);
    }

    public void initiateAutoChooser() {
        autoChooser.setDefaultOption("MiddleStart", leftStart);
        autoChooser.addOption("LeftStart", middleStart);

        SmartDashboard.putData("Choose Auto", autoChooser);
    }

    public void rumbleWhenReady() {
        if (shooter.readyToShoot()) {
            chassisCtrl.setRumble(RumbleType.kBothRumble, 0.5);
        }
        else {
            chassisCtrl.setRumble(RumbleType.kBothRumble, 0);
        }
    }
}
