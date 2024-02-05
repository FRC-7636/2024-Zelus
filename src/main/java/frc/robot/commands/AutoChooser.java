package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser extends Command {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final MiddleStart middleStart;
    private final LeftStart leftStart;

    public AutoChooser(MiddleStart middleStart, LeftStart leftStart) {
        this.middleStart = middleStart;
        this.leftStart = leftStart;
        autoChooser.setDefaultOption("MiddleStart", middleStart);
        autoChooser.addOption("LeftStart", leftStart);
    }

    @Override
    public void execute() {
        SmartDashboard.putData("Auto choices", autoChooser);
        autoChooser.getSelected();
    }
}
