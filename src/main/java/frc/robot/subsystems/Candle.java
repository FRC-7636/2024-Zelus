package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OthersConstants;

public class Candle extends SubsystemBase {
    private final CANdle candle = new CANdle(OthersConstants.CANDLE_ID, "cantivore");

    private final StrobeAnimation normalState = new StrobeAnimation(142, 87, 22);
    private final StrobeAnimation detectedState = new StrobeAnimation(0, 0, 255);

    public Candle() {
        candle.configFactoryDefault();

        candle.clearAnimation(0);
        candle.setLEDs(255, 255, 255);
    }

    public void backToNormalState() {
        candle.setLEDs(10, 10, 10);
    }

    public void noteDetectedState() {
        candle.setLEDs(255, 0, 255);
    }

    @Override
    public void periodic() {
        if (SmartDashboard.getBoolean("NOTE Detected?", false)) {
            noteDetectedState();
        }
        else {
            backToNormalState();
        }
    }
}

