package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OthersConstants;

public class Candle extends SubsystemBase {
    private final CANdle candle = new CANdle(OthersConstants.CANDLE_ID, "cantivore");

    private final StrobeAnimation normalState = new StrobeAnimation(255, 0, 255, 0, 0.5, 8);
    public Candle() {
        candle.configFactoryDefault();

        candle.clearAnimation(0);
    }

    public void backToNormalState() {
        candle.animate(normalState);
    }
}

