package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OthersConstants;

public class Candle extends SubsystemBase {
    private final CANdle candle = new CANdle(OthersConstants.CANDLE_ID, "cantivore");
    private final Shooter shooter = new Shooter();

    private final LarsonAnimation normalState = new LarsonAnimation(255, 255, 255);
    private final StrobeAnimation detectedState = new StrobeAnimation(255, 0, 255);
    private final StrobeAnimation shootingState = new StrobeAnimation(0, 255, 0);
    private final StrobeAnimation intakingState = new StrobeAnimation(0, 0, 255);
    private final StrobeAnimation ampState = new StrobeAnimation(255, 0, 0);
    private final double blinkSpeed = 0.4;

    private final double shooterVelocity = SmartDashboard.getNumber("Shooter Velocity", 0);
    private final double intakeVelocity = SmartDashboard.getNumber("Intake Velocity", 0);
    private final boolean noteDetect = SmartDashboard.getBoolean("NOTE Detected?", false);
    private final double climberPos = SmartDashboard.getNumber("Climber Position R", 0);
    private final double intakePos = SmartDashboard.getNumber("Intake Position", 0);
    
    public Candle() {
        candle.configFactoryDefault();

        candle.clearAnimation(0);
        candle.setLEDs(255, 255, 255);
    }

    public void backToNormalState() {
        candle.animate(normalState);
        normalState.setSpeed(0.25);
        normalState.setNumLed(43);
        normalState.setSize(5);
    }

    public void noteDetectedState() {
        candle.animate(detectedState);
        detectedState.setSpeed(blinkSpeed);
    }

    public void Shooting(){
        candle.animate(shootingState);
        shootingState.setSpeed(blinkSpeed);
    }

    public void Intaking(){
        candle.animate(intakingState);
        intakingState.setSpeed(blinkSpeed);
    }

    public void AMPing(){
        candle.animate(ampState);
        ampState.setSpeed(blinkSpeed);
    }
    @Override
    public void periodic() {
        if ((shooterVelocity > 1500) & noteDetect & (climberPos < 10)){
              Shooting();
        }
        else if ((intakeVelocity > 1) & !noteDetect & (climberPos < 10) & (intakePos > 100)){
             Intaking();
        }
        else if ((climberPos > 20) & !noteDetect){
             AMPing();
        }
        else if (noteDetect & (climberPos < 10)) {
             noteDetectedState();
        }
        else if(!noteDetect & (climberPos < 10)){
             backToNormalState();
        }
    }
}

