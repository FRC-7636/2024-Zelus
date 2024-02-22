package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final TalonFX frontMotor = new TalonFX(ClimberConstants.Config.F_ID);
    private final TalonFX rearMotor = new TalonFX(ClimberConstants.Config.R_ID);

    public Climber() {
        var frontMotorConfigurator = frontMotor.getConfigurator();
        var rearMotorConfigurator = rearMotor.getConfigurator();

        // reset the motor to factory defaults
        frontMotorConfigurator.apply(new TalonFXConfiguration());
        rearMotorConfigurator.apply(new TalonFXConfiguration());

        // set the motors to brake mode & invert motors
        frontMotor.setNeutralMode(NeutralModeValue.Brake);
        frontMotor.setInverted(ClimberConstants.Config.INVERTED);
        rearMotor.setNeutralMode(NeutralModeValue.Brake);
        rearMotor.setInverted(ClimberConstants.Config.INVERTED);

        // set feedback sensor as integrated sensor
        frontMotorConfigurator.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        rearMotorConfigurator.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity
        frontMotorConfigurator.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ClimberConstants.Config.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ClimberConstants.Config.MAX_VELOCITY));
        rearMotorConfigurator.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ClimberConstants.Config.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ClimberConstants.Config.MAX_VELOCITY));

        // setup PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ClimberConstants.PIDF.P;
        PIDConfig.kI = ClimberConstants.PIDF.I;
        PIDConfig.kD = ClimberConstants.PIDF.D;
        PIDConfig.kV = ClimberConstants.PIDF.F;  // kF is actually kV...?
        frontMotorConfigurator.apply(PIDConfig);
        rearMotorConfigurator.apply(PIDConfig);
    }

    public void setFloorLevel() {
        frontMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.FLOOR));
        rearMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.FLOOR));
    }

    public void setAmpLevel() {
        frontMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.AMP));
        rearMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.AMP));
    }

    public void setBalanceLevel() {
        frontMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.BALANCE));
        rearMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.BALANCE));
    }

    public void setTrapLevel() {
        frontMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.TRAP));
        rearMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Control.TRAP));
    }
}

