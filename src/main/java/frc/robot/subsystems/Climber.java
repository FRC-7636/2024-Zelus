package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final TalonFX frontMotor = new TalonFX(ClimberConstants.Config.F_ID);
    private final TalonFX rearMotor = new TalonFX(ClimberConstants.Config.R_ID);
    private final Slot0Configs PIDConfig = new Slot0Configs();
    public Climber() {
        // reset the motor to factory defaults
        frontMotor.getConfigurator().apply(new TalonFXConfiguration());
        rearMotor.getConfigurator().apply(new TalonFXConfiguration());

        // set the motors to brake mode
        frontMotor.setNeutralMode(NeutralModeValue.Brake);
        rearMotor.setNeutralMode(NeutralModeValue.Brake);

        // set feedback sensor as integrated sensor
        frontMotor.getConfigurator().apply(
                new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        rearMotor.getConfigurator().apply(
                new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // setup PIDConfig
        PIDConfig.kP = ClimberConstants.PIDF.P;
        PIDConfig.kI = ClimberConstants.PIDF.I;
        PIDConfig.kD = ClimberConstants.PIDF.D;
        PIDConfig.kV = ClimberConstants.PIDF.F;  // kF is actually kV...?
        frontMotor.getConfigurator().apply(PIDConfig);
        rearMotor.getConfigurator().apply(PIDConfig);
    }


}

