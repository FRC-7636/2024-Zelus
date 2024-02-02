package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake extends SubsystemBase{
    CANSparkMax pipeIntake = new CANSparkMax(IntakeConstants.pipeIntakePort, MotorType.kBrushless);
    CANSparkMax angleIntake = new CANSparkMax(IntakeConstants.angleIntakePort, MotorType.kBrushless);
    RelativeEncoder intakEncoder = angleIntake.getEncoder();
    SparkPIDController intakPidController = angleIntake.getPIDController();

    public  Intake(){
        pipeIntake.restoreFactoryDefaults();
        angleIntake.restoreFactoryDefaults();

        pipeIntake.setIdleMode(IdleMode.kBrake);
        angleIntake.setIdleMode(IdleMode.kBrake);

        pipeIntake.setInverted(false);
        angleIntake.setInverted(false);

        pipeIntake.setSmartCurrentLimit(35);
        angleIntake.setSmartCurrentLimit(35);

        intakPidController.setP(IntakeConstants.kP);
        intakPidController.setI(IntakeConstants.kI);
        intakPidController.setD(IntakeConstants.kD);
        intakPidController.setOutputRange(-1, 1);
    }

    public void intakeSuck(){
        pipeIntake.set(0.5);
    }

    public void intakeShoot(){

    }
    
}
