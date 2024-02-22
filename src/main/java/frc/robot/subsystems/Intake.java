package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake extends SubsystemBase{
    CANSparkMax pipeIntake = new CANSparkMax(IntakeConstants.Config.PIPE_ID, MotorType.kBrushless);
    CANSparkMax angleIntake = new CANSparkMax(IntakeConstants.Config.ANGLE_ID, MotorType.kBrushless);
    RelativeEncoder intakEncoder = angleIntake.getEncoder();
    SparkPIDController intakPidController = angleIntake.getPIDController();

    private SparkPIDController setPID(SparkPIDController sparkPIDController, double p, double i, double d){
        sparkPIDController.setP(p);
        sparkPIDController.setI(i);
        sparkPIDController.setD(d);
        return sparkPIDController;
    }

    public  Intake(){
        pipeIntake.restoreFactoryDefaults();
        angleIntake.restoreFactoryDefaults();

        pipeIntake.setIdleMode(IdleMode.kBrake);
        angleIntake.setIdleMode(IdleMode.kBrake);

        pipeIntake.setInverted(false);
        angleIntake.setInverted(false);

        pipeIntake.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);
        angleIntake.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);

        intakEncoder.setPosition(0);

        setPID(intakPidController, IntakeConstants.AnglePIDF.P, IntakeConstants.AnglePIDF.I, IntakeConstants.AnglePIDF.D);
        intakPidController.setOutputRange(-1, 1);
    }

    public void shoot(){
        pipeIntake.set(IntakeConstants.Control.SHOOT_SPEED);
    }

    public void suck(){
        pipeIntake.set(IntakeConstants.Control.SUCK_SPEED);
    }

    public void floorAngle(){
        intakPidController.setReference(IntakeConstants.Control.FLOOR_POSITION, ControlType.kSmartMotion);
    }

    public void ampAngle(){
        intakPidController.setReference(IntakeConstants.Control.AMP_POSITION, ControlType.kSmartMotion);
    }

    public void trapAngle(){
        intakPidController.setReference(IntakeConstants.Control.TRAP_POSITION, ControlType.kSmartMotion);
    }

    public void backToZero(){
        intakPidController.setReference(IntakeConstants.Control.ORIGIN_POSITION, ControlType.kSmartMotion);
    }
}
