package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake extends SubsystemBase{
    private final CANSparkMax pipeIntake = new CANSparkMax(IntakeConstants.Config.PIPE_ID, MotorType.kBrushless);
    private final CANSparkMax angleIntake = new CANSparkMax(IntakeConstants.Config.ANGLE_ID, MotorType.kBrushless);
    private final CANSparkMax intakeToShooter = new CANSparkMax(IntakeConstants.Config.ITS_ID, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = angleIntake.getEncoder();
    private final SparkPIDController intakePIDController = angleIntake.getPIDController();

    private void setPID(SparkPIDController sparkPIDController, double p, double i, double d){
        sparkPIDController.setP(p);
        sparkPIDController.setI(i);
        sparkPIDController.setD(d);
    }

    public Intake(){
        pipeIntake.restoreFactoryDefaults();
        angleIntake.restoreFactoryDefaults();
        intakeToShooter.restoreFactoryDefaults();

        pipeIntake.setIdleMode(IdleMode.kBrake);
        angleIntake.setIdleMode(IdleMode.kBrake);
        intakeToShooter.setIdleMode(IdleMode.kBrake);

        pipeIntake.setInverted(IntakeConstants.Config.PIPE_INVERTED);
        angleIntake.setInverted(IntakeConstants.Config.ANGLE_INVERTED);
        intakeToShooter.setInverted(IntakeConstants.Config.ITS_INVERTED);

        pipeIntake.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);
        angleIntake.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);
        intakeToShooter.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);

        // define current position as zero
        intakeEncoder.setPosition(0);

        setPID(intakePIDController, IntakeConstants.AnglePIDF.P, IntakeConstants.AnglePIDF.I, IntakeConstants.AnglePIDF.D);
        intakePIDController.setOutputRange(-1, 1);
    }

    public void shoot(){
        pipeIntake.set(IntakeConstants.Control.SHOOT_SPEED);
    }

    public void suck(){
        pipeIntake.set(IntakeConstants.Control.SUCK_SPEED);
    }

    public void startITS() {
        intakeToShooter.set(IntakeConstants.Control.ITS_SPEED);
    }

    public void stopIntake() {
        pipeIntake.stopMotor();
    }

    public void stopITS() {
        intakeToShooter.stopMotor();
    }

    public void floorAngle(){
        intakePIDController.setReference(IntakeConstants.Control.FLOOR_POSITION, ControlType.kSmartMotion);
    }

    public void ampAngle(){
        intakePIDController.setReference(IntakeConstants.Control.AMP_POSITION, ControlType.kSmartMotion);
    }

    public void trapAngle(){
        intakePIDController.setReference(IntakeConstants.Control.TRAP_POSITION, ControlType.kSmartMotion);
    }

    public void backToZero(){
        intakePIDController.setReference(IntakeConstants.Control.ORIGIN_POSITION, ControlType.kSmartMotion);
    }
}
