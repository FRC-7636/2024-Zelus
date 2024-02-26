package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private final CANSparkMax pipeIntake = new CANSparkMax(IntakeConstants.Config.PIPE_ID, MotorType.kBrushless);
    private final CANSparkMax angleIntake = new CANSparkMax(IntakeConstants.Config.ANGLE_ID, MotorType.kBrushless);
    private final CANSparkMax conveyor = new CANSparkMax(IntakeConstants.Config.CONVEYOR_ID, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder;
    private final SparkPIDController intakePIDController;

    private void setPID(SparkPIDController sparkPIDController, double p, double i, double d){
        sparkPIDController.setP(p);
        sparkPIDController.setI(i);
        sparkPIDController.setD(d);
    }

    public Intake(){
        pipeIntake.restoreFactoryDefaults();
        angleIntake.restoreFactoryDefaults();
        conveyor.restoreFactoryDefaults();

        pipeIntake.setIdleMode(IdleMode.kBrake);
        angleIntake.setIdleMode(IdleMode.kBrake);
        conveyor.setIdleMode(IdleMode.kBrake);

        pipeIntake.setInverted(IntakeConstants.Config.PIPE_INVERTED);
        angleIntake.setInverted(IntakeConstants.Config.ANGLE_INVERTED);
        conveyor.setInverted(IntakeConstants.Config.CONVEYOR_INVERTED);

        pipeIntake.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);
        angleIntake.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);
        conveyor.setSmartCurrentLimit(IntakeConstants.Config.CURRENT_LIMIT);

        intakeEncoder = angleIntake.getEncoder();

        // define current position as zero
        intakeEncoder.setPosition(0);

        intakePIDController = angleIntake.getPIDController();
        intakePIDController.setFeedbackDevice(intakeEncoder);
        setPID(intakePIDController, IntakeConstants.AnglePIDF.P, IntakeConstants.AnglePIDF.I, IntakeConstants.AnglePIDF.D);
        intakePIDController.setFF(0.0001);
        intakePIDController.setOutputRange(-1, 1);
    }

    public void shoot(){
        pipeIntake.set(IntakeConstants.Control.SHOOT_SPEED);
    }

    public void suck(){
        pipeIntake.set(IntakeConstants.Control.SUCK_SPEED);
    }

    public void startITS() {
        conveyor.set(IntakeConstants.Control.CONVEYOR_SPEED);
    }

    public void stopIntake() {
        pipeIntake.stopMotor();
    }

    public void stopITS() {
        conveyor.stopMotor();
    }

    public void stopAll() {
        stopIntake();
        stopITS();
    }

    public void floorAngle(){
        intakePIDController.setSmartMotionMaxAccel(1200, 0);
        intakePIDController.setSmartMotionMaxVelocity(4000, 0);
        intakePIDController.setReference(IntakeConstants.Control.FLOOR_POSITION/2, ControlType.kSmartMotion);
    }

    public void ampAngle(){
        intakePIDController.setReference(IntakeConstants.Control.AMP_POSITION, ControlType.kSmartMotion);
    }

    public void trapAngle(){
        intakePIDController.setReference(IntakeConstants.Control.TRAP_POSITION, ControlType.kSmartMotion);
    }

    public void backToZero(){
        intakePIDController.setSmartMotionMaxAccel(4000, 0);
        intakePIDController.setSmartMotionMaxVelocity(8000, 0);
        intakePIDController.setReference(-2.54, ControlType.kSmartMotion);
    }

    public void up() {
        angleIntake.set(0.5);
    }

    public void down() {
        angleIntake.set(-0.5);
    }

    public void stopAngle() {
        angleIntake.stopMotor();
    }

    public void ITSShoot() {
        conveyor.set(-IntakeConstants.Control.CONVEYOR_SPEED);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", intakeEncoder.getPosition());
    }
}
