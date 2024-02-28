package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.Config.L_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.Config.R_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final DigitalInput noteSensor = new DigitalInput(ShooterConstants.Config.SENSOR_ID);
    private final CANSparkMax transMotor = new CANSparkMax(ShooterConstants.Config.TRANS_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax angleMotor = new CANSparkMax(ShooterConstants.Config.ANGLE_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final SparkPIDController anglePIDController, leftPIDController, rightPIDController;

    private final RelativeEncoder leftMotorEncoder, rightMotorEncoder;
    private final AbsoluteEncoder angleEncoder;

    /**
     * set PID using SparkPIDController
     * @param sparkPIDController sparkPIDController which you want to set PID for
     * @param p set P
     * @param i set I
     * @param d set D
     */
    private void setPID(SparkPIDController sparkPIDController, double p, double i, double d) {
        sparkPIDController.setP(p);
        sparkPIDController.setI(i);
        sparkPIDController.setD(d);
    }


    public Shooter() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        transMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        transMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(ShooterConstants.Config.SHOOTER_INVERTED);
        rightMotor.setInverted(ShooterConstants.Config.SHOOTER_INVERTED);
        transMotor.setInverted(ShooterConstants.Config.TRANS_INVERTED);
        angleMotor.setInverted(ShooterConstants.Config.ANGLE_INVERTED);

        leftMotor.setSmartCurrentLimit(ShooterConstants.Config.CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(ShooterConstants.Config.CURRENT_LIMIT);
        transMotor.setSmartCurrentLimit(ShooterConstants.Config.CURRENT_LIMIT);
        angleMotor.setSmartCurrentLimit(ShooterConstants.Config.CURRENT_LIMIT);

        leftMotorEncoder = leftMotor.getEncoder();
        rightMotorEncoder = rightMotor.getEncoder();

        angleEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        angleEncoder.setPositionConversionFactor(360);

        // apply PIDs to motors
        anglePIDController = angleMotor.getPIDController();
        setPID(anglePIDController, ShooterConstants.AnglePIDF.P, ShooterConstants.AnglePIDF.I, ShooterConstants.AnglePIDF.D);
        leftPIDController = leftMotor.getPIDController();
        setPID(leftPIDController, ShooterConstants.LeftPIDF.P, ShooterConstants.LeftPIDF.I, ShooterConstants.LeftPIDF.D);
        rightPIDController = rightMotor.getPIDController();
        setPID(rightPIDController, ShooterConstants.RightPIDF.P, ShooterConstants.RightPIDF.I, ShooterConstants.RightPIDF.D);

        anglePIDController.setFeedbackDevice(angleEncoder);
        leftPIDController.setFeedbackDevice(leftMotorEncoder);
        rightPIDController.setFeedbackDevice(rightMotorEncoder);
        anglePIDController.setOutputRange(-0.35, 0.65);
        leftPIDController.setOutputRange(-1, 1);
        rightPIDController.setOutputRange(-1, 1);
        /* enable PID wrapping
         * after wrapping:
         *   set "0 -> 350", motor will go -10 instead of +350
         */
        anglePIDController.setPositionPIDWrappingEnabled(true);
        anglePIDController.setPositionPIDWrappingMinInput(0);
        anglePIDController.setPositionPIDWrappingMaxInput(360);

        leftMotor.burnFlash();
        rightMotor.burnFlash();
        transMotor.burnFlash();
        angleMotor.burnFlash();
    }

    /**
     * Set both shooter motors to the "shoot" status.
     * <p>(set the speed in {@link ShooterConstants.Control})
     */
    public void shoot() {
//         leftPIDController.setReference(ShooterConstants.Control.SHOOT_VELOCITY, ControlType.kVelocity);
//         rightPIDController.setReference(ShooterConstants.Control.SHOOT_VELOCITY, ControlType.kVelocity);
        leftMotor.set(0.7);
        rightMotor.set(0.7);
    }

    /**
     * Set both shooter motors to the "suck" status.
     * <p>(set the speed in {@link ShooterConstants.Control})
     */
    public void suck() {
        leftMotor.set(ShooterConstants.Control.SUCK_SPEED);
        rightMotor.set(ShooterConstants.Control.SUCK_SPEED);
    }

    public void standby() {
        leftPIDController.setReference(ShooterConstants.Control.STANDBY_SPEED, ControlType.kVelocity);
        rightPIDController.setReference(ShooterConstants.Control.STANDBY_SPEED, ControlType.kVelocity);
    }

    /**
     * Set transportation motor to the specified speed.
     * <p>(set the speed in {@link ShooterConstants.Control})
     */
    public void transport() {
        transMotor.set(ShooterConstants.Control.TRANS_SPEED);
    }

    /**
     * Reverse the transportation motor.
     */
    public void antiTransport() {
        transMotor.set(-ShooterConstants.Control.TRANS_SPEED);
    }

    /**
     * Stop the shooter motors.
     */
    public void stopShoot() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    /**
     * Stop the transportation motor.
     */
    public void stopTransport() {
        transMotor.stopMotor();
    }

    /**
     * Stop all motors.
     */
    public void stopAll() {
        stopShoot();
        stopTransport();
    }

    /**
     * Set the shooter to the desired position.
     * @param position  Desired shooter position, will be applied using {@link com.revrobotics.SparkPIDController}
     */
    public void setPosition(double position) {
        anglePIDController.setReference(position, ControlType.kPosition);
    }

    /**
     * Get the sensor value of the note sensor.
     * @return Whether the digital sensor is detecting a note.
     */
    public boolean noteDetected() {
        return noteSensor.get();
    }

    /**
     * @return true when both motors are ready
     */
    public boolean readyToShoot() {
        boolean leftReady = Math.abs(leftMotorEncoder.getVelocity() - ShooterConstants.Control.SHOOT_VELOCITY) <= 50;
        boolean rightReady = Math.abs(rightMotorEncoder.getVelocity() - ShooterConstants.Control.SHOOT_VELOCITY) <= 50;
        return (leftReady && rightReady);
    }

    /**
     * set the angle motor to the origin position
     */
    public void originAngle(){
        anglePIDController.setReference(ShooterConstants.Control.ORIGIN_POSITION, ControlType.kPosition);
    }

    public void up() {
        angleMotor.set(0.5);
    }

    public void down() {
        angleMotor.set(-0.5);
    }

    public void stopAngle() {
        angleMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Position", angleEncoder.getPosition());
        SmartDashboard.putNumber("Shooter Velocity", leftMotorEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Applied Output", angleMotor.getAppliedOutput());
    }
}
