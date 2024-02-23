package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.Config.L_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.Config.R_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final DigitalInput noteSensor = new DigitalInput(ShooterConstants.Config.SENSOR_ID);
    private final CANSparkMax transMotor = new CANSparkMax(ShooterConstants.Config.TRANS_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax angleMotor = new CANSparkMax(ShooterConstants.Config.ANGLE_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final SparkPIDController angleController, leftPIDController, rightPIDController;

    private final RelativeEncoder leftMotorEncoder, rightMotorEncoder;


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

        // apply PIDs to motors
        angleController = angleMotor.getPIDController();
        setPID(angleController, ShooterConstants.AnglePIDF.P, ShooterConstants.AnglePIDF.I, ShooterConstants.AnglePIDF.D);
        leftPIDController = leftMotor.getPIDController();
        setPID(leftPIDController, ShooterConstants.LeftPIDF.P, ShooterConstants.LeftPIDF.I, ShooterConstants.LeftPIDF.D);
        rightPIDController = rightMotor.getPIDController();
        setPID(rightPIDController, ShooterConstants.RightPIDF.P, ShooterConstants.RightPIDF.I, ShooterConstants.RightPIDF.D);

        leftMotorEncoder = leftMotor.getEncoder();
        rightMotorEncoder = rightMotor.getEncoder();
    }

    /**
     * Set both shooter motors to the "shoot" status.
     * (set the speed in {@link ShooterConstants.Control})
     */
    public void shoot() {
        leftPIDController.setReference(ShooterConstants.Control.SHOOT_VELOCITY, ControlType.kVelocity);
        rightPIDController.setReference(ShooterConstants.Control.SHOOT_VELOCITY, ControlType.kVelocity);
    }

    /**
     * Set both shooter motors to the "suck" status.
     * (set the speed in {@link ShooterConstants.Control})
     */
    public void suck() {
        leftMotor.set(ShooterConstants.Control.SUCK_SPEED);
        rightMotor.set(ShooterConstants.Control.SUCK_SPEED);
    }

    /**
     * Set transportation motor to the specified speed.
     * (set the speed in {@link ShooterConstants.Control})
     */
    public void transport() {
        transMotor.set(ShooterConstants.Control.TRANS_SPEED);
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
        angleController.setReference(position, ControlType.kSmartMotion);
    }

    /**
     * Get the sensor value of the note sensor.
     * @return Whether the note sensor is detecting a note.
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
}
