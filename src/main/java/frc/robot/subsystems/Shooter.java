package frc.robot.subsystems;

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
    private final SparkPIDController angleController;
    private final SparkPIDController leftPIDController;
    private final SparkPIDController rightPIDController;
    private final SparkPIDController setPID(SparkPIDController sparkPIDController , double p, double i, double d){
        sparkPIDController.setP(p);
        sparkPIDController.setI(i);
        sparkPIDController.setD(d);
        return sparkPIDController;
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

        // apply PID to the angle motor
        angleController = angleMotor.getPIDController();
        setPID(angleController, ShooterConstants.AnglePIDF.P, ShooterConstants.AnglePIDF.I, ShooterConstants.AnglePIDF.D);
        leftPIDController = leftMotor.getPIDController();
        setPID(leftPIDController, ShooterConstants.LeftPIDF.P, ShooterConstants.LeftPIDF.I, ShooterConstants.LeftPIDF.D);
        rightPIDController = rightMotor.getPIDController();
        setPID(rightPIDController, ShooterConstants.RightPIDF.P, ShooterConstants.RightPIDF.I, ShooterConstants.RightPIDF.D);
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
     * Set the shooter to the desired position.
     * @param position  Desired shooter position, will be applied using {@link com.revrobotics.SparkPIDController}
     */

    public void setPosition(double position) {
        angleController.setReference(position, ControlType.kPosition);
    }

    /**
     * Stop the shooter motors.
     */
    public void stopShoot() {
        leftMotor.set(0);
    }
}
