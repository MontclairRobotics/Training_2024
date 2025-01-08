package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Shooter {
    
    public SparkMax bottomMotor;
    public SparkMax topMotor;
    public SparkMax transportMotor;
    public CANcoder shooterEncoder;

    public Shooter() {
        topMotor = new SparkMax(Constants.ShooterConstants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        bottomMotor = new SparkMax(Constants.ShooterConstants.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        transportMotor = new SparkMax(Constants.ShooterConstants.TRANSPORT_MOTOR_ID, MotorType.kBrushless);

    }

    public void shoot() {
        topMotor.set(Constants.ShooterConstants.SHOOTER_SPEED);
        bottomMotor.set(-Constants.ShooterConstants.SHOOTER_SPEED);
        transportMotor.set(Constants.ShooterConstants.TRANSPORT_SPEED);
    }

    public void stop() {
        topMotor.set(0);
        bottomMotor.set(0);
        transportMotor.set(0);
    }

    public void reverseShooter() {
        topMotor.set(-Constants.ShooterConstants.SHOOTER_SPEED);
        bottomMotor.set(-Constants.ShooterConstants.SHOOTER_SPEED);
        transportMotor.set(-Constants.ShooterConstants.TRANSPORT_SPEED);
    }

    /*Commands */
    public Command shootCommand() {
        return Commands.runOnce(() -> shoot());
    }

    public Command reverseShooterCommand() {
        return Commands.runOnce(() -> reverseShooter());
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }
}
