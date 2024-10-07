// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DrivingTheDrivers;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drive extends SubsystemBase {
  
  /** Creates a new ExampleSubsystem. */
  public Drive() {
    pid = new PIDController(-1,-1,-1);
    canSparkLeftForward = new CANSparkMax(1, MotorType.kBrushless);
    canSparkRightForward = new CANSparkMax(2, MotorType.kBrushless);
    canSparkLeftBack = new CANSparkMax(3, MotorType.kBrushless);
    canSparkRightBack = new CANSparkMax(4, MotorType.kBrushless);
    canBusName = "rio";
    canCoder = new CANcoder(1, canBusName);
    canSparkLeftForward.set(pid.calculate((canCoder.getDistance(), PlaceholderSetpoint));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}