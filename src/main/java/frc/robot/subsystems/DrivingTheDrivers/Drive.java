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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drive extends SubsystemBase {
  private SwerveModule forwardRight;
  private SwerveModule forwardLeft;
  private SwerveModule backRight;
  private SwerveModule backLeft;
  private Translation2d posForwardRight;
  private Translation2d posForwardLeft;
  private Translation2d posBackRight;
  private Translation2d posBackLeft;
  public Drive() {
    forwardRight = new SwerveModule(1,1,1,forwardRightSwerveX,forwardRightSwerveY); //these x and y constants are all 0.0 for now, and found under constants
    forwardLeft = new SwerveModule(1,1,1,forwardLeftSwerveX,forwardLeftSwerveY);
    backRight = new SwerveModule(1,1,1,backRightSwerveX,backRightSwerveY);
    backLeft = new SwerveModule(1,1,1,backLeftSwerveX,backLeftSwerveY);
    posForwardRight = forwardRight.getPos();
    posForwardLeft = forwardLeft.getPos(); // bla bla unfinished 
    posBackRight = backRight.getPos();
    posBackLeft = backLeft.getPos();
  // here we  need an object of  SwerveDriveKinematics made with a Translation2d that cad has values for or somthing smh
        // SwerveModuleState[] swerveModuleStates =
        //     kinematics.toSwerveModuleStates(velocity, centerOfRotationMeters); // prety sure  all this stuff I'm doing goes in a different part of the file but I'll do real stuff tomawrrow this is just stuff abe told me about so now I'm writing it
  }
}