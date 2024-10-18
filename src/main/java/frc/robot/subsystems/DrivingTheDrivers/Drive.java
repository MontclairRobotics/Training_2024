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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  private SwerveDriveKinematics roboSwerve; // we should probably change the name later
  public static final double forwardRightSwerveX = 0.0; //we will get these numbers later from CAD or smth
  public static final double forwardRightSwerveY = 0.0;
  public static final double forwardLeftSwerveX = 0.0;
  public static final double forwardLeftSwerveY = 0.0;
  public static final double backRightSwerveX = 0.0;
  public static final double backRightSwerveY = 0.0;
  public static final double backLeftSwerveX = 0.0;
  public static final double backLeftSwerveY = 0.0;
  public Drive() {
    forwardRight = new SwerveModule(1,1,1,forwardRightSwerveX,forwardRightSwerveY); //these x and y constants are all 0.0 for now, and found under constants
    forwardLeft = new SwerveModule(1,1,1,forwardLeftSwerveX,forwardLeftSwerveY);
    backRight = new SwerveModule(1,1,1,backRightSwerveX,backRightSwerveY);
    backLeft = new SwerveModule(1,1,1,backLeftSwerveX,backLeftSwerveY);
    posForwardRight = forwardRight.getPos();
    posForwardLeft = forwardLeft.getPos(); // bla bla unfinished 
    posBackRight = backRight.getPos();
    posBackLeft = backLeft.getPos();

    roboSwerve = new SwerveDriveKinematics(posForwardLeft,posForwardRight,posBackLeft,posBackRight);
    //swerve kinematics - SwerveDriveKinematics object takes in 4 Translation2d objects, forwardleft, forwardright, backleft, and backright
  // here we  need an object of  SwerveDriveKinematics made with a Translation2d that cad has values for or somthing smh
        // SwerveModuleState[] swerveModuleStates =
        //     kinematics.toSwerveModuleStates(velocity, centerOfRotationMeters); // prety sure  all this stuff I'm doing goes in a different part of the file but I'll do real stuff tomawrrow this is just stuff abe told me about so now I'm writing it
  }
}