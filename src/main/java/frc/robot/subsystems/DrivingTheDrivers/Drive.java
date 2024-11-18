// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Drive extends SubsystemBase {

  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  public SwerveModuleState[] swerveModuleStatesArray; 
  public SwerveDriveKinematics roboSwerveKinematics;
  private CommandPS5Controller ps5Controller;
  private double xMoveSpeedTarget;
  private double rotationSpeedTarget; 
  private double yMoveSpeedTarget;
  private Pigeon2 gyro;
  private double inputRotationSpeed;
  private double inputXSpeed;
  private double inputYSpeed;
  

  
  public Drive() {


    frontLeftModule = new SwerveModule(10,1,5); //The values are the IDs for the motors
    frontRightModule = new SwerveModule(11,2,6); // creates the swerve module objects
    backLeftModule = new SwerveModule(9,3,7);
    backRightModule = new SwerveModule(12,4,8);

    ps5Controller = new CommandPS5Controller(0);

    gyro = new Pigeon2(25, "rio");

    roboSwerveKinematics = new SwerveDriveKinematics(Constants.SwerveModuleConstants.forwardLeftSwerve, Constants.SwerveModuleConstants.forwardRightSwerve, 
      Constants.SwerveModuleConstants.backRightSwerve, Constants.SwerveModuleConstants.backLeftSwerve); 
      // outputs in meters/second so we will be using this unit for everything now!

  }
  public void periodic() {
    inputRotationSpeed = ps5Controller.getRightX();
    inputXSpeed = ps5Controller.getLeftX();
    inputYSpeed = ps5Controller.getLeftY();

    rotationSpeedTarget = inputRotationSpeed * 2;
    xMoveSpeedTarget = inputXSpeed * 2; // 2 represents the max speed in m/s
    yMoveSpeedTarget = inputYSpeed * 2; 

    swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget, gyro.getRotation2d()));

    frontLeftModule.setState(swerveModuleStatesArray[0]);
    frontRightModule.setState(swerveModuleStatesArray[1]);
    backLeftModule.setState(swerveModuleStatesArray[2]);
    backRightModule.setState(swerveModuleStatesArray[3]);

    if (Math.abs(inputRotationSpeed) > 0.06 || Math.abs(inputXSpeed) > 0.06 || Math.abs(inputYSpeed) > 0.06) {
      System.out.println("this if statment is running");
      frontLeftModule.move();
      frontRightModule.move();
      frontLeftModule.move();
      backRightModule.move();
    }
  }
}