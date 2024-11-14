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
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Drive extends SubsystemBase {

  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  public SwerveModuleState[] swerveModuleStatesArray; 
  public SwerveDriveKinematics roboSwerveKinematics;
  private PS5Controller ps5Controller;
  private double xMoveSpeedTarget;
  private double rotationSpeedTarget; 
  private double yMoveSpeedTarget;
  private Pigeon2 gyro;
  

  
  public Drive() {


    frontLeftModule = new SwerveModule(1,1,1); //The values are the IDs for the motors
    frontRightModule = new SwerveModule(1,1,1); // creates the swerve module objects
    backLeftModule = new SwerveModule(1,1,1);
    backRightModule = new SwerveModule(1,1,1);

    ps5Controller = new PS5Controller(1);
    gyro = new Pigeon2(0, "rio");

    roboSwerveKinematics = new SwerveDriveKinematics(Constants.SwerveModuleConstants.forwardLeftSwerve, Constants.SwerveModuleConstants.forwardRightSwerve, 
      Constants.SwerveModuleConstants.backRightSwerve, Constants.SwerveModuleConstants.backLeftSwerve); 
      // outputs in meters/second so we will be using this unit for everything now!

  }
  public void periodic() {

    rotationSpeedTarget = ps5Controller.getRightX() * 2;
    xMoveSpeedTarget = ps5Controller.getLeftX() * 2; // 2 represents the max speed in m/s
    yMoveSpeedTarget = ps5Controller.getLeftY() * 2; 

    swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget, gyro.getRotation2d()));

    frontLeftModule.setState(swerveModuleStatesArray[0]);
    frontRightModule.setState(swerveModuleStatesArray[1]);
    backLeftModule.setState(swerveModuleStatesArray[2]);
    backRightModule.setState(swerveModuleStatesArray[3]);

    if (rotationSpeedTarget != 0 || xMoveSpeedTarget != 0 || yMoveSpeedTarget !=0) {
    frontLeftModule.move();
    frontRightModule.move();
    frontLeftModule.move();
    backRightModule.move();
    }
  }
}