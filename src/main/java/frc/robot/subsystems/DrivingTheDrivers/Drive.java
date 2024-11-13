// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DrivingTheDrivers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Drive extends SubsystemBase {

  private SwerveModule forwardLeftModule;
  private SwerveModule forwardRightModule; //TODO: figure out why this says not being used all wierd.
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  public SwerveModuleState[] swerveModuleStatesArray; 
  private double tempXTarget = 10; //TODO: remove and use the real values from the controller
  private double tempYTarget = 10;
  private double tempRotationTarget = 90;
  private Rotation2d tempRotation2d = new Rotation2d(0,0);
  public SwerveDriveKinematics roboSwerveKinematics;
  private PS5Controller ps5Controller;
  private double rotationTarget;
  private double xMoveSpeedTarget;
  private double rotationSpeedTarget; 
  private double yMoveSpeedTarget;
  
  public Drive() {


    forwardLeftModule = new SwerveModule(1,1,1,0); //First three values are the IDs for the motors and the last just tells it which module it is. id: 0 front left, 1 front right, 2 back left, 3 back right.
    forwardRightModule = new SwerveModule(1,1,1,1); // creates the swerve module objects
    backLeftModule = new SwerveModule(1,1,1,2);
    backRightModule = new SwerveModule(1,1,1,3);

    ps5Controller = new PS5Controller(1);


    roboSwerveKinematics = new SwerveDriveKinematics(Constants.SwerveModuleConstants.forwardLeftSwerve, Constants.SwerveModuleConstants.forwardRightSwerve, 
      Constants.SwerveModuleConstants.backRightSwerve, Constants.SwerveModuleConstants.backLeftSwerve); 
      // outputs in meters/second so we will be using this unit for everything now!


    
  }
  public void periodic() {
    rotationTarget = ps5Controller.getRightX();
    rotationSpeedTarget = ps5Controller.getRightY() * 2;
    xMoveSpeedTarget = ps5Controller.getLeftX() * 2; // 2 represents the max speed in m/s
    yMoveSpeedTarget = ps5Controller.getLeftY() * 2; 

    swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget, tempRotation2d));

    forwardLeftModule.setState(swerveModuleStatesArray[0]);
    forwardRightModule.setState(swerveModuleStatesArray[1]);
    backLeftModule.setState(swerveModuleStatesArray[2]);
    backRightModule.setState(swerveModuleStatesArray[3]);

  }
}