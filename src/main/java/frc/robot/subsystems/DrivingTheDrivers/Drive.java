// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DrivingTheDrivers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Drive extends SubsystemBase {

  private SwerveModule forwardLeftModule;
  private SwerveModule forwardRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  public SwerveModuleState[] swerveModuleStatesArray; 
  private Rotation2d tempRotation2d = new Rotation2d(0,0);
  public SwerveDriveKinematics roboSwerveKinematics;
  private PS5Controller ps5Controller;
  private double rotationTarget;  //TODO: figure out why this says not being used all wierd.
  private double xMoveSpeedTarget;
  private double rotationSpeedTarget; 
  private double yMoveSpeedTarget;
  
  public Drive() {


    forwardLeftModule = new SwerveModule(1,1,1); //The values are the IDs for the motors
    forwardRightModule = new SwerveModule(1,1,1); // creates the swerve module objects
    backLeftModule = new SwerveModule(1,1,1);
    backRightModule = new SwerveModule(1,1,1);

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