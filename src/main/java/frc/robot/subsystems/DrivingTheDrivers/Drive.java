// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  public double xMoveSpeedTarget;
  private double rotationSpeedTarget; 
  private double yMoveSpeedTarget;
  private Pigeon2 gyro;
  private double inputRotationSpeedWithDeadband;
  private double inputXSpeedWithDeadBand;
  private double inputYSpeedWithDeadBand;
  

  
  public Drive() {

    frontLeftModule = new SwerveModule(10,1,5); //Creates the swerve module objects
    frontRightModule = new SwerveModule(11,2,6); //The values are the IDs for the motors
    backLeftModule = new SwerveModule(9,3,7);
    backRightModule = new SwerveModule(12,4,8);

    ps5Controller = new CommandPS5Controller(0);

    gyro = new Pigeon2(25, "rio");

    roboSwerveKinematics = new SwerveDriveKinematics(Constants.SwerveModuleConstants.forwardLeftSwerve, Constants.SwerveModuleConstants.forwardRightSwerve, 
      Constants.SwerveModuleConstants.backLeftSwerve, Constants.SwerveModuleConstants.backRightSwerve); 
      // outputs in meters/second so we will be using this unit for everything now! We may or may not want to change to rotations or somthing idk TODO: Decide what units everythings going to be
  }

  public void setInput() { //TODO: at some point change this to a defalt command. (multible things need to be commands) A defalt comand will do the running all the time thing exept you can interupt it w/ another command
      inputRotationSpeedWithDeadband = MathUtil.applyDeadband(ps5Controller.getRightX(), Constants.DriveConstants.deadBand);  //TODO: find the inverted joystick axis in the controller tab of Driverstation
      inputXSpeedWithDeadBand = MathUtil.applyDeadband(ps5Controller.getLeftX(), Constants.DriveConstants.deadBand);
      inputYSpeedWithDeadBand = MathUtil.applyDeadband(ps5Controller.getLeftY(), Constants.DriveConstants.deadBand);
  }

  public void setTargetSpeed() { //TODO: maybe square or cube input
    rotationSpeedTarget = inputRotationSpeedWithDeadband * Constants.DriveConstants.maxRotationSpeed;
    xMoveSpeedTarget = inputXSpeedWithDeadBand * Constants.DriveConstants.maxDriveSpeed;
    yMoveSpeedTarget = inputYSpeedWithDeadBand * Constants.DriveConstants.maxDriveSpeed;
  }

  public void setSwerveModuleStateArray() { //this will work but why TODO: make this nolonger an instanance variable or somthing idk Abe said it would look better as somthing else
    swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget, gyro.getRotation2d())); //TODO: this assumes we will always use field reletive maybe in futer we will want a switch
  }

  public void setModules() {
    frontLeftModule.setState(swerveModuleStatesArray[0]);
    frontRightModule.setState(swerveModuleStatesArray[1]);
    backLeftModule.setState(swerveModuleStatesArray[2]);
    backRightModule.setState(swerveModuleStatesArray[3]);
  }

  public void moveModules() {
      frontLeftModule.move();
      frontRightModule.move();
      frontLeftModule.move();
      backRightModule.move();

  }
 

  public void periodic() { //TODO: use .optimize at some point
    
     setInput();
     setTargetSpeed();
     setSwerveModuleStateArray();
     setModules();
     moveModules();

  }
}