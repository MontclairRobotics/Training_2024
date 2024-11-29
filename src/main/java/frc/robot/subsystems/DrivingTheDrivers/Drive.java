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
  private Pigeon2 gyro;

  public double xMoveSpeedTarget;
  public double yMoveSpeedTarget;
  public double rotationSpeedTarget; 

  private double inputXSpeedWithDeadBand;
  private double inputYSpeedWithDeadBand;
  private double inputRotationSpeedWithDeadband;

  
  public Drive() {

    frontLeftModule = new SwerveModule(10,1,5); //Creates the swerve module objects
    frontRightModule = new SwerveModule(11,2,6); //The values are the IDs for the motors
    backLeftModule = new SwerveModule(9,3,7);
    backRightModule = new SwerveModule(12,4,8);

    gyro = new Pigeon2(25, "rio");

    roboSwerveKinematics = new SwerveDriveKinematics(Constants.SwerveModuleConstants.FRONT_LEFT_SWERVE_POSITION, Constants.SwerveModuleConstants.FRONT_RIGHT_SWERVE_POSITION, 
      Constants.SwerveModuleConstants.BACK_LEFT_SWERVE_POSITION, Constants.SwerveModuleConstants.BACK_RIGHT_SWERVE_POSITION); 
      // outputs in meters/second so we will be using this unit for everything now! Also we now use radians
  }

  public void setTargetSpeed(CommandPS5Controller controller) { //This is now called as a defalt command in robot container (thats why it looks like this now) (multible other things need to be commands I think) 

    inputRotationSpeedWithDeadband = MathUtil.applyDeadband(controller.getRightX(), Constants.DriveConstants.CONTROLLER_DEAD_BAND);  //there doesnt seem to be any inverted axis of the controlers but keep an eye out
    inputXSpeedWithDeadBand = MathUtil.applyDeadband(controller.getLeftX(), Constants.DriveConstants.CONTROLLER_DEAD_BAND);
    inputYSpeedWithDeadBand = MathUtil.applyDeadband(controller.getLeftY(), Constants.DriveConstants.CONTROLLER_DEAD_BAND);
    
    rotationSpeedTarget = inputRotationSpeedWithDeadband * Constants.DriveConstants.MAX_ROTATION_SPEED;
    xMoveSpeedTarget = inputXSpeedWithDeadBand * Constants.DriveConstants.MAX_DRIVE_SPEED;
    yMoveSpeedTarget = inputYSpeedWithDeadBand * Constants.DriveConstants.MAX_DRIVE_SPEED; //TODO: maybe square or cube input
  }

  public void setSwerveModuleStateArray() { //this will work but why TODO: make this nolonger an instanance variable or somthing idk Abe said it would look better as somthing else
    swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget, gyro.getRotation2d())); //TODO: this assumes we will always use field reletive maybe in the future we will want a switch
  }

  public void setAndMoveModules() {
      frontLeftModule.setStateAndMove(swerveModuleStatesArray[0]);
      frontRightModule.setStateAndMove(swerveModuleStatesArray[1]);
      frontLeftModule.setStateAndMove(swerveModuleStatesArray[2]);
      backRightModule.setStateAndMove(swerveModuleStatesArray[3]);
  }
  
  public void periodic() { //TODO: use .optimize at some point
     setSwerveModuleStateArray(); //setTargetSpeed is not here because its a defalt command in the robot container
     setAndMoveModules();
  }
}