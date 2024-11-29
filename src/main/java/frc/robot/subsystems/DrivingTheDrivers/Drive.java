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

  public SwerveDriveKinematics roboSwerveKinematics;
  private Pigeon2 gyro;

  public double xMoveSpeedTarget;
  public double yMoveSpeedTarget;
  public double rotationSpeedTarget; 

  
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

    double inputRotationSpeedWithDeadband = MathUtil.applyDeadband(controller.getRightX(), Constants.DriveConstants.CONTROLLER_DEAD_BAND);  //there doesnt seem to be any inverted axis of the controlers but keep an eye out
    double inputXSpeedWithDeadBand = MathUtil.applyDeadband(controller.getLeftX(), Constants.DriveConstants.CONTROLLER_DEAD_BAND);
    double inputYSpeedWithDeadBand = MathUtil.applyDeadband(controller.getLeftY(), Constants.DriveConstants.CONTROLLER_DEAD_BAND);
    
    rotationSpeedTarget = Math.pow(inputRotationSpeedWithDeadband, 3) * Constants.DriveConstants.MAX_ROTATION_SPEED; //cubed because its easier to controll (still gives value between -1 and 1) 
    xMoveSpeedTarget = Math.pow(inputXSpeedWithDeadBand, 3) * Constants.DriveConstants.MAX_DRIVE_SPEED;
    yMoveSpeedTarget = Math.pow(inputYSpeedWithDeadBand, 3) * Constants.DriveConstants.MAX_DRIVE_SPEED;
  }

  public void setAndMoveModules() {
    SwerveModuleState[] swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget, gyro.getRotation2d())); //TODO: this assumes we will always use field reletive maybe in the future we will want a switch
  
      frontLeftModule.setStateAndMove(swerveModuleStatesArray[0]);
      frontRightModule.setStateAndMove(swerveModuleStatesArray[1]);
      frontLeftModule.setStateAndMove(swerveModuleStatesArray[2]);
      backRightModule.setStateAndMove(swerveModuleStatesArray[3]);
  }
  
  public void periodic() {
    setAndMoveModules(); //setTargetSpeed is not here because its a defalt command in the robot container
  }
}