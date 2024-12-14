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

  public SwerveModule frontLeftModule;
  public SwerveModule frontRightModule;
  public SwerveModule backLeftModule;
  public SwerveModule backRightModule;

  public SwerveDriveKinematics roboSwerveKinematics;
  public Pigeon2 gyro;

  public double xMoveSpeedTarget;
  private double yMoveSpeedTarget;
  private double rotationSpeedTarget;
  public boolean fieldRelative = true; //This decides if we drive field or robo relative


  
  public Drive() {
    
    frontLeftModule = new SwerveModule(10,1,5,2.70438768); //Creates the swerve module objects
    frontRightModule = new SwerveModule(11,2,6,1.99892559); //The values are the IDs for the motors
    backLeftModule = new SwerveModule(9,3,7,1.2470378); //absalute encoder offsets in radians
    backRightModule = new SwerveModule(12,4,8,3.78876074);
    
    gyro = new Pigeon2(25, "rio"); //gyroscope tells us what direction the whole robot is facing

    roboSwerveKinematics = new SwerveDriveKinematics(Constants.SwerveModuleConstants.FRONT_LEFT_SWERVE_POSITION, Constants.SwerveModuleConstants.FRONT_RIGHT_SWERVE_POSITION, 
      Constants.SwerveModuleConstants.BACK_LEFT_SWERVE_POSITION, Constants.SwerveModuleConstants.BACK_RIGHT_SWERVE_POSITION); 
      //A helper class used later that does math with our for module positions. Uses meters (our unit for everything)
  }

  public void zeroGyro() {
    gyro.reset(); // sets the gyroscope's 0 to its current angle
  }
  
  public void fieldRelative() {
    if (fieldRelative == true) {
      fieldRelative = false;
    } else {
      fieldRelative = true;
    }
  }

  public void setTargetSpeed(CommandPS5Controller controller) { //This is now called as a defalt command in robot container
    
    double inputRotationSpeedWithDeadband = MathUtil.applyDeadband(controller.getRightX(), Constants.OperatorConstants.CONTROLLER_DEAD_BAND);  //this does not seem to be the case for now however one axis of a controller may be inverted
    double inputXSpeedWithDeadBand = MathUtil.applyDeadband(controller.getLeftX(), Constants.OperatorConstants.CONTROLLER_DEAD_BAND);
    double inputYSpeedWithDeadBand = MathUtil.applyDeadband(controller.getLeftY(), Constants.OperatorConstants.CONTROLLER_DEAD_BAND);
    
    rotationSpeedTarget = Math.pow(inputRotationSpeedWithDeadband, 3) * Constants.DriveConstants.MAX_ROTATION_SPEED; //cubed because its easier to controll (still gives value between -1 and 1) 
    xMoveSpeedTarget = Math.pow(inputXSpeedWithDeadBand, 3) * Constants.DriveConstants.MAX_DRIVE_SPEED; //When controller all the way should output 2 (meters persecond)
    yMoveSpeedTarget = Math.pow(inputYSpeedWithDeadBand, 3) * Constants.DriveConstants.MAX_DRIVE_SPEED;

    if(fieldRelative) {
      driveFieldRelative(xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget);
    } else {
      driveRobotRelative(xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget);
    }
  }

  public void setSwerveModules(SwerveModuleState[] swerveModuleStatesArray) {
    frontLeftModule.setStateAndMove(swerveModuleStatesArray[0]);
    frontRightModule.setStateAndMove(swerveModuleStatesArray[1]);
    backLeftModule.setStateAndMove(swerveModuleStatesArray[2]);
    backRightModule.setStateAndMove(swerveModuleStatesArray[3]);
  }

  public void driveRobotRelative(double xMoveSpeedTarget, double yMoveSpeedTarget, double rotationSpeedTarget) {
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget);

    SwerveModuleState[] swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    setSwerveModules(swerveModuleStatesArray);
  }

  public void driveFieldRelative(double xMoveSpeedTarget, double yMoveSpeedTarget, double rotationSpeedTarget) {

    SwerveModuleState[] swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget, gyro.getRotation2d()));

    setSwerveModules(swerveModuleStatesArray);
  }
}