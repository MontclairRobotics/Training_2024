// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DrivingTheDrivers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS4Controller;
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
  private SwerveDriveKinematics roboSwerveKinematics; // TODO: we should probably change the name later
  private SwerveModuleState[] swerveModuleStates; //TODO: figure out why this says not being used all wierd.
  private double tempXTarget = 10; //remove soon this represents the speeds were trying to get to and stuff (this is me writing code out probobly all wrong)
  private double tempYTarget = 10;
  private double tempRotationTarget = 90;
  private Rotation2d tempRotation2d = new Rotation2d(0,0);
  private PS4Controller ps4Controller; //absolutly not we are not naming like this 
  
  
  public Drive() {

    //everything below is wrong!!!!!!!!! we need to do the swerve module stuff here now.


    // forwardRight = new SwerveModule(1,1,1,Constants.SwerveModuleConstants.forwardRightSwerveX,Constants.SwerveModuleConstants.forwardRightSwerveY); //these x and y constants are all 0.0 for now, and found under constants
    // forwardLeft = new SwerveModule(1,1,1,Constants.SwerveModuleConstants.forwardLeftSwerveX,Constants.SwerveModuleConstants.forwardLeftSwerveY); //if CAD won't tell us we can tottally just look in the old robot code of this. 
    // backRight = new SwerveModule(1,1,1,Constants.SwerveModuleConstants.backRightSwerveX,Constants.SwerveModuleConstants.backRightSwerveY);
    // backLeft = new SwerveModule(1,1,1,Constants.SwerveModuleConstants.backLeftSwerveX,Constants.SwerveModuleConstants.backLeftSwerveY);

    //roboSwerveKinematics = new SwerveDriveKinematics(posForwardLeft,posForwardRight,posBackLeft,posBackRight); //output in meters/second so we will be using this unit for everything now!
    
    //swerveModuleStates = roboSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
    //tempXTarget, tempYTarget, tempRotationTarget, tempRotation2d)); //TODO: fix this
  }
}
    // next we need to do the PID stuff.The Swerve module states are what we put into PID (AND if tuned right should output volteges to feedinto out motors). For the forwardLeft one, we input forwardLeft's current state, and the swerve states with an index of 0. Then forward right for current and states index 1 would be for changing forward right, etc. This is how wpilib has it: motor.set(pid.calculate(encoder.getDistance(), setpoint));
    // for PID we may have to do different stuff just because the motors may require their own methods. PID also has a setpoint method




    //swerve kinematics - SwerveDriveKinematics object takes in 4 Translation2d objects, forwardleft, forwardright, backleft, and backright
  // here we  need an object of  SwerveDriveKinematics made with a Translation2d that cad has values for or somthing smh