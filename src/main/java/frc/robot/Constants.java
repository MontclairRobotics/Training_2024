// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * I think the distances for each swerve module (as Translation2d) should go in constants
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    
    

  }
  public static class SwerveModuleConstants {
  public static final Translation2d forwardRightSwerve = new Translation2d(0.301625, 0.301625); 
  public static final Translation2d forwardLeftSwerve = new Translation2d(0.301625, 0.301625);
  public static final Translation2d backRightSwerve = new Translation2d(0.301625, 0.301625);
  public static final Translation2d backLeftSwerve = new Translation2d(0.301625, 0.301625); 
  }
  public static class DriveConstants {
    public static final double maxSpeed = 2.0; // In meters per second
    public static final double deadBand = 0.06;
  }
} 
