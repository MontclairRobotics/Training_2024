// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final double forwardRightSwerveX = 0.0; //we will get these numbers later from CAD or smth
  public static final double forwardRightSwerveY = 0.0;
  public static final double forwardLeftSwerveX = 0.0;
  public static final double forwardLeftSwerveY = 0.0;
  public static final double backRightSwerveX = 0.0;
  public static final double backRightSwerveY = 0.0;
  public static final double backLeftSwerveX = 0.0;
  public static final double backLeftSwerveY = 0.0;
  }
}
