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
 */
public final class Constants {
  public static class OperatorConstants {

    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
    public static final double DRIVER_CONTROLLER_DEAD_BAND = 0.08;
    public static final double OPERATOR_CONTROLLER_DEAD_BAND = 0.2;

  }
  public static class SwerveModuleConstants {

    public static final Translation2d FRONT_LEFT_SWERVE_POSITION = new Translation2d(0.301625, 0.301625);
    public static final Translation2d FRONT_RIGHT_SWERVE_POSITION = new Translation2d(0.301625, -0.301625); 
    public static final Translation2d BACK_LEFT_SWERVE_POSITION = new Translation2d(-0.301625, 0.301625); 
    public static final Translation2d BACK_RIGHT_SWERVE_POSITION = new Translation2d(-0.301625, -0.301625);
  }
  public static class DriveConstants {

    public static final double MAX_DRIVE_SPEED = 2.0; //Meters per second
    public static final double MAX_ROTATION_SPEED = Math.PI; //RADIANS

    public static final double DRIVE_ENCODER_ROTATION_TO_METERS_OF_THE_WHEEL_RATIO = 0.407;
  }

  public static class IntakeConstants {

    public static final double INTAKE_SPEED = 0.6;

    public static final int INTAKE_TOP_MOTOR_ID = 20;
    public static final int INTAKE_BOTTOM_MOTOR_ID = 21;
  }

  public static class ClimberConstants {

    public static final double CLIMBER_SPEED = 0.2;

    public static final int CLIMBER_LEFT_MOTOR_ID = 48;
    public static final int CLIMBER_RIGHT_MOTOR_ID = 49;
    public static final int CLIMBER_LEFT_LIMIT_SWITCH_ID= 5;
    public static final int CLIMBER_RIGHT_LIMIT_SWITCH_ID = 4;
  }

  public static class SprocketConstants {
  
    public static final double SPROCKET_SPEED = 0.15;

    public static final int SPROCKET_RIGHT_MOTOR_ID = 31;
    public static final int SPROCKET_LEFT_MOTOR_ID = 30;    
  }

  public static class ShooterConstants {
    public static final double SHOOTER_SPEED = 0.8;
    public static final double TRANSPORT_SPEED = 0.2;
    
    public static final int TOP_SHOOTER_MOTOR_ID = 28;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 29;
    public static final int TRANSPORT_MOTOR_ID = 27;
  }
} 
