// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;


public class Drive extends SubsystemBase {

  public SwerveModule frontLeftModule;
  public SwerveModule frontRightModule;
  public SwerveModule backLeftModule;
  public SwerveModule backRightModule;

  public SwerveDriveKinematics roboSwerveKinematics;
  public Pigeon2 gyro;

  public boolean isFieldRelative = true; //This decides if we drive field or robo relative

  
  public Drive() {
    
    frontLeftModule = new SwerveModule(10,1,5,2.70438768); //Creates the swerve module objects
    frontRightModule = new SwerveModule(11,2,6,1.99892559); //The values are the IDs for the motors
    backLeftModule = new SwerveModule(9,3,7,1.2470378); //Absolute encoder offsets in radians
    backRightModule = new SwerveModule(12,4,8,3.78876074);
    
    gyro = new Pigeon2(25, "rio"); //gyroscope tells us what direction the whole robot is facing

    roboSwerveKinematics = new SwerveDriveKinematics(Constants.SwerveModuleConstants.FRONT_LEFT_SWERVE_POSITION, Constants.SwerveModuleConstants.FRONT_RIGHT_SWERVE_POSITION, 
      Constants.SwerveModuleConstants.BACK_LEFT_SWERVE_POSITION, Constants.SwerveModuleConstants.BACK_RIGHT_SWERVE_POSITION); 
      //A helper class used later that does math with our for module positions. Uses meters (our unit for everything)
  }

  public void setTargetSpeedFromController(CommandPS5Controller controller) { //This is now called as a default command in robot container
    
    double inputRotationSpeedWithDeadband = -MathUtil.applyDeadband(controller.getRightX(), Constants.OperatorConstants.DRIVER_CONTROLLER_DEAD_BAND);  //this does not seem to be the case for now however one axis of a controller may be inverted
    double inputXSpeedWithDeadBand = -MathUtil.applyDeadband(controller.getLeftY(), Constants.OperatorConstants.DRIVER_CONTROLLER_DEAD_BAND);
    double inputYSpeedWithDeadBand = -MathUtil.applyDeadband(controller.getLeftX(), Constants.OperatorConstants.DRIVER_CONTROLLER_DEAD_BAND);
    
    double rotationSpeedTarget = Math.pow(inputRotationSpeedWithDeadband, 3) * Constants.DriveConstants.MAX_ROTATION_SPEED; //cubed because its easier to control (still gives value between -1 and 1) 
    double xMoveSpeedTarget = Math.pow(inputXSpeedWithDeadBand, 3) * Constants.DriveConstants.MAX_DRIVE_SPEED; //When controller all the way should output 2 (meters /s)
    double yMoveSpeedTarget = Math.pow(inputYSpeedWithDeadBand, 3) * Constants.DriveConstants.MAX_DRIVE_SPEED;

    if(isFieldRelative) {
      driveFieldRelative(xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget);
    } else {
      driveRobotRelative(xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget);
    }
  }

  public void driveFieldRelative(double xMoveSpeedTarget, double yMoveSpeedTarget, double rotationSpeedTarget) {

    SwerveModuleState[] swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget, gyro.getRotation2d()));

    setSwerveModules(swerveModuleStatesArray);
  }

  public void driveRobotRelative(double xMoveSpeedTarget, double yMoveSpeedTarget, double rotationSpeedTarget) {
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xMoveSpeedTarget, yMoveSpeedTarget, rotationSpeedTarget);

    SwerveModuleState[] swerveModuleStatesArray = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    setSwerveModules(swerveModuleStatesArray);
  }

  public void setSwerveModules(SwerveModuleState[] swerveModuleStatesArray) {
    frontLeftModule.setStateAndMove(swerveModuleStatesArray[0]);
    frontRightModule.setStateAndMove(swerveModuleStatesArray[1]);
    backLeftModule.setStateAndMove(swerveModuleStatesArray[2]);
    backRightModule.setStateAndMove(swerveModuleStatesArray[3]);
  }

  public void voltageDrive(double driveVoltage, double rotationVoltage) {
    frontLeftModule.driveVoltageDrive(driveVoltage);
    frontRightModule.driveVoltageDrive(driveVoltage);
    backLeftModule.driveVoltageDrive(driveVoltage);
    backRightModule.driveVoltageDrive(driveVoltage);

    frontRightModule.driveVoltageDrive(rotationVoltage);
    frontRightModule.driveVoltageDrive(rotationVoltage);
    backLeftModule.driveVoltageDrive(rotationVoltage);
    backRightModule.driveVoltageDrive(rotationVoltage);

  }

  public void toggleRobotRelative() {
    if (isFieldRelative == true) {
      isFieldRelative = false;
    } else {
      isFieldRelative = true;
    }
  }



  /*
    * SysId routine for characterizing translation. This is used to find PID gains
    * for the drive motors.
    */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
          new SysIdRoutine.Config(
                  null, // Use default ramp rate (1 V/s)
                  Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                  null, // Use default timeout (10 s)
                  // Log state with SignalLogger class
                  state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
                  output -> voltageDrive(output.magnitude(), 0),
                  null,
                  this));

  /*
    * SysId routine for characterizing steer. This is used to find PID gains for
    * the steer motors.
    */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
          new SysIdRoutine.Config(
                  null, // Use default ramp rate (1 V/s)
                  Volts.of(7), // Use dynamic voltage of 7 V
                  null, // Use default timeout (10 s)
                  // Log state with SignalLogger class
                  state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
                  volts -> voltageDrive(volts.magnitude(), 0.0),
                  null,
                  this));

  /*
    * SysId routine for characterizing rotation.
    * This is used to find PID gains for the FieldCentricFacingAngle
    * HeadingController.
    * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
    * importing the log to SysId.
    */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
          new SysIdRoutine.Config(
                  /* This is in radians per second², but SysId only supports "volts per second" */
                  Volts.of(Math.PI / 6).per(Second),
                  /* This is in radians per second, but SysId only supports "volts" */
                  Volts.of(Math.PI),
                  null, // Use default timeout (10 s)
                  // Log state with SignalLogger class
                  state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
                  output -> {
                      /* output is actually radians per second, but SysId only supports "volts" */
                      voltageDrive(0, output.magnitude());
                      /* also log the requested output for SysId */
                      SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                  },
                  null,
                  this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutineToApply.dynamic(direction);
  }

  //Commands
  public Command toggleRobotRelativeCommand() {
    return Commands.runOnce( ()-> toggleRobotRelative());
  }
}