// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    Shuffleboard.getTab("drive").addDouble("direction", ()-> RobotContainer.drive.gyro.getRotation2d().getRadians());
    Shuffleboard.getTab("drive").addDouble("front left voltage", ()-> RobotContainer.drive.frontLeftModule.driveVoltage);
    Shuffleboard.getTab("drive").addDouble("front left turn voltage", ()-> RobotContainer.drive.frontLeftModule.turnVoltage);
    Shuffleboard.getTab("drive").addDouble("front right voltage", ()-> RobotContainer.drive.frontRightModule.driveVoltage);
    Shuffleboard.getTab("drive").addDouble("front right turn voltage", ()-> RobotContainer.drive.frontRightModule.turnVoltage);
    Shuffleboard.getTab("drive").addDouble("back left voltage", ()-> RobotContainer.drive.backLeftModule.driveVoltage);
    Shuffleboard.getTab("drive").addDouble("back left turn voltage", ()-> RobotContainer.drive.backLeftModule.turnVoltage);
    Shuffleboard.getTab("drive").addDouble("back right voltage", ()-> RobotContainer.drive.backRightModule.driveVoltage);
    Shuffleboard.getTab("drive").addDouble("back right turn voltage", ()-> RobotContainer.drive.backRightModule.turnVoltage);


    // Shuffleboard.getTab("acutally helpfull").addDouble("front right turn encoder value", ()-> RobotContainer.drive.frontRightModule.canCoder.getPosition().getValueAsDouble()*360);
    // Shuffleboard.getTab("acutally helpfull").addDouble("front right turn target value", ()-> RobotContainer.drive.swerveModuleStatesArray[2].angle.getDegrees());

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}