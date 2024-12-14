// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveTrain.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static Drive drive = new Drive();
  public static Intake intake = new Intake();
  public static Climbers climbers = new Climbers();
  public static ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Controllers
  private final CommandPS5Controller drivePS5Controller =
      new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER);

  private final CommandPS5Controller operatorPS5Controller =
      new CommandPS5Controller(OperatorConstants.OPERATOR_CONTROLLER);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drive.setDefaultCommand(  //A defalt comand will do the running all the time thing exept you can interupt it w/ another command
      Commands.run(
      ()-> {
        drive.setTargetSpeedFromController(drivePS5Controller); //lambda for setting target speed whether it be robo or field relative
      }, drive
    ));
  }

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    //Drive
    drivePS5Controller.circle().onTrue(drive.toggleRobotRelativeCommand());
    drivePS5Controller.touchpad().onTrue(drive.zeroGyroscopeCommand());

    //Intake
    operatorPS5Controller.cross().whileTrue(intake.outTakeNoteCommand()).onFalse(intake.stopCommand());
    operatorPS5Controller.triangle().whileTrue(intake.inTakeNoteCommand()).onFalse(intake.stopCommand());
    
    //Climbers
    operatorPS5Controller.square().onTrue(climbers.downCommand()).onFalse(climbers.stopCommand());
    operatorPS5Controller.cross().onTrue(climbers.upCommand()).onFalse(climbers.stopCommand());    
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
