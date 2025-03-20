// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.subsystems.DriveTrain.Drive;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  public static Sprocket sprocket = new Sprocket();
  public static Shooter shooter = new Shooter();
  public static ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Controllers
  private final CommandPS5Controller drivePS5Controller =
      new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER);
  //x and y switched on drive controller 

  private final CommandPS5Controller operatorPS5Controller =
      new CommandPS5Controller(OperatorConstants.OPERATOR_CONTROLLER);
  //operator controller is inverted on the y axis

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drive.setDefaultCommand(  //A default command will do the running all the time thing except you can interrupt it w/ another command
      Commands.run(
      ()-> {
        drive.setTargetSpeedFromController(drivePS5Controller); //lambda for setting target speed whether it be robo or field relative
      }, drive
    ));

    sprocket.setDefaultCommand(
      Commands.run(
        ()-> {
          sprocket.inputFromController(drivePS5Controller);
        }, sprocket
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

    GenericHID operatorHID = operatorPS5Controller.getHID();
    POVButton operatorUpDPad = new POVButton(operatorHID, 0);
    @SuppressWarnings("unused")
    POVButton operatorRightDPad = new POVButton(operatorHID, 90);
    POVButton operatorDownDPad = new POVButton(operatorHID, 180);
    @SuppressWarnings("unused")
    POVButton operatorLeftDPad = new POVButton(operatorHID, 270);

    GenericHID driverHID = drivePS5Controller.getHID();
    POVButton driverUpDPad = new POVButton(driverHID, 0);
    POVButton driverRightDPad = new POVButton(driverHID, 90);
    POVButton driverDownDPad = new POVButton(driverHID, 180);
    POVButton driverLeftDPad = new POVButton(driverHID, 270);
    

    //Drive
    drivePS5Controller.circle().onTrue(drive.toggleRobotRelativeCommand());
    drivePS5Controller.touchpad().onTrue(Commands.runOnce(()-> drive.gyro.reset()));

    //Intake
    operatorPS5Controller.triangle().whileTrue(intake.inTakeNoteCommand()).onFalse(intake.stopCommand());
    operatorPS5Controller.cross().whileTrue(intake.outTakeNoteCommand()).onFalse(intake.stopCommand());
    
    //Climbers
    operatorUpDPad.onTrue(climbers.upCommand()).onFalse(climbers.stopCommand());    
    operatorDownDPad.onTrue(climbers.downCommand()).onFalse(climbers.stopCommand());
    //Sprocket

    //Shooter
    drivePS5Controller.triangle().onTrue(shooter.shootCommand()).onFalse(shooter.stopCommand());
    drivePS5Controller.square().onTrue(shooter.reverseShooterCommand()).onFalse(shooter.stopCommand());




    drivePS5Controller.L1().onTrue(Commands.runOnce(() -> SignalLogger.start()));
    drivePS5Controller.R1().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
    
    driverDownDPad.whileTrue(
      drive.sysIdDynamic(Direction.kForward)
    );
    driverLeftDPad.whileTrue(
      drive.sysIdDynamic(Direction.kReverse)
    );
    driverRightDPad.whileTrue(
      drive.sysIdQuasistatic(Direction.kForward)
    );
    driverUpDPad.whileTrue(
      drive.sysIdQuasistatic(Direction.kReverse)
    );
  }

  public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("TRAINING_2024/src/main/java/frc/robot/paths/1_b.path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

}
