package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase{

    CANSparkMax leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
    CANSparkMax rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);
    DigitalInput leftlimitSwitch = new DigitalInput(Constants.ClimberConstants.CLIMBER_LEFT_LIMIT_SWITCH_ID);
    DigitalInput rightlimitSwitch = new DigitalInput(Constants.ClimberConstants.CLIMBER_RIGHT_LIMIT_SWITCH_ID);

    boolean canRightClimberGoDown = true;
    boolean canLeftClimberGoDown = true;

    public Climbers(){
//        leftClimberMotor.setInverted(true); //we might not need this TODO: Test Climbers and find out
//        rightClimberMotor.setInverted(true);
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
    }
    //start climber motors
    public void up(){
        leftClimberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED);
        rightClimberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED);
    }
    //stop the climber motors
    public void stop(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }
    //start in the reverse direction
    public void down(){
        if(canLeftClimberGoDown){ //this stuff checks the limit switch to make sure it can go down further first
            leftClimberMotor.set(-Constants.ClimberConstants.CLIMBER_SPEED);
        } else {
            leftClimberMotor.set(0);
        }
        if(canRightClimberGoDown){
            rightClimberMotor.set(-Constants.ClimberConstants.CLIMBER_SPEED);
        } else {
            rightClimberMotor.set(0);
        }
    }
    //Start Command
    public Command upCommand(){
        return Commands.runOnce(()-> up());
    }
    //stop Command
    public Command stopCommand(){
        return Commands.runOnce(()-> stop());
    }
    //reverse Command
    public Command downCommand(){
        return Commands.runOnce(()-> down());
    }

    public void periodic(){
        if (rightlimitSwitch.get()) {
            canRightClimberGoDown = false;
        } else {
            canRightClimberGoDown = true;
        }
        if (leftlimitSwitch.get()) {
            canLeftClimberGoDown = false;
        } else {
            canLeftClimberGoDown = true;
        }
    }
}
