package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase{

    SparkMax leftClimberMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
    SparkMax rightClimberMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    DigitalInput leftlimitSwitch = new DigitalInput(Constants.ClimberConstants.CLIMBER_LEFT_LIMIT_SWITCH_ID);
    DigitalInput rightlimitSwitch = new DigitalInput(Constants.ClimberConstants.CLIMBER_RIGHT_LIMIT_SWITCH_ID);

    public Climbers(){

        //leftClimberMotor.setInverted(true); //we might not need this Test Climbers and find out
        //rightClimberMotor.setInverted(true);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        rightClimberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    //Rais climbers (start climber motors foward)
    public void up(){
        leftClimberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED);
        rightClimberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED);
    }
    //stop the climber motors
    public void stop(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }
    //If possible lower climbers (start motors in the reverse dirction)
    public void down(){
        if(leftlimitSwitch.get()){ //this checks the limit switch to make sure it can go down further first. If it cant it stops instead of going further.
            leftClimberMotor.set(0);
        } else {
            leftClimberMotor.set(-Constants.ClimberConstants.CLIMBER_SPEED);
        }
        if(rightlimitSwitch.get()){
            rightClimberMotor.set(0);
        } else {
            rightClimberMotor.set(-Constants.ClimberConstants.CLIMBER_SPEED);
        }
    }

    /*Commands*/

    //Start Command
    public Command upCommand(){
        return Commands.runOnce(()-> up());
    }
    //stop Command
    public Command stopCommand(){
        return Commands.runOnce(()-> stop());
    }
    //down Command
    public Command downCommand(){
        return Commands.runOnce(()-> down());
    }
}
