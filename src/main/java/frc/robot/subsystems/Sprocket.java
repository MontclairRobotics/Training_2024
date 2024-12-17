package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Sprocket {
    
    private CANSparkMax leftNEO;
    private CANSparkMax rightNEO;

    public Sprocket() {
        leftNEO = new CANSparkMax(31, MotorType.kBrushless);
        rightNEO = new CANSparkMax(30, MotorType.kBrushless);
    }

    public void up() {
        leftNEO.set(Constants.SprocketConstants.SPROCKET_SPEED);
        rightNEO.set(Constants.SprocketConstants.SPROCKET_SPEED);
    }
    public void down() {
        leftNEO.set(-Constants.SprocketConstants.SPROCKET_SPEED);
        rightNEO.set(-Constants.SprocketConstants.SPROCKET_SPEED);
    }
    public void stop() {
        leftNEO.set(0);
        rightNEO.set(0);
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
