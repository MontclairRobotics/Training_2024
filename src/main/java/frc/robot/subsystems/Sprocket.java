package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Sprocket extends SubsystemBase{
    
    private CANSparkMax leftNEO;
    private CANSparkMax rightNEO;

    public Sprocket() {
        leftNEO = new CANSparkMax(30, MotorType.kBrushless);
        rightNEO = new CANSparkMax(31, MotorType.kBrushless);
    }

    public void inputFromController(CommandPS5Controller controller) {
        double speed = Math.pow(MathUtil.applyDeadband(controller.getLeftY(), Constants.OperatorConstants.OPERATOR_CONTROLLER_DEAD_BAND),3) - 0.2;
        move(speed);
    }

    public void move(double speed) {
        leftNEO.set(speed);
        rightNEO.set(speed);
    }

    //Button based sprocket below

    // public void up() {
    //     leftNEO.set(Constants.SprocketConstants.SPROCKET_SPEED);
    //     rightNEO.set(Constants.SprocketConstants.SPROCKET_SPEED);
    // }
    // public void down() {
    //     leftNEO.set(-Constants.SprocketConstants.SPROCKET_SPEED);
    //     rightNEO.set(-Constants.SprocketConstants.SPROCKET_SPEED);
    // }
    // public void stop() {
    //     leftNEO.set(0);
    //     rightNEO.set(0);
    // }

    /*Commands*/

    //Start Command
    // public Command upCommand(){
    //     return Commands.runOnce(()-> up());
    // }
    // //stop Command
    // public Command stopCommand(){
    //     return Commands.runOnce(()-> stop());
    // }
    // //down Command
    // public Command downCommand(){
    //     return Commands.runOnce(()-> down());
    // }
}
